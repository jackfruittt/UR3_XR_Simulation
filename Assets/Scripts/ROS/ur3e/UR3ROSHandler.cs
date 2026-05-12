// Author: Jackson Russell

using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using UnityEngine;

/// Centralised ROS interface for the UR3e.
///
/// Provides a single point of control for:
///   - Robot mode selection (local physics / ROS sim / real hardware).
///   - All topic string configuration (robot and camera).
///   - Joint command routing: LocalPhysics drives ArticulationBodies directly;
///     ROSSimulation and RealHardware publish commands over ROS-TCP.
///   - Joint state reception: fires OnJointStateReceived in all modes; updates
///     ArticulationBody drives in LocalPhysics and ROSSimulation modes.
///
/// Usage:
///   Access via UR3ROSHandler.Instance from any MonoBehaviour.
///   Call SendJointCommand(degrees) without caring about the current mode.
///   Subscribe to OnJointStateReceived for live joint feedback.
///
/// Camera topic strings are exposed as read-only properties so SimpleImageSubscriber,
/// ROSPointCloudRenderer, and IMUSubscriber can optionally read from here rather than
/// duplicating hardcoded strings. Those scripts continue to work unchanged if you
/// prefer to keep their own inspector values.
public class UR3ROSHandler : MonoBehaviour
{
    // Singleton

    public static UR3ROSHandler Instance { get; private set; }

    // Robot mode

    /// Controls how joint commands are dispatched and how joint state feedback
    /// is applied to the scene.
    public enum RobotMode
    {
        /// Commands drive ArticulationBody targets directly. No ROS publisher
        /// needed. Joint state feedback from ROS is still accepted and applies
        /// physics drives when received (useful for monitoring real hardware while
        /// controlling the virtual arm independently).
        LocalPhysics,

        /// Commands are published to ROS. The ROS side forwards them to a
        /// simulator (e.g. UR Sim, Gazebo). Joint state feedback from ROS is
        /// applied to the ArticulationBodies for visual tracking.
        ROSSimulation,

        /// Commands are published to ROS. The ROS side forwards them to the
        /// real UR3e controller. Joint state feedback mirrors the physical arm.
        RealHardware,
    }

    [Header("Robot Mode")]
    [SerializeField] RobotMode _mode = RobotMode.LocalPhysics;

    /// Current robot mode. Change via SetMode() to trigger re-registration.
    public RobotMode Mode => _mode;

    // Robot topics

    [Header("Robot Topics")]
    [Tooltip("Joint state topic published by the robot driver or simulator.")]
    public string jointStateTopic = "/joint_states";

    [Tooltip("Joint command topic consumed by the robot driver or simulator. " +
             "On a real UR3e with ur_robot_driver this is typically " +
             "/scaled_joint_trajectory_controller/... but /unity_joint_commands " +
             "works when a relay node remaps to the correct action server.")]
    public string jointCommandTopic = "/unity_joint_commands";

    [Header("Servo (RealHardware mode)")]
    [Tooltip("MoveIt Servo incoming Cartesian velocity topic. Must match servo_node config.")]
    public string servoTwistTopic = "/servo_node/delta_twist_cmds";

    [Tooltip("Seconds without a /joint_states message before IsRobotConnected reports false.")]
    public float robotConnectedTimeoutSec = 2f;

    // Camera topics (read-only properties for external subscribers)

    [Header("Camera Topics")]
    [Tooltip("D455 colour image (sensor_msgs/Image, rgb8).")]
    public string colorTopic = "/camera/camera/color/image_raw";

    [Tooltip("D455 colour image compressed (sensor_msgs/CompressedImage, jpeg).")]
    public string colorCompressedTopic = "/camera/camera/color/image_raw/compressed";

    [Tooltip("D455 depth image (sensor_msgs/Image, 16UC1, millimetres).")]
    public string depthTopic = "/camera/camera/depth/image_rect_raw";

    [Tooltip("D455 depth aligned to colour frame (sensor_msgs/Image, 16UC1).")]
    public string alignedDepthTopic = "/camera/camera/aligned_depth_to_color/image_raw";

    [Tooltip("D455 colour camera intrinsics (sensor_msgs/CameraInfo).")]
    public string cameraInfoTopic = "/camera/camera/color/camera_info";

    [Tooltip("D455 gyroscope (sensor_msgs/Imu). Use the _relay variant when the imu_qos_relay " +
             "node is running to bridge BEST_EFFORT to RELIABLE QoS.")]
    public string gyroTopic = "/camera/camera/gyro/sample_relay";

    [Tooltip("D455 accelerometer (sensor_msgs/Imu). Use the _relay variant.")]
    public string accelTopic = "/camera/camera/accel/sample_relay";

    // Dependencies

    [Header("Dependencies")]
    [Tooltip("UR3SourceDestinationPublisher that owns the ArticulationBody chain. " +
             "Auto-wired if left empty.")]
    [SerializeField] UR3SourceDestinationPublisher _publisher;

    // Events

    /// Fired on the main thread whenever a /joint_states message is received.
    /// Argument is a 6-element array of joint positions in degrees,
    /// ordered: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.
    public event Action<float[]> OnJointStateReceived;

    // Private state

    ROSConnection _ros;
    bool          _rosRegistered       = false;
    bool          _servoRegistered     = false;
    float         _lastJointStateTime  = -999f;
    readonly float[] _jointStateDeg = new float[6];

    static readonly string[] JointNames = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    // Properties

    /// ArticulationBody array from the underlying publisher.
    public ArticulationBody[] JointBodies => _publisher != null ? _publisher.JointBodies : null;

    /// True when the ROS-TCP connection exists and has been initialised.
    public bool IsROSActive => _ros != null;

    /// True when a /joint_states message has arrived within robotConnectedTimeoutSec.
    /// Used by JacobianIKSolver to gate servo publishing and detect connection loss.
    public bool IsRobotConnected =>
        (UnityEngine.Time.realtimeSinceStartup - _lastJointStateTime) < robotConnectedTimeoutSec;

    // MonoBehaviour

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning("[UR3ROSHandler] Duplicate instance destroyed.");
            Destroy(gameObject);
            return;
        }
        Instance = this;

        if (_publisher == null)
            _publisher = FindObjectOfType<UR3SourceDestinationPublisher>();
    }

    void Start()
    {
        RegisterROS();
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    // Public interface

    /// Send a 6-joint command (degrees). Routing is determined by the current mode:
    ///   LocalPhysics   -> ArticulationBody drives updated directly.
    ///   ROSSimulation  -> published to jointCommandTopic.
    ///   RealHardware   -> published to jointCommandTopic.
    public void SendJointCommand(float[] deg)
    {
        if (deg == null || deg.Length != 6)
        {
            Debug.LogError("[UR3ROSHandler] SendJointCommand requires a 6-element array.");
            return;
        }

        if (_mode == RobotMode.LocalPhysics)
        {
            for (int i = 0; i < 6; i++)
                SetJointAngleLocally(i, deg[i]);
        }
        else
        {
            PublishJointCommandROS(deg);
        }
    }

    /// Set a single joint to a target angle (degrees), always via the
    /// ArticulationBody regardless of mode. Use this for direct local overrides
    /// (e.g. homing in simulation before a calibration session).
    public void SetJointAngleLocally(int jointIndex, float angleDeg)
    {
        if (_publisher != null)
            _publisher.SetJointAngleLocally(jointIndex, angleDeg);
    }

    /// Move all joints to the home configuration. Routing follows current mode.
    public void MoveHome()
    {
        float[] home = GetHomePosition();
        if (home == null) return;

        if (_mode == RobotMode.LocalPhysics)
            _publisher?.MoveToHomePosition();
        else
            PublishJointCommandROS(home);
    }

    /// Returns a copy of the home joint configuration (degrees).
    public float[] GetHomePosition()
        => _publisher != null ? _publisher.GetHomePosition() : null;

    /// Fills buffer with actual physics joint positions (degrees, length 6).
    /// Returns false if the publisher or bodies are not yet initialised.
    public bool GetActualJointAnglesInto(float[] buffer)
        => _publisher != null && _publisher.GetActualJointAnglesInto(buffer);

    /// Returns actual physics joint positions as a new array. Prefer
    /// GetActualJointAnglesInto in hot paths to avoid heap allocation.
    public float[] GetActualJointAngles()
        => _publisher != null ? _publisher.GetActualJointAngles() : null;

    /// Switch the robot mode at runtime. Triggers ROS re-registration when
    /// switching into or out of a ROS mode.
    public void SetMode(RobotMode newMode)
    {
        if (newMode == _mode) return;

        bool wasROS = _mode != RobotMode.LocalPhysics;
        bool willROS = newMode != RobotMode.LocalPhysics;

        _mode = newMode;

        // Keep UR3SourceDestinationPublisher's own manual-control flag in sync.
        // manualControlMode = true prevents joint states from overriding drives.
        if (_publisher != null)
            _publisher.manualControlMode = (_mode == RobotMode.LocalPhysics);

        // Re-register publisher if transitioning between local and ROS modes.
        if (willROS && !wasROS)
            EnsurePublisherRegistered();

        Debug.Log("[UR3ROSHandler] Mode set to: " + _mode);
    }

    // ROS setup

    void RegisterROS()
    {
        _ros = ROSConnection.GetOrCreateInstance();

        // Joint state subscription is active in all modes.
        // It provides feedback regardless of command routing.
        _ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateMsg);

        if (_mode != RobotMode.LocalPhysics)
            EnsurePublisherRegistered();

        // Keep UR3SourceDestinationPublisher's manual-control flag consistent.
        if (_publisher != null)
            _publisher.manualControlMode = (_mode == RobotMode.LocalPhysics);

        Debug.Log("[UR3ROSHandler] ROS registered. Mode=" + _mode
            + "  jointState=" + jointStateTopic
            + "  jointCmd=" + jointCommandTopic);
    }

    void EnsurePublisherRegistered()
    {
        if (_rosRegistered) return;
        if (_ros == null) _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<JointStateMsg>(jointCommandTopic);
        _rosRegistered = true;
    }

    // Joint state reception

    void OnJointStateMsg(JointStateMsg msg)
    {
        // Parse the message into a degrees array in the fixed joint order.
        for (int i = 0; i < JointNames.Length; i++)
        {
            for (int j = 0; j < msg.name.Length; j++)
            {
                if (msg.name[j] == JointNames[i])
                {
                    _jointStateDeg[i] = (float)msg.position[j] * Mathf.Rad2Deg;
                    break;
                }
            }
        }

        // In LocalPhysics mode the ArticulationBody drives are the source of
        // truth. Do not overwrite them with incoming joint states since that
        // would create a feedback conflict with the IK solver.
        if (_mode != RobotMode.LocalPhysics && _publisher != null)
            ApplyJointStateToBodies(msg);

        // Record the timestamp so IsRobotConnected can detect connection loss.
        _lastJointStateTime = UnityEngine.Time.realtimeSinceStartup;

        // Fire event for any listener (IK, calibration collector, HUD, etc.)
        OnJointStateReceived?.Invoke(_jointStateDeg);
    }

    void ApplyJointStateToBodies(JointStateMsg msg)
    {
        ArticulationBody[] bodies = _publisher.JointBodies;
        if (bodies == null) return;

        for (int i = 0; i < JointNames.Length; i++)
        {
            if (bodies[i] == null) continue;
            for (int j = 0; j < msg.name.Length; j++)
            {
                if (msg.name[j] == JointNames[i])
                {
                    var drive    = bodies[i].xDrive;
                    drive.target = Mathf.Rad2Deg * (float)msg.position[j];
                    bodies[i].xDrive = drive;
                    break;
                }
            }
        }
    }

    // Servo Cartesian velocity publishing

    // Publishes linearROS and angularROS as a TwistStamped to MoveIt Servo.
    // Both vectors must already be in ROS FLU coordinates in the base_link frame.
    public void SendServoTwist(Vector3 linearUnity, Vector3 angularUnity)
    {
        if (_ros == null)
        {
            Debug.LogWarning("[UR3ROSHandler] Cannot publish servo twist: ROS not initialised.");
            return;
        }

        EnsureServoPublisherRegistered();

        var linearROS  = linearUnity.To<FLU>();
        var angularROS = angularUnity.To<FLU>();

        var msg = new TwistStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "base_link",
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec     = (int)Time.time,
                    nanosec = (uint)((Time.time - (int)Time.time) * 1e9f)
                }
            },
            twist = new TwistMsg
            {
                linear  = new RosMessageTypes.Geometry.Vector3Msg(linearROS.x,  linearROS.y,  linearROS.z),
                angular = new RosMessageTypes.Geometry.Vector3Msg(angularROS.x, angularROS.y, angularROS.z)
            }
        };

        _ros.Publish(servoTwistTopic, msg);
    }

    void EnsureServoPublisherRegistered()
    {
        if (_servoRegistered) return;
        if (_ros == null) _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<TwistStampedMsg>(servoTwistTopic);
        _servoRegistered = true;
    }

    // Joint command publishing

    void PublishJointCommandROS(float[] deg)
    {
        if (_ros == null)
        {
            Debug.LogWarning("[UR3ROSHandler] Cannot publish: ROS connection not initialised.");
            return;
        }

        EnsurePublisherRegistered();

        var msg = new JointStateMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec      = (int)Time.time,
                    nanosec  = (uint)((Time.time - (int)Time.time) * 1e9f)
                }
            },
            name     = JointNames,
            position = new double[6],
            velocity = new double[6],
            effort   = new double[6]
        };

        for (int i = 0; i < 6; i++)
            msg.position[i] = deg[i] * Mathf.Deg2Rad;

        _ros.Publish(jointCommandTopic, msg);
    }
}
