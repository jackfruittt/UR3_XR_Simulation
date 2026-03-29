// References:
// Ark_Revan, "IMU Sensor and Quaternion", Unity Discussions, Jul 2017
// https://discussions.unity.com/t/imu-sensor-and-quaternion/190038
//
// emthele, "[Solved] Quaternion from IMU sensor to GameObject Orientation problem", Unity Discussions, Jun 2016
// https://discussions.unity.com/t/solved-quaternion-from-imu-sensor-to-gameobject-orientation-problem/167120

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// ROS IMU subscriber - fuses gyro + accel via 6-DOF Madgwick (no magnetometer).
///  - Orientation from Madgwick + position from AprilTag (Detection.cs) -> full 6-DOF pose.
///  - Last known tag position held when tag not visible (double-integration drift makes IMU position unreliable).
///  - Pose written to EEFTargetController each FixedUpdate for JacobianIKSolver to follow.
public class IMUSubscriber : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("D455 IMU topic. Default: /camera/camera/imu")]
    public string imuTopic = "/camera/camera/imu";

    [Header("References")]
    public EEFTargetController targetController;
    public Detection detection; // for AprilTag position fusion

    [Header("AprilTag Fusion")]
    [Tooltip("ID of the AprilTag used as the world-space position anchor.")]
    public int targetTagId = 0;
    [Tooltip("World-space Transform placed at the physical location of the AprilTag anchor in the scene.")]
    public Transform tagWorldAnchor;

    [Header("Madgwick Filter")]
    [Tooltip("Must match the actual IMU publish rate set in the RealSense launch file. " +
             "D455 options: gyro 200/400 Hz, accel 63/250 Hz. Wrong value causes drift. [1]")]
    public float sampleFrequency = 200f;
    [Tooltip("Filter gain beta. Higher = faster convergence but noisier. 0.033 is Madgwick's default.")]
    public float beta = 0.033f;

    [Header("IMU-to-Camera Extrinsic")]
    [Tooltip("Factory-calibrated rotation from IMU frame to colour optical frame. " +
             "Read from /camera/camera/imu_optical_frame TF or pyrealsense2 get_extrinsics_to(). " +
             "Leave identity until measured — axis misalignment will be visible immediately.")]
    public Vector3 imuToCameraRotationEuler = Vector3.zero;

    // Madgwick filter state: quaternion [w, x, y, z]
    float _q0 = 1f, _q1 = 0f, _q2 = 0f, _q3 = 0f;

    // Thread-safe queue: ROS callback enqueues, FixedUpdate dequeues
    readonly Queue<ImuMsg> _msgQueue = new Queue<ImuMsg>();

    // Last known world-space position from AprilTag (held when tag not visible)
    Vector3 _lastTagWorldPos;

    ROSConnection _ros;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<ImuMsg>(imuTopic, EnqueueIMUMessage);
    }

    // ROS callback, enqueue only, never touch Unity objects here (wrong thread)
    void EnqueueIMUMessage(ImuMsg msg)
    {
        // lock _msgQueue and enqueue msg
        lock (_msgQueue)
        {
            _msgQueue.Enqueue(msg);
        }
    }

    // FixedUpdate per [1]: consistent timestep is critical for filter stability
    void FixedUpdate()
    {
        // dequeue one message (lock _msgQueue)
        ImuMsg msg = null;
        lock (_msgQueue)
        {
            if (_msgQueue.Count > 0)
            {
                msg = _msgQueue.Dequeue();
            }
        }

        if (msg == null) return;

        // 1. Extract gyro + accel from msg
        Vector3 gyro  = new Vector3((float)msg.angular_velocity.x,    (float)msg.angular_velocity.y,    (float)msg.angular_velocity.z);
        Vector3 accel = new Vector3((float)msg.linear_acceleration.x, (float)msg.linear_acceleration.y, (float)msg.linear_acceleration.z);

        // 2. Remap axes ROS IMU frame -> Unity frame (RemapROSIMUToUnity)
            Vector3 gyroUnity = RemapROSIMUToUnity(gyro);
            Vector3 accelUnity = RemapROSIMUToUnity(accel);
    
            // Debug.Log($"Gyro (rad/s): {gyroUnity}, Accel (m/s²): {accelUnity}");

        // 3. Call MadgwickUpdate6DOF(gx, gy, gz, ax, ay, az, 1f / sampleFrequency)
        MadgwickUpdate6DOF(gyroUnity.x, gyroUnity.y, gyroUnity.z,
                          accelUnity.x, accelUnity.y, accelUnity.z,
                          1f / sampleFrequency);

        // 4. Build Quaternion from _q0/_q1/_q2/_q3
        Quaternion imuOrientation = new Quaternion(_q1, _q2, _q3, _q0); // Unity's Quaternion constructor is (x, y, z, w)

        // 5. Quaternion.Inverse() per [2] - filter outputs earth-wrt-sensor, not sensor-wrt-earth
        Quaternion cameraOrientation = Quaternion.Inverse(imuOrientation);

        // 6. Apply imuToCameraRotationEuler extrinsic
        cameraOrientation *= Quaternion.Euler(imuToCameraRotationEuler);

        // 7. Call GetFusedPosition() for world-space position
        Vector3 cameraPosition = GetFusedPosition();

        // 8. Write pose to targetController (needs SetTarget method on EEFTargetController)
        targetController.SetTarget(cameraPosition, cameraOrientation);

    }

    /// 6-DOF Madgwick filter (gyro + accel, no magnetometer; D455 has none).
    /// gyro: rad/s in Unity frame. accel: m/s^2 in Unity frame.
    /// Updates _q0/_q1/_q2/_q3 in place.
    void MadgwickUpdate6DOF(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt)
    {
        // 1. Normalise accel (skip update if near-zero - free-fall / bad data)
        float accelNorm = Mathf.Sqrt(ax * ax + ay * ay + az * az);
        if (accelNorm < 1e-6f) return; // avoid division
        ax /= accelNorm;
        ay /= accelNorm;
        az /= accelNorm;

        // 2. Compute gradient descent step s0..s3 from objective f = estimated_gravity - measured_gravity
        //    where estimated_gravity = [2(q1q3 - q0q2), 2(q0q1 + q2q3), q0^2 - q1^2 - q2^2 + q3^2]
        //    and measured_gravity = [ax, ay, az]
        float f1 = 2f * (_q1 * _q3 - _q0 * _q2) - ax;
        float f2 = 2f * (_q0 * _q1 + _q2 * _q3) - ay;
        
        // q0^2-q1^2-q2^2+q3^2 == 1-2q1^2-2q2^2 for a unit quaternion - matches Madgwick f3 exactly.
        float f3 = (_q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3) - az;
        float s0 = -2f * _q2 * f1 + 2f * _q1 * f2;
        float s1 =  2f * _q3 * f1 + 2f * _q0 * f2 - 4f * _q1 * f3;
        float s2 = -2f * _q0 * f1 + 2f * _q3 * f2 - 4f * _q2 * f3;
        float s3 =  2f * _q1 * f1 + 2f * _q2 * f2;

        // 3. Normalise s0..s3
        float sNorm = Mathf.Sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sNorm > 1e-6f) // avoid division
        {
            s0 /= sNorm;
            s1 /= sNorm;
            s2 /= sNorm;
            s3 /= sNorm;
        }

        // 4. Compute qDot from gyro integration minus beta * gradient step
        float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz) - beta * s0;
        float qDot1 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy) - beta * s1;
        float qDot2 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx) - beta * s2;
        float qDot3 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx) - beta * s3; 

        // 5. Integrate qDot * dt into _q0.._q3
        _q0 += qDot0 * dt;
        _q1 += qDot1 * dt;
        _q2 += qDot2 * dt;
        _q3 += qDot3 * dt;  

        // 6. Normalise _q0.._q3
        float qNorm = Mathf.Sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
        if (qNorm > 1e-6f) // avoid division
        {
            _q0 /= qNorm;
            _q1 /= qNorm;
            _q2 /= qNorm;
            _q3 /= qNorm;
        }
    }

    /// Remaps a vector from the D455 IMU frame to Unity world frame.
    /// Verify axis signs against /camera/camera/imu_optical_frame TF when connected.
    static Vector3 RemapROSIMUToUnity(Vector3 ros)
    {
        // D455 IMU frame: x forward, y left, z up (right-handed)
        // Unity world frame: x right, y up, z forward (left-handed)
        return new Vector3(ros.y, ros.z, ros.x);
    }

    /// Returns world-space camera position.
    /// Uses AprilTag absolute fix when visible; holds last known position otherwise.
    Vector3 GetFusedPosition()
    {
        // Iterate detection.DetectedTags
        // tag.Position = tag position in camera space (from AprilTag detector)
        // tagWorldAnchor.position = known world-space position of the physical tag
        // camera_world_pos = tag_world_pos - camera_rotation * tag_camera_space_pos
        if (detection != null && detection.DetectedTags != null)
        {
            foreach (var tag in detection.DetectedTags)
            {
                if (tag.ID == targetTagId)
                {
                    Vector3 tagWorldPos = tagWorldAnchor != null ? tagWorldAnchor.position : _lastTagWorldPos;
                    _lastTagWorldPos = tagWorldPos - targetController.transform.rotation * tag.Position;
                    return _lastTagWorldPos;
                }
            }
        }
        return _lastTagWorldPos; // hold last known position when tag not visible
    }
}
