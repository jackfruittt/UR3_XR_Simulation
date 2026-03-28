// References:
// [1] Ark_Revan, "IMU Sensor and Quaternion", Unity Discussions, Jul 2017
//     https://discussions.unity.com/t/imu-sensor-and-quaternion/190038
//     Key lesson: run Madgwick in FixedUpdate; sampleFreq MUST match actual IMU publish rate
//     (D455 gyro: 200 or 400 Hz, accel: 63 or 250 Hz). Wrong frequency causes drift or spin.
//
// [2] emthele, "[Solved] Quaternion from IMU sensor to GameObject Orientation problem", Unity Discussions, Jun 2016
//     https://discussions.unity.com/t/solved-quaternion-from-imu-sensor-to-gameobject-orientation-problem/167120
//     Key lesson: Madgwick outputs orientation of earth-frame w.r.t. sensor, NOT sensor w.r.t. earth.
//     Must call Quaternion.Inverse() on the result to get the actual sensor orientation in world space.

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// Subscribes to /camera/camera/imu (sensor_msgs/Imu) and fuses gyroscope +
/// accelerometer via a 6-DOF Madgwick filter (no magnetometer; D455 has none).
///
/// Fused IMU orientation is combined with AprilTag absolute position from
/// Detection.cs to produce a full 6DOF camera pose. When the tag is not
/// visible, the last known position is held (IMU alone cannot give reliable
/// position due to double-integration drift).
///
/// Output is written to EEFTargetController each FixedUpdate so the
/// JacobianIKSolver drives the sim arm to follow the physical camera.
public class IMUSubscriber : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("D455 IMU topic. Default: /camera/camera/imu")]
    public string imuTopic = "/camera/camera/imu";

    [Header("References")]
    public EEFTargetController targetController;
    public Detection detection;           // for AprilTag position fusion

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
        // TODO: get ROSConnection, subscribe imuTopic -> EnqueueIMUMessage
        // TODO: auto-wire targetController and detection if null
    }

    // ROS callback, enqueue only, never touch Unity objects here (wrong thread)
    void EnqueueIMUMessage(ImuMsg msg)
    {
        // TODO: lock _msgQueue and enqueue msg
    }

    // FixedUpdate per [1]: consistent timestep is critical for filter stability
    void FixedUpdate()
    {
        // TODO: dequeue one message (lock _msgQueue)
        // TODO: extract gyro + accel from msg
        // TODO: remap axes ROS IMU frame → Unity frame (RemapROSIMUToUnity)
        // TODO: call MadgwickUpdate6DOF(gx, gy, gz, ax, ay, az, 1f / sampleFrequency)
        // TODO: build Quaternion from _q0/_q1/_q2/_q3
        // TODO: Quaternion.Inverse() per [2] - filter outputs earth-wrt-sensor, not sensor-wrt-earth
        // TODO: apply imuToCameraRotationEuler extrinsic
        // TODO: call GetFusedPosition() for world-space position
        // TODO: write pose to targetController (needs SetTarget method on EEFTargetController)
    }

    /// 6-DOF Madgwick filter (gyro + accel, no magnetometer; D455 has none).
    /// gyro: rad/s in Unity frame. accel: m/s² in Unity frame.
    /// Updates _q0/_q1/_q2/_q3 in place.
    void MadgwickUpdate6DOF(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt)
    {
        // TODO: normalise accel (skip update if near-zero - free-fall / bad data)
        // TODO: compute gradient descent step s0..s3 from objective f = estimated_gravity - measured_gravity
        // TODO: normalise s0..s3
        // TODO: compute qDot from gyro integration minus beta * gradient step
        // TODO: integrate qDot * dt into _q0.._q3
        // TODO: normalise _q0.._q3
    }

    /// Remaps a vector from the D455 IMU frame to Unity world frame.
    /// Verify axis signs against /camera/camera/imu_optical_frame TF when connected.
    static Vector3 RemapROSIMUToUnity(Vector3 ros)
    {
        // TODO
        return Vector3.zero;
    }

    /// Returns world-space camera position.
    /// Uses AprilTag absolute fix when visible; holds last known position otherwise.
    Vector3 GetFusedPosition()
    {
        // TODO: iterate detection.DetectedTags
        // TODO: if tag visible, compute camera world pos from known tag world Transform
        //       camera world pos = tagWorldTransform.position - cameraOrientation * tagCameraSpacePos
        // TODO: update _lastTagWorldPos and return it
        return _lastTagWorldPos;
    }
}
