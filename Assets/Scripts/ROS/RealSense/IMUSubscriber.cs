// Author: Jackson Russell

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// Subscribes to RealSense D455 gyro and accel topics and fuses them with a
/// 6-DOF Madgwick filter (Y-up gravity convention) to produce a world-space pose.
public class IMUSubscriber : MonoBehaviour
{
    [Header("ROS Topics")]
    [Tooltip("Gyroscope topic (sensor_msgs/Imu).")]
    public string gyroTopic  = "/camera/camera/gyro/sample";
    [Tooltip("Accelerometer topic (sensor_msgs/Imu).")]
    public string accelTopic = "/camera/camera/accel/sample";

    [Header("References")]
    [Tooltip("EEF controller to drive when Free-Hand Mode is off.")]
    public EEFTargetController targetController;
    [Tooltip("AprilTag detector used for position fusion.")]
    public Detection detection;

    [Header("Free-Hand Mode")]
    [Tooltip("Apply IMU pose directly to FreeHandCameraTransform instead of the robot arm.")]
    public bool freeHandMode = false;
    [Tooltip("Transform driven in Free-Hand Mode (e.g. ROSPointCloudRenderer root).")]
    public Transform freeHandCameraTransform;

    [Header("AprilTag Fusion")]
    [Tooltip("ID of the AprilTag used as the world-space position anchor.")]
    public int targetTagId = 0;
    [Tooltip("World-space scene Transform at the physical AprilTag location.")]
    public Transform tagWorldAnchor;

    [Header("Madgwick Filter")]
    [Tooltip("Gyro publish rate (Hz). Used as initial dt before first messages arrive.")]
    public float sampleFrequency = 200f;
    [Tooltip("Filter gain beta — higher corrects drift faster but increases noise.")]
    public float beta = 0.05f;

    [Header("Gyro Bias Calibration")]
    [Tooltip("Number of gyro samples to average for zero-rate bias. Camera must be still. 0 = skip.")]
    public int calibrationSamples = 200;
    [HideInInspector] public Vector3 gyroBias;

    [Header("Axis Mapping — D455 to Unity")]
    [Tooltip("Sign multipliers for raw gyro axes. Default (1,1,-1) maps D455 Z-backward to Unity Z-forward.")]
    public Vector3 gyroAxisScale  = new Vector3(1f, 1f, -1f);
    [Tooltip("Sign multipliers for raw accel axes. At rest Y should read ≈ +9.81 m/s².")]
    public Vector3 accelAxisScale = new Vector3(1f, 1f, -1f);

    [Header("IMU-to-Camera Extrinsic")]
    [Tooltip("Euler offset (degrees) aligning IMU output to the colour optical frame.")]
    public Vector3 imuToCameraRotationEuler = Vector3.zero;

    [Header("Floor Mapping")]
    [Tooltip("Camera height above floor at startup (metres). Seeds world position before first tag fix.")]
    public float initialCameraHeight = 0.915f;

    // Filter state
    float _q0 = 1f, _q1 = 0f, _q2 = 0f, _q3 = 0f;

    readonly Queue<Vector3> _gyroQueue = new Queue<Vector3>();

    volatile float _ax, _ay, _az;
    volatile bool  _hasAccel;

    float _gyroDt;
    float _lastGyroTimestamp = -1f;

    Vector3 _biasAccum;
    int     _biasSamplesCollected;
    bool    _biasCalibrated;

    Vector3       _lastTagWorldPos;
    ROSConnection _ros;

    const int k_ImuWindowSize = 120;
    float[]   _imuTimestamps  = new float[k_ImuWindowSize];
    int       _imuTsHead      = 0;
    int       _imuTsCount     = 0;

    // Public properties

    /// World-space position of the D455 colour optical centre (metres).
    public Vector3    CameraWorldPosition { get; private set; }

    /// World-space orientation of the D455 optical frame.
    public Quaternion CameraWorldRotation { get; private set; } = Quaternion.identity;

    /// True once the first gyro+accel pair and AprilTag fix have been processed.
    public bool PoseValid { get; private set; }

    /// Gyro message receive rate (Hz), rolling 120-sample window.
    public float IMUReceiveHz { get; private set; } = 0f;

    /// Current filter output as signed Euler angles (degrees).
    public Vector3 FilterEulerAngles { get; private set; }

    /// Last raw accel in Unity frame (m/s^2). Should be ~=(0,+9.81,0) at rest.
    public Vector3 LastAccelUnity { get; private set; }

    /// Gyro messages queued but not yet integrated.
    public int IMUMsgQueueDepth { get { lock (_gyroQueue) return _gyroQueue.Count; } }

    /// True when Free-Hand Mode is active.
    public bool IsHandHeld => freeHandMode;

    // Lifecycle

    void Start()
    {
        _gyroDt = 1f / sampleFrequency;
        _lastTagWorldPos    = new Vector3(0f, initialCameraHeight, 0f);
        CameraWorldPosition = _lastTagWorldPos;

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<ImuMsg>(gyroTopic.Trim(),  OnGyroReceived);
        _ros.Subscribe<ImuMsg>(accelTopic.Trim(), OnAccelReceived);

        Debug.Log($"[IMUSubscriber] gyro={gyroTopic.Trim()}  accel={accelTopic.Trim()}  "
                + $"height={initialCameraHeight:F3} m");
    }

    // ROS callbacks

    void OnGyroReceived(ImuMsg msg)
    {
        Vector3 raw   = new Vector3((float)msg.angular_velocity.x,
                                    (float)msg.angular_velocity.y,
                                    (float)msg.angular_velocity.z);
        Vector3 unity = new Vector3(raw.x * gyroAxisScale.x,
                                    raw.y * gyroAxisScale.y,
                                    raw.z * gyroAxisScale.z);

        if (calibrationSamples > 0 && !_biasCalibrated)
        {
            _biasAccum += unity;
            if (++_biasSamplesCollected >= calibrationSamples)
            {
                gyroBias        = _biasAccum / _biasSamplesCollected;
                _biasCalibrated = true;
                Debug.Log($"[IMUSubscriber] Gyro bias: ({gyroBias.x:F4}, {gyroBias.y:F4}, {gyroBias.z:F4}) rad/s");
            }
            return;
        }

        unity -= gyroBias;
        lock (_gyroQueue) _gyroQueue.Enqueue(unity);

        float t = Time.realtimeSinceStartup;
        if (_lastGyroTimestamp > 0f)
        {
            float dt = t - _lastGyroTimestamp;
            if (dt > 0.001f && dt < 1f) _gyroDt = dt;
        }
        _lastGyroTimestamp = t;
        RecordImuTimestamp(t);
    }

    void OnAccelReceived(ImuMsg msg)
    {
        _ax = (float)msg.linear_acceleration.x * accelAxisScale.x;
        _ay = (float)msg.linear_acceleration.y * accelAxisScale.y;
        _az = (float)msg.linear_acceleration.z * accelAxisScale.z;
        _hasAccel = true;
    }

    void RecordImuTimestamp(float t)
    {
        _imuTimestamps[_imuTsHead] = t;
        _imuTsHead = (_imuTsHead + 1) % k_ImuWindowSize;
        if (_imuTsCount < k_ImuWindowSize) _imuTsCount++;
        if (_imuTsCount >= 2)
        {
            float oldest = _imuTimestamps[_imuTsHead % k_ImuWindowSize];
            float span   = t - oldest;
            if (span > 0f) IMUReceiveHz = (_imuTsCount - 1) / span;
        }
    }

    // FixedUpdate

    void FixedUpdate()
    {
        if (!_hasAccel) return;

        bool anyUpdate = false;
        while (true)
        {
            Vector3 gyro;
            lock (_gyroQueue)
            {
                if (_gyroQueue.Count == 0) break;
                gyro = _gyroQueue.Dequeue();
            }
            MadgwickUpdate6DOF_YUp(gyro.x, gyro.y, gyro.z, _ax, _ay, _az, _gyroDt);
            anyUpdate = true;
        }
        if (!anyUpdate) return;

        Quaternion filterQ           = new Quaternion(_q1, _q2, _q3, _q0);
        Quaternion cameraOrientation = Quaternion.Inverse(filterQ);
        cameraOrientation           *= Quaternion.Euler(imuToCameraRotationEuler);

        Vector3 cameraPosition = GetFusedPosition(cameraOrientation);

        if (freeHandMode)
        {
            // Guard: never rotate the transform if it lives under a FirstPersonCamera rig.
            // In that case freeHandCameraTransform is mis-wired to the player camera;
            // driving it would snap/spin the view whenever the FPS cursor is not locked.
            bool underFPSRig = freeHandCameraTransform != null &&
                               freeHandCameraTransform.GetComponentInParent<FirstPersonCamera>() != null;
            if (freeHandCameraTransform != null && !underFPSRig &&
                Cursor.lockState != CursorLockMode.Locked)
                freeHandCameraTransform.rotation = cameraOrientation;
        }
        else
        {
            if (targetController != null)
                targetController.SetTarget(cameraPosition, cameraOrientation);
        }

        CameraWorldPosition = cameraPosition;
        CameraWorldRotation = cameraOrientation;
        PoseValid           = true;

        Vector3 eul = cameraOrientation.eulerAngles;
        eul.x = eul.x > 180f ? eul.x - 360f : eul.x;
        eul.y = eul.y > 180f ? eul.y - 360f : eul.y;
        eul.z = eul.z > 180f ? eul.z - 360f : eul.z;
        FilterEulerAngles = eul;
        LastAccelUnity    = new Vector3(_ax, _ay, _az);
    }

    // Madgwick 6-DOF (Y-up gravity reference)
    // Objective function with g_ref = [0,1,0]:
    //   f1 = 2(q1*q2 - q0*q3) - ax
    //   f2 = 1 - 2(q1^2 + q3^2)  - ay
    //   f3 = 2(q2*q3 + q0*q1) - az
    void MadgwickUpdate6DOF_YUp(float gx, float gy, float gz,
                                 float ax, float ay, float az, float dt)
    {
        float aNorm = Mathf.Sqrt(ax*ax + ay*ay + az*az);
        if (aNorm < 1e-6f) return;
        ax /= aNorm; ay /= aNorm; az /= aNorm;

        float f1 = 2f * (_q1 * _q2 - _q0 * _q3) - ax;
        float f2 = 1f - 2f * (_q1 * _q1 + _q3 * _q3) - ay;
        float f3 = 2f * (_q2 * _q3 + _q0 * _q1) - az;

        float s0 = -2f * _q3 * f1                    + 2f * _q1 * f3;
        float s1 =  2f * _q2 * f1 - 4f * _q1 * f2   + 2f * _q0 * f3;
        float s2 =  2f * _q1 * f1                    + 2f * _q3 * f3;
        float s3 = -2f * _q0 * f1 - 4f * _q3 * f2   + 2f * _q2 * f3;

        float sNorm = Mathf.Sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (sNorm > 1e-6f) { s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm; }

        float qDot0 = 0.5f * (-_q1*gx - _q2*gy - _q3*gz) - beta * s0;
        float qDot1 = 0.5f * ( _q0*gx + _q2*gz - _q3*gy) - beta * s1;
        float qDot2 = 0.5f * ( _q0*gy - _q1*gz + _q3*gx) - beta * s2;
        float qDot3 = 0.5f * ( _q0*gz + _q1*gy - _q2*gx) - beta * s3;

        _q0 += qDot0 * dt; _q1 += qDot1 * dt;
        _q2 += qDot2 * dt; _q3 += qDot3 * dt;

        float qNorm = Mathf.Sqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
        if (qNorm > 1e-6f) { _q0 /= qNorm; _q1 /= qNorm; _q2 /= qNorm; _q3 /= qNorm; }
    }

    // Position fusion

    Vector3 GetFusedPosition(Quaternion currentCamOrientation)
    {
        if (detection?.DetectedTags != null)
        {
            foreach (var tag in detection.DetectedTags)
            {
                if (tag.ID != targetTagId) continue;
                Vector3 tagWorldPos = tagWorldAnchor != null
                    ? tagWorldAnchor.position
                    : _lastTagWorldPos;
                _lastTagWorldPos = tagWorldPos - currentCamOrientation * tag.Position;
                return _lastTagWorldPos;
            }
        }
        return _lastTagWorldPos;
    }
}
