// Author: Jackson Russell

using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using UnityEngine;

public class UR3SourceDestinationPublisher : MonoBehaviour
{
    ROSConnection _ros;
    public string rosServiceName = "ur3_moveit";
    public string jointCommandTopic = "/unity_joint_commands";

	private static readonly string[] LinkNames =
	{
	    "base_link/base_link_inertia/shoulder_link",
	    "base_link/base_link_inertia/shoulder_link/upper_arm_link",
	    "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link",
	    "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link",
	    "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link",
	    "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link"
	};

    private ArticulationBody[] jointArticulationBodies;

    // Exposes the ArticulationBody array for the IK solver to read joint axes
    public ArticulationBody[] JointBodies => jointArticulationBodies;

    public GameObject target;
    public GameObject targetPlacement;
    public GameObject ur3;
    
    [Header("Control Mode")]
    public bool manualControlMode = true; // Set true to use sliders without ROS interference

    // Default ArticulationBody stiffness causes PD overshoot; these give a critically-damped-ish response.
    [Header("Drive Tuning")]
    public float driveStiffness  = 400f;
    public float driveDamping    = 80f;
    public float driveForceLimit = 300f;

    // Elbow-up ready pose; wrist_2=-90 avoids wrist singularity, elbow=90 avoids elbow-straight singularity.
    private static readonly float[] HomePosition = { 0f, -90f, 90f, -90f, -90f, 0f }; // In degrees

void Start()
{
    _ros = ROSConnection.GetOrCreateInstance();
    _ros.Subscribe<JointStateMsg>("/joint_states", UpdateRobotJoints);
    _ros.RegisterPublisher<JointStateMsg>(jointCommandTopic);
    
    // Initialize the joint articulation bodies array
    jointArticulationBodies = new ArticulationBody[LinkNames.Length];
    
        for (int i = 0; i < LinkNames.Length; i++)
    {
        Transform linkTransform = ur3.transform.Find(LinkNames[i]);
        if (linkTransform != null)
        {
            jointArticulationBodies[i] = linkTransform.GetComponent<ArticulationBody>();
            if (jointArticulationBodies[i] == null)
                Debug.LogError($"ArticulationBody not found on: {LinkNames[i]}");
        }
        else
        {
            Debug.LogError($"Transform not found: {LinkNames[i]}");
        }
    }
    
    // Tune drives before moving to home so the first commanded step is already damped.
    TuneAllDrives();

    // Set robot to home position
    StartCoroutine(SetHomePositionAfterInit());
}

System.Collections.IEnumerator SetHomePositionAfterInit()
{
    yield return null;
    for (int i = 0; i < HomePosition.Length && i < jointArticulationBodies.Length; i++)
        SetJointAngleLocally(i, HomePosition[i]);
}

void TuneAllDrives()
{
    if (jointArticulationBodies == null) return;
    foreach (var body in jointArticulationBodies)
    {
        if (body == null) continue;
        var drive         = body.xDrive;
        drive.stiffness   = driveStiffness;
        drive.damping     = driveDamping;
        drive.forceLimit  = driveForceLimit;
        body.xDrive       = drive;
    }
}

void FindArticulationBodies(Transform t, string path) { }

    void UpdateRobotJoints(JointStateMsg msg)
{
    // Ignore ROS updates when in manual control mode
    if (manualControlMode)
    {
        return;
    }
    
    if (jointArticulationBodies == null) return;

    string[] urJointNames = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    for (var i = 0; i < urJointNames.Length; i++)
    {
        if (jointArticulationBodies[i] == null)
        {
            Debug.LogError($"jointArticulationBodies[{i}] is null!");
            continue;
        }
        for (var j = 0; j < msg.name.Length; j++)
        {
            if (msg.name[j] == urJointNames[i])
            {
                var drive = jointArticulationBodies[i].xDrive;
                drive.target = Mathf.Rad2Deg * (float)msg.position[j];
                jointArticulationBodies[i].xDrive = drive;
                break;
            }
        }
    }
}

    public void Publish()
    {
        Debug.Log("Publish called - messages not yet generated.");
    }
    
    public void PublishJointCommand(float[] jointAngles)
    {
        if (jointAngles == null || jointAngles.Length != 6)
        {
            Debug.LogError("Invalid joint angles array. Expected 6 values.");
            return;
        }
        
        JointStateMsg msg = new JointStateMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                }
            },
            name = new string[]
            {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            },
            position = new double[6],
            velocity = new double[6],
            effort = new double[6]
        };
        
        // Convert degrees to radians for ROS2
        for (int i = 0; i < 6; i++)
        {
            msg.position[i] = jointAngles[i] * Mathf.Deg2Rad;
        }
        
        _ros.Publish(jointCommandTopic, msg);
    }
    
    /// Returns commanded drive targets (degrees).
    /// NOTE: lags physics position - prefer GetActualJointAngles() for IK feedback.
    public float[] GetCurrentJointAngles()
    {
        if (jointArticulationBodies == null)
            return null;
            
        float[] angles = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (jointArticulationBodies[i] != null)
                angles[i] = jointArticulationBodies[i].xDrive.target;
        }
        return angles;
    }

    /// Returns true physics joint positions (degrees) from ArticulationBody.jointPosition[0].
    /// Use for IK feedback - avoids command/actual mismatch.
    public float[] GetActualJointAngles()
    {
        if (jointArticulationBodies == null)
            return null;

        float[] angles = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (jointArticulationBodies[i] != null)
                // jointPosition is in radians for a 1-DOF revolute body
                angles[i] = jointArticulationBodies[i].jointPosition[0] * Mathf.Rad2Deg;
        }
        return angles;
    }

    /// Zero-allocation variant: fills <paramref name="buffer"/> in-place (must be length 6).
    /// Returns true on success, false if bodies are not yet initialised.
    /// Prefer this over GetActualJointAngles() in hot paths (e.g. FixedUpdate) to avoid
    /// heap allocations every frame.
    public bool GetActualJointAnglesInto(float[] buffer)
    {
        if (jointArticulationBodies == null || buffer == null || buffer.Length < 6)
            return false;

        for (int i = 0; i < 6; i++)
        {
            buffer[i] = jointArticulationBodies[i] != null
                ? jointArticulationBodies[i].jointPosition[0] * Mathf.Rad2Deg
                : 0f;
        }
        return true;
    }
    
    // Get the home position values
    public float[] GetHomePosition()
    {
        return (float[])HomePosition.Clone();
    }
    
    // Move robot to home position
    public void MoveToHomePosition()
    {
        for (int i = 0; i < HomePosition.Length && i < jointArticulationBodies.Length; i++)
            SetJointAngleLocally(i, HomePosition[i]);
    }
    
    // Direct local control of individual joints (works without ROS)
    public void SetJointAngleLocally(int jointIndex, float angleDegrees)
    {
        if (jointArticulationBodies == null || jointIndex < 0 || jointIndex >= jointArticulationBodies.Length)
            return;

        if (jointArticulationBodies[jointIndex] != null)
        {
            var drive = jointArticulationBodies[jointIndex].xDrive;
            drive.target = angleDegrees;
            jointArticulationBodies[jointIndex].xDrive = drive;
        }
    }
}
