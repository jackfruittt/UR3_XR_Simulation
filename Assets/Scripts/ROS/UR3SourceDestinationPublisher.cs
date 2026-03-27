using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using UnityEngine;

public class UR3SourceDestinationPublisher : MonoBehaviour
{
    ROSConnection ros;
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

    public GameObject target;
    public GameObject targetPlacement;
    public GameObject ur3;
    
    [Header("Control Mode")]
    public bool manualControlMode = true; // Set true to use sliders without ROS interference
    
    // UR3 Home position (safe starting position)
    private static readonly float[] HomePosition = { 0f, -90f, 0f, -90f, 0f, -90f }; // In degrees

void Start()
{
    ros = ROSConnection.GetOrCreateInstance();
    ros.Subscribe<JointStateMsg>("/joint_states", UpdateRobotJoints);
    ros.RegisterPublisher<JointStateMsg>(jointCommandTopic);
    
    // Initialize the joint articulation bodies array
    jointArticulationBodies = new ArticulationBody[LinkNames.Length];
    
    for (int i = 0; i < LinkNames.Length; i++)
    {
        Transform linkTransform = ur3.transform.Find(LinkNames[i]);
        if (linkTransform != null)
        {
            jointArticulationBodies[i] = linkTransform.GetComponent<ArticulationBody>();
            if (jointArticulationBodies[i] != null)
            {
                Debug.Log($"Found ArticulationBody {i}: {LinkNames[i]}");
            }
            else
            {
                Debug.LogError($"ArticulationBody not found on: {LinkNames[i]}");
            }
        }
        else
        {
            Debug.LogError($"Transform not found: {LinkNames[i]}");
        }
    }
    
    // Print all ArticulationBodies for verification
    FindArticulationBodies(ur3.transform, ur3.name);
    
    // Set robot to home position
    StartCoroutine(SetHomePositionAfterInit());
}

System.Collections.IEnumerator SetHomePositionAfterInit()
{
    // Wait one frame for physics to initialize
    yield return null;
    
    Debug.Log("Setting robot to home position...");
    for (int i = 0; i < HomePosition.Length && i < jointArticulationBodies.Length; i++)
    {
        if (jointArticulationBodies[i] != null)
        {
            SetJointAngleLocally(i, HomePosition[i]);
        }
    }
    Debug.Log("Robot set to home position");
}
void FindArticulationBodies(Transform t, string path)
{
    if (t.GetComponent<ArticulationBody>() != null)
        Debug.Log("AB found: " + path);
    foreach (Transform child in t)
        FindArticulationBodies(child, path + "/" + child.name);
}

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
        
        ros.Publish(jointCommandTopic, msg);
        Debug.Log($"Published joint command: [{string.Join(", ", jointAngles)}] degrees");
    }
    
    public float[] GetCurrentJointAngles()
    {
        if (jointArticulationBodies == null)
            return null;
            
        float[] angles = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (jointArticulationBodies[i] != null)
            {
                angles[i] = jointArticulationBodies[i].xDrive.target;
            }
        }
        return angles;
    }
    
    // Get the home position values
    public float[] GetHomePosition()
    {
        return (float[])HomePosition.Clone();
    }
    
    // Move robot to home position
    public void MoveToHomePosition()
    {
        Debug.Log("Moving to home position...");
        for (int i = 0; i < HomePosition.Length && i < jointArticulationBodies.Length; i++)
        {
            if (jointArticulationBodies[i] != null)
            {
                SetJointAngleLocally(i, HomePosition[i]);
            }
        }
    }
    
    // Direct local control of individual joints (works without ROS)
    public void SetJointAngleLocally(int jointIndex, float angleDegrees)
    {
        Debug.Log($"SetJointAngleLocally({jointIndex}, {angleDegrees})");
        if (jointArticulationBodies == null || jointIndex < 0 || jointIndex >= jointArticulationBodies.Length)
        {
            Debug.LogError($"Invalid joint index: {jointIndex}");
            return;
        }
        
        if (jointArticulationBodies[jointIndex] != null)
        {
            var drive = jointArticulationBodies[jointIndex].xDrive;
            drive.target = angleDegrees;
            jointArticulationBodies[jointIndex].xDrive = drive;
            Debug.Log($"Set joint {jointIndex} target to {angleDegrees} degrees");
        }
        else
        {
            Debug.LogError($"ArticulationBody at index {jointIndex} is null!");
        }
    }
}
