using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
#if !ROS2
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;
#else
using RosMessageTypes.UniqueIdentifier;
using RosMessageTypes.ActionMsgs;
#endif

namespace RosMessageTypes.Moveit
{
    public class ExecuteTrajectoryActionResult : ActionResult<ExecuteTrajectoryResult>
    {
        public const string k_RosMessageName = "moveit_msgs/ExecuteTrajectoryActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTrajectoryActionResult() : base()
        {
            this.result = new ExecuteTrajectoryResult();
        }

#if !ROS2
        public ExecuteTrajectoryActionResult(HeaderMsg header, GoalStatusMsg status, ExecuteTrajectoryResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public ExecuteTrajectoryActionResult(sbyte status, ExecuteTrajectoryResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static ExecuteTrajectoryActionResult Deserialize(MessageDeserializer deserializer) => new ExecuteTrajectoryActionResult(deserializer);

        ExecuteTrajectoryActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ExecuteTrajectoryResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
#if !ROS2
            serializer.Write(this.header);
            serializer.Write(this.status);
#else
            serializer.Write(this.status);
#endif
            serializer.Write(this.result);
        }


#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
