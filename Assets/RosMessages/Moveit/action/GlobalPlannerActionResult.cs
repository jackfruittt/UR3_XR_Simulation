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
    public class GlobalPlannerActionResult : ActionResult<GlobalPlannerResult>
    {
        public const string k_RosMessageName = "moveit_msgs/GlobalPlannerActionResult";
        public override string RosMessageName => k_RosMessageName;


        public GlobalPlannerActionResult() : base()
        {
            this.result = new GlobalPlannerResult();
        }

#if !ROS2
        public GlobalPlannerActionResult(HeaderMsg header, GoalStatusMsg status, GlobalPlannerResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public GlobalPlannerActionResult(sbyte status, GlobalPlannerResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static GlobalPlannerActionResult Deserialize(MessageDeserializer deserializer) => new GlobalPlannerActionResult(deserializer);

        GlobalPlannerActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = GlobalPlannerResult.Deserialize(deserializer);
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
