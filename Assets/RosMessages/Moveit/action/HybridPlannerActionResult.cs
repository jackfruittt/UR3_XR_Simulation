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
    public class HybridPlannerActionResult : ActionResult<HybridPlannerResult>
    {
        public const string k_RosMessageName = "moveit_msgs/HybridPlannerActionResult";
        public override string RosMessageName => k_RosMessageName;


        public HybridPlannerActionResult() : base()
        {
            this.result = new HybridPlannerResult();
        }

#if !ROS2
        public HybridPlannerActionResult(HeaderMsg header, GoalStatusMsg status, HybridPlannerResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public HybridPlannerActionResult(sbyte status, HybridPlannerResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static HybridPlannerActionResult Deserialize(MessageDeserializer deserializer) => new HybridPlannerActionResult(deserializer);

        HybridPlannerActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = HybridPlannerResult.Deserialize(deserializer);
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
