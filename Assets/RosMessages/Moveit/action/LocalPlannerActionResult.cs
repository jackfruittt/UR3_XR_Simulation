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
    public class LocalPlannerActionResult : ActionResult<LocalPlannerResult>
    {
        public const string k_RosMessageName = "moveit_msgs/LocalPlannerActionResult";
        public override string RosMessageName => k_RosMessageName;


        public LocalPlannerActionResult() : base()
        {
            this.result = new LocalPlannerResult();
        }

#if !ROS2
        public LocalPlannerActionResult(HeaderMsg header, GoalStatusMsg status, LocalPlannerResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public LocalPlannerActionResult(sbyte status, LocalPlannerResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static LocalPlannerActionResult Deserialize(MessageDeserializer deserializer) => new LocalPlannerActionResult(deserializer);

        LocalPlannerActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = LocalPlannerResult.Deserialize(deserializer);
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
