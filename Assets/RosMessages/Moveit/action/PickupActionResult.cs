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
    public class PickupActionResult : ActionResult<PickupResult>
    {
        public const string k_RosMessageName = "moveit_msgs/PickupActionResult";
        public override string RosMessageName => k_RosMessageName;


        public PickupActionResult() : base()
        {
            this.result = new PickupResult();
        }

#if !ROS2
        public PickupActionResult(HeaderMsg header, GoalStatusMsg status, PickupResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public PickupActionResult(sbyte status, PickupResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static PickupActionResult Deserialize(MessageDeserializer deserializer) => new PickupActionResult(deserializer);

        PickupActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = PickupResult.Deserialize(deserializer);
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
