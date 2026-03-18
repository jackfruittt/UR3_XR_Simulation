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
    public class MoveGroupSequenceActionResult : ActionResult<MoveGroupSequenceResult>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupSequenceActionResult";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupSequenceActionResult() : base()
        {
            this.result = new MoveGroupSequenceResult();
        }

#if !ROS2
        public MoveGroupSequenceActionResult(HeaderMsg header, GoalStatusMsg status, MoveGroupSequenceResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public MoveGroupSequenceActionResult(sbyte status, MoveGroupSequenceResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static MoveGroupSequenceActionResult Deserialize(MessageDeserializer deserializer) => new MoveGroupSequenceActionResult(deserializer);

        MoveGroupSequenceActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = MoveGroupSequenceResult.Deserialize(deserializer);
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
