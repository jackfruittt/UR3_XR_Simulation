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
    public class MoveGroupActionResult : ActionResult<MoveGroupResult>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupActionResult";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupActionResult() : base()
        {
            this.result = new MoveGroupResult();
        }

#if !ROS2
        public MoveGroupActionResult(HeaderMsg header, GoalStatusMsg status, MoveGroupResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public MoveGroupActionResult(sbyte status, MoveGroupResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static MoveGroupActionResult Deserialize(MessageDeserializer deserializer) => new MoveGroupActionResult(deserializer);

        MoveGroupActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = MoveGroupResult.Deserialize(deserializer);
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
