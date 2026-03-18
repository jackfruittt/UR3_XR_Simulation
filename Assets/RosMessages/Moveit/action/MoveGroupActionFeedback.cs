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
    public class MoveGroupActionFeedback : ActionFeedback<MoveGroupFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupActionFeedback() : base()
        {
            this.feedback = new MoveGroupFeedback();
        }

#if !ROS2
        public MoveGroupActionFeedback(HeaderMsg header, GoalStatusMsg status, MoveGroupFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public MoveGroupActionFeedback(UUIDMsg goal_id, MoveGroupFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static MoveGroupActionFeedback Deserialize(MessageDeserializer deserializer) => new MoveGroupActionFeedback(deserializer);

        MoveGroupActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = MoveGroupFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
#if !ROS2
            serializer.Write(this.header);
            serializer.Write(this.status);
#else
            serializer.Write(this.goal_id);
#endif
            serializer.Write(this.feedback);
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
