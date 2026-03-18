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
    public class PickupActionFeedback : ActionFeedback<PickupFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/PickupActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public PickupActionFeedback() : base()
        {
            this.feedback = new PickupFeedback();
        }

#if !ROS2
        public PickupActionFeedback(HeaderMsg header, GoalStatusMsg status, PickupFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public PickupActionFeedback(UUIDMsg goal_id, PickupFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static PickupActionFeedback Deserialize(MessageDeserializer deserializer) => new PickupActionFeedback(deserializer);

        PickupActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = PickupFeedback.Deserialize(deserializer);
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
