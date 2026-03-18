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
    public class PlaceActionFeedback : ActionFeedback<PlaceFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/PlaceActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public PlaceActionFeedback() : base()
        {
            this.feedback = new PlaceFeedback();
        }

#if !ROS2
        public PlaceActionFeedback(HeaderMsg header, GoalStatusMsg status, PlaceFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public PlaceActionFeedback(UUIDMsg goal_id, PlaceFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static PlaceActionFeedback Deserialize(MessageDeserializer deserializer) => new PlaceActionFeedback(deserializer);

        PlaceActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = PlaceFeedback.Deserialize(deserializer);
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
