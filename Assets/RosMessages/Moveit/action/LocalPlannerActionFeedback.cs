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
    public class LocalPlannerActionFeedback : ActionFeedback<LocalPlannerFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/LocalPlannerActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public LocalPlannerActionFeedback() : base()
        {
            this.feedback = new LocalPlannerFeedback();
        }

#if !ROS2
        public LocalPlannerActionFeedback(HeaderMsg header, GoalStatusMsg status, LocalPlannerFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public LocalPlannerActionFeedback(UUIDMsg goal_id, LocalPlannerFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static LocalPlannerActionFeedback Deserialize(MessageDeserializer deserializer) => new LocalPlannerActionFeedback(deserializer);

        LocalPlannerActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = LocalPlannerFeedback.Deserialize(deserializer);
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
