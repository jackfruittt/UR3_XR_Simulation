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
    public class GlobalPlannerActionFeedback : ActionFeedback<GlobalPlannerFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/GlobalPlannerActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public GlobalPlannerActionFeedback() : base()
        {
            this.feedback = new GlobalPlannerFeedback();
        }

#if !ROS2
        public GlobalPlannerActionFeedback(HeaderMsg header, GoalStatusMsg status, GlobalPlannerFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public GlobalPlannerActionFeedback(UUIDMsg goal_id, GlobalPlannerFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static GlobalPlannerActionFeedback Deserialize(MessageDeserializer deserializer) => new GlobalPlannerActionFeedback(deserializer);

        GlobalPlannerActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = GlobalPlannerFeedback.Deserialize(deserializer);
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
