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
    public class HybridPlannerActionFeedback : ActionFeedback<HybridPlannerFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/HybridPlannerActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public HybridPlannerActionFeedback() : base()
        {
            this.feedback = new HybridPlannerFeedback();
        }

#if !ROS2
        public HybridPlannerActionFeedback(HeaderMsg header, GoalStatusMsg status, HybridPlannerFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public HybridPlannerActionFeedback(UUIDMsg goal_id, HybridPlannerFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static HybridPlannerActionFeedback Deserialize(MessageDeserializer deserializer) => new HybridPlannerActionFeedback(deserializer);

        HybridPlannerActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = HybridPlannerFeedback.Deserialize(deserializer);
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
