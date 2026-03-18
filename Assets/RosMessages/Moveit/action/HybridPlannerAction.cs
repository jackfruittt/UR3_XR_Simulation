using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class HybridPlannerAction : Action<HybridPlannerActionGoal, HybridPlannerActionResult, HybridPlannerActionFeedback, HybridPlannerGoal, HybridPlannerResult, HybridPlannerFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/HybridPlannerAction";
        public override string RosMessageName => k_RosMessageName;


        public HybridPlannerAction() : base()
        {
            this.action_goal = new HybridPlannerActionGoal();
            this.action_result = new HybridPlannerActionResult();
            this.action_feedback = new HybridPlannerActionFeedback();
        }

        public static HybridPlannerAction Deserialize(MessageDeserializer deserializer) => new HybridPlannerAction(deserializer);

        HybridPlannerAction(MessageDeserializer deserializer)
        {
            this.action_goal = HybridPlannerActionGoal.Deserialize(deserializer);
            this.action_result = HybridPlannerActionResult.Deserialize(deserializer);
            this.action_feedback = HybridPlannerActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
#if !ROS2
#else
#endif
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
