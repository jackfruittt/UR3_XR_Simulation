using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class GlobalPlannerAction : Action<GlobalPlannerActionGoal, GlobalPlannerActionResult, GlobalPlannerActionFeedback, GlobalPlannerGoal, GlobalPlannerResult, GlobalPlannerFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/GlobalPlannerAction";
        public override string RosMessageName => k_RosMessageName;


        public GlobalPlannerAction() : base()
        {
            this.action_goal = new GlobalPlannerActionGoal();
            this.action_result = new GlobalPlannerActionResult();
            this.action_feedback = new GlobalPlannerActionFeedback();
        }

        public static GlobalPlannerAction Deserialize(MessageDeserializer deserializer) => new GlobalPlannerAction(deserializer);

        GlobalPlannerAction(MessageDeserializer deserializer)
        {
            this.action_goal = GlobalPlannerActionGoal.Deserialize(deserializer);
            this.action_result = GlobalPlannerActionResult.Deserialize(deserializer);
            this.action_feedback = GlobalPlannerActionFeedback.Deserialize(deserializer);
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
