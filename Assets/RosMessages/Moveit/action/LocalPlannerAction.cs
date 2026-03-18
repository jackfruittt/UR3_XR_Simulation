using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Moveit
{
    public class LocalPlannerAction : Action<LocalPlannerActionGoal, LocalPlannerActionResult, LocalPlannerActionFeedback, LocalPlannerGoal, LocalPlannerResult, LocalPlannerFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/LocalPlannerAction";
        public override string RosMessageName => k_RosMessageName;


        public LocalPlannerAction() : base()
        {
            this.action_goal = new LocalPlannerActionGoal();
            this.action_result = new LocalPlannerActionResult();
            this.action_feedback = new LocalPlannerActionFeedback();
        }

        public static LocalPlannerAction Deserialize(MessageDeserializer deserializer) => new LocalPlannerAction(deserializer);

        LocalPlannerAction(MessageDeserializer deserializer)
        {
            this.action_goal = LocalPlannerActionGoal.Deserialize(deserializer);
            this.action_result = LocalPlannerActionResult.Deserialize(deserializer);
            this.action_feedback = LocalPlannerActionFeedback.Deserialize(deserializer);
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
