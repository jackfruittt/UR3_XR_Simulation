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
    public class ExecuteTrajectoryActionFeedback : ActionFeedback<ExecuteTrajectoryFeedback>
    {
        public const string k_RosMessageName = "moveit_msgs/ExecuteTrajectoryActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTrajectoryActionFeedback() : base()
        {
            this.feedback = new ExecuteTrajectoryFeedback();
        }

#if !ROS2
        public ExecuteTrajectoryActionFeedback(HeaderMsg header, GoalStatusMsg status, ExecuteTrajectoryFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
#else
        public ExecuteTrajectoryActionFeedback(UUIDMsg goal_id, ExecuteTrajectoryFeedback feedback) : base(goal_id)
        {
            this.feedback = feedback;
        }
#endif
        public static ExecuteTrajectoryActionFeedback Deserialize(MessageDeserializer deserializer) => new ExecuteTrajectoryActionFeedback(deserializer);

        ExecuteTrajectoryActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ExecuteTrajectoryFeedback.Deserialize(deserializer);
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
