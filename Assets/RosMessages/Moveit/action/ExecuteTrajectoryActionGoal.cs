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
    public class ExecuteTrajectoryActionGoal : ActionGoal<ExecuteTrajectoryGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/ExecuteTrajectoryActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ExecuteTrajectoryActionGoal() : base()
        {
            this.goal = new ExecuteTrajectoryGoal();
        }

#if !ROS2
        public ExecuteTrajectoryActionGoal(HeaderMsg header, GoalIDMsg goal_id, ExecuteTrajectoryGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public ExecuteTrajectoryActionGoal(UUIDMsg goal_id, ExecuteTrajectoryGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static ExecuteTrajectoryActionGoal Deserialize(MessageDeserializer deserializer) => new ExecuteTrajectoryActionGoal(deserializer);

        ExecuteTrajectoryActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ExecuteTrajectoryGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
#if !ROS2
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
#else
            serializer.Write(this.goal_id);
#endif
            serializer.Write(this.goal);
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
