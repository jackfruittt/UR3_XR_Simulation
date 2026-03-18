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
    public class LocalPlannerActionGoal : ActionGoal<LocalPlannerGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/LocalPlannerActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public LocalPlannerActionGoal() : base()
        {
            this.goal = new LocalPlannerGoal();
        }

#if !ROS2
        public LocalPlannerActionGoal(HeaderMsg header, GoalIDMsg goal_id, LocalPlannerGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public LocalPlannerActionGoal(UUIDMsg goal_id, LocalPlannerGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static LocalPlannerActionGoal Deserialize(MessageDeserializer deserializer) => new LocalPlannerActionGoal(deserializer);

        LocalPlannerActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = LocalPlannerGoal.Deserialize(deserializer);
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
