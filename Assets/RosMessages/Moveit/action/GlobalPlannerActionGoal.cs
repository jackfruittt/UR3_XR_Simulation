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
    public class GlobalPlannerActionGoal : ActionGoal<GlobalPlannerGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/GlobalPlannerActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public GlobalPlannerActionGoal() : base()
        {
            this.goal = new GlobalPlannerGoal();
        }

#if !ROS2
        public GlobalPlannerActionGoal(HeaderMsg header, GoalIDMsg goal_id, GlobalPlannerGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public GlobalPlannerActionGoal(UUIDMsg goal_id, GlobalPlannerGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static GlobalPlannerActionGoal Deserialize(MessageDeserializer deserializer) => new GlobalPlannerActionGoal(deserializer);

        GlobalPlannerActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = GlobalPlannerGoal.Deserialize(deserializer);
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
