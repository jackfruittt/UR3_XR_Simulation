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
    public class HybridPlannerActionGoal : ActionGoal<HybridPlannerGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/HybridPlannerActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public HybridPlannerActionGoal() : base()
        {
            this.goal = new HybridPlannerGoal();
        }

#if !ROS2
        public HybridPlannerActionGoal(HeaderMsg header, GoalIDMsg goal_id, HybridPlannerGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public HybridPlannerActionGoal(UUIDMsg goal_id, HybridPlannerGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static HybridPlannerActionGoal Deserialize(MessageDeserializer deserializer) => new HybridPlannerActionGoal(deserializer);

        HybridPlannerActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = HybridPlannerGoal.Deserialize(deserializer);
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
