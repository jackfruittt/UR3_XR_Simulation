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
    public class PickupActionGoal : ActionGoal<PickupGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/PickupActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public PickupActionGoal() : base()
        {
            this.goal = new PickupGoal();
        }

#if !ROS2
        public PickupActionGoal(HeaderMsg header, GoalIDMsg goal_id, PickupGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public PickupActionGoal(UUIDMsg goal_id, PickupGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static PickupActionGoal Deserialize(MessageDeserializer deserializer) => new PickupActionGoal(deserializer);

        PickupActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = PickupGoal.Deserialize(deserializer);
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
