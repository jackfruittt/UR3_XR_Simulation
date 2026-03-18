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
    public class MoveGroupSequenceActionGoal : ActionGoal<MoveGroupSequenceGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupSequenceActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupSequenceActionGoal() : base()
        {
            this.goal = new MoveGroupSequenceGoal();
        }

#if !ROS2
        public MoveGroupSequenceActionGoal(HeaderMsg header, GoalIDMsg goal_id, MoveGroupSequenceGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public MoveGroupSequenceActionGoal(UUIDMsg goal_id, MoveGroupSequenceGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static MoveGroupSequenceActionGoal Deserialize(MessageDeserializer deserializer) => new MoveGroupSequenceActionGoal(deserializer);

        MoveGroupSequenceActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = MoveGroupSequenceGoal.Deserialize(deserializer);
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
