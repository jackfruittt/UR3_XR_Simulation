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
    public class MoveGroupActionGoal : ActionGoal<MoveGroupGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/MoveGroupActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public MoveGroupActionGoal() : base()
        {
            this.goal = new MoveGroupGoal();
        }

#if !ROS2
        public MoveGroupActionGoal(HeaderMsg header, GoalIDMsg goal_id, MoveGroupGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public MoveGroupActionGoal(UUIDMsg goal_id, MoveGroupGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static MoveGroupActionGoal Deserialize(MessageDeserializer deserializer) => new MoveGroupActionGoal(deserializer);

        MoveGroupActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = MoveGroupGoal.Deserialize(deserializer);
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
