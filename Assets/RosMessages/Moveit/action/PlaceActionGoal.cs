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
    public class PlaceActionGoal : ActionGoal<PlaceGoal>
    {
        public const string k_RosMessageName = "moveit_msgs/PlaceActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public PlaceActionGoal() : base()
        {
            this.goal = new PlaceGoal();
        }

#if !ROS2
        public PlaceActionGoal(HeaderMsg header, GoalIDMsg goal_id, PlaceGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
#else
        public PlaceActionGoal(UUIDMsg goal_id, PlaceGoal goal) : base(goal_id)
        {
            this.goal = goal;
        }
#endif
        public static PlaceActionGoal Deserialize(MessageDeserializer deserializer) => new PlaceActionGoal(deserializer);

        PlaceActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = PlaceGoal.Deserialize(deserializer);
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
