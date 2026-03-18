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
    public class PlaceActionResult : ActionResult<PlaceResult>
    {
        public const string k_RosMessageName = "moveit_msgs/PlaceActionResult";
        public override string RosMessageName => k_RosMessageName;


        public PlaceActionResult() : base()
        {
            this.result = new PlaceResult();
        }

#if !ROS2
        public PlaceActionResult(HeaderMsg header, GoalStatusMsg status, PlaceResult result) : base(header, status)
        {
            this.result = result;
        }
#else
        public PlaceActionResult(sbyte status, PlaceResult result) : base(status)
        {
            this.result = result;
        }
#endif
        public static PlaceActionResult Deserialize(MessageDeserializer deserializer) => new PlaceActionResult(deserializer);

        PlaceActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = PlaceResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
#if !ROS2
            serializer.Write(this.header);
            serializer.Write(this.status);
#else
            serializer.Write(this.status);
#endif
            serializer.Write(this.result);
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
