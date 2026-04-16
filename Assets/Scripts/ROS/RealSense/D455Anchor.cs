// Author: Jackson Russell
//
// D455Anchor - standalone root GameObject that acts as the world-space pivot for
// the live D455 point cloud. No connection to the robot arm or the player rig.
//
// Geometry:
//   ROSPointCloudRenderer (child, local offset 0, 0, 0.065) outputs vertices in
//   D455 camera-local space. The vertex shader computes:
//     clip = P * V * M * camLocalPoint
//   where V = inv(cameraWorldMatrix), M = this anchor's local-to-world.
using UnityEngine;

[DefaultExecutionOrder(100)]   // run after IMUSubscriber (order 0 default)
public class D455Anchor : MonoBehaviour
{
    [Tooltip("The IMUSubscriber whose CameraWorldRotation drives this anchor's rotation.")]
    public IMUSubscriber imuSubscriber;

    void LateUpdate()
    {
        if (imuSubscriber == null || !imuSubscriber.PoseValid) return;
        // Rotation only — anchor position stays at its scene-placed world origin.
        // The stitcher handles world position independently via CameraWorldPosition.
        transform.rotation = imuSubscriber.CameraWorldRotation;
    }
}
