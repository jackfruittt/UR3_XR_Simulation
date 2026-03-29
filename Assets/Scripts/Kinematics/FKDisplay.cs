using UnityEngine;
using TMPro;

// Reads FK state from RobotFKSolver and actual joint angles from the publisher,
// then pushes them to two TMP text fields every frame for live debugging.
public class FKDisplay : MonoBehaviour
{
    [Header("References")]
    public RobotFKSolver fkSolver;
    public UR3SourceDestinationPublisher publisher;

    [Header("UI Text")]
    public TMP_Text taskSpaceText;
    public TMP_Text jointSpaceText;

    void Update()
    {
        if (fkSolver != null && taskSpaceText != null)
        {
            Vector3 pos   = fkSolver.GetEEFPosition();
            Vector3 euler = fkSolver.GetEEFRotation().eulerAngles;

            taskSpaceText.text =
                $"EEF Position\nX: {pos.x:F3}  Y: {pos.y:F3}  Z: {pos.z:F3}\n" +
                $"EEF Rotation\nRx: {euler.x:F1}  Ry: {euler.y:F1}  Rz: {euler.z:F1}";
        }

        if (publisher != null && jointSpaceText != null)
        {
            // Use actual physics angles (jointPosition[0]) rather than drive targets,
            // which can lag behind the true robot state.
            float[] angles = publisher.GetActualJointAngles();
            if (angles != null && angles.Length == 6)
            {
                jointSpaceText.text =
                    $"Joint Angles (deg)\n" +
                    $"J1: {angles[0]:F2}  J2: {angles[1]:F2}  J3: {angles[2]:F2}\n" +
                    $"J4: {angles[3]:F2}  J5: {angles[4]:F2}  J6: {angles[5]:F2}";
            }
        }
    }
}
