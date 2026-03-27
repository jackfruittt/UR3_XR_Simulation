using UnityEngine;
using TMPro;

public class FKDisplay : MonoBehaviour
{
    [Header("References")]
    public RobotFKSolver fkSolver;
    public SimpleJointController jointController;

    [Header("UI Text")]
    public TMP_Text taskSpaceText;
    public TMP_Text jointSpaceText;

    void Update()
    {
        if (fkSolver != null && taskSpaceText != null)
        {
            Vector3 pos = fkSolver.GetEEFPosition();
            Vector3 euler = fkSolver.GetEEFRotation().eulerAngles;

            taskSpaceText.text =
                $"EEF Position\nX: {pos.x:F3}  Y: {pos.y:F3}  Z: {pos.z:F3}\n" +
                $"EEF Rotation\nRx: {euler.x:F1}  Ry: {euler.y:F1}  Rz: {euler.z:F1}";
        }

        if (jointController != null && jointSpaceText != null)
        {
            float[] angles = jointController.GetCurrentJointAngles();
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
