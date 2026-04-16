// Author: Jackson Russell

using UnityEngine;

/// Artificial potential field collision avoidance (Khatib 1986).
///
/// References:
///   Khatib (1986) - Real-time obstacle avoidance for manipulators and mobile robots.
///   https://doi.org/10.1177/027836498600500106
///
/// Method (direct joint-space, no null-space projection):
///   For each monitored link sphere, query Physics for colliders within the influence
///   radius d0. For each nearby obstacle surface point compute a repulsive world-space
///   force, then map it to a joint velocity correction via the link's translational
///   Jacobian column. The correction is added to q_dot after the RMRC/GD blend:
///
///   q_dot = q_dot_RMRC_GD + q_dot_rep
///
///   Repulsive potential:   U_rep   = (gain/2) * (1/d - 1/d0)^2      for d less than d0
///   Repulsive force:       F_rep   = gain * (1/d - 1/d0) / d^2 * n_hat
///   Link Jacobian (trans): J_j     = z_j cross (p_link - p_j)        for joint j less than or equal to link index
///   Joint correction:      q_dot_rep[j] += dot(J_j, F_rep)

public class SelfCollisionAvoider : MonoBehaviour
{
    [Header("References")]
    public UR3SourceDestinationPublisher publisher;

    [Header("Potential Field Parameters")]
    [Tooltip("Influence radius d0 (metres). Repulsion activates when surface-to-surface distance is less than d0.")]
    public float influenceRadius = 0.08f;

    [Tooltip("Repulsion gain. Larger values produce a stronger push-away response.")]
    public float repulsionGain = 0.8f;

    [Tooltip("Per-joint soft cap on the repulsive velocity correction (rad/s). " +
             "Prevents the correction dominating the task motion. Tune to less than or equal to maxJointVelRad.")]
    public float maxRepulsiveVelRad = 0.25f;

    [Tooltip("Layer mask for obstacle colliders. Include platform, base, camera body, and arm " +
             "link layers for self-collision avoidance. Adjacent links in the chain (parent, " +
             "self, child) are automatically skipped so only non-adjacent link pairs repel.")]
    public LayerMask obstacleLayerMask = Physics.DefaultRaycastLayers;

    [Tooltip("Max colliders tested per link per FixedUpdate. 8 to 16 is sufficient for a sparse scene.")]
    public int maxHitsPerLink = 16;

    [Tooltip("Enable or disable the potential field. Leave false until LinkSpheres are fully configured.")]
    public bool apfEnabled = false;

    // One entry per monitored arm link (shoulder through wrist3 / tool0).
    [System.Serializable]
    public struct LinkSphere
    {
        [Tooltip("The link Transform (drag from Hierarchy).")]
        public Transform link;

        [Tooltip("Offset from the link origin to the sphere centre, in local link space (metres). " +
                 "Shift to the geometric centre of the link segment for best coverage.")]
        public Vector3 localOffset;

        [Tooltip("Sphere radius (metres). Should roughly bound the physical link cross-section.")]
        public float radius;
    }

    [Header("Link Spheres")]
    [Tooltip("One entry per arm joint link, ordered shoulder to wrist3. " +
             "Index i corresponds to joint body i in JointBodies.")]
    public LinkSphere[] linkSpheres = new LinkSphere[6];

    // Pre-allocated buffers - no heap allocation per FixedUpdate.
    private Collider[] _hits;
    private readonly float[] _qdotRep = new float[6];

    void Start()
    {
        _hits = new Collider[maxHitsPerLink];
    }

    /// Computes repulsive joint velocity from the potential field and adds it into qdot[].
    /// Call after the RMRC/GD blend and after LimitVelocities.
    public void AccumulateRepulsiveQdot(float[] qdot)
    {
        if (!apfEnabled || publisher == null || linkSpheres == null) return;

        ArticulationBody[] bodies = publisher.JointBodies;
        if (bodies == null) return;

        for (int j = 0; j < 6; j++) _qdotRep[j] = 0f;

        int numLinks = Mathf.Min(linkSpheres.Length, 6);

        for (int li = 0; li < numLinks; li++)
        {
            ref LinkSphere ls = ref linkSpheres[li];
            if (ls.link == null) continue;

            // World-space centre of the bounding sphere for this link.
            Vector3 sphereCenter = ls.link.TransformPoint(ls.localOffset);

            // Query all obstacle colliders within (influenceRadius + linkRadius) of the sphere.
            int hitCount = Physics.OverlapSphereNonAlloc(
                sphereCenter,
                influenceRadius + ls.radius,
                _hits,
                obstacleLayerMask,
                QueryTriggerInteraction.Ignore);

            for (int h = 0; h < hitCount; h++)
            {
                Collider col = _hits[h];
                if (col == null) continue;

                // Skip this link and its immediate chain neighbours (li-1, li, li+1).
                // Adjacent links are always within influence range by design; repelling them
                // would fight the task motion. Non-adjacent links (e.g. upper_arm vs wrist_3)
                // are allowed through, giving proper self-collision avoidance.
                bool adjacent = false;
                for (int skip = li - 1; skip <= li + 1; skip++)
                {
                    if (skip < 0 || skip >= numLinks) continue;
                    if (linkSpheres[skip].link != null &&
                        col.transform.IsChildOf(linkSpheres[skip].link))
                    { adjacent = true; break; }
                }
                if (adjacent) continue;

                // Closest point on the obstacle surface to the link sphere centre.
                Vector3 closestPt = Physics.ClosestPoint(
                    sphereCenter,
                    col,
                    col.transform.position,
                    col.transform.rotation);

                // Surface-to-surface distance: subtract the link sphere radius.
                Vector3 diff = sphereCenter - closestPt;
                float d = diff.magnitude - ls.radius;

                // Skip if outside influence zone or already overlapping (physics handles hard contact).
                if (d >= influenceRadius || d < 1e-4f) continue;

                float inv_d  = 1f / Mathf.Max(d, 1e-4f);
                float inv_d0 = 1f / influenceRadius;

                // F_rep = gain * (1/d - 1/d0) / d^2 * n_hat
                Vector3 F_rep = repulsionGain * (inv_d - inv_d0) * inv_d * inv_d
                                * diff.normalized;

                // Map F_rep to joint velocities via the translational link Jacobian.
                // Joint j can affect link li only if j <= li (chain kinematics).
                for (int j = 0; j <= li && j < 6; j++)
                {
                    if (bodies[j] == null) continue;

                    // Joint axis in world space - same convention as BuildJacobian in JacobianIKSolver.
                    Vector3 z_j = bodies[j].transform.rotation
                                  * bodies[j].anchorRotation
                                  * Vector3.right;

                    // Translational Jacobian column: z_j cross (p_link - p_joint)
                    Vector3 Jt = Vector3.Cross(z_j, sphereCenter - bodies[j].transform.position);

                    _qdotRep[j] += Vector3.Dot(Jt, F_rep);
                }
            }
        }

        // Add clamped repulsive corrections into the caller's qdot array.
        for (int j = 0; j < 6; j++)
            qdot[j] += Mathf.Clamp(_qdotRep[j], -maxRepulsiveVelRad, maxRepulsiveVelRad);
    }

    // Gizmo: draw all link spheres and their influence shells in the Scene view.
    void OnDrawGizmosSelected()
    {
        if (linkSpheres == null) return;
        foreach (var ls in linkSpheres)
        {
            if (ls.link == null) continue;
            Vector3 c = ls.link.TransformPoint(ls.localOffset);
            Gizmos.color = new Color(0f, 1f, 0.4f, 0.25f);
            Gizmos.DrawSphere(c, ls.radius);
            Gizmos.color = new Color(1f, 0.5f, 0f, 0.15f);
            Gizmos.DrawWireSphere(c, ls.radius + influenceRadius);
        }
    }
}
