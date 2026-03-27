using UnityEngine;
using System.Collections.Generic;

/// Manages a pool of GameObjects to visualise detected AprilTags in 3D.
/// One child GO per tag ID - moved/activated each frame, hidden when not detected.
public class TagDrawer : System.IDisposable
{
    // Parent transform to keep hierarchy tidy
    Transform _root;

    // Map from tag ID to its visualisation GameObject
    Dictionary<int, GameObject> _tagObjects = new Dictionary<int, GameObject>();

    Material _material;

    // TODO: replace the primitive cube with a proper quad + axis indicator prefab
    public TagDrawer(Material material)
    {
        _root = new GameObject("AprilTagOverlays").transform;
        _material = material;
    }

    public void Dispose()
    {
        Object.Destroy(_root.gameObject);
    }

    /// Update or create the overlay for a single tag.
    /// position and rotation are in world space (already transformed out of camera space by detection.cs).
    public void Draw(int id, Vector3 position, Quaternion rotation, float tagSize)
    {
        if(!_tagObjects.ContainsKey(id))
        {
            CreateTagObject(id);
        }
        GameObject tagObject = _tagObjects[id];
        tagObject.transform.position = position;
        tagObject.transform.rotation = rotation;
        tagObject.transform.localScale = Vector3.one * tagSize;
        tagObject.SetActive(true);
    }

    /// Call once per frame after Draw() calls to hide tags that were not detected this frame.
    /// Pass the set of IDs that were detected so everything else gets hidden.
    public void HideUndetected(IEnumerable<int> detectedIds)
    {
        HashSet<int> detectedSet = new HashSet<int>(detectedIds);
        foreach (var pair in _tagObjects)
        {
            if (!detectedSet.Contains(pair.Key))
                pair.Value.SetActive(false);
        }
    }

    /// Create a simple visual representation for a new tag ID.
    GameObject CreateTagObject(int id)
    {
        GameObject tagObject = GameObject.CreatePrimitive(PrimitiveType.Quad);
        tagObject.name = "Tag_" + id;
        tagObject.GetComponent<MeshRenderer>().material = _material;
        tagObject.transform.SetParent(_root, false);

        // Quad primitives include a MeshCollider by default - remove it,
        // Don't need physics on a visualisation overlay
        Object.Destroy(tagObject.GetComponent<MeshCollider>());

        _tagObjects[id] = tagObject;
        return tagObject;
    }
}
