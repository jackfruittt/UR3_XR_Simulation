using System;
using UnityEngine;
using UnityEngine.VFX;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class RealSenseToVFX : MonoBehaviour
{


    // CONFIG
    [Header("ROS Settings")]
    [Tooltip("The ROS topic to subscribe to for the RealSense camera's depth image: /camera/camera/depth/colour/points")]
    [SerializeField] string pclTopic = "/camera/camera/depth/color/points";

    [Header("VFX Output - Assign these to your VFX Graph")]
    [Tooltip("GPU texture encoding 3D positions (RGB = XYZ, A = valid flag)")]
    public RenderTexture positionMap;
    
    [Tooltip("GPU texture encoding RGB colors for each point")]
    public RenderTexture colorMap;
    
    [Header("Settings")]
    [Tooltip("Width of point cloud (must match ROS publisher). D455 default: 848 or 640")]
    [SerializeField] int width = 640;
    
    [Tooltip("Height of point cloud (must match ROS publisher). D455 default: 480")]
    [SerializeField] int height = 480;
    
    [Tooltip("Scale factor for positions (adjust if points too close/far). Unity units per meter.")]
    [SerializeField] float positionScale = 1.0f;
    
    [Tooltip("Enable ROS to Unity coordinate conversion (usually keep enabled)")]
    [SerializeField] bool flipYZ = true;

    // CPU textures for data manipulation before GPU upload
    private Texture2D posTexture;
    private Texture2D colorTexture;
    
    // CPU arrays holding processed point data
    private Color[] positionData;  // RGBA = (X, Y, Z, valid)
    private Color[] colorData; 
    // Assuming you have the ROS message fields for x, y, z offsets
    int xOffset, yOffset, zOffset;

    // Example of converting ROS coordinates to Unity coordinates   

    void Start()
    {
        // Position map needs float for accurate 3D Pos
        positionMap = new RenderTexture(width, height, 0, 
        RenderTextureFormat.ARGBFloat);
        positionMap.enableRandomWrite = true;

        // Compute Shader Access
        positionMap.filterMode = FilterMode.Point; // No interpolation for point clouds
        positionMap.Create();

        // colour map can use bytes (0-255)
        colorMap = new RenderTexture(width, height, 0,
        RenderTextureFormat.ARGB32);
        colorMap.enableRandomWrite = true;
        colorMap.filterMode = FilterMode.Point;
        colorMap.Create();

        // Create CPU Textures for Rendering
        posTexture = new Texture2D(width, height, 
        TextureFormat.RGBAFloat, false);
        colorTexture = new Texture2D(width, height, 
        TextureFormat.RGBA32, false);

        // Pre-allocate CPU arrays for point data
        positionData = new Color[width * height];
        colorData = new Color[width * height];

        // Subscribe to ROS topic
        ROSConnection.instance.Subscribe<PointCloud2Msg>(
            pclTopic, // Topic subscrbed to
            ReceivePointCloud // Callback
        );

        UnityEngine.Debug.Log($"[RealSenseToVFX] Subscribed to {pclTopic} via ROS-TCP");
        UnityEngine.Debug.Log($"[RealSenseToVFX] Expecting point cloud size: {width}x{height}");

        // Auto-assign textures to VFX Graph component
        VisualEffect vfx = GetComponent<VisualEffect>();
        if (vfx != null)
        {
            vfx.SetTexture("PositionMap", positionMap);
            vfx.SetTexture("ColorMap", colorMap);
            UnityEngine.Debug.Log("[RealSenseToVFX] Textures automatically assigned to VFX Graph");
        }
        else
        {
            UnityEngine.Debug.LogWarning("[RealSenseToVFX] No VisualEffect component found on this GameObject. Add one to see the point cloud!");
        }
    }

    // Cleanup when game object gets destroyed
    void OnDestroy()
    {
        // Release GPU render textures 
        positionMap?.Release();
        colorMap?.Release();

        // Destroy CPU textures
        if (posTexture != null) Destroy(posTexture);
        if (colorTexture != null) Destroy(colorTexture);

        UnityEngine.Debug.Log("[RealSenseToVFX] Cleaned up textures and render textures");
    }

    // ROS Message Handling 
    void ReceivePointCloud(PointCloud2Msg msg)
    {
        if(msg.width != width || msg.height != height)
        {
            UnityEngine.Debug.LogError($"[RealSenseToVFX] Received point cloud with unexpected dimensions: {msg.width}x{msg.height}. Expected: {width}x{height}");
            return;
        }   

        // Process the PCL Data
        ProcessPointCloudData(msg);

        // Upload to GPU Textures
        UpdateTextures();
    }

    // PCL Processing
    void ProcessPointCloudData(PointCloud2Msg msg)
    {
        int pointStep = (int)msg.point_step;
        int rowStep = (int)msg.row_step;
        
        // Find field offsets
        int xOffset = -1, yOffset = -1, zOffset = -1, rgbOffset = -1;
        foreach (var field in msg.fields)
        {
            switch (field.name)
            {
                case "x": xOffset = (int)field.offset; break;
                case "y": yOffset = (int)field.offset; break;
                case "z": zOffset = (int)field.offset; break;
                case "rgb":
                case "rgba": rgbOffset = (int)field.offset; break;
            }
        }
        
        bool hasColour = rgbOffset >= 0;
        byte[] data = msg.data;  // Cache reference for performance
        
        for (int y = 0; y < height; y++)
        {
            int rowOffset = y * rowStep;  // Precompute row offset
            
            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;
                int offset = rowOffset + x * pointStep;
                
                if (offset + pointStep > data.Length)
                {
                    positionData[idx] = Color.clear;
                    colorData[idx] = Color.clear;
                    continue;
                }
                
                // Extract XYZ (already optimal)
                float px = BitConverter.ToSingle(data, offset + xOffset);
                float py = BitConverter.ToSingle(data, offset + yOffset);
                float pz = BitConverter.ToSingle(data, offset + zOffset);
                
                // Validate data
                if (float.IsNaN(px) || float.IsInfinity(px))
                {
                    positionData[idx] = Color.clear;
                    colorData[idx] = Color.clear;
                    continue;
                }
                
                // Convert coordinates from ROS to Unity
                UnityEngine.Vector3 rosPos = new UnityEngine.Vector3(px, py, pz);
                UnityEngine.Vector3 unityPos = rosPos.To<FLU>().toUnity;
                
                // Store position
                positionData[idx] = new Color(
                    unityPos.x * positionScale,
                    unityPos.y * positionScale,
                    unityPos.z * positionScale,
                    1.0f
                );
                
                // OPTIMISED: Extract RGB in one read - 
                if (hasColour)
                {
                    uint rgb = BitConverter.ToUInt32(data, offset + rgbOffset);
                    
                    // Assuming BGR order (common in RealSense)
                    colorData[idx] = new Color32(
                        (byte)(rgb >> 16),  // R
                        (byte)(rgb >> 8),   // G
                        (byte)(rgb),        // B
                        255
                    );
                }
                else
                {
                    colorData[idx] = Color.white;
                }
            }
        }
    }

    // GPU UPLOAD
    void UpdateTextures()
    {
        // Update CPU Textures with processed data
        posTexture.SetPixels(positionData);
        posTexture.Apply();

        colorTexture.SetPixels(colorData);
        colorTexture.Apply();

        // Copy CPU textures to GPU render textures
        Graphics.Blit(posTexture, positionMap);
        Graphics.Blit(colorTexture, colorMap);
    }
}

