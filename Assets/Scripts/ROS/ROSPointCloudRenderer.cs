using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

/// GPU-accelerated point cloud renderer for ROS depth and colour image streams.
///  Author: Jackson Russell
/// Implementation based on the Intel RealSense Unity SDK approach (RsPointCloudRenderer.cs):
/// - Utilises MeshTopology.Points for efficient point rendering
/// - GPU compute shader performs depth pixel to XYZ coordinate conversion
/// - Reference: librealsense/wrappers/unity/Assets/RealSenseSDK2.0/Scripts/RsPointCloudRenderer.cs
/// 
/// Key advantages compared to subscribing to PointCloud2 topics:
/// - Subscribes to depth and colour IMAGE topics (significantly reduced data: approximately 1.5MB)
/// - GPU compute shader performs depth to XYZ conversion utilising Intel's optimised SDK
/// - Eliminates CPU deserialisation bottleneck inherent in PointCloud2 messages
/// - Performance: 30+ FPS compared to 0-10 FPS when using PointCloud2 topics

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ROSPointCloudRenderer : MonoBehaviour
{
    [Header("ROS Topics")]
    [Tooltip("Depth image topic (raw 16-bit data). Default: /camera/camera/aligned_depth_to_color/image_raw")]
    public string depthTopic = "/camera/camera/aligned_depth_to_color/image_raw";

    [Tooltip("Colour image topic (RGB format). Default: /camera/camera/color/image_raw")]
    public string colorTopic = "/camera/camera/color/image_raw";

    [Header("Camera Intrinsics - RealSense D455 Depth Camera")]
    [Tooltip("Focal length X component in pixels. RealSense D455 at 640x480: approximately 385")]
    public float fx = 385.0f;

    [Tooltip("Focal length Y component in pixels. RealSense D455 at 640x480: approximately 385")]
    public float fy = 385.0f;

    [Tooltip("Principal point X coordinate (typically width/2). RealSense D455: approximately 320")]
    public float cx = 320.0f;

    [Tooltip("Principal point Y coordinate (typically height/2). RealSense D455: approximately 240")]
    public float cy = 240.0f;

    [Header("Settings")]
    [Tooltip("Image width in pixels. RealSense D455 default: 640")]
    public int width = 640;

    [Tooltip("Image height in pixels. RealSense D455 default: 480")]
    public int height = 480;

    [Tooltip("Depth scale factor: 0.001 for millimetre to metre conversion")]
    public float depthScale = 0.001f;

    [Tooltip("Minimum acceptable depth threshold in metres (filters noise)")]
    public float minDepth = 0.1f;

    [Tooltip("Maximum acceptable depth threshold in metres")]
    public float maxDepth = 10.0f;

    [Tooltip("Convert ROS coordinate system (Y-down) to Unity coordinate system (Y-up)")]
    public bool flipYZ = true;

    [Header("Compute Shader")]
    [Tooltip("Reference to the compute shader asset: DepthToPointCloud.compute")]
    public ComputeShader depthToXYZShader;

    [Header("Rendering")]
    [Tooltip("Point size for rendering (configurable in shader or material)")]
    public float pointSize = 0.005f;

    [Header("Debug")]
    public bool showDebugInfo = true;

    // ROS TCP connection instance
    private ROSConnection ros;

    // GPU texture resources for depth and colour data
    private Texture2D depthTexture;
    private Texture2D colorTexture;

    // Compute shader GPU buffers
    private ComputeBuffer vertexBuffer;
    private ComputeBuffer colorBuffer;

    // Point cloud mesh components
    private Mesh mesh;
    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    // Synchronisation and performance tracking
    private bool hasDepth = false;
    private bool hasColor = false;
    private int frameCount = 0;
    private float lastUpdateTime = 0f;

    // pre-allocated CPU buffers (eliminates per-frame heap allocation) ---
    private float[] depthData;
    private byte[] rgbData;

    // cached kernel index and shader property IDs (avoids per-frame string lookup) ---
    private int kernel;
    private int shaderID_DepthTexture;
    private int shaderID_ColorTexture;
    private int shaderID_VertexBuffer;
    private int shaderID_ColorBuffer;
    private int shaderID_flipBGR;
    private int matID_VertexBuffer;
    private int matID_ColorBuffer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        if (ros == null)
        {
            Debug.LogError("[ROSPointCloudRenderer] Failed to establish ROS connection.");
            enabled = false;
            return;
        }

        if (depthToXYZShader == null)
        {
            Debug.LogError("[ROSPointCloudRenderer] Compute shader not assigned. Please assign DepthToPointCloud.compute in the Inspector.");
            enabled = false;
            return;
        }

        InitializeMesh();
        CacheShaderIDs();
        SetStaticShaderParameters();

        string depthTopicTrimmed = depthTopic.Trim();
        string colorTopicTrimmed = colorTopic.Trim();

        ros.Subscribe<ImageMsg>(depthTopicTrimmed, OnDepthImageReceived);
        ros.Subscribe<ImageMsg>(colorTopicTrimmed, OnColorImageReceived);

        Debug.Log($"[ROSPointCloudRenderer] Subscribed to depth: {depthTopicTrimmed}");
        Debug.Log($"[ROSPointCloudRenderer] Subscribed to color: {colorTopicTrimmed}");
        Debug.Log($"[ROSPointCloudRenderer] Camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}");
    }

    /// Initialises mesh with point topology utilising the Intel RsPointCloudRenderer pattern.
    /// Reference implementation: RsPointCloudRenderer.cs ResetMesh() method.
    /// Also pre-allocates CPU-side buffers to eliminate per-frame heap allocation.
    void InitializeMesh()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        int pointCount = width * height;

        mesh = new Mesh
        {
            indexFormat = IndexFormat.UInt32
        };
        mesh.MarkDynamic();

        Vector3[] vertices = new Vector3[pointCount];

        int[] indices = new int[pointCount];
        for (int i = 0; i < pointCount; i++)
            indices[i] = i;

        Vector2[] uvs = new Vector2[pointCount];
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;
                uvs[idx] = new Vector2((float)x / width, (float)y / height);
            }
        }

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 20f);

        meshFilter.mesh = mesh;

        vertexBuffer = new ComputeBuffer(pointCount, sizeof(float) * 3);
        colorBuffer  = new ComputeBuffer(pointCount, sizeof(float) * 4);

        depthTexture = new Texture2D(width, height, TextureFormat.RFloat, false);
        depthTexture.filterMode = FilterMode.Point;

        colorTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        colorTexture.filterMode = FilterMode.Point;

        // pre-allocate reusable CPU buffers
        depthData = new float[pointCount];
        rgbData   = new byte[pointCount * 3];

        Debug.Log($"[ROSPointCloudRenderer] Mesh initialised: {pointCount} points ({width}x{height})");
    }

    /// Caches all shader property and kernel IDs once at startup.
    /// Avoids repeated string hashing inside the per-frame hot path.
    void CacheShaderIDs()
    {
        kernel = depthToXYZShader.FindKernel("DepthToXYZ");

        // Compute shader inputs
        shaderID_DepthTexture = Shader.PropertyToID("DepthTexture");
        shaderID_ColorTexture = Shader.PropertyToID("ColorTexture");
        shaderID_VertexBuffer = Shader.PropertyToID("VertexBuffer");
        shaderID_ColorBuffer  = Shader.PropertyToID("ColorBuffer");
        shaderID_flipBGR      = Shader.PropertyToID("flipBGR");

        // Material buffer bindings
        matID_VertexBuffer = Shader.PropertyToID("_VertexBuffer");
        matID_ColorBuffer  = Shader.PropertyToID("_ColorBuffer");
    }

    /// Uploads shader parameters that never change after startup (intrinsics, scale, dimensions).
    void SetStaticShaderParameters()
    {
        depthToXYZShader.SetFloat("fx",         fx);
        depthToXYZShader.SetFloat("fy",         fy);
        depthToXYZShader.SetFloat("cx",         cx);
        depthToXYZShader.SetFloat("cy",         cy);
        depthToXYZShader.SetFloat("depthScale", depthScale);
        depthToXYZShader.SetFloat("minDepth",   minDepth);
        depthToXYZShader.SetFloat("maxDepth",   maxDepth);
        depthToXYZShader.SetInt("width",        width);
        depthToXYZShader.SetInt("height",       height);
        depthToXYZShader.SetBool("flipYZ",      flipYZ);

        // Bind output buffers once — these never change between frames
        depthToXYZShader.SetBuffer(kernel, shaderID_VertexBuffer, vertexBuffer);
        depthToXYZShader.SetBuffer(kernel, shaderID_ColorBuffer,  colorBuffer);
    }

    void OnDepthImageReceived(ImageMsg msg)
    {
        if (msg.encoding != "16UC1" && msg.encoding != "mono16")
        {
            Debug.LogError($"[ROSPointCloudRenderer] Depth callback received incorrect encoding: {msg.encoding} (expected 16UC1 or mono16).");
            return;
        }

        if (msg.width != width || msg.height != height)
        {
            Debug.LogWarning($"[ROSPointCloudRenderer] Depth dimension mismatch: expected {width}x{height}, received {msg.width}x{msg.height}");
            return;
        }

        int pixelCount = width * height;

        // Extract 16-bit depth values into pre-allocated float array (no allocation here)
        for (int i = 0; i < pixelCount; i++)
        {
            ushort depthMM = (ushort)(msg.data[i * 2] | (msg.data[i * 2 + 1] << 8));
            depthData[i] = depthMM;
        }

        depthTexture.SetPixelData(depthData, 0);
        depthTexture.Apply(false, false); // Suppress mipmap generation

        hasDepth = true;

        if (hasColor)
            UpdatePointCloud();
    }

    void OnColorImageReceived(ImageMsg msg)
    {
        if (msg.encoding != "rgb8" && msg.encoding != "bgr8")
        {
            Debug.LogError($"[ROSPointCloudRenderer] Colour callback received incorrect encoding: {msg.encoding} (expected rgb8 or bgr8).");
            return;
        }

        if (msg.width != width || msg.height != height)
        {
            Debug.LogWarning($"[ROSPointCloudRenderer] Colour dimension mismatch: expected {width}x{height}, received {msg.width}x{msg.height}");
            return;
        }

        // Load raw bytes directly regardless of channel order; the compute shader swizzles if needed.
        // eliminates the per-frame CPU byte-swap loop entirely.
        colorTexture.LoadRawTextureData(msg.data);
        colorTexture.Apply(false, false); // Suppress mipmap generation

        // Inform compute shader whether to swizzle BGR->RGB (GPU swizzle is effectively free)
        depthToXYZShader.SetBool(shaderID_flipBGR, msg.encoding == "bgr8");

        hasColor = true;
    }

    /// Executes compute shader to perform depth-to-XYZ conversion.
    /// Buffers are passed directly to the material, no GPU readback occurs.
    void UpdatePointCloud()
    {
        if (!hasDepth || !hasColor)
            return;

        hasDepth = false;
        hasColor = false;

        // Bind per-frame input textures (these change every frame)
        depthToXYZShader.SetTexture(kernel, shaderID_DepthTexture, depthTexture);
        depthToXYZShader.SetTexture(kernel, shaderID_ColorTexture, colorTexture);

        // Dispatch compute shader — output written directly to GPU buffers
        int threadGroupsX = Mathf.CeilToInt(width  / 8.0f);
        int threadGroupsY = Mathf.CeilToInt(height / 8.0f);
        depthToXYZShader.Dispatch(kernel, threadGroupsX, threadGroupsY, 1);

        // Vertex shader reads positions and colours from these buffers via SV_VertexID.
        meshRenderer.material.SetBuffer(matID_VertexBuffer, vertexBuffer);
        meshRenderer.material.SetBuffer(matID_ColorBuffer,  colorBuffer);

        // Performance diagnostics (FPS only - valid point count removed as it required CPU readback)
        frameCount++;
        if (showDebugInfo && Time.time - lastUpdateTime > 2.0f)
        {
            float fps = frameCount / (Time.time - lastUpdateTime);
            Debug.Log($"[ROSPointCloudRenderer] Performance: {fps:F1} FPS");
            frameCount = 0;
            lastUpdateTime = Time.time;
        }
    }

    void OnDestroy()
    {
        vertexBuffer?.Release();
        colorBuffer?.Release();

        if (depthTexture != null) Destroy(depthTexture);
        if (colorTexture != null) Destroy(colorTexture);
        if (mesh != null)         Destroy(mesh);
    }
}
