// Author: Jackson Russell

using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;
using System.Collections.Concurrent;
using System.Threading;

/// GPU-accelerated point cloud renderer for ROS depth and colour image streams.
/// Based on the Intel RealSense Unity SDK (RsPointCloudRenderer.cs).
///
/// Design:
///   - Depth arrives as raw 16UC1 bytes -> LoadRawTextureData -> R16 texture (no CPU loop)
///   - Colour arrives as rgb8/bgr8 bytes -> LoadRawTextureData -> RGB24 texture
///   - Compute shader (DepthToPointCloud.compute) converts depth pixels to XYZ on GPU
///   - Vertex shader reads XYZ+colour from StructuredBuffers via SV_VertexID
///   - All GPU work dispatched from LateUpdate (main thread only)
///   - ROS callbacks fire-and-forget: they enqueue/slot bytes and return immediately
///   - Depth and colour textures sized independently to avoid resize loops

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ROSPointCloudRenderer : MonoBehaviour
{
    // Inspector

    [Header("ROS Topics")]
    public string depthTopic               = "camera/camera/depth/image_rect_raw";
    public string colorTopic               = "camera/camera/color/image_raw";
    public string colorCompressedTopic     = "/camera/camera/color/image_raw/compressed";
    public string cameraInfoTopic          = "/camera/camera/color/camera_info";

    [Tooltip("Use compressed JPEG colour stream instead of raw rgb8. At 480x270 raw is ~34 MB/s;\n" +
             "prefer raw (false) for 90 Hz since JPEG decode blocks the main thread each frame.")]
    public bool useCompressedColor = false;

    [Header("Camera Intrinsics (Inspector fallback until camera_info arrives)")]
    public float fx = 385f;
    public float fy = 385f;
    public float cx = 320f;
    public float cy = 240f;

    [Header("Settings")]
    public int   width      = 640;
    public int   height     = 480;
    public float depthScale = 0.001f;   // mm -> m
    public float minDepth   = 0.2f;
    public float maxDepth   = 10f;
    [Tooltip("Convert ROS Y-down to Unity Y-up.")]
    public bool  flipYZ       = true;
    [Tooltip("Flip the row used to READ the depth texture. Enable when depth is stored " +
             "top-to-bottom but colour is bottom-to-top (or vice versa), causing depth " +
             "values to pair with the wrong colour pixels in the point cloud.")]
    public bool  flipDepthTexY = true;

    [Header("Compute Shader")]
    public ComputeShader depthToXYZShader;

    [Header("Rendering")]
    [Tooltip("World-space quad half-size per metre of depth. Matches D455 pixel spacing at 640×480: " +
             "gap ~= z × 0.00234m. Default 0.003 adds ~30% overlap to eliminate visible holes.")]
    public float quadScale = 0.003f;

    // Public API

    public Texture2D  DepthTexture        => depthTexture;
    public Texture2D  ColorTexture        => colorTexture;
    public bool       IntrinsicsFromDevice { get; private set; }
    public string     IntrinsicsSource    => IntrinsicsFromDevice ? "camera_info" : "inspector fallback";
    public int        ColorWidth          => colorTexture != null ? colorTexture.width  : width;
    public int        ColorHeight         => colorTexture != null ? colorTexture.height : height;
    public float      CameraReceiveFPS    { get; private set; }

    // Private: mesh / GPU

    private Mesh           mesh;
    private MeshFilter     meshFilter;
    private MeshRenderer   meshRenderer;
    private Material       _mat;

    private ComputeBuffer  vertexBuffer;
    private ComputeBuffer  colorBuffer;

    private Texture2D      depthTexture;
    private Texture2D      colorTexture;
    private Texture2D      _compressedStagingTex;

    private int  kernel;
    private int  _threadGroupsX, _threadGroupsY;

    // Private: shader IDs

    private int shaderID_DepthTexture;
    private int shaderID_ColorTexture;
    private int shaderID_VertexBuffer;
    private int shaderID_ColorBuffer;
    private int shaderID_flipBGR;
    private int matID_VertexBuffer;
    private int matID_ColorBuffer;
    private int matID_QuadScale;

    // Private: ROS thread -> main thread handoff

    // Depth: bounded queue absorbs burst delivery without blocking ROS thread.
    private ConcurrentQueue<byte[]> _depthQueue = new ConcurrentQueue<byte[]>();
    private const int depthBufferCap = 3;

    // Colour raw: lockless single-slot (only newest frame matters).
    private byte[] _colorSlot;
    private int    _colorSlotVer;
    private int    _colorReadVer = -1;
    private bool   _colorFlipBGR;

    // Colour compressed: same single-slot pattern.
    private byte[] _compressedSlot;
    private int    _compressedSlotVer;
    private int    _compressedReadVer = -1;

    // Pending resize – depth and colour sized independently.
    // Only depth resize rebuilds the mesh/buffers; colour resize only swaps the Texture2D.
    private volatile bool _pendingDepthResize;
    private int  _pendingDepthW, _pendingDepthH;

    private volatile bool _pendingColorResize;
    private int  _pendingColorW, _pendingColorH;
    private int  _colorWidth, _colorHeight;

    // Pending intrinsics from camera_info (set on ROS thread, applied on main thread).
    private volatile bool  _pendingIntrinsics;
    private float _pendingFx, _pendingFy, _pendingCx, _pendingCy;

    // State
    private bool   hasDepth, hasColor;
    private bool   _loggedFirstDepth, _loggedFirstColor;

    // Receive-rate counter - incremented on the ROS thread, read on the main thread.
    // Interlocked.Increment is the only safe cross-thread primitive here; no lock needed.
    // We count depth frames (one per sensor cycle) over a 0.5s window to get true Hz.
    private int    _depthFrameCount;          // accumulates since last window reset
    private float  _fpsWindowStart = -1f;     // Time.realtimeSinceStartup of window start

    // Lifecycle

    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        if (ros == null)
        {
            Debug.LogError("[ROSPointCloudRenderer] No ROS connection.");
            enabled = false;
            return;
        }
        if (depthToXYZShader == null)
        {
            Debug.LogError("[ROSPointCloudRenderer] Compute shader not assigned.");
            enabled = false;
            return;
        }

        InitializeMesh();
        CacheShaderIDs();
        SetStaticShaderParameters();

        // Target 90 fps to match the D455 sensor rate.
        // Must be set before the first frame is rendered; Start() is early enough.
        Application.targetFrameRate = 90;

        ros.Subscribe<ImageMsg>(depthTopic.Trim(), OnDepthImageReceived);
        ros.Subscribe<CameraInfoMsg>(cameraInfoTopic.Trim(), OnCameraInfoReceived);

        if (useCompressedColor)
            ros.Subscribe<CompressedImageMsg>(colorCompressedTopic.Trim(), OnCompressedColorReceived);
        else
            ros.Subscribe<ImageMsg>(colorTopic.Trim(), OnColorImageReceived);

        Debug.Log($"[ROSPointCloudRenderer] Started. depth={depthTopic.Trim()}  " +
                  $"color={(useCompressedColor ? colorCompressedTopic.Trim() : colorTopic.Trim())}  " +
                  $"intrinsics={IntrinsicsSource}");
    }

    void Update()
    {
        // Receive FPS

        float now2 = Time.realtimeSinceStartup;
        if (_fpsWindowStart < 0f) _fpsWindowStart = now2;
        float elapsed = now2 - _fpsWindowStart;
        if (elapsed >= 0.5f)
        {
            int count = Interlocked.Exchange(ref _depthFrameCount, 0);
            float measured = count / elapsed;
            CameraReceiveFPS = CameraReceiveFPS <= 0f ? measured : Mathf.Lerp(CameraReceiveFPS, measured, 0.3f);
            _fpsWindowStart = now2;
        }

        // Apply resolution changes at frame start, before any GPU work this tick.

        if (_pendingDepthResize)
        {
            _pendingDepthResize = false;
            ReinitDepthResolution(_pendingDepthW, _pendingDepthH);
        }

        if (_pendingColorResize)
        {
            _pendingColorResize = false;
            ReinitColorTexture(_pendingColorW, _pendingColorH);
        }

        // Apply intrinsics received from camera_info on the ROS thread.
        if (_pendingIntrinsics)
        {
            _pendingIntrinsics = false;
            fx = _pendingFx; fy = _pendingFy; cx = _pendingCx; cy = _pendingCy;
            depthToXYZShader.SetFloat("invFx",      1f / fx);
            depthToXYZShader.SetFloat("invFy",      1f / fy);
            depthToXYZShader.SetFloat("cx_over_fx", cx / fx);
            depthToXYZShader.SetFloat("cy_over_fy", cy / fy);
            IntrinsicsFromDevice = true;
            Debug.Log($"[ROSPointCloudRenderer] Intrinsics from camera_info: fx={fx:F1} fy={fy:F1} cx={cx:F1} cy={cy:F1}");
        }
    }

    void LateUpdate()
    {
        // Drain entire depth queue, upload only the newest frame.
        // Draining all prevents the queue from backing up when TCP delivers bursts;
        // uploading only the latest keeps the point cloud current (no stale frames).
        byte[] latestDepth = null;
        while (_depthQueue.TryDequeue(out byte[] depthBytes))
            latestDepth = depthBytes;
        if (latestDepth != null)
        {
            // R16 unsigned normalised: LoadRawTextureData accepts raw uint16 bytes directly.
            // Compute shader reads [0,1] and multiplies by 65535 to recover millimetres.
            depthTexture.LoadRawTextureData(latestDepth);
            depthTexture.Apply(false, false);
            hasDepth = true;
        }

        // Upload colour
        if (useCompressedColor)
        {
            int ver = _compressedSlotVer;
            if (ver != _compressedReadVer)
            {
                byte[] data = Interlocked.Exchange(ref _compressedSlot, null);
                _compressedReadVer = ver;
                if (data != null) DecodeCompressed(data);
            }
        }
        else
        {
            int ver = _colorSlotVer;
            if (ver != _colorReadVer)
            {
                byte[] data = Interlocked.Exchange(ref _colorSlot, null);
                _colorReadVer = ver;
                if (data != null)
                {
                    colorTexture.LoadRawTextureData(data);
                    colorTexture.Apply(false, false);
                    depthToXYZShader.SetInt(shaderID_flipBGR, _colorFlipBGR ? 1 : 0);
                    hasColor = true;
                }
            }
        }

        if (hasDepth && hasColor)
            DispatchCompute();
    }

    // Compute dispatch

    void DispatchCompute()
    {
        hasDepth = false;
        hasColor = false;

        depthToXYZShader.Dispatch(kernel, _threadGroupsX, _threadGroupsY, 1);
    }

    // Initialisation

    void InitializeMesh()
    {
        meshFilter   = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        int pointCount = width * height;

        mesh = new Mesh { indexFormat = IndexFormat.UInt32 };
        // MeshTopology.Triangles: each point expands into 4 vertices / 2 triangles.
        int vertCount  = pointCount * 4;
        int indexCount = pointCount * 6;
        int[] indices  = new int[indexCount];
        for (int i = 0; i < pointCount; i++)
        {
            int v   = i * 4;
            int idx = i * 6;
            // Two triangles per quad (CCW winding): TL-BL-TR, BL-BR-TR.
            indices[idx + 0] = v + 0; // TL
            indices[idx + 1] = v + 1; // BL
            indices[idx + 2] = v + 2; // TR
            indices[idx + 3] = v + 1; // BL
            indices[idx + 4] = v + 3; // BR
            indices[idx + 5] = v + 2; // TR
        }
        mesh.vertices  = new Vector3[vertCount];    // sets vertex count; values unused on GPU
        mesh.SetIndices(indices, MeshTopology.Triangles, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 1000f);
        mesh.UploadMeshData(true);                  // free CPU copy - immutable index/vertex data

        meshFilter.mesh = mesh;

        // float4 stride (16 bytes) avoids the Vulkan/SPIR-V std430 alignment issue where
        // StructuredBuffer<float3> generates ArrayStride=16 but a stride=12 ComputeBuffer
        // would shift every element past index 0 by 4 bytes, corrupting far-field depth.
        // The w component carries a validity flag: w=1 valid, w=0 invalid.
        vertexBuffer = new ComputeBuffer(pointCount, sizeof(float) * 4);
        colorBuffer  = new ComputeBuffer(pointCount, sizeof(float) * 4);

        // R16 unsigned normalised: byte[] from ROS is already raw uint16 data.
        // linear=true: depth is NOT a colour texture and must never have sRGB gamma applied.
        // Intel RealSense Unity SDK (RsStreamTextureRenderer.cs) always uses linear=true for
        // non-colour streams (Stream != Color && Stream != Infrared).
        depthTexture = new Texture2D(width, height, TextureFormat.R16, false, true);
        depthTexture.filterMode = FilterMode.Point;

        int cw = _colorWidth  > 0 ? _colorWidth  : width;
        int ch = _colorHeight > 0 ? _colorHeight : height;
        // Colour texture: linear=false so Unity handles sRGB correctly for display.
        colorTexture = new Texture2D(cw, ch, TextureFormat.RGB24, false, false);
        colorTexture.filterMode = FilterMode.Point;

        if (useCompressedColor && _compressedStagingTex == null)
            _compressedStagingTex = new Texture2D(2, 2, TextureFormat.RGBA32, false);

        _mat = meshRenderer.material;
        meshRenderer.shadowCastingMode          = ShadowCastingMode.Off;
        meshRenderer.receiveShadows             = false;
        meshRenderer.reflectionProbeUsage       = ReflectionProbeUsage.Off;
        meshRenderer.lightProbeUsage            = LightProbeUsage.Off;
        meshRenderer.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
    }

    void CacheShaderIDs()
    {
        kernel = depthToXYZShader.FindKernel("DepthToXYZ");

        shaderID_DepthTexture = Shader.PropertyToID("DepthTexture");
        shaderID_ColorTexture = Shader.PropertyToID("ColorTexture");
        shaderID_VertexBuffer = Shader.PropertyToID("VertexBuffer");
        shaderID_ColorBuffer  = Shader.PropertyToID("ColorBuffer");
        shaderID_flipBGR      = Shader.PropertyToID("flipBGR");

        matID_VertexBuffer = Shader.PropertyToID("_VertexBuffer");
        matID_ColorBuffer  = Shader.PropertyToID("_ColorBuffer");
        matID_QuadScale    = Shader.PropertyToID("_QuadScale");
    }

    void SetStaticShaderParameters()
    {
        // Compute shader uses precomputed reciprocals to avoid per-thread division on GPU.
        depthToXYZShader.SetFloat("invFx",      1f / fx);
        depthToXYZShader.SetFloat("invFy",      1f / fy);
        depthToXYZShader.SetFloat("cx_over_fx", cx / fx);
        depthToXYZShader.SetFloat("cy_over_fy", cy / fy);
        depthToXYZShader.SetFloat("depthScale", depthScale);
        depthToXYZShader.SetFloat("minDepth",   minDepth);
        depthToXYZShader.SetFloat("maxDepth",   maxDepth);
        depthToXYZShader.SetInt  ("width",      width);
        depthToXYZShader.SetInt  ("height",     height);
        depthToXYZShader.SetInt  ("flipYZ",        flipYZ        ? 1 : 0);
        depthToXYZShader.SetInt  ("flipDepthTexY", flipDepthTexY ? 1 : 0);

        int cw = colorTexture != null ? colorTexture.width  : width;
        int ch = colorTexture != null ? colorTexture.height : height;
        depthToXYZShader.SetInt  ("colorWidth",   cw);
        depthToXYZShader.SetInt  ("colorHeight",  ch);
        depthToXYZShader.SetFloat("colorScaleX", (float)cw / width);
        depthToXYZShader.SetFloat("colorScaleY", (float)ch / height);

        // Bind buffers and textures once - only their contents change per frame.
        depthToXYZShader.SetBuffer (kernel, shaderID_VertexBuffer, vertexBuffer);
        depthToXYZShader.SetBuffer (kernel, shaderID_ColorBuffer,  colorBuffer);
        depthToXYZShader.SetTexture(kernel, shaderID_DepthTexture, depthTexture);
        depthToXYZShader.SetTexture(kernel, shaderID_ColorTexture, colorTexture);

        _mat.SetBuffer(matID_VertexBuffer, vertexBuffer);
        _mat.SetBuffer(matID_ColorBuffer,  colorBuffer);
        _mat.SetFloat (matID_QuadScale,    quadScale);

        // Cache thread group counts (16x16 = 256 threads per group, fills a GPU wavefront).
        _threadGroupsX = Mathf.CeilToInt(width  / 16f);
        _threadGroupsY = Mathf.CeilToInt(height / 16f);
    }

    // Resize helpers

    /// Rebuilds the full mesh, buffers, and depth texture for a new depth resolution.
    /// Called on the main thread from Update().
    void ReinitDepthResolution(int w, int h)
    {
        vertexBuffer?.Release();
        colorBuffer?.Release();
        if (depthTexture != null) Destroy(depthTexture);
        if (mesh != null)         Destroy(mesh);

        width  = w;
        height = h;

        InitializeMesh();
        CacheShaderIDs();
        SetStaticShaderParameters();

        hasDepth = hasColor = false;
        _loggedFirstDepth = false;
        _depthQueue = new ConcurrentQueue<byte[]>();

        Debug.Log($"[ROSPointCloudRenderer] Depth resolution reinit -> {w}×{h}");
    }

    /// Creates a new colour texture for a different colour resolution.
    /// Does NOT affect the mesh, buffers, or depth pipeline.
    /// Called on the main thread from Update().
    void ReinitColorTexture(int w, int h)
    {
        if (colorTexture != null) Destroy(colorTexture);
        _colorWidth  = w;
        _colorHeight = h;
        colorTexture = new Texture2D(w, h, TextureFormat.RGB24, false, false);
        colorTexture.filterMode = FilterMode.Point;

        depthToXYZShader.SetInt  ("colorWidth",   w);
        depthToXYZShader.SetInt  ("colorHeight",  h);
        depthToXYZShader.SetFloat("colorScaleX", (float)w / width);
        depthToXYZShader.SetFloat("colorScaleY", (float)h / height);
        depthToXYZShader.SetTexture(kernel, shaderID_ColorTexture, colorTexture);

        _colorSlotVer  = 0;
        _colorReadVer  = -1;
        _loggedFirstColor = false;
        hasColor = false;

        Debug.Log($"[ROSPointCloudRenderer] Colour texture reinit -> {w}×{h}");
    }

    // ROS callbacks (run on background threads - NO Unity/GPU API calls here)

    void OnDepthImageReceived(ImageMsg msg)
    {
        if (msg.encoding != "16UC1" && msg.encoding != "mono16")
        {
            Debug.LogError($"[ROSPointCloudRenderer] Unexpected depth encoding: '{msg.encoding}' (expected 16UC1)");
            return;
        }

        if ((int)msg.width != width || (int)msg.height != height)
        {
            // Schedule depth resize on the main thread. Only latch the first mismatch.
            if (!_pendingDepthResize)
            {
                _pendingDepthW = (int)msg.width;
                _pendingDepthH = (int)msg.height;
                _pendingDepthResize = true;
            }
            return;
        }

        // Enqueue raw bytes: main thread uploads without any CPU conversion.
        _depthQueue.Enqueue(msg.data);
        while (_depthQueue.Count > depthBufferCap)
            _depthQueue.TryDequeue(out _);

        // Count every received frame for the receive-rate measurement in Update().
        Interlocked.Increment(ref _depthFrameCount);

        if (!_loggedFirstDepth)
        {
            Debug.Log($"[ROSPointCloudRenderer] First depth frame: {msg.width}×{msg.height} {msg.encoding}");
            _loggedFirstDepth = true;
        }
    }

    void OnColorImageReceived(ImageMsg msg)
    {
        if (msg.encoding != "rgb8" && msg.encoding != "bgr8") return;

        if ((int)msg.width != _colorWidth || (int)msg.height != _colorHeight)
        {
            // Schedule an independent colour-only resize. Does NOT touch depth mesh/buffers.
            if (!_pendingColorResize)
            {
                _pendingColorW = (int)msg.width;
                _pendingColorH = (int)msg.height;
                _pendingColorResize = true;
            }
            // Track new expected size so subsequent frames match.
            _colorWidth  = (int)msg.width;
            _colorHeight = (int)msg.height;
            return;
        }

        _colorFlipBGR = msg.encoding == "bgr8";
        Interlocked.Exchange(ref _colorSlot, msg.data);
        Interlocked.Increment(ref _colorSlotVer);

        if (!_loggedFirstColor)
        {
            Debug.Log($"[ROSPointCloudRenderer] First colour frame: {msg.width}×{msg.height} {msg.encoding}");
            _loggedFirstColor = true;
        }
    }

    void OnCompressedColorReceived(CompressedImageMsg msg)
    {
        if (msg.data == null || msg.data.Length == 0) return;
        Interlocked.Exchange(ref _compressedSlot, msg.data);
        Interlocked.Increment(ref _compressedSlotVer);

        if (!_loggedFirstColor)
        {
            Debug.Log($"[ROSPointCloudRenderer] First compressed colour: fmt={msg.format} bytes={msg.data.Length}");
            _loggedFirstColor = true;
        }
    }

    void OnCameraInfoReceived(CameraInfoMsg msg)
    {
        float nFx = (float)msg.k[0], nFy = (float)msg.k[4];
        float nCx = (float)msg.k[2], nCy = (float)msg.k[5];
        if (Mathf.Approximately(nFx, fx) && Mathf.Approximately(nFy, fy) &&
            Mathf.Approximately(nCx, cx) && Mathf.Approximately(nCy, cy)) return;
        _pendingFx = nFx; _pendingFy = nFy; _pendingCx = nCx; _pendingCy = nCy;
        _pendingIntrinsics = true;
    }

    // Compressed JPEG decode (main thread only)
    void DecodeCompressed(byte[] data)
    {
        try
        {
            _compressedStagingTex.LoadImage(data);

            bool rebuild = _compressedStagingTex.width  != _colorWidth
                        || _compressedStagingTex.height != _colorHeight
                        || colorTexture == null
                        || colorTexture.format != _compressedStagingTex.format;

            if (rebuild)
            {
                _colorWidth  = _compressedStagingTex.width;
                _colorHeight = _compressedStagingTex.height;
                if (colorTexture != null) Destroy(colorTexture);
                colorTexture = new Texture2D(_colorWidth, _colorHeight, _compressedStagingTex.format, false, false);
                colorTexture.filterMode = FilterMode.Point;
                depthToXYZShader.SetInt  ("colorWidth",   _colorWidth);
                depthToXYZShader.SetInt  ("colorHeight",  _colorHeight);
                depthToXYZShader.SetFloat("colorScaleX", (float)_colorWidth  / width);
                depthToXYZShader.SetFloat("colorScaleY", (float)_colorHeight / height);
                depthToXYZShader.SetTexture(kernel, shaderID_ColorTexture, colorTexture);
                Debug.Log($"[ROSPointCloudRenderer] Colour texture rebuilt: {_colorWidth}×{_colorHeight} {_compressedStagingTex.format}");
            }

            Graphics.CopyTexture(_compressedStagingTex, colorTexture);
            depthToXYZShader.SetInt(shaderID_flipBGR, false ? 1 : 0); // JPEG always RGB
            hasColor = true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[ROSPointCloudRenderer] Compressed decode failed: {e.Message}");
        }
    }

    // Cleanup

    void OnDestroy()
    {
        vertexBuffer?.Release();
        colorBuffer?.Release();
        if (depthTexture != null)          Destroy(depthTexture);
        if (colorTexture != null)          Destroy(colorTexture);
        if (_compressedStagingTex != null) Destroy(_compressedStagingTex);
        if (mesh != null)                  Destroy(mesh);
        if (_mat != null)                  Destroy(_mat);
    }
}
