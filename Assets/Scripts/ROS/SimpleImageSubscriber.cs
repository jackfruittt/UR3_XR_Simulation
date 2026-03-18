using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Simple ROS image subscriber - supports both raw and compressed images
/// </summary>
public class SimpleImageSubscriber : MonoBehaviour
{
    [Header("ROS Topic")]
    public string imageTopic = "/camera/camera/color/image_raw";
    public bool useCompressed = false; // Set true to use /compressed variant (JPEG/PNG)
    public bool useDepthCamera = false; // Use depth camera topic
    public bool useAlignedDepth = false; // Use aligned depth (depth aligned to color camera)
    
    [Header("Display")]
    public Renderer targetRenderer;
    public bool flipVertically = true; // Flip raw images (compressed usually don't need it)
    
    [Header("Depth Visualization")]
    public float maxDepthMM = 5000f; // Max depth in millimeters to visualize
    
    [Header("Performance")]
    public bool showFPS = true;
    
    private ROSConnection ros;
    private Texture2D imageTexture;
    private Material displayMaterial;
    
    private int frameCount = 0;
    private float lastFPSTime = 0f;
    private float currentFPS = 0f;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        Debug.Log($"[SimpleImageSubscriber] Mode flags - useAlignedDepth: {useAlignedDepth}, useDepthCamera: {useDepthCamera}, useCompressed: {useCompressed}");
        
        // Switch topic based on mode
        string topic;
        if (useAlignedDepth)
        {
            topic = "/camera/camera/aligned_depth_to_color/image_raw";
            Debug.Log("[SimpleImageSubscriber] Using ALIGNED DEPTH mode");
        }
        else if (useDepthCamera)
        {
            topic = "/camera/camera/depth/image_rect_raw";
            Debug.Log("[SimpleImageSubscriber] Using DEPTH mode");
        }
        else
        {
            topic = imageTopic;
            Debug.Log("[SimpleImageSubscriber] Using COLOR mode");
        }
        
        // Depth cameras always use raw (16-bit doesn't compress well)
        bool useCompression = (useDepthCamera || useAlignedDepth) ? false : useCompressed;
        
        if (useCompression)
        {
            // Automatically append /compressed if not already there
            string compressedTopic = topic.EndsWith("/compressed") ? topic : topic + "/compressed";
            ros.Subscribe<CompressedImageMsg>(compressedTopic, UpdateCompressedImage);
            Debug.Log($"[SimpleImageSubscriber] ✓ Subscribed to COMPRESSED topic: {compressedTopic}");
        }
        else
        {
            ros.Subscribe<ImageMsg>(topic, UpdateRawImage);
            Debug.Log($"[SimpleImageSubscriber] ✓ Subscribed to RAW topic: {topic}");
        }
        
        if (targetRenderer != null)
        {
            displayMaterial = targetRenderer.material;
            Debug.Log($"[SimpleImageSubscriber] Target renderer assigned: {targetRenderer.name}");
        }
        else
        {
            Debug.LogError("[SimpleImageSubscriber] No target renderer assigned!");
        }
    }
    
    void UpdateCompressedImage(CompressedImageMsg msg)
    {
        Debug.Log($"[SimpleImageSubscriber] Received compressed image: format={msg.format}, data size: {msg.data.Length}");
        
        // Create texture if needed
        if (imageTexture == null)
        {
            imageTexture = new Texture2D(2, 2);
        }
        
        // Unity's LoadImage automatically handles JPEG/PNG decompression
        if (imageTexture.LoadImage(msg.data))
        {
            // Compressed images from ROS usually don't need flipping
            // (Unity's LoadImage handles orientation correctly)
            
            if (displayMaterial != null)
            {
                displayMaterial.mainTexture = imageTexture;
            }
            
            UpdateFPS();
        }
        else
        {
            Debug.LogError($"[SimpleImageSubscriber] Failed to decompress image. Format: {msg.format}");
        }
    }
    
    void UpdateRawImage(ImageMsg msg)
    {
        int width = (int)msg.width;
        int height = (int)msg.height;
        
        Debug.Log($"[SimpleImageSubscriber] Received image: {width}x{height}, encoding: {msg.encoding}, data size: {msg.data.Length}");
        
        // Create texture if needed
        if (imageTexture == null || imageTexture.width != width || imageTexture.height != height)
        {
            // Determine format based on encoding
            TextureFormat format = TextureFormat.RGB24;
            if (msg.encoding == "16UC1" || msg.encoding == "mono16")
            {
                format = TextureFormat.RGBA32;
            }
            
            imageTexture = new Texture2D(width, height, format, false);
            Debug.Log($"[SimpleImageSubscriber] Created texture: {width}x{height}, encoding: {msg.encoding}");
        }
        
        // Handle 16-bit depth (16UC1)
        if (msg.encoding == "16UC1" || msg.encoding == "mono16")
        {
            Debug.Log($"[SimpleImageSubscriber] Processing DEPTH image");
            ProcessDepthImage(msg.data, width, height);
        }
        // Handle RGB/BGR color
        else if (msg.encoding == "rgb8" || msg.encoding == "bgr8")
        {
            Debug.Log($"[SimpleImageSubscriber] Processing COLOR image (BGR={msg.encoding == "bgr8"})");
            ProcessColorImage(msg.data, width, height, msg.encoding == "bgr8");
        }
        else
        {
            Debug.LogWarning($"[SimpleImageSubscriber] Unsupported encoding: {msg.encoding}");
            return;
        }
        
        if (displayMaterial != null)
        {
            displayMaterial.mainTexture = imageTexture;
        }
        
        imageTexture.Apply(false);
        UpdateFPS();
    }
    
    void ProcessDepthImage(byte[] data, int width, int height)
    {
        Color32[] pixels = new Color32[width * height];
        
        for (int i = 0; i < pixels.Length; i++)
        {
            int idx = i * 2;
            ushort depthMM = (ushort)(data[idx] | (data[idx + 1] << 8));
            
            // Convert depth to grayscale (0 = black/close, 255 = white/far)
            byte gray = (byte)Mathf.Clamp((depthMM / maxDepthMM) * 255, 0, 255);
            pixels[i] = new Color32(gray, gray, gray, 255);
        }
        
        if (flipVertically)
        {
            FlipVertical(pixels, width, height);
        }
        
        imageTexture.SetPixels32(pixels);
    }
    
    void ProcessColorImage(byte[] data, int width, int height, bool isBGR)
    {
        // Convert BGR to RGB if needed
        if (isBGR)
        {
            for (int i = 0; i < data.Length; i += 3)
            {
                byte temp = data[i];
                data[i] = data[i + 2];
                data[i + 2] = temp;
            }
        }
        
        // Flip vertically if needed
        if (flipVertically)
        {
            FlipVerticalRaw(data, width, height, 3);
        }
        
        imageTexture.LoadRawTextureData(data);
    }
    
    void FlipVertical(Color32[] pixels, int width, int height)
    {
        for (int y = 0; y < height / 2; y++)
        {
            int topRow = y * width;
            int bottomRow = (height - 1 - y) * width;
            
            for (int x = 0; x < width; x++)
            {
                Color32 temp = pixels[topRow + x];
                pixels[topRow + x] = pixels[bottomRow + x];
                pixels[bottomRow + x] = temp;
            }
        }
    }
    
    void FlipVerticalRaw(byte[] data, int width, int height, int bytesPerPixel)
    {
        int rowSize = width * bytesPerPixel;
        byte[] tempRow = new byte[rowSize];
        
        for (int y = 0; y < height / 2; y++)
        {
            int topRow = y * rowSize;
            int bottomRow = (height - 1 - y) * rowSize;
            
            // Swap rows
            System.Array.Copy(data, topRow, tempRow, 0, rowSize);
            System.Array.Copy(data, bottomRow, data, topRow, rowSize);
            System.Array.Copy(tempRow, 0, data, bottomRow, rowSize);
        }
    }
    
    void FlipTextureVertically()
    {
        // For textures that are already loaded (compressed images)
        Color32[] pixels = imageTexture.GetPixels32();
        int width = imageTexture.width;
        int height = imageTexture.height;
        
        for (int y = 0; y < height / 2; y++)
        {
            int topRow = y * width;
            int bottomRow = (height - 1 - y) * width;
            
            for (int x = 0; x < width; x++)
            {
                Color32 temp = pixels[topRow + x];
                pixels[topRow + x] = pixels[bottomRow + x];
                pixels[bottomRow + x] = temp;
            }
        }
        
        imageTexture.SetPixels32(pixels);
        imageTexture.Apply(false);
    }
    
    void UpdateFPS()
    {
        frameCount++;
        float currentTime = Time.time;
        if (currentTime - lastFPSTime >= 1f)
        {
            currentFPS = frameCount / (currentTime - lastFPSTime);
            frameCount = 0;
            lastFPSTime = currentTime;
        }
    }
    
    void OnGUI()
    {
        if (showFPS && imageTexture != null)
        {
            string mode;
            if (useAlignedDepth)
                mode = "Aligned Depth";
            else if (useDepthCamera)
                mode = "Depth";
            else
                mode = useCompressed ? "Color (Compressed)" : "Color (Raw)";
            
            GUI.Label(new Rect(10, 10, 450, 30), 
                $"{mode} | FPS: {currentFPS:F1} | Size: {imageTexture.width}x{imageTexture.height}");
        }
    }
    
    public Texture2D GetTexture()
    {
        return imageTexture;
    }
}
