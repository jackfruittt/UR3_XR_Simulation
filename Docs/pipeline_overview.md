# RealSense → Unity Streaming Pipeline
### 90 Hz Depth + Colour + IMU

---

## 1. System Overview

```mermaid
flowchart LR
    subgraph D455["Intel D455 (USB 3.x)"]
        DS["Depth\n640×480 @ 90 Hz\n16UC1 Z16"]
        CS["Colour\n480×270 @ 90 Hz\nrgb8"]
        GY["Gyro\n~500 Hz"]
        AC["Accel\n~250 Hz"]
    end

    subgraph ROS2["ROS 2 Humble"]
        RL["realsense2_camera\nalign_depth.enable = true"]
        EP["ros_tcp_endpoint\nport 10000"]
    end

    subgraph TCP["TCP Bridge"]
        direction TB
        TH["ReaderThread\nasync Task\n10 ms poll"]
        IQ["ConcurrentQueue\ncap = 16 msgs\ndrop oldest on overflow"]
    end

    subgraph Unity["Unity (90 fps target)"]
        direction TB
        UPD["Update()\nmain thread\nresize / intrinsics"]
        LU["LateUpdate()\nmain thread\ntexture upload + dispatch"]
        GPU["GPU\nDepthToPointCloud.compute\n16×16 thread groups"]
        VP["Vertex / Geometry shader\npoint → screen quad"]
        DISP["Display"]
    end

    D455 --> RL --> EP
    EP -->|TCP| TH --> IQ
    IQ -->|"dequeue all,\nupload latest"| LU
    LU --> GPU --> VP --> DISP
    UPD -. "resize / intrinsics\napplied before LateUpdate" .-> LU
```

---

## 2. SimpleImageSubscriber Pipeline

Used for the calibration HUD colour overlay and AprilTag detection.

```mermaid
sequenceDiagram
    participant ROS as ROS 2 Topic<br/>/color/image_raw
    participant BG  as ROS Background Thread
    participant MQ  as ROSConnection<br/>ConcurrentQueue (cap 16)
    participant UPD as Unity Update()<br/>main thread
    participant TEX as Texture2D<br/>RGB24 480×270
    participant MAT as Material<br/>mainTexture

    loop 90 Hz
        ROS  ->> BG  : ImageMsg bytes arrive via TCP
        BG   ->> MQ  : Enqueue raw bytes<br/>drop oldest if > 16
        MQ   ->> UPD : TryDequeue (called each frame)
        UPD  ->> TEX : LoadRawTextureData()<br/>no CPU pixel loop
        UPD  ->> TEX : Apply(false, false)
        UPD  ->> MAT : mainTexture = tex
    end
```

### Performance decisions

| Decision | Reason |
|---|---|
| `raw rgb8` not JPEG | `LoadImage()` (JPEG decode) blocks main thread per frame; at 90 Hz that is ~11 ms decode budget consumed before any rendering |
| No per-frame `Debug.Log` | String allocation + console write: ~270 allocs/sec at 90 Hz; triggers GC pressure and frame stalls |
| `LoadRawTextureData()` | Byte-copy into GPU memory; zero pixel-loop CPU cost compared to manual `Color32[]` iteration |
| `Apply(false, false)` | Skips mipmap generation and does not upload to GPU twice |

---

## 3. ROSPointCloudRenderer Pipeline

Full GPU point cloud: depth back-projected to XYZ on the compute shader, coloured from the aligned colour stream.

```mermaid
flowchart TB
    subgraph ROSThread["ROS Background Thread (async)"]
        DR["OnDepthImageReceived()\n16UC1 bytes"]
        CR["OnColorImageReceived()\nrgb8 bytes"]
        CI["OnCameraInfoReceived()\nfx fy cx cy"]
    end

    subgraph Queues["Lock-free Handoff"]
        DQ["ConcurrentQueue&lt;byte[]&gt;\ndepthBufferCap = 3\n(absorbs TCP burst)"]
        CS["Interlocked slot\nbyte[] _colorSlot\n(only newest frame kept)"]
        PI["volatile bool\n_pendingIntrinsics"]
    end

    subgraph MainThread["Unity Main Thread"]
        direction TB
        UPD["Update()\n• apply resize\n• apply intrinsics from _pendingIntrinsics\n• measure CameraReceiveFPS (0.5 s window)"]
        LU["LateUpdate()\n① drain entire depth queue\n   upload LATEST frame only\n② read _colorSlot (if version changed)\n③ DispatchCompute()"]
    end

    subgraph GPU["GPU"]
        direction LR
        DT["DepthTexture\nR16 640×480"]
        CT["ColorTexture\nRGB24 480×270"]
        CS2["DepthToXYZ.compute\n16×16 thread groups\n= 40×30 dispatches\n1 200 GPU threads\nper depth frame"]
        VB["VertexBuffer\nfloat4 × 307 200\n(XYZ + validity)"]
        CB["ColorBuffer\nfloat4 × 307 200"]
        VS["Vertex shader\nSV_VertexID → position"]
        GS["Geometry shader\npoint → 2-tri quad\nquadScale per metre"]
        FB["Framebuffer"]
    end

    DR --> DQ
    CR --> CS
    CI --> PI
    DQ --> LU
    CS --> LU
    PI --> UPD
    UPD -.->|"intrinsics written\nbefore LateUpdate"| LU
    LU -->|LoadRawTextureData| DT
    LU -->|LoadRawTextureData| CT
    LU --> CS2
    DT --> CS2
    CT --> CS2
    CS2 --> VB & CB
    VB & CB --> VS --> GS --> FB
```

### Thread model detail

```mermaid
flowchart LR
    subgraph bg["Background (TCP reader, ~continuous)"]
        T1["ReadMessageContents()\nasync await"]
        T2["Enqueue to ConcurrentQueue\nor Interlocked.Exchange slot"]
    end

    subgraph main["Main Thread (per frame ~11 ms at 90 Hz)"]
        M1["Update() ~0.1 ms\nresize / intrinsics only"]
        M2["LateUpdate() ~1–2 ms\ndrain queue\nLoadRawTextureData\nDispatch"]
        M3["GPU ~3–5 ms\ncompute + vertex + geometry"]
    end

    bg -->|"zero-copy byte[]\nno lock"| main
    M1 --> M2 --> M3
```

### Performance decisions

| Decision | Reason |
|---|---|
| `Application.targetFrameRate = 90` | Without this Unity targets 30 fps in builds; VSync in the editor also caps at monitor refresh |
| Depth: `ConcurrentQueue` + drain all | TCP can deliver frames in bursts; draining all and uploading only the latest ensures the point cloud is never stale and the queue cannot grow unboundedly |
| Colour: `Interlocked` single slot | Only the newest colour frame is needed; a slot exchange is O(1) and lock-free, cheaper than a queue for single-consumer single-value semantics |
| `R16` depth texture | `LoadRawTextureData` accepts the raw `uint16` bytes from the `16UC1` ROS message directly — zero CPU conversion |
| Compute shader `invFx`, `invFy` | Pre-computed reciprocals avoid per-thread GPU division (no `fdiv` in the inner loop) |
| `float4` StructuredBuffer stride | SPIR-V `std430` layout assigns `ArrayStride=16` regardless of component count; using `float3` would silently corrupt every element past index 0 on Vulkan |
| `mesh.UploadMeshData(true)` | Frees the CPU-side copy; the index buffer is immutable so this saves ~1.2 MB RAM at 640×480 |
| `ROSConnection.incomingQueueCapacity = 16` | Hard cap on the ROSConnection raw byte queue; without it the queue grows at `(90 fps − Unity fps) × topics/sec`, producing a slow-motion display seconds behind reality |

---

## 4. Combined Data Flow (wall-clock view)

```mermaid
gantt
    title Single 11 ms frame budget at 90 Hz
    dateFormat x
    axisFormat %L ms

    section Main Thread
    Update() intrinsics/resize   :a1, 0, 1
    LateUpdate() drain + upload  :a2, 1, 3
    DispatchCompute()            :a3, 3, 4

    section GPU (async)
    Compute back-projection      :b1, 4, 7
    Vertex + Geometry shader     :b2, 7, 9

    section Budget remaining
    Rendering / scripts / XR     :c1, 9, 11
```

---

## 5. Key Numbers

| Metric | Value |
|---|---|
| Depth stream | 640 × 480 × 90 Hz × 2 B = **55.3 MB/s** |
| Colour stream (raw) | 480 × 270 × 90 Hz × 3 B = **35.0 MB/s** |
| Colour stream (JPEG) | ~3–5 MB/s but adds ~5–10 ms decode/frame |
| IMU gyro | ~500 Hz, negligible bandwidth |
| GPU threads per depth frame | 640 × 480 = **307 200** |
| Frame budget at 90 Hz | **11.1 ms** total |
| Estimated compute budget used | ~4–6 ms (depth + colour upload + dispatch) |

---

## 6. AprilTag Detection Pipeline

```mermaid
flowchart TB
    subgraph ROS2["ROS 2 Humble"]
        CI2["color/camera_info\nfx fy cx cy"]
        COL["/color/image_raw\nrgb8 480×270 @ 90 Hz"]
        DEP["/depth/image_rect_raw\n16UC1 640×480 @ 90 Hz"]
        IMU2["/imu/data\n~500 Hz gyro + accel"]
    end

    subgraph TCP["ROS-TCP-Connector"]
        TQ["ConcurrentQueue\ncap = 16\ntrims oldest on overflow"]
    end

    subgraph Unity_Main["Unity Main Thread"]
        direction TB
        PCR["ROSPointCloudRenderer\nColorTexture GPU-resident\nDepthTexture R16\nfx fy cx cy live"]
        DET["Detection.cs\nLateUpdate()\nthrottle = 15 Hz"]
        PIX["ColourTexture.GetPixels32()\nCPU readback\n480×270 Color32 array"]
        BURST["TagDetector.ProcessImage()\nUnity Burst Job\nImageU8 greyscale\ndecimation ÷2\nPoseEstimationJob"]
        DTAGS["DetectedTags\nTagPose[]\nPosition + Rotation\ncamera-local space"]
    end

    subgraph PoseEst["PoseEstimation.cs"]
        direction TB
        CAM_POSE["GetDepthRefinedCameraPose()\n① monocular Z from tag size + FOV\n② project onto colour image → UV\n③ SampleDepth 9x9 median patch\n④ replace Z with depth reading\n⑤ reproject X Y via pinhole\nresult: mm-accurate camera-space Pose"]
        IMU_PATH["CameraToWorldFromPose()\ncamWorldPos + Rot × tagCamPos\nuses Madgwick filter output"]
        FK_PATH["CameraToWorld()\nfallback: D455_Camera.transform\nused when IMU has no fix"]
        EEF_POSE["EEF frame\neefTF.InverseTransformPoint(worldPos)"]
        BASE_POSE["Base frame\nWorldToBase()\nrobotBase.InverseTransformPoint(worldPos)"]
    end

    subgraph IMU["IMUSubscriber.cs"]
        MADG["Madgwick filter\ngyro + accel fusion\n~500 Hz on ROS bg thread"]
        CWR["CameraWorldRotation\nCameraWorldPosition\nPoseValid flag"]
    end

    subgraph HUD["CalibrationHUD.cs"]
        SB["StringBuilder\nTAG n\n  CAM t r\n  EEF t r\n  BASE t r\n──\nIMU header\naccel\norientation"]
        TMP["TextMeshPro\nsingle element\nfontSize 24\nwhite forced every frame"]
        CORNR["Corner markers\n4 × RectTransform dots"]
        AXIS["Axis lines\n3 × RectTransform lines\nX red Y cyan Z blue"]
    end

    ROS2 -->|TCP| TQ --> PCR
    IMU2 -->|TCP| MADG --> CWR
    PCR -->|ColorTexture\nreused, no dup sub| DET
    DET --> PIX --> BURST --> DTAGS
    DTAGS --> CAM_POSE
    PCR -->|DepthTexture + fx fy cx cy| CAM_POSE
    CAM_POSE --> IMU_PATH
    CAM_POSE --> FK_PATH
    CWR -->|PoseValid=true| IMU_PATH
    IMU_PATH --> EEF_POSE & BASE_POSE
    FK_PATH --> EEF_POSE & BASE_POSE
    EEF_POSE & BASE_POSE --> SB --> TMP
    DTAGS --> CORNR & AXIS
```

### Throttle + timing

```mermaid
sequenceDiagram
    participant FR  as Unity Frame<br/>~11 ms @ 90 Hz
    participant DET as Detection.LateUpdate()
    participant GP  as GetPixels32()<br/>CPU readback
    participant BST as Burst Job<br/>ProcessImage()
    participant CA  as CalibrationHUD.LateUpdate()
    participant TMP as TextMeshPro

    loop Every frame (90 Hz)
        FR  ->> DET : LateUpdate()
        alt unscaledTime ≥ nextDetectionTime  (15 Hz gate)
            DET ->> GP  : GetPixels32()  ~1 ms
            GP  ->> BST : ProcessImage(pixels, fovV, tagSize)  ~2–4 ms
            BST -->> DET: DetectedTags updated
            Note over DET: nextDetectionTime += 1/15
        else
            DET -->> DET: skip detection, DrawTags() only
        end
        FR  ->> CA  : LateUpdate()
        CA  ->> TMP : tagBlock + imuBlock written once
    end
```

### Pose estimation math

```mermaid
flowchart LR
    A["tag.Position\ncamera-space XYZ\nfrom Burst monocular"] --> B["ProjectToUV()\nu = x/z·fx + cx\nv = -y/z·fy + cy"]
    B --> C["SampleDepth()\n9×9 patch on R16\nmedian filter\nmm → metres"]
    C --> D["Refined XYZ\nx = (u−cx)/fx · D\ny = (v−cy)/fy · D\nz = D"]
    D --> E{"IMU\nPoseValid?"}
    E -->|yes| F["camWorld + camRot × tagCam\n= tag world pose (IMU)"]
    E -->|no| G["D455_Camera.transform\n.TransformPoint(tagCam)\n= tag world pose (FK)"]
    F & G --> H["InverseTransformPoint()\ninto EEF / base frame"]
```

### Performance decisions

| Decision | Reason |
|---|---|
| 15 Hz detection gate | `GetPixels32()` is a full CPU readback + Burst job: ~3–5 ms; running at 90 Hz would consume the entire frame budget stalling all rendering |
| Reuse ROSPointCloudRenderer colour texture | Avoids a second ROS subscription, second TCP queue, and second GPU upload for the same stream |
| 9×9 median depth patch | Single-pixel depth on a tag centre can be occluded, noisy, or zero; the patch collects up to 81 readings and returns the median, rejecting outliers |
| IMU → world pose, not FK | FK-based `D455_Camera.transform` is static in the scene unless RobotFKSolver moves it; the Madgwick filter reflects the physical camera orientation in real-time |
| Decimation = 2 | Halves the input image to `240×135`; Burst job is 4× faster; detection quality is unchanged for tags ≥ 5 cm at < 2 m |

---

## 7. TCP Optimization

```mermaid
flowchart TB
    subgraph Physical["Physical Layer"]
        USB["D455 USB 3.x\n~90 MB/s raw depth + colour"]
        LAN["Localhost / LAN\nros_tcp_endpoint port 10000"]
    end

    subgraph ROS["ROS 2 Humble"]
        PUB_D["depth publisher\n16UC1 @ 90 Hz\n~614 KB/frame → 55 MB/s"]
        PUB_C["colour publisher\nrgb8 @ 90 Hz\n~374 KB/frame → 34 MB/s"]
        PUB_CI["camera_info\n@ 90 Hz\n~1 KB/frame"]
        PUB_IMU["imu/data\n~500 Hz\n~200 B/msg"]
        EP["ros_tcp_endpoint\nfanout → one TCP conn/topic"]
    end

    subgraph TCPStack["ROS-TCP-Connector (Unity)"]
        direction TB
        RT["ReaderThread (async Task)\ncontinuous socket read\nno sleep on data\nframing: 4-byte topic len\n+ 4-byte msg len + bytes"]
        IQ2["ConcurrentQueue\nTuple string,byte\nincomingQueueCapacity = 16\ntrim oldest when > 16\n⟹ always ≤ 16 messages\n   lag at most ~178 ms @90Hz"]
        UPDQ["ROSConnection.Update()\nmain thread\nmaxMessageProcessingMs = 0\n→ process ALL queued messages\nper frame (no budget cap)"]
        DISP["Dispatch to subscriber callbacks\nOnDepthImageReceived\nOnColorImageReceived\nOnCameraInfoReceived\nOnImuReceived"]
    end

    subgraph Handoff["Per-topic queues (lock-free)"]
        DQ2["Depth ConcurrentQueue\ncap = 3\ndrains all in LateUpdate\nuploads LATEST only"]
        CS2["Colour Interlocked slot\nbyte[] _colorSlot\nInterlocked.Exchange\nonly newest kept"]
        CI2["volatile bool _pendingIntrinsics\nfx fy cx cy copied atomically"]
        IMU_Q["IMU ring buffer\nMadgwick runs on bg thread\nresult in CameraWorldRotation"]
    end

    USB --> LAN --> EP
    PUB_D & PUB_C & PUB_CI & PUB_IMU --> EP
    EP -->|TCP socket| RT --> IQ2 --> UPDQ --> DISP
    DISP --> DQ2 & CS2 & CI2 & IMU_Q
```

### Latency budget

```mermaid
flowchart LR
    A["D455 frame captured\nt = 0"] --> B["ROS realsense2_camera\nDMA + align_depth\n~2 ms"]
    B --> C["TCP transmit\nlocalhost ~0 ms\nLAN ~1–3 ms"]
    C --> D["ReaderThread\nbytes enqueued\n< 0.1 ms"]
    D --> E["Queue wait\n0–11 ms\n(next Unity frame)"]
    E --> F["ROSConnection.Update()\ndispatch callback\n< 0.2 ms"]
    F --> G["LateUpdate()\nLoadRawTextureData\nDispatchCompute\n~2 ms"]
    G --> H["Rendered frame\ntotal latency\n~5–17 ms"]
```

### Overflow strategy per topic

```mermaid
flowchart TB
    subgraph Strategy["Drop policy by data type"]
        direction LR
        DEPTH_S["Depth frames\nConcurrentQueue cap 3\nmatch-drain: only latest\nrendered each frame\nstale frames silently dropped"]
        COLOR_S["Colour frames\nInterlocked.Exchange slot\nbyte → always newest\nno queue growth possible"]
        CI_S["camera_info\nvolatile bool flag\nno queue - single bool\ng-race safe enough for\nread-once startup"]
        IMU_S["IMU messages\nprocessed on bg thread\nMadgwick filter\nbuffers gyro+accel internally\nresult polled by main thread"]
        RAW_S["ROSConnection raw queue\ncap 16 across ALL topics\noldest raw bytes trimmed\nprevents unbounded memory\nand multi-second replay lag"]
    end
```

### Key parameters

| Parameter | Value | Effect |
|---|---|---|
| `incomingQueueCapacity` | 16 | Hard global cap on the ROSConnection raw byte queue; without it the queue grows at `(pub Hz − Unity fps) × nTopics`, building multi-second lag |
| `maxMessageProcessingMs` | 0 (unlimited) | Process every queued message per frame; a budget cap here would leave stale depth/colour in the queue and increase visual latency |
| Depth `ConcurrentQueue` cap | 3 | Absorbs short TCP burst jitter; `LateUpdate` drains all and uploads only the latest, so the display is never more than 1 frame stale |
| Colour `Interlocked` slot | 1 | O(1) lock-free; only the newest colour frame needed; eliminates queue allocation and GC entirely for the colour stream |
| Detection reuse of `ColorTexture` | shared ref | Zero-cost: same `Texture2D` pointer; no second TCP subscription, no second `LoadRawTextureData`, no second GPU upload |
| `align_depth.enable = true` | ROS launch | Depth → colour alignment on the D455 DSP; Unity does not need to handle depth-colour pixel registration in software |

