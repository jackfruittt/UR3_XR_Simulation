# GPU-Accelerated Point Cloud Renderer

## Overview

The point cloud renderer converts live RealSense depth and colour image streams into a 3D point cloud rendered inside Unity, achieving **30+ FPS** by keeping the depth-to-3D conversion entirely on the GPU.

The implementation is split across three files:

| File | Type | Purpose |
|------|------|---------|
| `Assets/Scripts/ROS/ROSPointCloudRenderer.cs` | C# MonoBehaviour | ROS subscription, GPU dispatch, mesh management |
| `Assets/Shaders/RealSense/DepthToPointCloud.compute` | HLSL Compute Shader | Depth pixel → XYZ + colour (GPU) |
| `Assets/Shaders/RealSense/PointCloudVertexColor.shader` | HLSL Shader | Renders the resulting point mesh |

Architecture is adapted from the Intel RealSense Unity SDK (`RsPointCloudRenderer.cs` in librealsense) with modifications for ROS-TCP image delivery.

---

## Background Theory

This section explains the GPU computing and shader concepts from the perspective of someone with a ROS/robotics background. If you are already comfortable with Unity shaders, skip ahead to [Architecture & Data Flow](#architecture--data-flow).

### GPU Computing vs CPU Computing — the Mental Model

On the CPU you write code that runs **sequentially** (or across a handful of threads). In a ROS node, a depth callback unpacks one pixel, does some math, moves to the next pixel, and so on. For a 640×480 image that is 307 200 iterations — fine for a desktop callback, but too slow to stay inside a single 33 ms render frame while also running the rest of Unity.

A GPU is a different kind of processor. It has **thousands of small cores** designed to do the same operation to many pieces of data simultaneously. The deal is:

- You cannot branch wildly or share state easily between cores.
- But if every piece of work is independent (or close to it) the GPU does it all **in parallel**.

Converting a depth pixel to a 3D point is perfectly parallel — pixel (100, 200) does not depend on pixel (101, 200). So instead of a for-loop on the CPU, we launch one GPU thread per pixel, all 307 200 running at the same time.

The HLSL file `DepthToPointCloud.compute` is that program. Each invocation of the kernel function receives `id` — the index of the one pixel it is responsible for — and does its work independently of every other invocation.

### What is a Compute Shader?

A **compute shader** is a GPU program that does **general-purpose computation** rather than drawing pixels to the screen. Think of it as a ROS node that runs on the GPU:

| ROS concept | GPU compute equivalent |
|-------------|----------------------|
| A ROS service or callback | A compute shader kernel function |
| `ros2 service call` | `depthToXYZShader.Dispatch(...)` |
| Input topic message | Input texture / structured buffer bound to the shader |
| Output topic message | Output structured buffer written by the shader |
| Node parameters | Shader uniform variables (`fx`, `fy`, etc.) |
| Running in parallel via multiple nodes | Thread groups — the GPU runs thousands of kernel invocations simultaneously |

The `.compute` file is written in **HLSL** (High-Level Shading Language). It looks like C with some extra keywords. The key addition is `SV_DispatchThreadID` — a built-in variable that tells each thread which pixel it owns, equivalent to knowing your own array index in a parallel for-loop.

### Thread Groups — How the GPU Schedules Work

When the C# code calls:

```csharp
depthToXYZShader.Dispatch(kernel, 80, 60, 1);
```

it is launching an 80×60 grid of **thread groups**. Each thread group runs the kernel with `[numthreads(8, 8, 1)]`, meaning 64 threads in an 8×8 tile. Total threads:

$$80 \times 60 \times 8 \times 8 = 307\,200$$

One thread per pixel. The GPU schedules these groups across its cores; Unity just fires the dispatch and continues on the CPU. There is no waiting, no polling — the GPU runs in parallel with the rest of the game loop.

### GPU Memory — Buffers and Textures

From a ROS perspective you are used to messages flowing through topics. On the GPU the equivalent is **memory objects** bound to the shader before dispatch.

There are two kinds used here:

**Texture2D** — a 2D array of pixels with a fixed format. Unity uploads the CPU `Texture2D` to the GPU via `SetPixelData` / `LoadRawTextureData` + `Apply()`. The `.Apply()` call is the equivalent of `ros2 topic pub` — it finalises the upload and makes the new data visible to the GPU.

**StructuredBuffer / RWStructuredBuffer** — a flat array of arbitrary structs on the GPU. `RW` means the shader can write to it. This is how the compute kernel outputs its results:

```
vertexBuffer[0]  = XYZ position of pixel (0,0)
vertexBuffer[1]  = XYZ position of pixel (1,0)
...
vertexBuffer[307199] = XYZ position of pixel (639,479)
```

Neither buffer ever comes back to the CPU. After the compute shader fills `vertexBuffer`, the *vertex shader* (the next stage in the pipeline, described below) reads from it directly. This is the biggest performance advantage — no `GPU → CPU readback`, which would stall the pipeline.

### The Rendering Pipeline — How Unity Draws the Points

After the compute shader runs, Unity needs to actually **draw** the points on screen. This is where the standard GPU rendering pipeline takes over.

In "normal" 3D rendering you might think of Unity as: *load a mesh → apply a material → draw it*. Under the hood, drawing a mesh involves the GPU running two more programs in sequence:

```
CPU sends: "draw this mesh with this material"
     │
     ▼
┌─────────────────────────────────────────────────┐
│  VERTEX SHADER  (runs once per vertex/point)    │
│  Input:  vertex index (SV_VertexID)             │
│  Reads:  vertexBuffer[SV_VertexID] → XYZ        │
│  Output: clip-space position + colour to pass   │
│          to fragment shader                     │
└──────────────────┬──────────────────────────────┘
                   │  Rasterisation
                   │  (GPU converts geometry to pixels)
                   ▼
┌─────────────────────────────────────────────────┐
│  FRAGMENT SHADER  (runs once per screen pixel)  │
│  Input:  colour from vertex shader              │
│  Output: final pixel colour → frame buffer      │
└─────────────────────────────────────────────────┘
```

The mesh uses `MeshTopology.Points` — each "triangle" is actually a single vertex rendered as a point sprite. So the vertex shader runs once per depth pixel, reads its pre-computed XYZ position from the buffer, transforms it into screen space, and passes the colour through. The fragment shader just outputs that colour — no lighting, no texture sampling.

`SV_VertexID` is a built-in that gives the vertex shader its index in the draw call — exactly like how `SV_DispatchThreadID` gave the compute shader its pixel index.

### Why Is HLSL Used Instead of C#?

HLSL code runs **on the GPU**, not on the CPU. Unity cannot execute C# on the GPU. HLSL is compiled by Unity's shader compiler into GPU machine code at import time. The `.compute` and `.shader` files are the GPU programs; the C# MonoBehaviour is the CPU manager that sets up inputs and fires the dispatch.

A useful mental model: the C# is like a ROS launch file — it configures parameters and starts things running. The HLSL files are the actual node code that does the computation.

---

### HLSL — The Language

**HLSL** (High-Level Shading Language) is Microsoft's C-like language for writing GPU programs. It looks almost identical to C:

- Same primitive types: `int`, `float`, `bool`
- Same control flow: `if`, `for`, `return`
- Same function syntax

The additions that make it GPU-specific:

**Vector/matrix types built in**

```hlsl
float3 position = float3(1.0, 2.0, 3.0);  // a 3-component vector
float4 colour   = float4(0.8, 0.4, 0.0, 1.0);  // RGBA
position.xy     // swizzle — grab just X and Y as a float2
colour.rgb      // grab just the RGB channels
colour.bgr      // grab them in a different order (used for BGR swap)
```

Swizzling is just syntax sugar for accessing components by name. `col.bgr` is equivalent to `float3(col.b, col.g, col.r)` — it reorders the channels in a single instruction.

**Texture and buffer types**

```hlsl
Texture2D<float>  DepthTexture;              // read-only 2D texture of floats
RWStructuredBuffer<float3> VertexBuffer;     // RW = read/write flat array
```

`RW` means the shader can write to it. Without `RW`, the binding is read-only. The difference matters because the GPU needs to know ahead of time which resources a shader will write to, so it can manage memory coherency between thread groups.

**Semantic annotations (the `: SV_` syntax)**

```hlsl
void DepthToXYZ(uint3 id : SV_DispatchThreadID)
```

The `: SV_DispatchThreadID` part is called a **semantic**. It is how HLSL connects a variable to a GPU-system-provided value rather than data you pass in yourself. `SV_` stands for *system value*. Common ones:

| Semantic | Shader type | What it gives you |
|----------|-------------|-------------------|
| `SV_DispatchThreadID` | Compute | Global thread index — which pixel this invocation owns |
| `SV_VertexID` | Vertex | Which vertex in the draw call this invocation is processing |
| `SV_Target` | Fragment | Marks the return value as the output pixel colour |
| `POSITION` | Vertex | The clip-space position to pass to the rasteriser |
| `COLOR` | Vertex/Fragment | Per-vertex colour to interpolate across a polygon |
| `PSIZE` | Vertex | Point sprite size in screen pixels |

You cannot read these values from a ROS message or a buffer — the GPU fills them in automatically based on what stage of the pipeline is running. They are the GPU's equivalent of reading `rospy.get_time()` — values the runtime knows without you providing them.

---

### `#pragma` Directives — Shader Compiler Instructions

`#pragma` lines are **not executable code** — they are instructions to the **shader compiler** about how to compile the file. The analogy in ROS would be metadata in a `package.xml` or flags passed to `colcon build`.

#### In `.compute` files

```hlsl
#pragma kernel DepthToXYZ
```

This tells Unity's shader compiler: *"there is an entry-point function called `DepthToXYZ` in this file — expose it as a named kernel."* Without this line, the function exists in HLSL but Unity's C# API cannot find it by name with `FindKernel("DepthToXYZ")`. You can have multiple kernels in one file, each with its own `#pragma kernel` line.

#### In `.shader` files

```hlsl
#pragma vertex vert
#pragma fragment frag
```

Same idea — these tell the compiler which function is the vertex shader entry point and which is the fragment shader entry point. The names `vert` and `frag` are conventional but not required.

```hlsl
#pragma target 3.0
```

Specifies the minimum shader model (hardware capability level) required. Higher numbers allow more features (more registers, texture reads in vertex shaders, etc.). If you leave this out Unity picks a default that may be too low for compute buffers.

---

### `#include "UnityCG.cginc"` — Unity's Shader Standard Library

```hlsl
#include "UnityCG.cginc"
```

This is equivalent to `#include <ros/ros.h>` — it pulls in Unity's built-in helper functions. The most commonly used one in this shader:

```hlsl
o.vertex = UnityObjectToClipPos(v.vertex);
```

`UnityObjectToClipPos` applies the full **Model-View-Projection (MVP)** matrix transform to convert a vertex position from *object/local space* into *clip space* (the GPU's normalised coordinate system before the screen projection is applied). Without this, you would have to manually multiply three matrices together every frame and keep them in sync with the camera — `UnityCG.cginc` handles that automatically.

Other useful helpers in `UnityCG.cginc`: `WorldSpaceViewDir`, `TRANSFORM_TEX`, `LinearEyeDepth`. You will see these in more advanced shaders.

---

### Cg vs HLSL vs ShaderLab — Clearing up the naming

Unity's `.shader` files contain three different languages layered together:

```
Shader "ROS/PointCloudVertexColor"     ← ShaderLab (Unity-specific wrapper)
{
    Properties { ... }                 ← ShaderLab (Inspector-exposed inputs)
    SubShader
    {
        Pass
        {
            CGPROGRAM                  ← starts a Cg/HLSL block
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            // ... actual HLSL code here
            
            ENDCG                      ← ends the Cg/HLSL block
        }
    }
}
```

| Language | Role |
|----------|------|
| **ShaderLab** | Unity's own declarative wrapper. Declares properties, render state (blending, culling, depth testing), and which GPU passes to run. Not HLSL. |
| **Cg** | NVIDIA's old GPU shading language, nearly identical to HLSL. Unity historically used `CGPROGRAM`/`ENDCG` blocks. Mostly interchangeable with HLSL for simple shaders. |
| **HLSL** | Microsoft's language, now the preferred option. Use `HLSLPROGRAM`/`ENDHLSL` for newer Unity URP/HDRP shaders. |

For the simple `PointCloudVertexColor.shader` used here, the difference is irrelevant — the code inside `CGPROGRAM`/`ENDCG` is valid HLSL either way.

`.compute` files skip ShaderLab entirely — they are pure HLSL.

---

## Architecture & Data Flow

```
ROS Topics                 CPU (C#)                     GPU
─────────────────────────────────────────────────────────────────────
/aligned_depth_to_color ──► OnDepthImageReceived()
  [16-bit raw, ~0.6 MB]      unpack ushort → float[]
                             SetPixelData() ──────────► DepthTexture (RFloat)

/color/image_raw ─────────► OnColorImageReceived()
  [RGB24, ~0.9 MB]           LoadRawTextureData() ────► ColorTexture (RGB24)

                            UpdatePointCloud()
                             SetTexture() ────────────►
                             Dispatch(w/8, h/8, 1) ───► DepthToXYZ kernel
                                                            per pixel:
                                                            depth → float Z
                                                            (x-cx)*Z/fx → X
                                                            (y-cy)*Z/fy → Y
                                                         VertexBuffer[i] = XYZ
                                                         ColorBuffer[i]  = RGBA

                             SetBuffer() ─────────────► Vertex Shader reads
                                                         VertexBuffer via SV_VertexID
                                                         → MeshTopology.Points
```

**Key design decision**: subscribing to depth and colour *images* (~1.5 MB/frame combined) rather than `PointCloud2` (~4.9 MB/frame) eliminates the heaviest part of the CPU deserialisation bottleneck. The 3D reprojection math that would otherwise run on the CPU is instead dispatched to the GPU in parallel across all 307 200 pixels simultaneously.

---

## ROSPointCloudRenderer.cs

### Inspector Properties

#### ROS Topics
| Property | Default | Description |
|----------|---------|-------------|
| `depthTopic` | `/camera/camera/aligned_depth_to_color/image_raw` | 16-bit raw depth topic |
| `colorTopic` | `/camera/camera/color/image_raw` | RGB colour topic |

Using the **aligned** depth topic is important — it ensures each depth pixel corresponds to the colour pixel at the same UV coordinate, so colours map correctly onto 3D points.

#### Camera Intrinsics (RealSense D455 @ 640×480)
| Property | Default | Description |
|----------|---------|-------------|
| `fx` | 385.0 | Focal length X in pixels |
| `fy` | 385.0 | Focal length Y in pixels |
| `cx` | 320.0 | Principal point X (optical centre) |
| `cy` | 240.0 | Principal point Y (optical centre) |

These values come from the camera's calibration. To retrieve exact runtime values:
```bash
ros2 topic echo /camera/camera/color/camera_info --once
# Read fx=K[0], fy=K[4], cx=K[2], cy=K[5]
```

#### Settings
| Property | Default | Description |
|----------|---------|-------------|
| `width` | 640 | Image width in pixels |
| `height` | 480 | Image height in pixels |
| `depthScale` | 0.001 | Converts raw depth units (mm) to metres |
| `minDepth` | 0.1 m | Pixels below this threshold are discarded (noise) |
| `maxDepth` | 10.0 m | Pixels beyond this are discarded |
| `flipYZ` | true | Converts ROS Y-down → Unity Y-up coordinate system |
| `pointSize` | 0.005 | Point size passed to the material |
| `showDebugInfo` | true | Logs FPS to console every 2 seconds |

#### Compute Shader
Drag `DepthToPointCloud.compute` from `Assets/Shaders/RealSense/` into this slot.

---

### Startup Sequence (`Start`)

1. Acquires a `ROSConnection` instance (creates one if absent).
2. Validates the compute shader is assigned; disables the component if not.
3. Calls `InitializeMesh()` → builds the GPU resources.
4. Calls `CacheShaderIDs()` → resolves all `Shader.PropertyToID` values once.
5. Calls `SetStaticShaderParameters()` → uploads intrinsics and depth settings to the compute shader (done once, not per frame).
6. Subscribes to both ROS image topics.

---

### Mesh and GPU Buffer Initialisation (`InitializeMesh`)

```
pointCount = width × height  (307 200 for 640×480)
```

| Resource | Type | Size | Purpose |
|----------|------|------|---------|
| `mesh` (vertices) | `Vector3[]` | 307 200 × 12 B | Placeholder positions; actual positions come from `VertexBuffer` |
| `mesh` (UVs) | `Vector2[]` | 307 200 × 8 B | Normalised pixel coordinates `(x/W, y/H)` |
| `mesh` (indices) | `int[]` | 307 200 × 4 B | `[0, 1, 2 … N-1]` for `MeshTopology.Points` |
| `vertexBuffer` | `ComputeBuffer` | 307 200 × 12 B | GPU positions output (`float3`) |
| `colorBuffer` | `ComputeBuffer` | 307 200 × 16 B | GPU colour output (`float4`) |
| `depthTexture` | `Texture2D` RFloat | 640×480 | Depth values uploaded from CPU each frame |
| `colorTexture` | `Texture2D` RGB24 | 640×480 | Colour values uploaded from CPU each frame |

Pre-allocated CPU arrays (`depthData`, `rgbData`) eliminate heap allocations in the ROS callback hot path.

---

### Depth Callback (`OnDepthImageReceived`)

Accepts only `16UC1` / `mono16` encoded messages. Each ROS message carries raw 16-bit little-endian depth values in millimetres.

```csharp
// Unpack two bytes per pixel into a float (no division — depthScale applied in shader)
ushort depthMM = (ushort)(msg.data[i * 2] | (msg.data[i * 2 + 1] << 8));
depthData[i] = depthMM;           // stored as float in RFloat texture
```

The raw millimetre value is stored as a `float` in an `RFloat` texture. Conversion to metres happens in the compute shader (`depthMM * depthScale`), keeping the CPU path allocation-free and numerically simple.

After uploading: `depthTexture.Apply(false, false)` — the second `false` suppresses GPU mipmap generation (unnecessary for point data).

If both depth and colour have arrived, `UpdatePointCloud()` is triggered immediately.

---

### Colour Callback (`OnColorImageReceived`)

Accepts `rgb8` and `bgr8` encodings. Raw bytes are loaded directly into the colour texture with `LoadRawTextureData` — no CPU byte-swap loop. The compute shader is told which channel order arrived via the `flipBGR` boolean, and does the swizzle on the GPU at effectively zero cost.

---

### Compute Dispatch (`UpdatePointCloud`)

```csharp
int threadGroupsX = Mathf.CeilToInt(width  / 8.0f);   // 80 groups
int threadGroupsY = Mathf.CeilToInt(height / 8.0f);   // 60 groups
depthToXYZShader.Dispatch(kernel, threadGroupsX, threadGroupsY, 1);
// → 4 800 thread groups × 64 threads = 307 200 threads running in parallel
```

After dispatch the compute buffers (`vertexBuffer`, `colorBuffer`) are bound to the mesh renderer's material. The vertex shader reads positions directly from `_VertexBuffer` using `SV_VertexID` — **no CPU readback ever occurs**.

---

### Resource Cleanup (`OnDestroy`)

All GPU resources are explicitly released to avoid memory leaks:
```csharp
vertexBuffer?.Release();
colorBuffer?.Release();
Destroy(depthTexture);
Destroy(colorTexture);
Destroy(mesh);
```

---

## DepthToPointCloud.compute

### Kernel: `DepthToXYZ`

Thread group size: `[numthreads(8, 8, 1)]` — each thread processes one pixel.

#### Inputs

| Binding | HLSL Type | Description |
|---------|-----------|-------------|
| `DepthTexture` | `Texture2D<float>` | RFloat texture — raw depth in mm |
| `ColorTexture` | `Texture2D<float4>` | RGB24 colour texture |
| `VertexBuffer` | `RWStructuredBuffer<float3>` | Output XYZ positions |
| `ColorBuffer` | `RWStructuredBuffer<float4>` | Output RGBA colours |

#### Scalar Parameters

| Parameter | Type | Set when | Description |
|-----------|------|----------|-------------|
| `fx`, `fy` | float | Startup | Focal lengths in pixels |
| `cx`, `cy` | float | Startup | Principal point in pixels |
| `depthScale` | float | Startup | mm → metres conversion (0.001) |
| `minDepth`, `maxDepth` | float | Startup | Valid depth range in metres |
| `width`, `height` | int | Startup | Image dimensions |
| `flipYZ` | bool | Startup | Y-axis flip for Unity coordinate system |
| `flipBGR` | bool | Per-frame | Set when colour encoding is `bgr8` |

#### Execution Logic

```hlsl
float depthMM = DepthTexture[id.xy];          // exact texel, no interpolation
float depthM  = depthMM * depthScale;

// Sentinel and range checks
if (depthMM == 0.0 || depthM < minDepth || depthM > maxDepth)
{
    VertexBuffer[index] = float3(0, 0, INVALID_Z);  // INVALID_Z = 999.0
    ColorBuffer[index]  = float4(0, 0, 0, 0);       // transparent
    return;
}

// Pinhole camera back-projection
float x = ((float)id.x - cx) * depthM / fx;
float y = ((float)id.y - cy) * depthM / fy;
float z = depthM;

// ROS: X-right, Y-down, Z-forward
// Unity: X-right, Y-up,   Z-forward
VertexBuffer[index] = flipYZ ? float3(x, -y, z) : float3(x, y, z);

// Optional BGR swizzle
float4 col = ColorTexture[id.xy];
if (flipBGR) col = float4(col.b, col.g, col.r, 1.0);
ColorBuffer[index] = col;
```

**Why direct indexing?** `DepthTexture[id.xy]` reads the exact texel with no filtering. Using `SampleLevel` with a sampler would introduce sub-pixel interpolation that corrupts metric depth values.

**Why `INVALID_Z = 999.0`?** Invalid points are pushed far behind the scene rather than collapsed to the origin, which avoids the visual artefact of a dense cluster of black points at world-space zero.

---

## PointCloudVertexColor.shader

A minimal Cg/HLSL shader in the `ROS/PointCloudVertexColor` namespace.

### Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `_PointSize` | Float | 0.005 | Controls the rendered point size via `PSIZE` |
| `_Color` | Color | white | Global tint multiplied with per-vertex colour |

### Vertex Shader

```hlsl
v2f vert(appdata v)
{
    o.vertex = UnityObjectToClipPos(v.vertex);  // standard MVP transform
    o.color  = v.color * _Color;                // tint vertex colour
    o.size   = _PointSize;                      // point rasterisation size
}
```

The `PSIZE` semantic sets the screen-space point size in pixels. Note: `PSIZE` support varies across graphics APIs — it is well supported on DirectX (PC) and Vulkan but may be ignored on Metal (macOS/iOS).

### Fragment Shader

```hlsl
fixed4 frag(v2f i) : SV_Target
{
    return i.color;  // output the interpolated vertex colour directly
}
```

No lighting calculation — colours come directly from the colour camera, so diffuse shading would be incorrect.

---

## Performance Reference

| Approach | Bandwidth per frame | XYZ conversion | Typical FPS |
|----------|--------------------|--------------------|-------------|
| PointCloud2 subscription | ~4.9 MB | CPU deserialisation | 0–10 |
| Images + GPU compute (this) | ~1.5 MB | GPU parallel (307 200 threads) | 30+ |

The roughly 3× bandwidth reduction comes from the fact that `PointCloud2` encodes 32 bytes per point (XYZ + padding + RGB + padding) for all 307 200 points, while a 16-bit depth image encodes 2 bytes per pixel and the colour image 3 bytes per pixel.

---

## Setup Guide

### 1. Create the GameObject

1. `GameObject → Create Empty`, name it `ROSPointCloud`.
2. Add a **MeshFilter** and **MeshRenderer** component (or let `[RequireComponent]` add them automatically when you attach the script).
3. Attach `ROSPointCloudRenderer.cs`.

### 2. Create and Assign a Material

1. `Right-click in Project → Create → Material`, name it `PointCloudMaterial`.
2. Set the shader to `ROS/PointCloudVertexColor`.
3. Drag the material onto the `MeshRenderer` on the GameObject.

### 3. Configure the Script

In the Inspector:
- **Compute Shader** — drag `DepthToPointCloud.compute` into the slot.
- **Depth/Colour Topics** — confirm they match your ROS camera namespace.
- **Camera Intrinsics** — adjust `fx`, `fy`, `cx`, `cy` to match your camera info topic if needed.
- **Flip YZ** — leave enabled for correct Unity orientation.

### 4. Verify ROS Side

```bash
# Check topics are publishing
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/camera/color/image_raw

# Verify Unity subscribed (after pressing Play)
ros2 topic info /camera/camera/aligned_depth_to_color/image_raw
# Subscription count should be 1
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No points visible | Compute shader not assigned | Drag `DepthToPointCloud.compute` into Inspector slot |
| Points all black | Colour topic not receiving | Check colour topic name and encoding |
| Points misaligned with colour | Wrong intrinsics or non-aligned depth topic | Use `aligned_depth_to_color` topic; verify `fx/fy/cx/cy` |
| Points flipped upside-down | `flipYZ` is false | Enable `flipYZ` in Inspector |
| Low FPS | Network bandwidth | Reduce resolution in ROS launch file and update `width`/`height` fields |
| Dense cluster at origin | Old Unity version ignoring `INVALID_Z` | Increase `minDepth` to push valid range away from zero |
