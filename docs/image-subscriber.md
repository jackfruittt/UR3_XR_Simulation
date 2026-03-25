# Simple Image Subscriber

## Overview

`SimpleImageSubscriber` is a Unity MonoBehaviour that subscribes to a single RealSense image topic over ROS-TCP and displays the result on a `Renderer` component (e.g., a quad in the scene). It supports colour images (raw and JPEG/PNG compressed) as well as raw 16-bit depth images visualised as a greyscale gradient.

**File**: `Assets/Scripts/ROS/SimpleImageSubscriber.cs`

This component is intended for **diagnostic and visualisation purposes** — displaying a camera feed on an in-scene surface. For performant 3D point cloud rendering, see [point-cloud-renderer.md](point-cloud-renderer.md).

---

## Inspector Properties

### ROS Topic

| Property | Default | Description |
|----------|---------|-------------|
| `imageTopic` | `/camera/camera/color/image_raw` | Base topic name. Overridden by the depth flags below |
| `useCompressed` | false | Subscribe to the `/compressed` variant (JPEG/PNG). Ignored for depth topics |
| `useDepthCamera` | false | Switches to `/camera/camera/depth/image_rect_raw` |
| `useAlignedDepth` | false | Switches to `/camera/camera/aligned_depth_to_color/image_raw` |

**Topic selection order** (evaluated at `Start`):
1. If `useAlignedDepth` → aligned depth topic (raw, ignores `useCompressed`)
2. Else if `useDepthCamera` → depth rect topic (raw, ignores `useCompressed`)
3. Else → `imageTopic` value, optionally with `/compressed` appended

### Display

| Property | Default | Description |
|----------|---------|-------------|
| `targetRenderer` | — | The `Renderer` whose `mainTexture` is updated each frame (required) |
| `flipVertically` | true | Mirrors the image top-to-bottom. Raw ROS images are stored top-row-first; Unity textures expect bottom-row-first |

### Depth Visualisation

| Property | Default | Description |
|----------|---------|-------------|
| `maxDepthMM` | 5000 mm | Depth values at or beyond this distance map to white (255). Values closer map proportionally darker |

### Performance

| Property | Default | Description |
|----------|---------|-------------|
| `showFPS` | true | Draws a HUD label with the current mode, FPS, and texture dimensions |

---

## Subscription Modes

### Colour Raw (`rgb8` / `bgr8`)

Subscribes as `ImageMsg`. On receipt:
1. Creates (or resizes) a `Texture2D` with `TextureFormat.RGB24`.
2. If `bgr8`, swaps R and B channels in the raw byte array in-place on the CPU.
3. Optionally flips vertically using a row-swap algorithm.
4. Uploads via `LoadRawTextureData` and calls `Apply(false)`.

### Colour Compressed (JPEG / PNG)

Subscribes as `CompressedImageMsg`. On receipt:
- Calls `Texture2D.LoadImage(msg.data)` — Unity's built-in JPEG/PNG decompressor.
- No vertical flip is applied because Unity's `LoadImage` already orients the image correctly.

### Depth Raw (`16UC1` / `mono16`)

Subscribes as `ImageMsg`. On receipt:
1. Creates a `Texture2D` with `TextureFormat.RGBA32`.
2. For each pixel, unpacks two bytes into a `ushort` depth value in millimetres:
   ```csharp
   ushort depthMM = (ushort)(data[idx] | (data[idx + 1] << 8));
   ```
3. Normalises to greyscale:
   ```csharp
   byte gray = (byte)Mathf.Clamp((depthMM / maxDepthMM) * 255, 0, 255);
   ```
   - 0 mm (no data) → black
   - `maxDepthMM` or beyond → white
4. Optionally flips vertically.
5. Uploads via `SetPixels32`.

---

## Key Methods

### `UpdateRawImage(ImageMsg msg)`
Main handler for raw (uncompressed) image messages. Dispatches to `ProcessDepthImage` or `ProcessColorImage` based on `msg.encoding`, then assigns the result to `targetRenderer.material.mainTexture`.

### `UpdateCompressedImage(CompressedImageMsg msg)`
Handler for compressed image messages. Uses Unity's native `LoadImage` for decompression — supports JPEG and PNG automatically.

### `ProcessDepthImage(byte[] data, int width, int height)`
Converts 16-bit little-endian depth bytes to a `Color32` greyscale array using `maxDepthMM` as the normalisation range.

### `ProcessColorImage(byte[] data, int width, int height, bool isBGR)`
Optionally swaps BGR→RGB in-place, then loads raw bytes into the texture. Note: this modifies the `data` array in-place; be aware if the same array is referenced elsewhere.

### `FlipVertical(Color32[] pixels, int width, int height)`
Swaps rows around the horizontal midline using a double-pointer walk.

### `FlipVerticalRaw(byte[] data, int width, int height, int bytesPerPixel)`
Row-swap on raw byte data using a single temporary row buffer, without any additional heap allocation beyond the initial `tempRow`.

### `GetTexture()`
Public accessor returning the current `Texture2D`. Useful if another script needs to sample the displayed image.

---

## HUD Display (`OnGUI`)

When `showFPS` is enabled and a texture has been received, a label is drawn at the top-left of the screen:

```
Color (Raw) | FPS: 28.3 | Size: 640x480
Aligned Depth | FPS: 8.5 | Size: 640x480
```

The mode string reflects which subscription mode is active.

---

## Setup

1. Add a **Quad** (or any GameObject with a `Renderer`) to your scene.
2. Attach `SimpleImageSubscriber` to it (or to a separate manager GameObject).
3. Assign the quad's `Renderer` component to the **Target Renderer** field.
4. Select the desired mode (`useDepthCamera`, `useAlignedDepth`, or leave both off for colour).
5. Click Play — the texture will appear on the renderer as soon as the first message arrives.

---

## Performance Notes

`SimpleImageSubscriber` performs all image processing on the **CPU** and allocates `Color32` arrays per-frame when processing depth images. This is acceptable for a single diagnostic feed but is not suitable for high-frequency point cloud data. For that use case, see `ROSPointCloudRenderer` which does the conversion on the GPU with no per-frame heap allocation.

| Feed type | Processing | Per-frame allocation |
|-----------|-----------|----------------------|
| Colour raw | CPU byte-swap + optional flip | None (raw bytes loaded directly) |
| Colour compressed | Unity native decompress | Internal to `LoadImage` |
| Depth 16-bit | CPU normalisation loop | `Color32[width × height]` |
