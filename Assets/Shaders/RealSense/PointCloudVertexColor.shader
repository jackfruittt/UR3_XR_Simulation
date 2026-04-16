// Point Cloud Shader - vertex-only quad expansion. No geometry shader stage.
//
// Each depth pixel is pre-expanded into 4 vertices / 2 triangles in the mesh index buffer
// (MeshTopology.Triangles). The vertex shader computes the quad corner from SV_VertexID % 4.
//
// Equation:
//   corner offsets (id%4): 0=TL(-hs,+hs)  1=BL(-hs,-hs)  2=TR(+hs,+hs)  3=BR(+hs,-hs)
//   hs = _QuadScale * z
// Each corner is projected individually so perspective is exact.
//
// Default _QuadScale = 0.003:
//   D455 at 640x480 has angular pixel spacing ~= 86deg / 640 ~= 0.134deg.
//   Physical gap at depth z ~= z * tan(0.134deg) ~= z * 0.00234 m.
//   0.003 adds ~30% overlap so there are no visible gaps between quads at any depth.

Shader "ROS/PointCloudVertexColor"
{
    Properties
    {
        _QuadScale("Quad Scale (half-size per metre of depth)", Float) = 0.003
        _Color("Tint Color", Color) = (1, 1, 1, 1)
    }

    SubShader
    {
        // Transparent queue: URP skips DepthOnly and DepthNormals prepasses for this object.
        Tags { "RenderType"="Transparent" "Queue"="Transparent" }
        Cull Off
        ZWrite Off
        LOD 100

        Pass
        {
            Tags { "LightMode"="UniversalForward" }
            HLSLPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            // float4 stride (16 bytes) - avoids Vulkan SPIR-V std430 vec3 alignment issue.
            // w component: 1.0 = valid point, 0.0 = invalid (no depth / out of range).
            // Indexed as _VertexBuffer[SV_VertexID / 4] since 4 mesh verts share one point.
            StructuredBuffer<float4> _VertexBuffer;
            StructuredBuffer<float4> _ColorBuffer;

            float  _QuadScale;
            float4 _Color;

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float4 color : COLOR;
            };

            v2f vert(uint id : SV_VertexID)
            {
                // Each point occupies 4 consecutive vertex IDs (one per quad corner).
                uint pointIdx  = id >> 2;         // id / 4
                uint cornerIdx = id &  3u;        // id % 4

                float4 data  = _VertexBuffer[pointIdx];
                float4 col   = _ColorBuffer[pointIdx];

                v2f o;
                o.color = col * _Color;

                // Invalid point: push off-screen without a branch on the clip path.
                // All four corners land outside the frustum so the triangles are culled.
                if (data.w < 0.5)
                {
                    o.pos = float4(2.0, 2.0, 2.0, 1.0);
                    return o;
                }

                float3 wp = data.xyz;
                float  hs = _QuadScale * wp.z;

                // Corner offsets: cornerIdx encodes X and Y sign bits.
                //   0 = TL: (-hs, +hs)    1 = BL: (-hs, -hs)
                //   2 = TR: (+hs, +hs)    3 = BR: (+hs, -hs)
                float sx = (cornerIdx & 2u) != 0u ? +hs : -hs;
                float sy = (cornerIdx & 1u) != 0u ? -hs : +hs;

                o.pos = UnityObjectToClipPos(float4(wp.x + sx, wp.y + sy, wp.z, 1.0));
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color;
            }

            ENDHLSL
        }
    }

    // No fallback: this shader targets a specific URP setup. A fallback would introduce
    // additional passes (ShadowCaster, etc.) that conflict with MeshTopology.Triangles
    // quad-expansion layout.
    Fallback Off
}
