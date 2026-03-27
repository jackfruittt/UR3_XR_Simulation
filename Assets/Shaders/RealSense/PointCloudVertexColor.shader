// Point Cloud Shader - reads vertex positions and colours from compute shader StructuredBuffers.
// Uses HLSLPROGRAM (not CGPROGRAM) for reliable StructuredBuffer support on Vulkan/Linux.

Shader "ROS/PointCloudVertexColor"
{
    Properties
    {
        _PointSize("Point Size (pixels)", Float) = 3.0
        _Color("Tint Color", Color) = (1, 1, 1, 1)
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            // Compute shader output buffers — bound each frame by ROSPointCloudRenderer.UpdatePointCloud()
            StructuredBuffer<float3> _VertexBuffer;
            StructuredBuffer<float4> _ColorBuffer;

            float  _PointSize;
            float4 _Color;

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float4 color : COLOR;
                float  size  : PSIZE;
            };

            v2f vert(uint id : SV_VertexID)
            {
                v2f o;
                float3 worldPos = _VertexBuffer[id];

                // Discard invalid points (written as z=999 by the compute shader)
                if (worldPos.z > 900.0)
                {
                    // Move off-screen by placing behind camera
                    o.pos   = float4(0, 0, -1, 1);
                    o.color = float4(0, 0, 0, 0);
                    o.size  = 0;
                    return o;
                }

                o.pos   = UnityObjectToClipPos(float4(worldPos, 1.0));
                o.color = _ColorBuffer[id] * _Color;
                o.size  = _PointSize;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDHLSL
        }
    }

    Fallback "Diffuse"
}
