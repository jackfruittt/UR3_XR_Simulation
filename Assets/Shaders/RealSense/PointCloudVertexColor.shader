// Simple Point Cloud Shader with Vertex Colors
// Simplified version for ROS point cloud rendering
// Based on Intel's PointCloud.shader but uses vertex colors directly

Shader "ROS/PointCloudVertexColor"
{
    Properties
    {
        _PointSize("Point Size", Float) = 0.005
        _Color("Tint Color", Color) = (1, 1, 1, 1)
    }
    
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100
        
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            struct appdata
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
            };
            
            struct v2f
            {
                float4 vertex : SV_POSITION;
                float4 color : COLOR;
                float size : PSIZE;
            };
            
            float _PointSize;
            float4 _Color;
            
            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.color = v.color * _Color;
                o.size = _PointSize;
                return o;
            }
            
            fixed4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
    
    Fallback "Diffuse"
}
