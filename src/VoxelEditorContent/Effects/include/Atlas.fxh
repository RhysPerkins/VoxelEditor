float2 AtlasDim;								// float2(1 / Width, 1 / Height)
texture AtlasTexture;

// Adjust uv coordinates so they use the correct texture from the texture atlas
float2 CalculateAtlasUV(float2 UV, float4 SourceRectangle)
{
	UV *= SourceRectangle.zw * AtlasDim;		// Scale uv
	UV += SourceRectangle.xy * AtlasDim;		// Shift uv (rectangle offset from left of atlas)

	return UV;
}