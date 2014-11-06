//=============================================
//---[Includes]--------------------------------
//=============================================

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

//texture sgrMap;
texture colorMap;
float4 HalfPixelTexel;							// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]
texture lightMap;
texture depthMap;
float3 AmbientLight = float3(0.5, 0.5, 0.5);	//float3(0.1, 0.1, 0.1);

//=============================================
//---[Texture Samplers]------------------------
//=============================================	

sampler colorSampler = sampler_state
{
    Texture = (colorMap);
    AddressU = CLAMP;
    AddressV = CLAMP;
    MagFilter = POINT;
    MinFilter = POINT;
    Mipfilter = POINT;
};

sampler lightSampler = sampler_state
{
    Texture = (lightMap);
    AddressU = CLAMP;
    AddressV = CLAMP;
    MagFilter = LINEAR;
    MinFilter = LINEAR;
    Mipfilter = LINEAR;
};

//sampler SGRSampler = sampler_state
//{
//    Texture = (sgrMap);
//    AddressU = CLAMP;
//    AddressV = CLAMP;
//    MagFilter = LINEAR;
//    MinFilter = LINEAR;
//    Mipfilter = LINEAR;
//};

sampler DepthSampler = sampler_state
{
    Texture = (depthMap);
    AddressU = CLAMP;
    AddressV = CLAMP;
    MagFilter = POINT;
    MinFilter = POINT;
    Mipfilter = POINT;
};

//=============================================
//---[Structs]---------------------------------
//=============================================

struct VertexShaderInput
{
    float4 Position						: POSITION0;
    float2 TextureCoordinates			: TEXCOORD0;
};

struct VertexShaderOutputToPS
{
	float4 Position						: POSITION0;
    float2 TextureCoordinates			: TEXCOORD0;
};

struct PixelShaderOutput
{
	float4 Colour						: COLOR0;

	// [Forward Rendering]
	// • The DEPTH sematic is required to allow forward rendering of geometry after deferred rendering
	// • The range of values is [0, 1] where 0 is near clipping plane and 1 is the far clipping plane
	// • 'z buffer' and 'z buffer write' render states MUST be enabled for this to work
	float Depth							: DEPTH;
};

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutputToPS VertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutputToPS output = (VertexShaderOutputToPS)0;

	// 'input.Position.w' should always equal 1
	input.Position.w = 1;

    output.Position = input.Position;

	// [Texel To Pixel Align] 
	// • Half pixel offset for correct texel centering)
	// • Should be done AFTER transformation
	output.Position.xy -= HalfPixelTexel.xy;

    output.TextureCoordinates = input.TextureCoordinates;

    return output;
}

//=============================================
//---[Pixel Shaders]---------------------------
//=============================================

PixelShaderOutput RenderScenePS(VertexShaderOutputToPS input)
{
	PixelShaderOutput output = (PixelShaderOutput)0;

	// As alpha blending is enabled for the lights ignore the alpha value when combining and adding an ambient light here
	// This doesn't need to be the case if a separate draw call is used with a separate ambient light class
	float4 colour = tex2D(colorSampler, input.TextureCoordinates);

	output.Colour.rgb = colour.rgb * (AmbientLight.xyz + tex2D(lightSampler, input.TextureCoordinates).rgb);
	output.Colour.a = colour.a;

	// [Depth]
	output.Depth = tex2D(DepthSampler, input.TextureCoordinates).r;

	// Flip to keep accuracy away from floating point issues
	output.Depth = 1.0 - output.Depth;

	return output;
}

//=============================================
//---[Techniques]------------------------------
//=============================================

technique RenderScene
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 VertexShaderFunction();
		PixelShader = compile ps_3_0 RenderScenePS();
	}
}