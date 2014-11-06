//=============================================
//---[Notes]-----------------------------------
//=============================================

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

texture DepthTexture;
float4 HalfPixelTexel;							// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]

//=============================================
//---[Texture Samplers]------------------------
//=============================================

sampler DepthSampler = sampler_state   
{   
    Texture = <DepthTexture>;   
       
    MinFilter = Point;   
    MagFilter = Point;   
    MipFilter = Point;
       
    AddressU = Clamp;  
    AddressV = Clamp;
};

//=============================================
//---[Structs]---------------------------------
//=============================================

struct VertexShaderInput
{
    float4 Position						: POSITION0;
	float2 TextureCoordinates			: TEXCOORD0;
};

struct VertexShaderOutput
{
    float4 Position						: POSITION0;
	float2 TextureCoordinates			: TEXCOORD0;
};

struct PixelShaderOutput
{
	float4 Colour0						: COLOR0;
	float4 Colour1						: COLOR1;
	float4 Colour2						: COLOR2;
	float Depth							: DEPTH;
};

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output;

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

PixelShaderOutput PixelShaderFunction(VertexShaderOutput input)
{
	PixelShaderOutput output;

	// Clear up to 3 render targets
	output.Colour0 = float4(0, 0, 0, 0);
	output.Colour1 = float4(0, 0, 0, 0);
	output.Colour2 = float4(0, 0, 0, 0);

	output.Depth = 1.0f - tex2D(DepthSampler, input.TextureCoordinates).r;

    return output;
}

//=============================================
//---[Techniques]------------------------------
//============================================= 

technique Default
{
    pass Pass0
    {				  
        VertexShader = compile vs_3_0 VertexShaderFunction();
		PixelShader = compile ps_3_0 PixelShaderFunction();
    }
}