//=============================================
//---[Notes]----------------------------------
//=============================================

//=============================================
//---[Includes]--------------------------------
//=============================================

#include "..\include\Constants.fxh"

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

texture Background;
texture Depth;
float4 HalfPixelTexel;							// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]
texture Texture0;
texture Texture1;

//=============================================
//---[Texture Samplers]------------------------
//=============================================

sampler BackgroundSampler = sampler_state   
{   
    Texture = <Background>;   
       
    MinFilter = Point;   
    MagFilter = Point;   
    MipFilter = Point;   
       
    AddressU = Clamp;  
    AddressV = Clamp;
};

sampler DepthSampler = sampler_state   
{   
    Texture = <Depth>;   
       
    MinFilter = Point;   
    MagFilter = Point;   
    MipFilter = Point;   
       
    AddressU = Clamp;  
    AddressV = Clamp;
};

sampler Texture0Sampler = sampler_state   
{   
    Texture = <Texture0>;   
       
    MinFilter = Point;   
    MagFilter = Point;   
    MipFilter = Point;   
       
    AddressU = Clamp;  
    AddressV = Clamp;
};

sampler Texture1Sampler = sampler_state   
{   
    Texture = <Texture1>;   
       
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
	float4 Colour						: COLOR0;
	float Depth							: DEPTH;
};

//=============================================
//---[Functions]-------------------------------
//=============================================

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output;

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

PixelShaderOutput PixelShaderFunction(VertexShaderOutput input)
{
	PixelShaderOutput output;

	output.Depth = 1.0f - tex2D(DepthSampler, input.TextureCoordinates.xy).r;

	// Background colour 'Cbg'
	float4 BackgroundColour = tex2D(BackgroundSampler, input.TextureCoordinates.xy);

	float4 SumColour = tex2D(Texture0Sampler, input.TextureCoordinates.xy);

	// [Depth complexity]
	// • Each time a pixel is written by the transparent object it adds 1 to the value [float4(1, 1, 1, 1)]
	// • This is why a render target with 16-bits per channel is required (SurfaceFormat.Rgba64)
	float n = tex2D(Texture1Sampler, input.TextureCoordinates.xy).r;

	// If no transparent pixel has been drawn at this location, return the background colour
	if (n == 0.0)	
	{
		output.Colour = BackgroundColour;
		return output;
	}

	SumColour = Upscale(SumColour);
	n = Upscale(n);

	// Average colour 'C'
	float3 AvgColour = SumColour.rgb / SumColour.a;

	// Average alpha based on depth complexity 'A'
	float AvgAlpha = SumColour.a / n;

	// T = (1 - A) ^ n
	float T = pow(abs(1.0 - AvgAlpha), n);

	// [Weighted Average Formula]
	// • Cdst = (C * A * (1 - (1 - A)^n) / A) + Cbg * (1 - A)^n
	// • Cdst = (C * A * (1 - T) / A) + Cbg * T
	// • Cdst = C * (1 - T) + Cbg * T;
	output.Colour.rgb = AvgColour * (1 - T); 
	output.Colour.rgb += BackgroundColour.rgb * T;
	output.Colour.a = 1;

    return output;
}

//=============================================
//---[Techniques]------------------------------
//============================================= 

technique TransparencyWeightedAverage
{
    pass Pass0
    {		
		//ZEnable = true;
			  
        VertexShader = compile vs_3_0 VertexShaderFunction();		// Vertex
        PixelShader = compile ps_3_0 PixelShaderFunction();			// Fragment
    }
}