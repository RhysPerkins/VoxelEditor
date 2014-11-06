//=============================================
//---[Notes]-----------------------------------
//=============================================

// - This effect should be used with CustomBlendState.Multiply
//   AlphaBlendFunction = BlendFunction.Add,
//   AlphaDestinationBlend = Blend.SourceAlpha,
//   AlphaSourceBlend = Blend.Zero,
//   ColorBlendFunction = BlendFunction.Add,
//   ColorDestinationBlend = Blend.SourceColor,
//   ColorSourceBlend = Blend.Zero,

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

float4 HalfPixelTexel;							// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]

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

struct PixelShaderOutput0
{
	float4 Colour0						: COLOR0;
	float4 Colour1						: COLOR1;
	float4 Colour2						: COLOR2;
};

struct PixelShaderOutput1
{
	float4 Colour0						: COLOR0;
	float4 Colour1						: COLOR1;
	float4 Colour2						: COLOR2;
};

struct PixelShaderOutput2
{
	float4 Colour0						: COLOR0;
	float4 Colour1						: COLOR1;
	float4 Colour2						: COLOR2;
};

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

PixelShaderOutput0 PixelShaderFunction0(VertexShaderOutput input)
{
	PixelShaderOutput0 output;

	output.Colour0 = float4(0, 0, 0, 0);
	output.Colour1 = float4(1, 1, 1, 1);
	output.Colour2 = float4(1, 1, 1, 1);

    return output;
}

PixelShaderOutput1 PixelShaderFunction1(VertexShaderOutput input)
{
	PixelShaderOutput1 output;

	output.Colour0 = float4(1, 1, 1, 1);
	output.Colour1 = float4(0, 0, 0, 0);
	output.Colour2 = float4(1, 1, 1, 1);

    return output;
}

PixelShaderOutput2 PixelShaderFunction2(VertexShaderOutput input)
{
	PixelShaderOutput2 output;

	output.Colour0 = float4(1, 1, 1, 1);
	output.Colour1 = float4(1, 1, 1, 1);
	output.Colour2 = float4(0, 0, 0, 0);

    return output;
}

//=============================================
//---[Techniques]------------------------------
//============================================= 

technique ClearRenderTarget0
{
    pass Pass0
    {				  
        VertexShader = compile vs_3_0 VertexShaderFunction();
		PixelShader = compile ps_3_0 PixelShaderFunction0();
    }
}

technique ClearRenderTarget1
{
    pass Pass0
    {				  
        VertexShader = compile vs_3_0 VertexShaderFunction();
		PixelShader = compile ps_3_0 PixelShaderFunction1();
    }
}

technique ClearRenderTarget2
{
    pass Pass0
    {				  
        VertexShader = compile vs_3_0 VertexShaderFunction();
		PixelShader = compile ps_3_0 PixelShaderFunction2();
    }
}