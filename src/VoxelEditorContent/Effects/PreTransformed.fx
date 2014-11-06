//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

float4x4 ViewProjection;

//=============================================
//---[Structs]---------------------------------
//=============================================

struct VertexShaderInput
{
    float4 Position						: POSITION0; 
    float4 Colour						: COLOR0;   
};

struct VertexShaderOutput
{
    float4 Position						: POSITION0;
    float4 Colour						: COLOR0;   
};

struct PixelShaderOutput
{
	float4 Colour						: COLOR0;
};

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output;

    output.Position = mul(input.Position, ViewProjection);   
	output.Colour = input.Colour;

    return output;
}

//=============================================
//---[Pixel Shaders]---------------------------
//=============================================

PixelShaderOutput PixelShaderFunction(VertexShaderOutput input)
{
	PixelShaderOutput output;
	
	output.Colour = input.Colour;
	
    return output;
}

//=============================================
//---[Techniques]------------------------------
//============================================= 

technique Default
{
    pass Pass0
    {		  
		//ZEnable = true;					// Setting to false will cause problems with WireShapeDrawer

        VertexShader = compile vs_2_0 VertexShaderFunction();
        PixelShader = compile ps_2_0 PixelShaderFunction();
    }
}