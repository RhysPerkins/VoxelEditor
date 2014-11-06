//=============================================
//---[Includes]--------------------------------
//=============================================

#include "..\include\Constants.fxh"

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

float3 Bias = float3(0.00001f, -0.0001f, 0.04f);	// [x = Shadow Bias, y = VSM Epsilon, z = Light Bleed Correction]
float3 CameraPosition;
float4x4 CameraViewProjectionInverse;				// Inverse of the camera's ViewProjection matrix
texture DepthMap;
float4 HalfPixelTexel;								// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]
float4 LightDiffuse;								// [xyz = Diffuse Colour, w = Diffuse Intensity]
float3 LightPosition;
float4x4 LightProjection;
float4 LightSpecular;								// [xyz = Specular Colour, w = Specular Intensity]
float4x4 LightView;
float4x4 LightViewInverse;							// Inverse of the light's view matrix
texture NormalMap;									// Normals (with specular power in the alpha channel)
texture ShadowMap;

//=============================================
//---[Texture Samplers]------------------------
//=============================================	

sampler NormalSampler = sampler_state
{
    Texture = (NormalMap);
    AddressU = CLAMP;
    AddressV = CLAMP;
    MagFilter = POINT;
    MinFilter = POINT;
    Mipfilter = POINT;
};

sampler DepthSampler = sampler_state
{
    Texture = (DepthMap);
    AddressU = CLAMP;
    AddressV = CLAMP;
    MagFilter = POINT;
    MinFilter = POINT;
    Mipfilter = POINT;
};

sampler ShadowSampler = sampler_state
{
    Texture = (ShadowMap);
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
};

struct VertexShaderOutputToPS
{
    float4 Position						: POSITION0;
	float4 PositionClone				: TEXCOORD0;
};

struct PixelShaderOutput
{
	float4 Color						: COLOR0;	// Color
	float4 SGR							: COLOR1;	// [R] Specular, [G] Glow and [B] Reflection values
	float4 Tangent						: COLOR2;	// Normal / Tangent map
	float4 Depth						: COLOR3;	// Depth map 
};

//=============================================
//---[Functions]-------------------------------
//=============================================

float2 WorldToScreenSpace(float4 position)
{
    float2 screenPosition = position.xy /= position.w;
    screenPosition = 0.5f * (float2(screenPosition.x, -screenPosition.y) + 1);	
	
	// Offset by the texel size
	screenPosition += HalfPixelTexel.zw;
	
	return screenPosition;
}

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutputToPS DirectionalLightVS(VertexShaderInput input)
{
    VertexShaderOutputToPS output = (VertexShaderOutputToPS)0;
    
	// 'input.Position.w' should always equal 1
	input.Position.w = 1;

	output.Position = input.Position;
	
	// [Texel To Pixel Align] 
	// • Half pixel offset for correct texel centering)
	// • Should be done AFTER transformation
	output.Position.xy -= HalfPixelTexel.xy;

	output.PositionClone = output.Position;

    return output;
}

//=============================================
//---[Pixel Shaders]---------------------------
//=============================================

float4 DirectionalLightNoShadowPS(VertexShaderOutputToPS input) : COLOR0
{
    // Obtain screen position
    input.PositionClone.xy /= input.PositionClone.w;

    float2 texCoord = 0.5f * (float2(input.PositionClone.x, -input.PositionClone.y) + 1);
	texCoord += HalfPixelTexel.zw;

	// [Normal]
    float3 normalData = tex2D(NormalSampler, texCoord);

	// Transform normal from [0, 1] texture coordinate range back into [-1, 1] range
    float3 N = normalData.xyz * 2.0f - 1.0f;
	N = normalize(N);
    
	float4 position;

	// [Screen Position]
	position.xy = input.PositionClone.xy;
	position.z = 1.0f - tex2D(DepthSampler, texCoord).r;
	position.w = 1.0f;

	// [World Position]
	position = mul(position, CameraViewProjectionInverse);
	position /= position.w;
	
	// ===============================================
	// --[Lighting Calculations]----------------------
	// ===============================================

    // Unnormalized light vector
    float3 dir_to_light = LightPosition - position;	// Direction from vertex
    //float dist_to_light = length(dir_to_light);

	// Normalise 'dir_to_light' vector for lighting calculations
	dir_to_light = normalize(dir_to_light);

	/* ===============================================
	 * --[Diffuse Light]------------------------------
	 * ===============================================
	 * I = Di * Dc * N.L										[float3]    DiffuseLight.xyz
	 *
	 * Where:
	 *
	 * I = Intensity Of Light
	 * Di = Diffuse Intensity									[float]
	 * Dc = Diffuse Colour										[float3]
	 * N = Surface Normal										[float3]
	 * L = Direction From Pixel To Light						[float3]
	 *
	 */

	float Di = LightDiffuse.w;
	float3 Dc = LightDiffuse.xyz;
	float NdotL = max(0, dot(N, dir_to_light));
	float3 DiffuseLight = Di * Dc * NdotL;

	/* ===============================================
	 * --[Specular Light]-----------------------------
	 * ===============================================
	 * I = Si * Sc * (R.V)^n									[float3]    SpecularLight.xyz
	 * 
	 * Where:
	 *
	 * I = Intensity Of Light
	 * Si = Specular Intensity									[float]		Range: [0, 1]
	 * Sc = Specular Colour										[float3]
	 * R = Reflection Vector = 2 * (N.L) * N - L				[float3]	Reflection vector of light ray
	 *	 = reflect(L, N)													HLSL Function
	 *	 = L - 2 * N * dot(N, L) 											
	 * V = Camera.WorldPosition	- Pixel.WorldPosition   		[float3]	Vertex To Eye vector
	 * n = Shininess factor										[float]		Range: [0, 255]
	 *
	 */	

	float Si = 1;	//LightSpecular.w;
	float3 Sc = LightSpecular.xyz;
	float3 R = 2.0f * N * dot(N, dir_to_light) - dir_to_light;
	float3 V = normalize(CameraPosition - position);
	float n = 255;

	//float4 sgr = tex2D(SGRSampler, input.TextureCoordinates);
	//float3 SpecularLight = Si * Sc * pow(max(0, saturate(dot(R, V))), n) * sgr.r;

	float3 SpecularLight = Si * Sc * pow(max(0, saturate(dot(R, V))), n);
    
    /* ===============================================
	 * --[Final Colour]-------------------------------
	 * ===============================================
	 */	

	float4 colour = float4(DiffuseLight + SpecularLight, 1);
	//colour.xyz *= atten_amount;

	return colour;
}

float4 DirectionalLightShadowPS(VertexShaderOutputToPS input) : COLOR0
{
    // Obtain screen position
    input.PositionClone.xy /= input.PositionClone.w;

    float2 texCoord = 0.5f * (float2(input.PositionClone.x, -input.PositionClone.y) + 1);
	texCoord += HalfPixelTexel.zw;

	// [Normal]
    float3 normalData = tex2D(NormalSampler, texCoord);

	// Transform normal from [0, 1] texture coordinate range back into [-1, 1] range
    float3 N = normalData.xyz * 2.0f - 1.0f;
	N = normalize(N);
    
	float4 position;

	// [Screen Position]
	position.xy = input.PositionClone.xy;
	position.z = 1.0f - tex2D(DepthSampler, texCoord).r;
	position.w = 1.0f;

	// [World Position]
	position = mul(position, CameraViewProjectionInverse);
	position /= position.w;
	
	// ===============================================
	// --[Shadow Calculations]------------------------
	// ===============================================

	// Shadow term (1 = no shadow)
	float shadow = 1;

	// Find screen position as seen by the light
	float4x4 ViewProjection = mul(LightView, LightProjection);
	float4 lightScreenPos = mul(position, ViewProjection);

	// Transform from light space to shadow map texture space
	float2 texShadow = WorldToScreenSpace(lightScreenPos);

	float depth = tex2D(ShadowSampler, texShadow).r;

	float lightDepth = lightScreenPos.z / lightScreenPos.w;

	// Works but has 'Peter Panning' problems
	shadow = depth + Bias.x < lightDepth ? 0.0f : 1.0f;

	// ===============================================
	// --[Lighting Calculations]----------------------
	// ===============================================

	// Unnormalized light vector
    float3 dir_to_light = LightPosition - position;	// Direction from vertex
    //float dist_to_light = length(dir_to_light);

	// Normalise 'dir_to_light' vector for lighting calculations
	dir_to_light = normalize(dir_to_light);

	/* ===============================================
	 * --[Diffuse Light]------------------------------
	 * ===============================================
	 * I = Di * Dc * N.L										[float3]    DiffuseLight.xyz
	 *
	 * Where:
	 *
	 * I = Intensity Of Light
	 * Di = Diffuse Intensity									[float]
	 * Dc = Diffuse Colour										[float3]
	 * N = Surface Normal										[float3]
	 * L = Direction From Pixel To Light						[float3]
	 *
	 */

	float Di = LightDiffuse.w;
	float3 Dc = LightDiffuse.xyz;
	float NdotL = max(0, dot(N, dir_to_light));
	float3 DiffuseLight = Di * Dc * NdotL;

	/* ===============================================
	 * --[Specular Light]-----------------------------
	 * ===============================================
	 * I = Si * Sc * (R.V)^n									[float3]    SpecularLight.xyz
	 * 
	 * Where:
	 *
	 * I = Intensity Of Light
	 * Si = Specular Intensity									[float]		Range: [0, 1]
	 * Sc = Specular Colour										[float3]
	 * R = Reflection Vector = 2 * (N.L) * N - L				[float3]	Reflection vector of light ray
	 *	 = reflect(L, N)													HLSL Function
	 *	 = L - 2 * N * dot(N, L) 											
	 * V = Camera.WorldPosition	- Pixel.WorldPosition   		[float3]	Vertex To Eye vector
	 * n = Shininess factor										[float]		Range: [0, 255]
	 *
	 */	

	float Si = 1;	//LightSpecular.w;
	float3 Sc = LightSpecular.xyz;
	float3 R = 2.0f * N * dot(N, dir_to_light) - dir_to_light;
	float3 V = normalize(CameraPosition - position);
	float n = 255;

	//float4 sgr = tex2D(SGRSampler, input.TextureCoordinates);
	//float3 SpecularLight = Si * Sc * pow(max(0, saturate(dot(R, V))), n) * sgr.r;

	float3 SpecularLight = Si * Sc * pow(max(0, saturate(dot(R, V))), n);
    
    /* ===============================================
	 * --[Final Colour]-------------------------------
	 * ===============================================
	 */	

	float4 colour = float4(DiffuseLight + SpecularLight, 1);
	//colour.xyz *= atten_amount;
	colour.xyz *= shadow;

	return colour;
}

//=============================================
//---[Techniques]------------------------------
//=============================================

technique NoShadow
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 DirectionalLightVS();
		PixelShader = compile ps_3_0 DirectionalLightNoShadowPS();
	}	
}

technique Shadow
{
	pass Pass0
	{
		VertexShader = compile vs_3_0 DirectionalLightVS();
		PixelShader = compile ps_3_0 DirectionalLightShadowPS();
	}	
}