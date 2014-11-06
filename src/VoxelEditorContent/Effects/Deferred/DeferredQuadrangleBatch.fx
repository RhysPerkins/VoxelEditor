//=============================================
//---[Includes]--------------------------------
//=============================================

#include "..\include\Atlas.fxh"
#include "..\include\Constants.fxh"

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

//texture GlowMap;
float4 HalfPixelTexel;							// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]
//texture ReflectionMap;
//texture SpecularMap;
float Time;										// Scroll Time
float4x4 ViewProjection;
float4 VoxelGrid;

//=============================================
//---[HLSL Variables]--------------------------
//=============================================

// Matrices can be accessed in HLSL as follows:

//	The zero-based row-column position: 
//	_m00, _m01, _m02, _m03
//	_m10, _m11, _m12, _m13
//	_m20, _m21, _m22, _m23
//	_m30, _m31, _m32, _m33

//	The one-based row-column position: 
//	_11, _12, _13, _14
//	_21, _22, _23, _24
//	_31, _32, _33, _34
//	_41, _42, _43, _44

//	[0][0], [0][1], [0][2], [0][3]
//	[1][0], [1][1], [1][2], [1][3]
//	[2][0], [2][1], [2][2], [2][3]
//	[3][0], [3][1], [3][2], [3][3]

//=============================================
//---[Texture Samplers]------------------------
//=============================================

sampler TextureSampler = sampler_state   
{   
    Texture = <AtlasTexture>;   
       
    MinFilter = Point;   
    MagFilter = Point;   
    MipFilter = Point;   
       
    AddressU = Clamp;  
    AddressV = Clamp;
};

//sampler SpecularSampler = sampler_state
//{
//	Texture = <SpecularMap>;	
//	AddressU = Wrap;
//	AddressV = Wrap;
//	MipFilter = LINEAR; 
//	MinFilter = LINEAR; 
//	MagFilter = LINEAR; 
//};

//sampler GlowSampler = sampler_state
//{
//	Texture = <GlowMap>;	
//	AddressU = Wrap;
//	AddressV = Wrap;
//	MipFilter = LINEAR; 
//	MinFilter = LINEAR; 
//	MagFilter = LINEAR; 
//};

//sampler ReflectionSampler = sampler_state
//{
//	Texture = <ReflectionMap>;	
//	AddressU = Wrap;
//	AddressV = Wrap;
//	MipFilter = LINEAR; 
//	MinFilter = LINEAR; 
//	MagFilter = LINEAR; 
//};

//=============================================
//---[Structs]---------------------------------
//=============================================

struct VertexShaderInput
{
    float4 Position					: POSITION0;		// Local Vertex Position
	float4 Normal					: NORMAL0;			// [x, y, z] Normal [w] Alpha
    float4 TexCoordDimension		: TEXCOORD0;		// [x, y] Texture Coordinates [z, w] Width & Height
    float4 Column1					: TEXCOORD1;		// M11...M14
    float4 Column2					: TEXCOORD2;		// M21...M24
    float4 Column3					: TEXCOORD3;		// M31...M34
    float4 Column4					: TEXCOORD4;		// M41...M44
    float4 SourceRectangle0			: TEXCOORD5;		// Source Rectangle (Texture Layer 0)
	float4 SourceRectangle1			: TEXCOORD6;		// Source Rectangle (Texture Layer 1)
	float4 ScrollVector				: TEXCOORD7;		// [x, y] Layer 0 Scroll Direction * Speed [z, w] Layer 1 Scroll Direction * Speed
	float4 NumTiles					: COLOR0;			// [x, y] Layer 0 uv tile [z, w] Layer 1 uv tile
	float4 WarpParameters0			: COLOR1;
	float4 WarpParameters1			: COLOR2;
	float4 VoxelGridFogEnable		: COLOR3;
};

struct VertexShaderOutput
{
    float4 Position					: POSITION0;
	float3 Normal					: NORMAL0;	
	//float3x3 Tangent				: TANGENT0;			// For bump mapping
    float4 TextureCoordinates		: TEXCOORD0;
	float4 SourceRectangle0			: TEXCOORD1;
	float4 SourceRectangle1			: TEXCOORD2;
	float4 ScrollVector				: TEXCOORD3;
	float4 NumTiles					: TEXCOORD4;
	float4 WarpParameters0			: TEXCOORD5;
	float4 WarpParameters1			: TEXCOORD6;
	float4 PositionClone			: TEXCOORD7;		// Final position values must be cloned to be used in PS calculations
	float4 VoxelGridFogEnable		: TEXCOORD8;
};

struct PixelShaderOutput
{
	float4 Color					: COLOR0;			// Color
	float4 SGR						: COLOR1;			// [R] Specular, [G] Glow and [B] Reflection values
	float4 Tangent					: COLOR2;			// Normal / Tangent map
	float4 Depth					: COLOR3;			// Depth map 
};

//=============================================
//---[Functions]-------------------------------
//=============================================

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutput DeferredVertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput Output = (VertexShaderOutput)0;

	// Set world transform here (don't use a static identity matrix as it is slower)
	float4x4 World =
	{
		// [Rotation, Scale]
		input.Column1.x * input.TexCoordDimension.z,		// * Scale X
		input.Column1.y * input.TexCoordDimension.z,
		input.Column1.z * input.TexCoordDimension.z,
		input.Column1.w,
		input.Column2.x * input.TexCoordDimension.w,		// * Scale Y
		input.Column2.y * input.TexCoordDimension.w,
		input.Column2.z * input.TexCoordDimension.w,
		input.Column2.w,
		input.Column3.x,
		input.Column3.y,
		input.Column3.z,
		input.Column3.w,
	
		// [Translation]
		input.Column4.x,
		input.Column4.y,
		input.Column4.z,
		1													// 'input.Position.w' should always equal 1
	};

	//float3 normal = float3(input.Column3.x, input.Column3.y, input.Column3.z);	// Z
	//float3 tangent0 = float3(input.Column1.x, input.Column1.y, input.Column1.z);	// X
	//float3 tangent1 = float3(input.Column2.x, input.Column2.y, input.Column2.z);	// Y

	// [Transformation]
	// • Multiplying input.Position by World, then the result by ViewProjection is fast
	// • Concatenating World and ViewProjection matrices then multiplying input.Position by the result is slower
	input.Position = mul(input.Position, World);		
	Output.Position = mul(input.Position, ViewProjection);
	
	// [Texel To Pixel Align] 
	// • Half pixel offset for correct texel centering)
	// • Should be done AFTER transformation
	Output.Position.xy -= HalfPixelTexel.xy;

	// [UV Coordinates]
	Output.TextureCoordinates.xy = input.TexCoordDimension.xy;
  
  	// [Texture Tiling]
	// • x = Number of tiles per row		- Layer0
	// • y = Number of tiles per column		- Layer0
	// • z = Number of tiles per row		- Layer1
	// • w = Number of tiles per column		- Layer1
	// Multiply by 'NumTiles' to tile the texture 'numTiles' times when using frac
	Output.NumTiles = input.NumTiles;

	// [Alpha]
	Output.TextureCoordinates.w = input.Normal.w;

	// [Source Rectangles]
	// • Pass the source rectangles to the pixel shader for processing
	Output.SourceRectangle0 = input.SourceRectangle0;
	Output.SourceRectangle1 = input.SourceRectangle1;

	// [Scroll Vector]
	// • Texture Scrolling and the use of 'frac' should be in the PixelShader
	Output.ScrollVector = input.ScrollVector;

	// [Normal]
	Output.Normal = float3(input.Column3.x, input.Column3.y, input.Column3.z);

	// [World Normal]
	// • Useful for models but not strictly needed for quadrangles
	//Output.Normal = mul(float4(input.Column3.x, input.Column3.y, input.Column3.z, 0), World).xyz;
	
	// [Bump Mapping]
	//Output.Tangent[0] = tangent0;							// X Axis
	//Output.Tangent[1] = tangent1;							// Y Axis
	//Output.Tangent[2] = Output.Normal;					// Z Axis
          
	// [Warp Parameters]
	Output.WarpParameters0 = input.WarpParameters0;
	Output.WarpParameters1 = input.WarpParameters1;
		   
	// [Position Clone]
	// • POSITION0 values can't be used in the pixel shader so they must be passed in via another semantic
	Output.PositionClone = Output.Position;

	// [Voxel Grid, Fog Enable]
	Output.VoxelGridFogEnable = input.VoxelGridFogEnable;
		     
    return Output;
}

//=============================================
//---[Pixel Shaders]---------------------------
//=============================================

PixelShaderOutput PSBasicTexture(VertexShaderOutput input)
{
	PixelShaderOutput output = (PixelShaderOutput)0;
	
	float2 OriginalTextureCoordinates = input.TextureCoordinates;

	float2 texCoords0 = input.TextureCoordinates.xy;
	float2 texCoords1 = input.TextureCoordinates.xy;

	// [Scroll Texture]
	float4 offset = input.ScrollVector * Time.x;
	texCoords0.x -= offset.x;
	texCoords0.y += offset.y;
	texCoords1.x -= offset.z;	
	texCoords1.y += offset.w;

	// [Warp Texture]
	// • Quake style water/teleport texture animation
	// • Produces incorrect edge textures with 'Clamping Method' but works with 'Scaling Method' in 'Quadrangle' class
	float s0 = Time * input.WarpParameters0.z;
	float s1 = Time * input.WarpParameters1.z;
	float f0 = TWOPI * input.WarpParameters0.x;
	float f1 = TWOPI * input.WarpParameters1.x;

	texCoords0.x += sin((texCoords0.y + s0) * f0) * input.WarpParameters0.y;
    texCoords0.y += sin((texCoords0.x + s0) * f0) * input.WarpParameters0.y;
	texCoords1.x += sin((texCoords1.y + s1) * f1) * input.WarpParameters1.y;
    texCoords1.y += sin((texCoords1.x + s1) * f1) * input.WarpParameters1.y;
	
	// Wrap coordinates in [0, 1) range (for both tiling and scrolling)
	// Use 'frac(texCoord) * n' to wrap in [0, n) range
	texCoords0.xy = frac(texCoords0.xy * input.NumTiles.xy);
	texCoords1.xy = frac(texCoords1.xy * input.NumTiles.zw);

	// Adjust uv coordinates so they use the correct texture from the texture atlas  
	texCoords0.xy = CalculateAtlasUV(texCoords0.xy, input.SourceRectangle0);
	texCoords1.xy = CalculateAtlasUV(texCoords1.xy, input.SourceRectangle1);
	float2 voxelGridCoords = CalculateAtlasUV(input.TextureCoordinates.xy, VoxelGrid);

	// Use the original uv coordinates for mip-map calculation
	//float r1 = ddx(OriginalTextureCoordinates.x);
	//float r2 = ddy(OriginalTextureCoordinates.y);

	//float4 colour0 = tex2D(TextureSampler, texCoords0.xy, r1, r2);
	//float4 colour1 = tex2D(TextureSampler, texCoords1.xy, r1, r2);
	//float4 voxelGridColour = tex2D(TextureSampler, voxelGridCoords, r1, r2);

	float4 colour0 = tex2D(TextureSampler, texCoords0);
	float4 colour1 = tex2D(TextureSampler, texCoords1);
	float4 voxelGridColour = tex2D(TextureSampler, voxelGridCoords);

	// [Linear Interpolation]
	// • Based on colour1 alpha (therefore colour1 takes precedence over colour0)
	// • output = lerp(A, B, C);
	// • output = A * (1 - C) + B * C;
	float4 colour = lerp(colour0, colour1, colour1.a);

	// Add a frame to the quadrangle when drawing voxels (depending on voxel grid value)
	colour = lerp(colour, lerp(colour, voxelGridColour, voxelGridColour.a), input.VoxelGridFogEnable.x);

	// Set alpha value based on input (used for fading)
	colour.a *= input.TextureCoordinates.w;

	// [Fog] - Doesn't appear to be working correctly (See: simulateFog2 source code for correct behaviour)
	//float3 fogColour = float3(1, 1, 1);
	//float fogEnabled = 1;
	//float fogFactor = ComputeLinearFogFactor(input.PositionClone.z / input.PositionClone.w, 0.1, 1.0);
	//float fogFactor = ComputeExponentialFogFactor(input.PositionClone.z / input.PositionClone.w, 0.5);
	//float fogFactor = ComputeExponentialSquaredFogFactor(input.PositionClone.z / input.PositionClone.w, 0.5);
	//colour.rgb = lerp(colour.rgb, fogColour, fogFactor * fogEnabled);

	// [Colour]
	output.Color = colour;
	//output.Color = float4(1,1,1,1);
		
	// [Tangent/Normal]
	// • Get value in the range of -1 to 1
	//float3 n = 2.0f * tex2D(BumpMapSampler, input.TextureCoordinates) - 1.0f;
	
	// Multiply by the tangent matrix
	//n = mul(n, input.Tangent);
	
	// Output normal
	output.Tangent.rgb = normalize(input.Normal) * 0.5f + 0.5f;
	output.Tangent.a = 1;
	
	// [Specular, Glow, Reflection] 
	// • Completely optional
	//output.SGR.r = tex2D(SpecularSampler, input.TextureCoordinates);
	//output.SGR.g = tex2D(GlowSampler, input.TextureCoordinates);
	//output.SGR.b = tex2D(ReflectionMap, input.TextureCoordinates);
	//output.SGR.w = 0;
	
	// [Depth]
	output.Depth.r = input.PositionClone.z / input.PositionClone.w;

	// Flip to keep accuracy away from floating point issues
	output.Depth.r = 1.0f - output.Depth.r; 

	output.Depth.a = 1;
	
	return output;
}

//=============================================
//---[Techniques]------------------------------
//=============================================

technique Deferred
{
    pass Pass0
    {		
		VertexShader = compile vs_3_0 DeferredVertexShaderFunction();
        PixelShader = compile ps_3_0 PSBasicTexture();
    }
}