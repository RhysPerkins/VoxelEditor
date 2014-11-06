//=============================================
//---[Notes]-----------------------------------
//=============================================

// [Align UV Coordinates]
// • Keep textures vertically aligned to world y Axis (0, 1, 0) [Quad.Rotation.Up = Vector3.Cross(Vector3.UnitY, normal)]
// • Textures are aligned to vector 'v' where [Rotation.Up = Vector3.Cross(v, normal)]          
//float3 v0 = input.Position.xyz - float3(input.Column4.x, input.Column4.y, input.Column4.z);
    
// • 1.0f / Width
//float widthInverse = 1.0f / input.TexCoordDimension.z;
	
// • 1.0f / Height	
//float heightInverse = 1.0f / input.TexCoordDimension.w;			
    
// • dot(v0, right)
//input.TexCoordDimension.x = dot(v0, normalize(float3(input.Column1.x, input.Column1.y, input.Column1.z))) * widthInverse + 0.5f;

// • dot(v0, up)
//input.TexCoordDimension.y = -dot(v0, normalize(float3(input.Column2.x, input.Column2.y, input.Column2.z))) * heightInverse + 0.5f;

// [MipMap Level]
// • Use original texture coordinates for calculation
// • Requires minimum of vs_3_0
// • mipMapLevel = log2(max(ddx(textureCoord.x) * textureWidth, ddy(textureCoord.y) * texureHeight));
//float mipMapLevel0 = log2(max(ddx(input.OriginalCoordinates.x) * AtlasDim.x, ddy(input.OriginalCoordinates.y) * AtlasDim.y));
//float mipMapLevel0 = log2(max(ddx(input.OriginalCoordinates.x) * input.SourceRectangle0.z, ddy(input.OriginalCoordinates.y) * input.SourceRectangle0.w));

// Use a pixel offset with powers of 2 as each mipMap level increases by a power of 2
//float onePixel = HalfPixel + HalfPixel;
//input.TextureCoordinates0.xy += onePixel * pow(2, mipMapLevel0);
//input.TextureCoordinates1.xy += onePixel * pow(2, mipMapLevel0);

	// ==========================================================================

	// This offset is absolutely correct (256 = Atlas Width, 512 = Atlas Height)
	//uv1 += float2((0.5 / 256) * input.SourceRectangle1.z, (0.5 / 512) * input.SourceRectangle1.w);

	//float2 Dim = AtlasDim;

	//SourceRectangle.x *= Dim.x;
	//SourceRectangle.x += (0.5 * Dim.x);

	//SourceRectangle.y *= Dim.y;
	//SourceRectangle.y += (0.5 * Dim.y);

	//SourceRectangle.z *= Dim.x;
	//SourceRectangle.z -= Dim.x;

	//SourceRectangle.w *= Dim.y;
	//SourceRectangle.w -= Dim.y;

	//UV *= SourceRectangle.zw; 				// Scale uv
	//UV += SourceRectangle.xy;				// Shift uv (rectangle offset from left of atlas)

	// ==========================================================================

	// I have to use a HalfTexel based on the dimension of the texture for the tiling to work correctly
	//float2 HalfTexel0 = 0.5 / (input.SourceRectangle0.zw * input.NumTiles.xy);
	//float2 HalfTexel1 = 0.5 / (input.SourceRectangle1.zw * input.NumTiles.zw);

	// [Clamping Method] - Faster but only works with quadrangles and causes problems with texture peturbations (warping)
	//float2 texCoords0 = max(input.TextureCoordinates.xy, HalfTexel0);
	//texCoords0 = min(texCoords0, 1.0 - HalfTexel0);
	
	//float2 texCoords1 = max(input.TextureCoordinates.xy, HalfTexel1);
	//texCoords1 = min(texCoords1, 1.0 - HalfTexel1);


//=============================================
//---[Includes]--------------------------------
//=============================================

#include "include\Atlas.fxh"
#include "include\Constants.fxh"

//=============================================
//---[XNA to HLSL Variables]-------------------
//=============================================

float4 HalfPixelTexel;							// Texel to pixel align [xy = HalfPixel, zw = HalfTexel]
float Time;
float4x4 ViewProjection;
float4 VoxelGrid;

//=============================================
//---[HLSL Variables]--------------------------
//=============================================

// [Matrices] 
// • Can be accessed in HLSL as follows:

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
    MipFilter = Point;		// None
       
    AddressU = Clamp;  
    AddressV = Clamp;
};

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
	float4 VoxelGridParameters		: COLOR3;
};

struct VertexShaderOutput
{
    float4 Position					: POSITION0;    
    float4 TextureCoordinates		: TEXCOORD0;
	float4 SourceRectangle0			: TEXCOORD1;
	float4 SourceRectangle1			: TEXCOORD2;
	float4 ScrollVector				: TEXCOORD3;
	float4 NumTiles					: TEXCOORD4;
	float4 WarpParameters0			: TEXCOORD5;
	float4 WarpParameters1			: TEXCOORD6;
	float4 VoxelGridParameters		: TEXCOORD7;
	float4 PositionClone			: TEXCOORD8;
};

struct PixelShaderForwardOutput
{
	float4 Colour					: COLOR0;
	float4 Depth					: COLOR1;		// For sceneDepth render target
};

struct PixelShaderTransparentOutput
{
	float4 Colour					: COLOR0;
	float4 DepthComplexity			: COLOR1;
};

//=============================================
//---[Functions]-------------------------------
//=============================================

//=============================================
//---[Vertex Shaders]--------------------------
//=============================================

VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
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
		input.Column4.w
	};

	// 'input.Position.w' should always equal 1
	input.Position.w = 1;
	
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

	// [Position Clone]
	// • POSITION0 values can't be used in the pixel shader so they must be passed in via another semantic
	Output.PositionClone = Output.Position;

	// [Source Rectangles]
	// • Pass the source rectangles to the pixel shader for processing
	Output.SourceRectangle0 = input.SourceRectangle0;
	Output.SourceRectangle1 = input.SourceRectangle1;

	// [Scroll Vector]
	// • Texture Scrolling and the use of 'frac' should be in the PixelShader
	Output.ScrollVector = input.ScrollVector;

	// [Warp Parameters]
	Output.WarpParameters0 = input.WarpParameters0;
	Output.WarpParameters1 = input.WarpParameters1;

	// [Voxel Grid]
	Output.VoxelGridParameters = input.VoxelGridParameters;

    return Output;
}

VertexShaderOutput VertexShaderFunctionPolygonOffset(VertexShaderInput input)
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
		input.Column4.w
	};

	// 'input.Position.w' should always equal 1
	input.Position.w = 1;
	
	// [Transformation]
	// • Multiplying input.Position by World, then the result by ViewProjection is fast
	// • Concatenating World and ViewProjection matrices then multiplying input.Position by the result is slower
	input.Position = mul(input.Position, World);	
	Output.Position = mul(input.Position, ViewProjection);

	// Introduce a small polygon offset in screen space to prevent z-fighting [0, 1]
	// Noticable when transparent triangles are coplanar with opaque triangles
	Output.Position.z -= 0.00001;
	
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

	// [Position Clone]
	// • POSITION0 values can't be used in the pixel shader so they must be passed in via another semantic
	Output.PositionClone = Output.Position;

	// [Source Rectangles]
	// • Pass the source rectangles to the pixel shader for processing
	Output.SourceRectangle0 = input.SourceRectangle0;
	Output.SourceRectangle1 = input.SourceRectangle1;

	// [Scroll Vector]
	// • Texture Scrolling and the use of 'frac' should be in the PixelShader
	Output.ScrollVector = input.ScrollVector;

	// [Warp Parameters]
	Output.WarpParameters0 = input.WarpParameters0;
	Output.WarpParameters1 = input.WarpParameters1;

	// [Voxel Grid]
	Output.VoxelGridParameters = input.VoxelGridParameters;

    return Output;
}

//=============================================
//---[Pixel Shaders]---------------------------
//=============================================

PixelShaderForwardOutput PixelShaderFunction(VertexShaderOutput input)
{
	PixelShaderForwardOutput output = (PixelShaderForwardOutput)0;

	float2 OriginalTextureCoordinates = input.TextureCoordinates;

	// [Scale Texture Coordinates] (Page 12 - Texture Atlas Paper)
    // • To access all texels of a texture of dimensions 'width' by 'height' once and only once, models need to use:
    // • u-coordinates in the range [.5 / width, 1 - (.5 / width)] 
    // • v-coordinates in the range [.5 / height, 1- (.5 / height)]
	// • This prevents bleeding of textures (like wrapping) when using an atlas with frac

	// [Clamping Method]
	// • Faster than scaling but only works with quadrangles and causes problems with texture peturbations (warping)
	
	//float2 HalfTexel0 = 0.5 / (input.SourceRectangle0.zw * input.NumTiles.xy);
	//float2 HalfTexel1 = 0.5 / (input.SourceRectangle1.zw * input.NumTiles.zw);

	//float2 texCoords0 = max(input.TextureCoordinates.xy, HalfTexel0);
	//texCoords0 = min(texCoords0, 1.0 - HalfTexel0);
	
	//float2 texCoords1 = max(input.TextureCoordinates.xy, HalfTexel1);
	//texCoords1 = min(texCoords1, 1.0 - HalfTexel1);

	// [Scaling Method]
	//float2 texCoords0 = input.TextureCoordinates.xy;

	//float tex0Width = max(16, input.SourceRectangle0.z) * input.NumTiles.x;	
	//float tex0Height = max(16, input.SourceRectangle0.w) * input.NumTiles.y;

	//texCoords0 *= float2((tex0Width - 1) / tex0Width, (tex0Height - 1) / tex0Height);
	//texCoords0 += float2(0.5 / tex0Width, 0.5 / tex0Height);

	//float2 texCoords1 = input.TextureCoordinates.xy;

	//float tex1Width = max(16, input.SourceRectangle1.z) * input.NumTiles.z;	
	//float tex1Height = max(16, input.SourceRectangle1.w) * input.NumTiles.w;

	//texCoords1 *= float2((tex1Width - 1) / tex1Width, (tex1Height - 1) / tex1Height);
	//texCoords1 += float2(0.5 / tex1Width, 0.5 / tex1Height);

	// [No Scaling]
	float2 texCoords0 = input.TextureCoordinates.xy;
	float2 texCoords1 = input.TextureCoordinates.xy;

	// [Scroll Texture]
	float4 offset = input.ScrollVector * Time;
	texCoords0.x -= offset.x;
	texCoords0.y += offset.y;
	texCoords1.x -= offset.z;	
	texCoords1.y += offset.w;

	// [Warp Texture]
	// • Quake style water/teleport texture animation
	// • Produces incorrect edge textures with 'Clamping Method' but works with 'Scaling Method' in 'Quadrangle' class 
	//texCoords0.x += sin((texCoords0.y + (WarpParameters0.x * WarpParameters0.y)) * TWOPI * WarpParameters0.z) * WarpParameters0.w;
    //texCoords0.y += sin((texCoords0.x + (WarpParameters0.x * WarpParameters0.y)) * TWOPI * WarpParameters0.z) * WarpParameters0.w;
	//texCoords1.x += sin((texCoords1.y + (WarpParameters1.x * WarpParameters1.y)) * TWOPI * WarpParameters1.z) * WarpParameters1.w;
    //texCoords1.y += sin((texCoords1.x + (WarpParameters1.x * WarpParameters1.y)) * TWOPI * WarpParameters1.z) * WarpParameters1.w;

	//=========================================

	//texCoords0 *= 1.0 - HalfTexel0;
	//texCoords0 += HalfTexel0 * 0.5f;
	//texCoords1 *= 1.0 - HalfTexel1;
	//texCoords1 += HalfTexel1 * 0.5f;

	//float2 scale = float2(255 / 256, 511 / 512);

	//input.TextureCoordinates.xy *= scale;
	//input.TextureCoordinates.x += 0.5 / 256.0;
	//input.TextureCoordinates.y += 0.5 / 512.0;

	//=========================================

	// Wrap coordinates in [0, 1) range (for both tiling and scrolling)
	// Use 'frac(texCoord) * n' to wrap in [0, n) range as only mantissa is used
	texCoords0 = frac(texCoords0.xy * input.NumTiles.xy);
	texCoords1 = frac(texCoords1.xy * input.NumTiles.zw);

	// Adjust uv coordinates so they use the correct texture from the texture atlas  
	texCoords0 = CalculateAtlasUV(texCoords0, input.SourceRectangle0);
	texCoords1 = CalculateAtlasUV(texCoords1, input.SourceRectangle1);
	float2 voxelGridCoords = CalculateAtlasUV(input.TextureCoordinates.xy, VoxelGrid);

	// Use the original uv coordinates for mip-map calculation (crashes PIX debugger)
	//float r1 = ddx(OriginalTextureCoordinates.x);
	//float r2 = ddy(OriginalTextureCoordinates.y);
	
	//float4 colour0 = tex2D(TextureSampler, texCoords0, r1, r2);
	//float4 colour1 = tex2D(TextureSampler, texCoords1, r1, r2);
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
	colour = lerp(colour, lerp(colour, voxelGridColour, voxelGridColour.a), input.VoxelGridParameters.x);

	// Set alpha value based on input (used for fading)
	colour.a *= input.TextureCoordinates.w;

	output.Colour = colour;

	// [Depth]
	output.Depth.r = input.PositionClone.z / input.PositionClone.w;

	// Flip to keep accuracy away from floating point issues
	output.Depth.r = 1.0f - output.Depth.r; 

	output.Depth.a = 1;

	return output;
}

PixelShaderTransparentOutput PixelShaderTransparentFunction(VertexShaderOutput input)
{
	PixelShaderTransparentOutput output = (PixelShaderTransparentOutput)0;

	float2 OriginalTextureCoordinates = input.TextureCoordinates;

	// [No Scaling]
	float2 texCoords0 = input.TextureCoordinates.xy;
	float2 texCoords1 = input.TextureCoordinates.xy;

	// [Scroll Texture]
	float4 offset = input.ScrollVector * Time;
	texCoords0.x -= offset.x;
	texCoords0.y += offset.y;
	texCoords1.x -= offset.z;	
	texCoords1.y += offset.w;

	// Wrap coordinates in [0, 1) range (for both tiling and scrolling)
	// Use 'frac(texCoord) * n' to wrap in [0, n) range as only mantissa is used
	texCoords0 = frac(texCoords0.xy * input.NumTiles.xy);
	texCoords1 = frac(texCoords1.xy * input.NumTiles.zw);

	// Adjust uv coordinates so they use the correct texture from the texture atlas  
	texCoords0 = CalculateAtlasUV(texCoords0, input.SourceRectangle0);
	texCoords1 = CalculateAtlasUV(texCoords1, input.SourceRectangle1);
	float2 voxelGridCoords = CalculateAtlasUV(input.TextureCoordinates.xy, VoxelGrid);

	float4 colour0 = tex2D(TextureSampler, texCoords0);
	float4 colour1 = tex2D(TextureSampler, texCoords1);
	float4 voxelGridColour = tex2D(TextureSampler, voxelGridCoords);

	// [Linear Interpolation]
	// • Based on colour1 alpha (therefore colour1 takes precedence over colour0)
	// • output = lerp(A, B, C);
	// • output = A * (1 - C) + B * C;
	float4 colour = lerp(colour0, colour1, colour1.a);

	// Add a frame to the quadrangle when drawing voxels (depending on voxel grid value)
	colour = lerp(colour, lerp(colour, voxelGridColour, voxelGridColour.a), input.VoxelGridParameters.x);

	// Set alpha value based on input (used for fading)
	colour.a *= input.TextureCoordinates.w;

	// [Order Independent Transparency]	
	// • Range is [0, 1] so take full advantage of floating point range using 'Downscale' method
	// • Value is scaled up back into normal range in 'WeightedAverageFinal.fx' using 'Upscale' method
	output.Colour = Downscale(float4(colour.rgb * colour.a, colour.a));
	
	// '1.0' will be added to the depth complexity for each pixel drawn at this location (using BlendFunction.Add)
	output.DepthComplexity.r = Downscale(1.0);

	return output;
}

//=============================================
//---[Techniques]------------------------------
//=============================================

technique Forward
{
    pass Pass0
    {		
		// All render states are set in XNA C# code

        VertexShader = compile vs_3_0 VertexShaderFunction();
        PixelShader = compile ps_3_0 PixelShaderFunction();
    }
}

technique TransparentWeightedAverage
{
    pass Pass0
    {		
		// All render states are set in XNA C# code

        VertexShader = compile vs_3_0 VertexShaderFunctionPolygonOffset();
        PixelShader = compile ps_3_0 PixelShaderTransparentFunction();
    }
}