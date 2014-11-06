#define e 2.71828
#define EPSILON 1e-6
#define MULTIPLIER 1024.0						// 65536 will not work with transparent objects that rely on this value for Downscale
#define INVMULTIPLIER 1.0 / MULTIPLIER
#define TWOPI 6.28318548

// [Shadow Mapping]

#define HEMISPHERE_BIAS 0.15f
#define SHADOW_SIZE 512
#define TEXEL_SIZE 1.0 / SHADOW_SIZE
#define HALFTEXEL_SIZE 0.5 / SHADOW_SIZE

// [Precision Improvement]

// 'DISTRIBUTEFACTOR' denotes where to split the value
// 8 bits works well for most situations (2^8 = 256)
#define DISTRIBUTEFACTOR 256.0 
#define INVDISTRIBUTEFACTOR 1.0 / DISTRIBUTEFACTOR

// [Global Debug Value]

float DEBUG = 0.0;

// ALWAYS use powers of two for the scaling factor (e.g. '1024.0')
// Multiplying by powers of two has no rounding error in binary floating point whereas other values do (e.g. '1000.0')
// Maximum range for 16-bit is 2^16 = 65536 (i.e. 'MULTIPLIER = 65536')

//=============================================
//---[Functions]-------------------------------
//=============================================

float4 DistributePrecision(float2 Value)  
{  
	// Split precision  
	float2 IntPart;  
	float2 FracPart = modf(Value * DISTRIBUTEFACTOR, IntPart);  
  
	// Compose outputs to make reconstruction cheap  
	return float4(IntPart * INVDISTRIBUTEFACTOR, FracPart);  
} 

float Downscale(float v)
{
	return v * INVMULTIPLIER;
}

float2 Downscale(float2 v)
{
	return v * INVMULTIPLIER;
}

float4 Downscale(float4 v)
{
	return v * INVMULTIPLIER;
}

/// <summary>
/// Pack a floating point value into an RGBA (32bpp)
///
/// Note that video cards apply some sort of bias (error?) to pixels,
/// so we must correct for that by subtracting the next component's
/// value from the previous component
/// </summary>
float4 Pack(float depth)
{
	float bV = 1.0 / 255.0;

    float4 bias = float4(bV, bV, bV, 0.0);

    float r = depth;
    float g = frac(r * 255.0);		// 8 bit
    float b = frac(g * 255.0);
    float a = frac(b * 255.0);

    float4 colour = float4(r, g, b, a);
    
    return colour - (colour.yzww * bias);
}

/// <summary>
/// Pack a floating point value into a Vector2 (16bpp)
/// </summary>
float2 PackHalf(float depth)
{
    float2 bias = float2(1.0 / 255.0, 0.0);
                            
    float2 colour = float2(depth, frac(depth * 255.0));
    
	return colour - (colour.yy * bias);
}

float2 RecombinePrecision(float4 Value)  
{   
	return (Value.zw * INVDISTRIBUTEFACTOR + Value.xy);  
} 

/// <summary>
/// Unpack an RGBA pixel to floating point value.
/// </summary>
float Unpack (float4 colour)
{
    float4 bitShifts = float4(
		1.0,
        1.0 / 255.0,
        1.0 / (255.0 * 255.0),
        1.0 / (255.0 * 255.0 * 255.0)
		);

    return dot(colour, bitShifts);
}

/// <summary>
/// Unpack a float2 to a floating point (used by VSM).
/// </summary>
float UnpackHalf (float2 colour)
{
    return colour.x + (colour.y / 255.0);
}

float Upscale(float v)
{
	return v * MULTIPLIER;
}

float2 Upscale(float2 v)
{
	return v * MULTIPLIER;
}

float4 Upscale(float4 v)
{
	return v * MULTIPLIER;
}