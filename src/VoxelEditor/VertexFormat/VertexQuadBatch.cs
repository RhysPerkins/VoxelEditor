using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace VoxelMeshEditor
{
    public struct VertexQuadBatch : IVertexType
    {
        public Vector3 Position;
        public Vector4 Normal;
        public Vector4 TexCoordDimension;
        public Vector4 Column1;
        public Vector4 Column2;
        public Vector4 Column3;
        public Vector4 Column4;
        public Vector4 SourceRectangle0;
        public Vector4 SourceRectangle1;
        public Vector4 ScrollVector;
        public Vector4 NumTiles;
        public Vector4 WarpParameters0;
        public Vector4 WarpParameters1;
        public Vector4 VoxelGridParameters;

        public VertexQuadBatch(
            Vector3 position,
            Vector4 normal,
            Vector4 texCoordDimension,
            Vector4 column1,
            Vector4 column2,
            Vector4 column3,
            Vector4 column4,
            Vector4 sourceRectangle0,
            Vector4 sourceRectangle1,
            Vector4 scrollVector,
            Vector4 numTiles,
            Vector4 warpParameters0,
            Vector4 warpParameters1,
            Vector4 voxelGridParameters
            )
        {
            Position = position;
            Normal = normal;
            TexCoordDimension = texCoordDimension;
            Column1 = column1;
            Column2 = column2;
            Column3 = column3;
            Column4 = column4;
            SourceRectangle0 = sourceRectangle0;
            SourceRectangle1 = sourceRectangle1;
            ScrollVector = scrollVector;
            NumTiles = numTiles;
            WarpParameters0 = warpParameters0;
            WarpParameters1 = warpParameters1;
            VoxelGridParameters = voxelGridParameters;
        }

        public readonly static VertexDeclaration VertexDeclaration = new VertexDeclaration
        (
            new VertexElement(
                0,                                      // sizeof(float) * 3 = 4 * 3 = 12
                VertexElementFormat.Vector3,
                VertexElementUsage.Position,
                0
                ),

            new VertexElement(                          // sizeof(float) * 4 = 4 * 4 = 16
                12,
                VertexElementFormat.Vector4,
                VertexElementUsage.Normal,
                0
                ),

            new VertexElement(
                28,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                0                                       // : TEXCOORD0
                ),

            new VertexElement(
                44,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                1                                       // : TEXCOORD1
                ),

            new VertexElement(
                60,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                2                                       // : TEXCOORD2
                ),

            new VertexElement(
                76,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                3                                       // : TEXCOORD3
                ),

            new VertexElement(
                92,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                4                                       // : TEXCOORD4
                ),

            new VertexElement(
                108,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                5                                       // : TEXCOORD5
                ),

            new VertexElement(
                124,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                6                                       // : TEXCOORD6
                ),

            new VertexElement(
                140,
                VertexElementFormat.Vector4,
                VertexElementUsage.TextureCoordinate,
                7                                       // : TEXCOORD7
                ),

            new VertexElement(
                156,
                VertexElementFormat.Vector4,
                VertexElementUsage.Color,
                0                                       // : COLOR0
                ),

            new VertexElement(
                172,
                VertexElementFormat.Vector4,
                VertexElementUsage.Color,
                1                                       // : COLOR1
                ),

            new VertexElement(
                188,
                VertexElementFormat.Vector4,
                VertexElementUsage.Color,
                2                                       // : COLOR2
                ),

            new VertexElement(
                204,
                VertexElementFormat.Vector4,
                VertexElementUsage.Color,
                3                                       // : COLOR3
                )
        );

        // Imagine a stream of vertices is sent to the graphics card.
        // Your graphics card would have a pretty hard time trying to figure out where to split the stream of bytes into vertices.
        // It needs to know the number of bytes occupied by one vertex so it can cut the stream nicely into separate vertices.
        public const int SizeInBytes = 220;

        VertexDeclaration IVertexType.VertexDeclaration { get { return VertexDeclaration; } }
    }
}