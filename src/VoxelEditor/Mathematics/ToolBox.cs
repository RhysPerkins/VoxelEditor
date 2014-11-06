using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;

// TODO: No variation of QuaternionFromYawPitchRoll and QuaternionToEulerAngles works with pitch equal to PI '(0, PI, 0)'
// TODO: See http://lab.polygonal.de/2007/05/10/bitwise-gems-fast-integer-math/ for ways of speeding up mathematical calculations

namespace VoxelEditor.Mathematics
{
    static class ToolBox
    {
        public const float EPSILON = 1e-5f;                 // 1e-5 is a favorite because it's about the smallest quantum for a 32-bit float at magnitude 1 (0.00001f)
        public const float ONEEIGHTYOVERPI = 57.29578f;
        public const float PI = 3.141592f;
        public const float PIOVER2 = 1.570796f;
        public const float PIOVER4 = 0.785398f;
        public const float PIOVER8 = 0.392699f;
        public const float PIOVER16 = 0.196349f;
        public const float PIOVER180 = 0.01745329f;
        public const float SQREPSILON = EPSILON * EPSILON;
        public const float SQRT2 = 1.414213f;
        public const float SQRT3 = 1.732050f;
        public const float TWOPI = 6.28318548f;
        public const float OOTWOPI = 1.0f / TWOPI;

        public static readonly Random Random = new Random();

        public static float Abs(float value)
        {
            return value < 0 ? -value : value;
        }

        public static int Abs(int value)
        {
            return value < 0 ? -value : value;
        }

        public static bool ArePointsCoplanar(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ad = d - a;

            // If the triple scalar product (volume) is 0 then the points are coplanar
            return Vector3.Dot(ab, Vector3.Cross(ac, ad)) == 0 ? true : false;
        }

        public static void BlendIntoAccumulator(float smoothRate, ref float newValue, ref float smoothedAccumulator)
        {
            smoothedAccumulator = Interpolate(Clamp(smoothRate, 0, 1), ref smoothedAccumulator, ref newValue);
        }

        public static void BlendIntoAccumulator(float smoothRate, ref Vector3 newValue, ref Vector3 smoothedAccumulator)
        {
            smoothedAccumulator = Interpolate(Clamp(smoothRate, 0, 1), ref smoothedAccumulator, ref newValue);
        }

        public static int Clamp(int value, int min, int max)
        {
            if (max < min)
            {
                Swap(ref max, ref min);
            }

            value = value > max ? max : value;
            value = value < min ? min : value;
            return value;
        }

        public static float Clamp(float value, float min, float max)
        {
            if (max < min)
            {
                Swap(ref max, ref min);
            }

            value = value > max ? max : value;
            value = value < min ? min : value;
            return value;
        }

        // [Screen Space Coordinates] 
        // • X, Y  go from (0, 0) [Top Left] to (Viewport.Width, Viewport.Height) [Bottom Right]
        //
        //  (0, 0)-----------(VP.W, 0)  [x]: 0 -----► VP.W
        //      |               |
        //      |               |       [y]: 0
        //      |     (0,0)     |            ▲
        //      |               |            |
        //      |               |            |
        //  (0, VP.H)------(VP.W, VP.H)     VP.H

        public static Vector2 ClipToScreenSpace(Vector2 clipSpacePos, Microsoft.Xna.Framework.Graphics.Viewport vp)
        {
            // Assumes clip space has already undergone homogenous divide 
            Vector2 screenSpacePoint = new Vector2();
            screenSpacePoint.X = (1 + clipSpacePos.X) * vp.Width * 0.5f + vp.X;
            screenSpacePoint.Y = (1 - clipSpacePos.Y) * vp.Height * 0.5f + vp.Y;

            // Multiply each component by the  homogenous divide
            //screenSpacePoint.X = (1 + clipSpacePos.X * ooW) * vp.Width * 0.5f + vp.X;
            //screenSpacePoint.Y = (1 - clipSpacePos.Y * ooW) * vp.Height * 0.5f + vp.Y;

            return screenSpacePoint;
        }

        public static Vector4 ClipToScreenSpace(Vector4 clipSpacePos, Microsoft.Xna.Framework.Graphics.Viewport vp)
        {
            // Assumes clip space has already undergone homogenous divide 
            Vector4 screenSpacePoint = new Vector4();
            screenSpacePoint.X = (1 + clipSpacePos.X) * vp.Width * 0.5f + vp.X;
            screenSpacePoint.Y = (1 - clipSpacePos.Y) * vp.Height * 0.5f + vp.Y;
            screenSpacePoint.Z = clipSpacePos.Z * (vp.MaxDepth - vp.MinDepth) + vp.MinDepth;

            // Multiply each component by the homogenous divide
            //screenSpacePoint.X = (1 + clipSpacePos.X * ooW) * vp.Width * 0.5f + vp.X;
            //screenSpacePoint.Y = (1 - clipSpacePos.Y * ooW) * vp.Height * 0.5f + vp.Y;
            //screenSpacePoint.Z = (clipSpacePos.Z * ooW) * (vp.MaxDepth - vp.MinDepth) + vp.MinDepth;

            screenSpacePoint.W = 1.0f;

            return screenSpacePoint;
        }

        private static bool ClipSegment(float min, float max, float a, float b, float d, ref float t0, ref float t1)
        {
            if (Abs(d) < EPSILON)
            {
                if (d > 0.0f)
                {
                    return !(b < min || a > max);
                }
                else
                {
                    return !(a < min || b > max);
                }
            }

            // Compute intersection t value of ray with near and far plane of slab (ood = one over d)
            float invd = 1f / d;
            float u0;
            float u1;

            u0 = (min - a) * invd;
            u1 = (max - a) * invd;

            // Make 'u0' be intersection with near plane, 'u1' with far plane
            if (u0 > u1)
            {
                Swap(ref u0, ref u1);
            }

            // Compute the intersection of slab intersection intervals
            t0 = Max(u0, t0);
            t1 = Min(u1, t1);

            // Exit with no collision as soon as slab intersection becomes empty
            if (t0 > t1)
            {
                return false;
            }

            return true;
        }

        private static bool ClipSegmentToAABB(ref Vector3 segmentA, ref Vector3 segmentB, Vector3 aabbMin, Vector3 aabbMax)
        {
            Vector3 origin = segmentA;
            Vector3 direction = segmentB - segmentA;

            float t0 = 0.0f;
            float t1 = 1.0f;

            if (!ClipSegment(aabbMin.X, aabbMax.X, segmentA.X, segmentB.X, direction.X, ref t0, ref t1))
                return false;

            if (!ClipSegment(aabbMin.Y, aabbMax.Y, segmentA.Y, segmentB.Y, direction.Y, ref t0, ref t1))
                return false;

            if (!ClipSegment(aabbMin.Z, aabbMax.Z, segmentA.Z, segmentB.Z, direction.Z, ref t0, ref t1))
                return false;

            segmentA = origin + (direction * t0);
            segmentB = origin + (direction * t1);

            return true;
        }

        /// <summary>
        /// Evaluates a single point on the gaussian falloff curve.
        /// Used for setting up the blur filter weightings.
        /// </summary>
        /// <param name="n">Sample offset</param>
        /// <param name="theta">Blur Amount</param>
        /// <returns></returns>
        public static float ComputeGaussian(float n, float theta)
        {
            // This is the equation of a Gaussian function in one dimension
            // Both dimensions are calculated separately (it is faster than blurring both x and y at the same time)
            // http://en.wikipedia.org/wiki/Gaussian_blur

            return (float)((1.0 / Math.Sqrt(TWOPI * theta)) * Math.Exp(-(n * n) / (2.0 * theta * theta)));
        }

        /// <summary>
        /// Returns a quaternion representing the orientation require to face target point [Column Major Version]
        /// </summary>
        /// <param name="target">Target point to look at</param>
        /// <param name="position">Object's position</param>      
        /// <param name="up">Object's up vector ("y-axis" relative to local orientation)</param>
        /// <returns></returns>
        public static Quaternion CreateLookAt_CM(Vector3 target, Vector3 position, Vector3 up)
        {
            Vector3 zAxis = Vector3.Normalize(position - target);

            Vector3 xAxis = Vector3.Cross(up, zAxis);

            // If x axis is parallel or pointing in the same direction as up
            if (xAxis.Length() < 0.01f)
            {
                // Pick another unit axis
                xAxis = Vector3.Cross(-Vector3.UnitZ, zAxis);
            }
            if (xAxis.Length() < 0.01f)
            {
                // Pick another unit axis
                xAxis = Vector3.Cross(Vector3.UnitX, zAxis);
            }

            xAxis.Normalize();

            Vector3 yAxis = Vector3.Normalize(Vector3.Cross(zAxis, xAxis));

            Matrix m = Matrix.Identity;

            // Column Major
            m.M11 = xAxis.X;
            m.M12 = yAxis.X;
            m.M13 = zAxis.X;

            m.M21 = xAxis.Y;
            m.M22 = yAxis.Y;
            m.M23 = zAxis.Y;

            m.M31 = xAxis.Z;
            m.M32 = yAxis.Z;
            m.M33 = zAxis.Z;

            // Projects position onto new local axes after rotation (translation part of matrix)
            // Not really needed as quaternions contain no position information but can be used with a world matrix
            //m.M41 = -Vector3.Dot(position, xAxis);
            //m.M42 = -Vector3.Dot(position, yAxis);
            //m.M43 = -Vector3.Dot(position, zAxis);

            return Quaternion.CreateFromRotationMatrix(m);
        }

        /// <summary>
        /// Returns a quaternion representing the orientation require to face target point [Row Major Version]
        /// </summary>
        /// <param name="target">Target point to look at</param>
        /// <param name="position">Object's position</param>      
        /// <param name="up">Object's up vector ("y-axis" relative to local orientation)</param>
        /// <returns></returns>
        public static Quaternion CreateLookAt_RM(Vector3 target, Vector3 position, Vector3 up)
        {
            Vector3 zAxis = Vector3.Normalize(position - target);

            Vector3 xAxis = Vector3.Cross(up, zAxis);

            // If x axis is parallel or pointing in the same direction as up
            if (xAxis.Length() < 0.01f)
            {
                // Pick another unit axis
                xAxis = Vector3.Cross(-Vector3.UnitZ, zAxis);
            }
            if (xAxis.Length() < 0.01f)
            {
                // Pick another unit axis
                xAxis = Vector3.Cross(Vector3.UnitX, zAxis);
            }

            xAxis.Normalize();

            Vector3 yAxis = Vector3.Normalize(Vector3.Cross(zAxis, xAxis));

            Matrix m = Matrix.Identity;

            // Row Major
            m.M11 = xAxis.X;
            m.M12 = xAxis.Y;
            m.M13 = xAxis.Z;

            m.M21 = yAxis.X;
            m.M22 = yAxis.Y;
            m.M23 = yAxis.Z;

            m.M31 = zAxis.X;
            m.M32 = zAxis.Y;
            m.M33 = zAxis.Z;

            return Quaternion.CreateFromRotationMatrix(m);
        }

        public static float Cross(Vector2 a, Vector2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Returns velocity of deflection
        /// </summary>
        /// <param name="currentVelocity">Velocity vector if item to be bounced</param>
        /// <param name="elasticity">Elasticity of item to be bounced</param>
        /// <param name="collisionNormal">Normal vector of plane the item is bouncing off of</param>
        /// <returns>Velocity vector of deflection</returns>
        public static Vector3 Deflection(Vector3 currentVelocity, float elasticity, Vector3 collisionNormal)
        {
            Vector3 newDirection = elasticity * (-2 * Vector3.Dot(currentVelocity, collisionNormal) * collisionNormal + currentVelocity);
            //Vector3 newDirection = Vector3.Reflect(CurrentVelocity, CollisionNormal) * Elasticity;

            return newDirection;
        }

        public static Vector2 DegreesToXYVector(float degrees)
        {
            Vector2 vector = new Vector2();
            vector.X = (float)Math.Sin(degrees * PIOVER180);
            vector.Y = (float)Math.Cos(degrees * PIOVER180);

            return vector;
        }

        public static Vector3 DegreesToXZVector(float degrees)
        {
            Vector3 vector = new Vector3();
            vector.X = (float)Math.Sin(degrees * PIOVER180);
            vector.Y = 0f;
            vector.Z = (float)Math.Cos(degrees * PIOVER180);

            return vector;
        }

        /// <summary>
        /// Returns the number of digits in an integer number
        /// </summary>
        /// <param name="number"></param>
        /// <returns></returns>
        public static int Digits(int number)
        {
            number = Abs(number);

            int digits = 1;
            int step = 10;

            while (step <= number)
            {
                digits++;
                step *= 10;
            }

            return digits;
        }

        /// <summary>
        /// Generates a random point on a disc with the concentration of points in the centre
        /// <para>See: http://mathworld.wolfram.com/DiskPointPicking.html </para>
        /// </summary>
        /// <param name="radius">Radius of the disc</param>
        /// <returns>A random point on the disc</returns>
        public static Vector3 DiscPointPickingCentre(float radius)
        {
            radius = Random.NextFloat(0f, radius);

            float theta = Random.NextFloat(0f, TWOPI - EPSILON);

            Vector3 point = new Vector3();
            point.X = radius * (float)Math.Cos(theta);
            point.Y = radius * (float)Math.Sin(theta);
            point.Z = 0f;

            return point;
        }

        /// <summary>
        /// Generates a random point on a disc with even distribution of the points (uses square root)
        /// </summary>
        /// <param name="radius">Radius of the disc</param>
        /// <returns>A random point on the disc</returns>
        public static Vector3 DiscPointPickingEven(float radius)
        {
            radius = Random.NextFloat(0f, radius);

            float sqrtR = (float)Math.Sqrt(radius);
            float theta = Random.NextFloat(0f, TWOPI - EPSILON);

            Vector3 point = new Vector3();
            point.X = sqrtR * (float)Math.Cos(theta);
            point.Y = sqrtR * (float)Math.Sin(theta);
            point.Z = 0f;

            return point;
        }

        // This method is a lot faster than using (int)Math.floor(x)
        private static int FastFloor(double x)
        {
            int xi = (int)x;
            return x < xi ? xi - 1 : xi;
        }

        public static List<Vector3> FindCells(ref CollisionDetection.ThreeDimensional.Ray ray, int gridWidth, int gridHeight, int gridLength, float cellSize)
        {
            List<Vector3> cellIndices = new List<Vector3>(50);

            Vector3 segmentA = ray.Origin;
            Vector3 segmentB = ray.Origin + (ray.Direction * ray.Length);

            Vector3 min = new Vector3();
            min.X = -cellSize * 0.5f;
            min.Y = -cellSize * 0.5f;
            min.Z = -cellSize * 0.5f;

            Vector3 max = new Vector3();
            max.X = min.X + (gridWidth * cellSize);
            max.Y = min.Y + (gridHeight * cellSize);
            max.Z = min.Z + (gridLength * cellSize);

            // Clip the segment to grid bounds, returning if it doesn't intersect
            if (!ClipSegmentToAABB(ref segmentA, ref segmentB, min, max))
                return cellIndices;

            float startX = segmentA.X;
            float startY = segmentA.Y;
            float startZ = segmentA.Z;
            float endX = segmentB.X;
            float endY = segmentB.Y;
            float endZ = segmentB.Z;

            // Prevent divide by 0 errors by using small epsilon value
            float dirX = ray.Direction.X + EPSILON;
            float dirY = ray.Direction.Y + EPSILON;
            float dirZ = ray.Direction.Z + EPSILON;

            // How far we must move in the ray direction before we encounter a new voxel (maximum possible value)
            float tDeltaX = cellSize / Abs(dirX);          // In x direction
            float tDeltaY = cellSize / Abs(dirY);          // In y direction
            float tDeltaZ = cellSize / Abs(dirZ);          // In z direction

            // Start voxel coordinates
            float x = (float)Math.Floor(WorldToGrid(startX, cellSize));   // Transform world coordinates to grid coordinates
            float y = (float)Math.Floor(WorldToGrid(startY, cellSize));
            float z = (float)Math.Floor(WorldToGrid(startZ, cellSize));

            // End voxel coordinates
            float x1 = (float)Math.Floor(WorldToGrid(endX, cellSize));
            float y1 = (float)Math.Floor(WorldToGrid(endY, cellSize));
            float z1 = (float)Math.Floor(WorldToGrid(endZ, cellSize));

            // Clamp voxel coordinates to grid boundaries to overcome floating point errors
            x = x < 0 ? 0 : x;
            x = x < gridWidth ? x : gridWidth - 1;
            x1 = x1 < 0 ? 0 : x1;
            x1 = x1 < gridWidth ? x1 : gridWidth - 1;
            y = y < 0 ? 0 : y;
            y = y < gridHeight ? y : gridHeight - 1;
            y1 = y1 < 0 ? 0 : y1;
            y1 = y1 < gridHeight ? y1 : gridHeight - 1;
            z = z < 0 ? 0 : z;
            z = z < gridLength ? z : gridLength - 1;
            z1 = z1 < 0 ? 0 : z1;
            z1 = z1 < gridLength ? z1 : gridLength - 1;

            // Decide which direction to start walking in
            int stepX = Math.Sign(dirX);
            int stepY = Math.Sign(dirY);
            int stepZ = Math.Sign(dirZ);

            // Calculate distance to first intersection in the voxel we start from           
            float tMaxX = dirX < 0 ? GridToWorld(x, cellSize) - startX : GridToWorld(x + 1, cellSize) - startX;
            tMaxX /= dirX;

            float tMaxY = dirY < 0 ? GridToWorld(y, cellSize) - startY : GridToWorld(y + 1, cellSize) - startY;
            tMaxY /= dirY;

            float tMaxZ = dirZ < 0 ? GridToWorld(z, cellSize) - startZ : GridToWorld(z + 1, cellSize) - startZ;
            tMaxZ /= dirZ;

            Vector3 cellIndex = new Vector3();

            while (true)
            {
                // Add current cell (voxel) to list
                cellIndex.X = x;
                cellIndex.Y = y;
                cellIndex.Z = z;
                cellIndices.Add(cellIndex);

                // Find out which direction to step in by finding the smallest distance out of the 3 directions
                // Min(tx, ty, tz) indicates how far one can travel along the segment and still remain in the current cell
                if (tMaxX <= tMaxY && tMaxX <= tMaxZ)
                {
                    // Cannot travel any further in this direction so this is the last cell
                    if (x == x1)
                        break;

                    tMaxX += tDeltaX;
                    x += stepX;
                }
                else if (tMaxY <= tMaxZ)
                {
                    // Cannot travel any further in this direction so this is the last cell
                    if (y == y1)
                        break;

                    tMaxY += tDeltaY;
                    y += stepY;
                }
                else
                {
                    // Cannot travel any further in this direction so this is the last cell
                    if (z == z1)
                        break;

                    tMaxZ += tDeltaZ;
                    z += stepZ;
                }
            }

            return cellIndices;
        }

        public static Vector2 ForwardXYPlaneFrom(float degrees)
        {
            Vector2 vector = new Vector2();
            vector.X = (float)Math.Sin(degrees * PIOVER180);
            vector.Y = (float)Math.Cos(degrees * PIOVER180);

            return vector;
        }

        public static Vector3 ForwardLHXZPlaneFrom(float degrees)
        {
            Vector3 vector = new Vector3();
            vector.X = (float)Math.Sin(degrees * PIOVER180);
            vector.Y = 0f;
            vector.Z = (float)Math.Cos(degrees * PIOVER180);

            return vector;
        }

        public static Vector3 ForwardRHXZPlaneFrom(float degrees)
        {
            Vector3 vector = new Vector3();
            vector.X = (float)Math.Sin(degrees * PIOVER180);
            vector.Y = 0f;
            vector.Z = -(float)Math.Cos(degrees * PIOVER180);

            return vector;
        }

        /// <summary>
        /// Returns the mode from a set of items
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="list"></param>
        /// <returns>The most frequent item (mode)</returns>
        public static T GetMostFrequent<T>(IEnumerable<T> list)
        {
            Dictionary<T, int> dictionary = new Dictionary<T, int>();

            int maxCount = 0;
            T maxItem = default(T);

            int currentCount = 0;

            foreach (T item in list)
            {
                if (dictionary.ContainsKey(item))
                    currentCount = ++dictionary[item];
                else
                    dictionary[item] = currentCount = 1;

                if (currentCount > maxCount)
                {
                    maxCount = currentCount;
                    maxItem = item;
                }
            }

            return maxItem;
        }

        /// <summary>
        /// classify a value relative to the interval between two bounds:
        /// returns -1 when below the lower bound
        /// returns  0 when between the bounds (inside the interval)
        /// returns +1 when above the upper bound
        /// </summary>
        /// <param name="x"></param>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        /// <returns></returns>
        public static int IntervalComparison(float x, float lowerBound, float upperBound)
        {
            if (x < lowerBound) return -1;
            if (x > upperBound) return +1;
            return 0;
        }

        public static float GridToWorld(float gridCoordinate, float cellSize)
        {
            float offset = cellSize * 0.5f;
            return (gridCoordinate * cellSize) - offset;
        }

        /// <summary>Retrieves an element of the matrix by its column and row index</summary>
        /// <param name="matrix">Matrix of which to retrieve an element</param>
        /// <param name="row">Index of the row from which to retrieve the element</param>
        /// <param name="col">Index of the column to retrieve</param>
        /// <returns>The element at the given row and column</returns>
        public static float Index(this Matrix matrix, int row, int col)
        {
            // The ~ (tilde) operator performs a bitwise complement on its single integer operand
            // Complementing a number means to change all the 0 bits to 1 and all the 1s to 0s
            if (((row | col) & ~3) != 0)
            {
                // This is unsafe code BUT it is useful for debugging
                throw new ArgumentOutOfRangeException(((row & ~3) != 0) ? "row" : "col");
            }

            // << = bitwise shift left 
            // << 4 in BINARY shifts integer n from [n * 2^0] to [n * 2^4] which is [n * 16] 
            switch ((row << 2) + col)
            {
                case 0x00: { return matrix.M11; }   // Hexadecimal values must start with "0x"
                case 0x01: { return matrix.M12; }
                case 0x02: { return matrix.M13; }
                case 0x03: { return matrix.M14; }

                case 0x04: { return matrix.M21; }
                case 0x05: { return matrix.M22; }
                case 0x06: { return matrix.M23; }
                case 0x07: { return matrix.M24; }

                case 0x08: { return matrix.M31; }
                case 0x09: { return matrix.M32; }
                case 0x0A: { return matrix.M33; }
                case 0x0B: { return matrix.M34; }

                case 0x0C: { return matrix.M41; }
                case 0x0D: { return matrix.M42; }
                case 0x0E: { return matrix.M43; }
                case 0x0F: { return matrix.M44; }

                default:
                    {
                        // This is unsafe code BUT it is useful for debugging
                        throw new ArgumentOutOfRangeException(
                            "Matrix row and/or column index out of range"
                        );
                    }
            }
        }

        public static float Interpolate(float alpha, float x0, float x1)
        {
            return x0 + ((x1 - x0) * alpha);
        }

        public static float Interpolate(float alpha, ref float x0, ref float x1)
        {
            return x0 + ((x1 - x0) * alpha);
        }

        public static Vector3 Interpolate(float alpha, Vector3 x0, Vector3 x1)
        {
            return x0 + ((x1 - x0) * alpha);
        }

        public static Vector3 Interpolate(float alpha, ref Vector3 x0, ref Vector3 x1)
        {
            return x0 + ((x1 - x0) * alpha);
        }

        /// <summary>
        /// Converts a Vector2 object to an array of three floats in the order of x and y
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static float[] ToFloatArray(ref Vector2 v)
        {
            float[] floats = { v.X, v.Y };
            return floats;
        }

        /// <summary>
        /// Converts a Vector3 object to an array of three floats in the order of x, y, and z
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static float[] ToFloatArray(ref Vector3 v)
        {
            float[] floats = { v.X, v.Y, v.Z };
            return floats;
        }

        /// <summary>
        /// Converts from an array of three floats in the order of x and y to a Vector2 object
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector2 Vector2FromFloats(float[] values)
        {
            return new Vector2(values[0], values[1]);
        }

        /// <summary>
        /// Converts a Vector2 object to an array of three floats in the order of x and y
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static float[] ToFloatArray(Vector2 v)
        {
            float[] floats = { v.X, v.Y };
            return floats;
        }

        /// <summary>
        /// Converts a Vector3 object to an array of three floats in the order of x, y, and z
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static float[] ToFloatArray(Vector3 v)
        {
            float[] floats = { v.X, v.Y, v.Z };
            return floats;
        }

        public static float[] ToFloatArray(Vector4 v)
        {
            float[] floats = { v.X, v.Y, v.Z, v.W };
            return floats;
        }

        /// <summary>
        /// Converts from an array of three floats in the order of x, y, and z to a Vector3 object
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector3 Vector3FromFloats(float[] values)
        {
            return new Vector3(values[0], values[1], values[2]);
        }

        /// <summary>
        /// Returns 2 times the signed triangle area.
        /// <para></para>
        /// Positive value = Counter Clockwise
        /// <para></para>
        /// Negative value = Clockwise
        /// <para></para>
        /// Zero = Degenerate
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public static float Is2DTriangleCCW(Vector2 a, Vector2 b, Vector2 c)
        {
            return (a.X - c.X) * (b.Y - c.Y) - (a.Y - c.Y) * (b.X - c.X);
        }

        /// <summary>
        /// Returns 2 times the signed triangle area.
        /// <para></para>
        /// Positive value = Counter Clockwise
        /// <para></para>
        /// Negative value = Clockwise
        /// <para></para>
        /// Zero = Degenerate
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns></returns>
        public static float Is2DTriangleCCW(Vector3 a, Vector3 b, Vector3 c)
        {
            return (a.X - c.X) * (b.Y - c.Y) - (a.Y - c.Y) * (b.X - c.X);
        }

        public static bool IsNumeric(object Expression)
        {
            if (Expression == null || Expression is DateTime)
            {
                return false;
            }

            if (Expression is Int16 ||
                Expression is Int32 ||
                Expression is Int64 ||
                Expression is Decimal ||
                Expression is Single ||
                Expression is Double ||
                Expression is Boolean)
            {
                return true;
            }

            try
            {
                if (Expression is string)
                {
                    Double.Parse(Expression as string);
                }
                else
                {
                    Double.Parse(Expression.ToString());
                }

                return true;
            }

            // Forget error messages and just return false
            catch { }

            return false;
        }

        /// <summary>
        /// Determines whether the order of the specified vertices is counter-clockwise with respect to a normal.
        /// </summary>
        /// <param name="normal">The triangle normal.</param>
        /// <param name="p1">The first vertex.</param>
        /// <param name="p2">The second vertex.</param>
        /// <param name="p3">The third vertex.</param>
        /// <returns>Returns a value indicating whether the vertices are specified in counter-clockwise order.</returns>
        public static bool IsTriangleCCW(ref Vector3 normal, ref Vector3 p1, ref Vector3 p2, ref Vector3 p3)
        {
            // > 0 for counterclockwise  
            // = 0 for none (degenerate)  
            // < 0 for clockwise  

            Vector3 c1 = new Vector3(p1.X - p2.X, p1.Y - p2.Y, p1.Z - p2.Z);
            Vector3 c2 = new Vector3(p1.X - p3.X, p1.Y - p3.Y, p1.Z - p3.Z);
            Vector3.Cross(ref c1, ref c2, out c1);

            return Vector3.Dot(normal, c1) >= EPSILON;
        }

        /// <summary>
        /// One dimensional linear interpolation
        /// </summary>
        /// <param name="start">Range min</param>
        /// <param name="end">Range max</param>
        /// <param name="amount">Amount to interpolate between min and max [0, 1]</param>
        /// <returns></returns>
        public static float Lerp1D(float start, float end, float amount)
        {
            return start + (end - start) * Min(amount, 1);
        }

        /// <summary>
        /// Two dimensional linear interpolation
        /// </summary>
        /// <param name="x1">X range min</param>
        /// <param name="x3">X range max</param>
        /// <param name="y1">Y range min</param>
        /// <param name="y2">Y range current value</param>
        /// <param name="y3">Y range max</param>
        /// <returns>Interpolated X value</returns>
        public static float Lerp2D(float x1, float x3, float y1, float y2, float y3)
        {
            // Clamp current value to the output range
            y2 = Clamp(y2, y1, y3);

            // x2 = ((y2 - y1)(x3 - x1) / (y3 - y1)) + x1
            float x2 = (y2 - y1);
            x2 *= (x3 - x1);
            x2 /= (y3 - y1) + EPSILON;
            x2 += x1;

            // Clamp the interpolated value to the input range
            x2 = Clamp(x2, x1, x3);

            return x2;
        }

        /// <summary>
        /// Does a "ceiling" or "floor" operation on the angle by which a given vector deviates from a given reference basis vector
        /// </summary>
        /// <param name="insideOrOutside">Is the 'source' vector forced to remain inside of outside of cone</param>
        /// <param name="source">Vector that may deviate from 'basis' vector</param>
        /// <param name="cosineOfConeAngle">Cone slope</param>
        /// <param name="basis">Cone basis</param>
        /// <returns></returns>
        public static Vector3 LimitDeviationAngleUtility(bool insideOrOutside, Vector3 source, float cosineOfConeAngle, Vector3 basis)
        {
            float sourceLength = source.Length();

            // immediately return zero length input vectors
            if (sourceLength == 0) 
                return source;

            // measure the angular diviation of "source" from "basis"
            Vector3 direction = source / sourceLength;
            float cosineOfSourceAngle = Vector3.Dot(direction, basis);

            // Simply return "source" if it already meets the angle criteria.
            // (note: we hope this top "if" gets compiled out since the flag
            // is a constant when the function is inlined into its caller)
            if (insideOrOutside)
            {
                // source vector is already inside the cone, just return it
                if (cosineOfSourceAngle >= cosineOfConeAngle) return source;
            }
            else
            {
                // source vector is already outside the cone, just return it
                if (cosineOfSourceAngle <= cosineOfConeAngle) return source;
            }

            // find the portion of "source" that is perpendicular to "basis"
            Vector3 perp = source.PerpendicularComponent(basis);

            // normalize that perpendicular
            Vector3 unitPerp = Vector3.Normalize(perp);

            // construct a new vector whose length equals the source vector,
            // and lies on the intersection of a plane (formed the source and
            // basis vectors) and a cone (whose axis is "basis" and whose
            // angle corresponds to cosineOfConeAngle)
            float perpDist = (float)Math.Sqrt(1 - (cosineOfConeAngle * cosineOfConeAngle));
            Vector3 c0 = basis * cosineOfConeAngle;
            Vector3 c1 = unitPerp * perpDist;

            return (c0 + c1) * sourceLength;
        }

        /// <summary>
        /// Enforce an upper bound on the angle by which a given arbitrary vector
        /// diviates from a given reference direction (specified by a unit basis vector).  
        /// <para>
        /// The effect is to clip the 'source' vector to be inside a cone defined by the basis and an angle
        /// </para>
        /// </summary>
        /// <param name="source"></param>
        /// <param name="cosineOfConeAngle"></param>
        /// <param name="basis"></param>
        /// <returns></returns>
        public static Vector3 LimitMaxDeviationAngle(Vector3 source, float cosineOfConeAngle, Vector3 basis)
        {
            // true = Force source INSIDE cone
            return LimitDeviationAngleUtility(true, source, cosineOfConeAngle, basis);
        }

        /// <summary>
        /// Enforce an upper bound on the angle by which a given arbitrary vector
        /// diviates from a given reference direction (specified by a unit basis vector).  
        /// <para>
        /// The effect is to clip the 'source' vector to be outside a cone defined by the basis and an angle
        /// </para>
        /// </summary>
        /// <param name="source"></param>
        /// <param name="cosineOfConeAngle"></param>
        /// <param name="basis"></param>
        /// <returns></returns>
        public static Vector3 LimitMinDeviationAngle(Vector3 source, float cosineOfConeAngle, Vector3 basis)
        {
            // true = Force source OUTSIDE cone
            return LimitDeviationAngleUtility(true, source, cosineOfConeAngle, basis);
        }

        /// <summary>
        /// Convert an n-dimensional array index into a 1D array index
        /// <para>
        /// E.g. LinearArrayIndex(3, new int[] { width, height, depth }, new int[] { xIndex, yIndex, zIndex });
        /// </para>
        /// </summary>
        /// <param name="numDimensions">Number of dimensions of the array</param>
        /// <param name="dimensionSizes">Size of array in each dimension</param>
        /// <param name="x">N-Dimensional index into array</param>
        /// <returns></returns>
        public static int LinearArrayIndex(int numDimensions, int[] dimensionSizes, int[] x)
        {
            // [1D Array To nD Array]
            // • For each dimension, the index is multiplied by the product of the cardinality of all dimensions minor to it:
            //      (z * width * height) + (y * width) + x
            //      (a * B * C) + (b * C) + (c * 1)                     // 3D
            //      (a * B * C * D) + (b * C * D) + (c * D) + (d * 1)   // 4D

            // int n = 4;                    // Number of dimensions of the array
            // int s[] = {10, 32, 40, 28};   // Size of array in each dimension
            // int x[] = {4, 10, 7, 9};      // N-dimensional index into array

            // Compute linear index 'i'
            int i = 0;
            int k = numDimensions;

            do
            {
                --k;
                i = dimensionSizes[k] * i + x[k];
            }
            while (k != 0);

            return i;
        }

        public static int Max(params int[] values)
        {
            int max = int.MinValue;

            int numValues = values.Length;

            for (int i = 0; i < numValues; ++i)
            {
                if (values[i] > max)
                    max = values[i];
            }

            return max;
        }

        public static int Max(int v0, int v1)
        {
            return v0 >= v1 ? v0 : v1;
        }

        public static float Max(params float[] values)
        {
            float max = float.MinValue;

            int numValues = values.Length;

            for (int i = 0; i < numValues; ++i)
            {
                if (values[i] > max)
                    max = values[i];
            }

            return max;
        }

        public static float Max3(float val1, float val2, float val3)
        {
            return Max(Max(val1, val2), val3);
        }

        public static float Max2Index(float val1, float val2, out float index)
        {
            if (val1 > val2)
            {
                index = 0;
                return val1;
            }
            else
            {
                index = 1;
                return val2;
            }
        }

        public static int Max3Index(float val1, float val2, float val3)
        {
            if (val1 >= val2 && val1 >= val3)
                return 0;
            else if (val2 >= val3)
                return 1;
            else
                return 2;
        }

        public static float Min(params float[] values)
        {
            float min = float.MaxValue;

            for (int i = 0; i < values.Length; ++i)
            {
                if (values[i] < min)
                    min = values[i];
            }

            return min;
        }

        public static int Min(int v0, int v1)
        {
            return v0 <= v1 ? v0 : v1;
        }

        public static int Min(params int[] values)
        {
            int min = int.MaxValue;

            for (int i = 0; i < values.Length; ++i)
            {
                if (values[i] < min)
                    min = values[i];
            }

            return min;
        }

        public static float Min3(float val1, float val2, float val3)
        {
            float min = Min(val1, val2);
            return Min(min, val3);
        }

        public static int Min3Index(float val1, float val2, float val3)
        {
            if (val1 <= val2 && val1 <= val3)
                return 0;
            else if (val2 <= val3)
                return 1;
            else
                return 2;
        }

        public static float Mod(float num, float modulo)
        {
            float ratio = num / modulo;
            return modulo * (ratio - (float)Math.Floor(ratio));
        }

        public static float NextFloat(this Random random)
        {
            return (float)random.NextDouble();
        }

        public static float NextFloat(this Random random, float min, float max)
        {
            if (max < min)
                throw new ArgumentException("Max cannot be less than min");

            return (float)random.NextDouble() * (max - min) + min;
        }

        public static Vector3 NextVector3(this Random random)
        {
            return new Vector3(
                random.NextFloat(),
                random.NextFloat(),
                random.NextFloat()
                );
        }

        public static Vector3 NextVector3(this Random random, Vector3 min, Vector3 max)
        {
            if (max.X < min.X)
                throw new ArgumentException("Max (X) cannot be less than min");

            if (max.Y < min.Y)
                throw new ArgumentException("Max (Y) cannot be less than min");
            
            if (max.Z < min.Z)
                throw new ArgumentException("Max (Z) cannot be less than min");

            return new Vector3(
                random.NextFloat(min.X, max.X),
                random.NextFloat(min.Y, max.Y),
                random.NextFloat(min.Z, max.Z)
                );
        }

        public static Vector4 NextVector4(this Random random)
        {
            return new Vector4(
                random.NextFloat(),
                random.NextFloat(),
                random.NextFloat(),
                random.NextFloat()
                );
        }

        public static Matrix Orientation(this Matrix matrix)
        {
            Matrix m = Matrix.Identity;
            m.M11 = matrix.M11;
            m.M12 = matrix.M12;
            m.M13 = matrix.M13;
            m.M21 = matrix.M21;
            m.M22 = matrix.M22;
            m.M23 = matrix.M23;
            m.M31 = matrix.M31;
            m.M32 = matrix.M32;
            m.M33 = matrix.M33;

            return m;
        }

        public static float Oscillate(float value, float min, float max)
        {
            if (min > max)
                Swap(ref min, ref max);

            float range = max - min;
            float cycleLength = range * 2f;

            float state = Mod(value - min, cycleLength);

            if (state > range)
                state = cycleLength - state;

            return state + min;
        }

        public static int Oscillate(int value, int min, int max)
        {
            if (min > max)
                Swap(ref min, ref max);

            // Bring input into range
            value -= min;

            int range = max - min;
            int cycleLength = range * 2;

            return min + Abs(((value + range) % cycleLength) - range);
        }

        /// <summary>
        /// Return component of vector parallel to a unit basis vector 
        /// <para>'unitBasis' must be have a length of 1 (normalised)</para>
        /// </summary>
        /// <param name="v"></param>
        /// <param name="unitBasis"></param>
        /// <returns></returns>
        public static Vector3 ParallelComponent(this Vector3 v, Vector3 unitBasis)
        {
            float projection = Vector3.Dot(v, unitBasis);

            return unitBasis * projection;
        }

        /// <summary>
        /// Return component of vector perpendicular to a unit basis vector
        /// <para>'unitBasis' must be have a length of 1 (normalised)</para>
        /// </summary>
        /// <param name="v"></param>
        /// <param name="unitBasis"></param>
        /// <returns></returns>
        public static Vector3 PerpendicularComponent(this Vector3 v, Vector3 unitBasis)
        {
            return v - v.ParallelComponent(unitBasis);
        }

        /// <summary>
        /// Like a 2D cross product
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float PerpendicularDotProduct(Vector2 a, Vector2 b)
        {
            return a.X * b.Y - a.Y - b.X;
        }

        public static Vector3 PieWedgePointPicking(float azimuthMin, float azimuthMax, float lattitudeMin, float lattitudeMax)
        {
            // [Pie Wedge Picking]
            // • Similar to sphere
            // • Restrict phi to range where inclination/lattitude of 0° = PIOVER2
            // • Restrict theta to range of view where azimuth of 0° = 0 OR TWOPI

            float theta = Random.NextFloat(0f, TWOPI);
            float u = Random.NextFloat(-1f, 1f);
            float d = (float)Math.Sqrt(1 - (u * u));

            Vector3 point = new Vector3();
            point.X = (float)Math.Cos(theta) * d;
            point.Y = (float)Math.Sin(theta) * d;
            point.Z = u;

            return point;
        }

        public static Matrix QuadrangleRotationFrom(Vector3 forward)
        {
            // Forward
            Vector3.Normalize(ref forward, out forward);

            // Right
            Vector3 right = Vector3.Cross(Vector3.UnitY, forward);

            if (right.LengthSquared() < EPSILON)
                right = Vector3.Cross(-Vector3.UnitZ, forward);

            Vector3.Normalize(ref right, out right);

            // Up
            Vector3 up = Vector3.Cross(forward, right);
            Vector3.Normalize(ref up, out up);

            // Rotation
            Matrix rotation = new Matrix();
            rotation.Forward = forward;
            rotation.Right = right;
            rotation.Up = up;
            rotation.M44 = 1f;

            return rotation;
        }

        public static Quaternion QuaternionFromEulerAngles(float yaw, float pitch, float roll)
        {
            // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm

            // Heading = Yaw
            // Attitude = Pitch
            // Bank = Roll

            yaw *= 0.5f;
            pitch *= 0.5f;
            roll *= 0.5f;

            // Assuming the angles are in radians.
            float c1 = (float)Math.Cos(yaw);
            float s1 = (float)Math.Sin(yaw);
            float c2 = (float)Math.Cos(pitch);
            float s2 = (float)Math.Sin(pitch);
            float c3 = (float)Math.Cos(roll);
            float s3 = (float)Math.Sin(roll);
            float c1c2 = c1 * c2;
            float s1s2 = s1 * s2;

            Quaternion quaternion = new Quaternion();
            quaternion.W = c1c2 * c3 - s1s2 * s3;
            quaternion.X = c1c2 * s3 + s1s2 * c3;
            quaternion.Y = s1 * c2 * c3 + c1 * s2 * s3;
            quaternion.Z = c1 * s2 * c3 - s1 * c2 * s3;

            return quaternion;
        }

        public static Quaternion QuaternionFromColumnMajorRotationMatrix(Matrix matrix)
        {
            Quaternion q = new Quaternion();

            float tr = matrix.M11 + matrix.M22 + matrix.M33;

            if (tr > 0)
            {
                float S = (float)Math.Sqrt(tr + 1.0) * 2; // S=4*qw 
                float ooS = 1f / S;

                q.W = 0.25f * S;
                q.X = (matrix.M32 - matrix.M23) * ooS;
                q.Y = (matrix.M13 - matrix.M31) * ooS;
                q.Z = (matrix.M21 - matrix.M12) * ooS;
            }
            else if ((matrix.M11 > matrix.M22) & (matrix.M11 > matrix.M33))
            {
                float S = (float)Math.Sqrt(1.0 + matrix.M11 - matrix.M22 - matrix.M33) * 2; // S=4*qx 
                float ooS = 1f / S;

                q.W = (matrix.M32 - matrix.M23) * ooS;
                q.X = 0.25f * S;
                q.Y = (matrix.M12 + matrix.M21) * ooS;
                q.Z = (matrix.M13 + matrix.M31) * ooS;
            }
            else if (matrix.M22 > matrix.M33)
            {
                float S = (float)Math.Sqrt(1.0 + matrix.M22 - matrix.M11 - matrix.M33) * 2; // S=4*qy
                float ooS = 1f / S;

                q.W = (matrix.M13 - matrix.M31) * ooS;
                q.X = (matrix.M12 + matrix.M21) * ooS;
                q.Y = 0.25f * S;
                q.Z = (matrix.M23 + matrix.M32) * ooS;
            }
            else
            {
                float S = (float)Math.Sqrt(1.0 + matrix.M33 - matrix.M11 - matrix.M22) * 2; // S=4*qz
                float ooS = 1f / S;

                q.W = (matrix.M21 - matrix.M12) * ooS;
                q.X = (matrix.M13 + matrix.M31) * ooS;
                q.Y = (matrix.M23 + matrix.M32) * ooS;
                q.Z = 0.25f * S;
            }

            return q;
        }

        public static Quaternion QuaternionFromRowMajorRotationMatrix(Matrix matrix)
        {
            Quaternion q = new Quaternion();

            float tr = matrix.M11 + matrix.M22 + matrix.M33;

            if (tr > 0)
            {
                float S = (float)Math.Sqrt(tr + 1.0) * 2; // S=4*qw 
                float ooS = 1f / S;

                q.W = 0.25f * S;
                q.X = (matrix.M23 - matrix.M32) * ooS;
                q.Y = (matrix.M31 - matrix.M13) * ooS;
                q.Z = (matrix.M12 - matrix.M21) * ooS;
            }
            else if ((matrix.M11 > matrix.M22) & (matrix.M11 > matrix.M33))
            {
                float S = (float)Math.Sqrt(1.0 + matrix.M11 - matrix.M22 - matrix.M33) * 2; // S=4*qx 
                float ooS = 1f / S;

                q.W = (matrix.M23 - matrix.M32) * ooS;
                q.X = 0.25f * S;
                q.Y = (matrix.M21 + matrix.M12) * ooS;
                q.Z = (matrix.M31 + matrix.M13) * ooS;
            }
            else if (matrix.M22 > matrix.M33)
            {
                float S = (float)Math.Sqrt(1.0 + matrix.M22 - matrix.M11 - matrix.M33) * 2; // S=4*qy
                float ooS = 1f / S;

                q.W = (matrix.M31 - matrix.M13) * ooS;
                q.X = (matrix.M21 + matrix.M12) * ooS;
                q.Y = 0.25f * S;
                q.Z = (matrix.M32 + matrix.M23) * ooS;
            }
            else
            {
                float S = (float)Math.Sqrt(1.0 + matrix.M33 - matrix.M11 - matrix.M22) * 2; // S=4*qz
                float ooS = 1f / S;

                q.W = (matrix.M12 - matrix.M21) * ooS;
                q.X = (matrix.M31 + matrix.M13) * ooS;
                q.Y = (matrix.M32 + matrix.M23) * ooS;
                q.Z = 0.25f * S;
            }

            return q;
        }

        public static Quaternion QuaternionFromYawPitchRollXNA(float yaw, float pitch, float roll)                  // XNA source original
        {
            float num = roll * 0.5f;
            float sinRollOver2 = (float)Math.Sin((double)num);
            float cosRollOver2 = (float)Math.Cos((double)num);
            float num4 = pitch * 0.5f;
            float sinPitchOver2 = (float)Math.Sin((double)num4);
            float cosPitchOver2 = (float)Math.Cos((double)num4);
            float num7 = yaw * 0.5f;
            float sinYawOver2 = (float)Math.Sin((double)num7);
            float cosYawOver2 = (float)Math.Cos((double)num7);

            Quaternion result;
            result.X = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;     // Z
            result.Y = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;     // W
            result.Z = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;     // Y
            result.W = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;     // X

            return result;
        }

        public static Quaternion QuaternionFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            float rollOver2 = roll * 0.5f;
            float sinRollOver2 = (float)Math.Sin((double)rollOver2);
            float cosRollOver2 = (float)Math.Cos((double)rollOver2);
            float pitchOver2 = pitch * 0.5f;
            float sinPitchOver2 = (float)Math.Sin((double)pitchOver2);
            float cosPitchOver2 = (float)Math.Cos((double)pitchOver2);
            float yawOver2 = yaw * 0.5f;
            float sinYawOver2 = (float)Math.Sin((double)yawOver2);
            float cosYawOver2 = (float)Math.Cos((double)yawOver2);

            // X = PI is giving incorrect result (pitch)

            // Heading = Yaw
            // Attitude = Pitch
            // Bank = Roll

            Quaternion result;
            //result.X = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;
            //result.Y = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;
            //result.Z = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;
            //result.W = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;

            result.W = cosYawOver2 * cosPitchOver2 * cosRollOver2 - sinYawOver2 * sinPitchOver2 * sinRollOver2;
            result.X = sinYawOver2 * sinPitchOver2 * cosRollOver2 + cosYawOver2 * cosPitchOver2 * sinRollOver2;
            result.Y = sinYawOver2 * cosPitchOver2 * cosRollOver2 + cosYawOver2 * sinPitchOver2 * sinRollOver2;
            result.Z = cosYawOver2 * sinPitchOver2 * cosRollOver2 - sinYawOver2 * cosPitchOver2 * sinRollOver2;

            result.Normalize();

            return result;
        }

        public static Vector3 RadiansToVector(float upDownRadians, float leftRightRadians)
        {
            Vector3 v = new Vector3();
            v.X = (float)(Math.Cos(leftRightRadians) * Math.Sin(upDownRadians));
            v.Y = (float)(Math.Sin(leftRightRadians) * Math.Sin(upDownRadians));
            v.Z = (float)Math.Cos(upDownRadians);
            return v;
        }

        static void VectorToRadians(Vector3 vector, out float upDownRadians, out float leftRightRadians)
        {
            upDownRadians = (float)System.Math.Acos(vector.Z / vector.Length());
            leftRightRadians = (float)System.Math.Atan2(vector.Y, vector.X);
        }

        /// <summary>
        /// Generates rays that are evenly distrbuted in a circular formation around a point
        /// </summary>
        /// <param name="point">The point around which the rays are generated</param>
        /// <param name="forward">The forward vector for which TWOPI is pointing at</param>
        /// <param name="numRays">Number of rays to generate</param>
        public static Microsoft.Xna.Framework.Ray[] RayDistributionCircular(Vector3 point, Vector3 forward, int numRays)
        {
            Microsoft.Xna.Framework.Ray[] rays = new Microsoft.Xna.Framework.Ray[numRays];

            // Current heading is based on xz plane with z as forward
            float currentHeading = (float)Math.Atan2(forward.Z, forward.X);
            float increment = TWOPI / (float)numRays;

            Vector3 f = new Vector3();

            for (int i = 0; i < numRays; ++i)
            {
                // Calculate the direction based on the current heading
                // Use xz plane only
                f.X = (float)Math.Cos(currentHeading);
                f.Y = 0f;
                f.Z = (float)Math.Sin(currentHeading);

                // Create the ray
                rays[i] = new Microsoft.Xna.Framework.Ray(point, f);

                // Increase the heading
                currentHeading += increment;
            }

            return rays;
        }

        /// <summary>
        /// Generates rays that are evenly distrbuted in a circular formation around a point but are restricted to an angle
        /// </summary>
        /// <param name="point">The point around which the rays are generated</param>
        /// <param name="forward">The forward vector from which the angle is taken</param>
        /// <param name="angle">The angle over which to distribute the rays (radians)</param>
        /// <param name="numRays">The number of rays to generate</param>
        /// <returns>Generated rays</returns>
        public static Microsoft.Xna.Framework.Ray[] RayDistributionCircular(Vector3 point, Vector3 forward, float angle, int numRays)
        {
            Microsoft.Xna.Framework.Ray[] rays = new Microsoft.Xna.Framework.Ray[numRays];

            // Current heading is based on xz plane
            float heading = (float)Math.Atan2(forward.Z, forward.X);
            float increment = angle / (float)numRays;

            Vector3 f = new Vector3();
            f.Y = 0f;

            if ((numRays & 2) == 0)
            {
                // Even number of rays

                float currentHeading = 0;
                currentHeading = heading - (increment * numRays * 0.5f);

                for (int i = 0; i < numRays; ++i)
                {
                    f.X = (float)Math.Cos(currentHeading);
                    f.Z = (float)Math.Sin(currentHeading);

                    // Create the ray
                    rays[i] = new Microsoft.Xna.Framework.Ray(point, f);

                    currentHeading += increment;
                }
            }
            else
            {
                // Odd number of rays

                // Calculate the direction based on the current heading
                // Use xz plane only
                f.X = (float)Math.Cos(heading);
                f.Z = (float)Math.Sin(heading);

                // Create the for the forward direction
                rays[0] = new Microsoft.Xna.Framework.Ray(point, f);

                float currentHeading = 0;
                int j = 1;
                float offset = 0;

                for (int i = 1; i < numRays; i += 2)
                {
                    offset = increment * j;

                    currentHeading = heading - offset;

                    f.X = (float)Math.Cos(currentHeading);
                    f.Z = (float)Math.Sin(currentHeading);

                    // Create the ray
                    rays[i] = new Microsoft.Xna.Framework.Ray(point, f);

                    currentHeading = heading + offset;

                    f.X = (float)Math.Cos(currentHeading);
                    f.Z = (float)Math.Sin(currentHeading);

                    // Create the ray
                    rays[i + 1] = new Microsoft.Xna.Framework.Ray(point, f);

                    j++;
                }
            }

            return rays;
        }

        /// <summary>
        /// Creates a specified number of random rays within a conical shape denoted by the attitude and heading ranges
        /// </summary>
        /// <param name="origin">Origin of all the rays generated</param>
        /// <param name="forward">Direction around which the random rays will be generated</param>
        /// <param name="attitudeRange">Attitude (up down) angle over which to distribute the rays [Radians]</param>
        /// <param name="headingRange">Heading (left right) angle over which to distribute the rays [Radians]</param>
        /// <param name="numRays">Number of rays to generate</param>
        /// <returns>The random rays generated by the method</returns>
        public static Microsoft.Xna.Framework.Ray[] RandomRayCone(Vector3 origin, Vector3 forward, float attitudeRange, float headingRange, int numRays)
        {
            // See: http://mathworld.wolfram.com/SphericalCoordinates.html

            // r = radial = radius from a point to the origin
            // theta = heading = azimuthal
            // phi = attitude = polar/zenith/colatitude  

            // [Z = Up]
            // x = r * cosTheta * sinPhi
            // y = r * sinTheta * sinPhi
            // z = r * cosPhi

            // phi = cos^-1(z / r) = Math.Acos(z / r)
            // theta = tan^-1(y / x) = Math.Atan2(y / x)

            // [Y = Up]
            // 'y' and 'z' are swapped

            Microsoft.Xna.Framework.Ray[] rays = new Microsoft.Xna.Framework.Ray[numRays];

            float currentAttitude = (float)Math.Acos(forward.Y / forward.Length());
            float currentHeading = (float)Math.Atan2(forward.Z, forward.X);

            attitudeRange *= 0.5f;
            headingRange *= 0.5f;

            float minAttitude = currentAttitude - attitudeRange;
            float maxAttitude = currentAttitude + attitudeRange;

            float minHeading = currentHeading - headingRange;
            float maxHeading = currentHeading + headingRange;

            for (int i = 0; i < numRays; ++i)
            {
                // Create a random direction for the ray within the attitude and heading ranges
                currentAttitude = Random.NextFloat(minAttitude, maxAttitude);
                currentHeading = Random.NextFloat(minHeading, maxHeading);

                forward.X = (float)Math.Cos(currentHeading) * (float)Math.Sin(currentAttitude);
                forward.Y = (float)Math.Cos(currentAttitude);
                forward.Z = (float)Math.Sin(currentHeading) * (float)Math.Sin(currentAttitude);

                // Create the ray
                rays[i] = new Microsoft.Xna.Framework.Ray(origin, forward);
            }

            return rays;
        }

        public static float Range(float a, float b)
        {
            return Abs(a - b);
        }

        public static int Range(int a, int b)
        {
            return Abs(a - b);
        }

        public static Microsoft.Xna.Framework.Ray[] RayDistributionSpherical(Vector3 origin, Vector3 forward, int numRays)
        {
            // See: http://mathworld.wolfram.com/SphericalCoordinates.html

            // r = radial = radius from a point to the origin
            // theta = heading = azimuthal
            // phi = attitude = polar/zenith/colatitude  

            // [Z = Up]
            // x = r * cosTheta * sinPhi
            // y = r * sinTheta * sinPhi
            // z = r * cosPhi

            // phi = cos^-1(z / r) = Math.Acos(z / r)
            // theta = tan^-1(y / x) = Math.Atan2(y / x)

            // [Y = Up]
            // 'y' and 'z' are swapped

            Microsoft.Xna.Framework.Ray[] rays = new Microsoft.Xna.Framework.Ray[numRays];

            float currentAttitude = (float)Math.Acos(forward.Y / forward.Length());
            float currentHeading = (float)Math.Atan2(forward.Z, forward.X);

            float increment = TWOPI / (float)numRays;

            Vector3 f = new Vector3();

            for (int i = 0; i < numRays; ++i)
            {
                f.X = (float)Math.Cos(currentHeading) * (float)Math.Sin(currentAttitude);
                f.Y = (float)Math.Cos(currentAttitude);
                f.Z = (float)Math.Sin(currentHeading) * (float)Math.Sin(currentAttitude);

                // Create the ray
                rays[i] = new Microsoft.Xna.Framework.Ray(origin, f);

                // Increase the attitude and heading
                //currentAttitude += increment;
                currentHeading += increment;
            }

            return rays;
        }

        public static void RegenerateOrthonormalBasis(ref Matrix localSpace, Vector3 forward)
        {
            Vector3.Normalize(ref forward, out forward);

            // Derive new side basis vector from NEW forward and OLD up

            //if (rightHanded)
            //    _side.cross(_forward, _up);
            //else
            //    _side.cross(_up, _forward);
            //_side = _side.normalize();

            Vector3 right = Vector3.Cross(forward, localSpace.Up);

            // Derive new Up basis vector from new Side and new Forward
            // Should have unit length since Side and Forward are perpendicular and unit length

            //if (rightHanded())
            //    _up.cross(_side, _forward);
            //else
            //    _up.cross(_forward, _side);

            Vector3 up = Vector3.Cross(right, forward);

            localSpace.Forward = forward;
            localSpace.Right = right;
            localSpace.Up = up;
        }

        /// <summary>
        /// Left handed version of RotationFrom
        /// </summary>
        /// <param name="forward"></param>
        /// <returns></returns>
        public static Matrix RotationFrom(Vector3 forward)
        {
            // Forward
            Vector3.Normalize(ref forward, out forward);

            // Right
            Vector3 right = Vector3.Cross(forward, Vector3.UnitY);

            if (right.LengthSquared() < EPSILON)
                right = Vector3.Cross(forward, Vector3.UnitZ * Math.Sign(forward.Y));

            Vector3.Normalize(ref right, out right);

            // Up
            Vector3 up = Vector3.Cross(right, forward);
            Vector3.Normalize(ref up, out up);

            // Rotation
            Matrix rotation = new Matrix();
            rotation.Forward = forward;
            rotation.Right = right;
            rotation.Up = up;
            rotation.M44 = 1f;

            return rotation;
        }

        public static void SetIndex(ref Vector2 v, int index, float value)
        {
            switch (index)
            {
                case 0: v.X = value; break;
                case 1: v.Y = value; break;
                default: throw new ArgumentOutOfRangeException("Vector component index out of range");
            }
        }

        public static void SetLength(ref Vector2 v, float length)
        {
            v.Normalize();
            v *= length;
        }

        public static void SetLength(ref Vector3 v, float length)
        {
            v.Normalize();
            v *= length;
        }

        /// <summary>
        /// Returns the unsigned angle between two 2D vectors (Radians)
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float SignedAngleBetweenVectors(Vector2 a, Vector2 b)
        {
            float c = PerpendicularDotProduct(a, b);

            // Unsigned angle
            return (float)Math.Atan2(c, Vector2.Dot(a, b));
        }

        /// <summary>
        /// Returns the signed signed angle between two 3D vectors (Radians)
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float SignedAngleBetweenVectors(Vector3 a, Vector3 b)
        {
            Vector3 c = Vector3.Cross(a, b);

            // Unsigned angle
            float angle = (float)Math.Atan2(c.Length(), Vector3.Dot(a, b));

            // Signed angle obtained from reference vector (UnitY)
            return Vector3.Dot(c, Vector3.UnitY) < 0.0f ? -angle : angle;
        }

        /// Find the angle between two vectors. This will not only give the angle difference, but the direction.
        /// For example, it may give you -1 radian, or 1 radian, depending on the direction. Angle given will be the 
        /// angle from the FromVector to the DestVector, in radians.
        /// </summary>
        /// <param name="FromVector">Vector to start at.</param>
        /// <param name="DestVector">Destination vector.</param>
        /// <param name="DestVectorsRight">Right vector of the destination vector</param>
        /// <returns>Signed angle, in radians</returns>        
        /// <remarks>All three vectors must lie along the same plane.</remarks>
        public static double SignedAngleBetweenVectors(Vector3 FromVector, Vector3 DestVector, Vector3 DestVectorsRight)
        {
            FromVector.Normalize();
            DestVector.Normalize();
            DestVectorsRight.Normalize();

            float forwardDot = Vector3.Dot(FromVector, DestVector);
            float rightDot = Vector3.Dot(FromVector, DestVectorsRight);

            // Keep dot in range to prevent rounding errors
            forwardDot = MathHelper.Clamp(forwardDot, -1.0f, 1.0f);

            double angleBetween = Math.Acos(forwardDot);

            if (rightDot < 0.0f)
                angleBetween *= -1.0f;

            return angleBetween;
        }

        /// <summary>
        /// Sets the matrix to be a skew-symmetric matrix based on the given vector.
        /// The cross product can be expressed in matrix form as the product of a skew-symmetric matrix and a vector
        /// <para>So if a and b are vectors, [Vector3.Cross(a,b)] a X b = a_SS * b (Where a_SS is the skew-symmetric form of a)</para>
        /// </summary>
        /// <param name="matrix_1"></param>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static Matrix Skew_Symmetric(Matrix matrix_1, Vector3 vector)
        {
            matrix_1.M11 = matrix_1.M22 = matrix_1.M33 = 0;

            matrix_1.M12 = -vector.Z;
            matrix_1.M13 = vector.Y;
            matrix_1.M21 = vector.Z;
            matrix_1.M23 = -vector.X;
            matrix_1.M31 = -vector.Y;
            matrix_1.M32 = vector.X;

            // For 4x4 matrices only (Make sure there is no data residing in there)
            matrix_1.M14 = 0f;
            matrix_1.M24 = 0f;
            matrix_1.M34 = 0f;
            matrix_1.M44 = 1f;      // Set to 1 in an identity matrix so must remain at 1

            return matrix_1;
        }

        /// <summary>
        /// Generates a random point on the surface of a unit sphere with even distribution
        /// </summary>
        /// <returns>Random point on sphere's surface</returns>
        public static Vector3 SpherePointPickingEven()
        {
            float theta = Random.NextFloat(0f, TWOPI);
            float u = Random.NextFloat(-1f, 1f);
            float d = (float)Math.Sqrt(1 - (u * u));

            Vector3 point = new Vector3();
            point.X = (float)Math.Cos(theta) * d;
            point.Y = (float)Math.Sin(theta) * d;
            point.Z = u;

            return point;
        }

        /// <summary>
        /// Generates a random point on the surface of a unit sphere with a tendency to bunch near poles (avoids using square root)
        /// </summary>
        /// <returns>Random point on sphere's surface</returns>
        public static Vector3 SpherePointPickingPoles()
        {
            // rho = Distance from centre [0, 1]
            float phi = Random.NextFloat(0f, PI - EPSILON);             // Lattitude
            float theta = Random.NextFloat(0f, TWOPI - EPSILON);

            Vector3 point = new Vector3();
            point.X = (float)(Math.Cos(theta) * Math.Sin(phi));
            point.Y = (float)(Math.Sin(theta) * Math.Sin(phi));
            point.Z = (float)Math.Cos(phi);

            return point;
        }

        /// <summary>
        /// Swaps a with b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public static void Swap(ref int a, ref int b)
        {
            int t;

            t = a;
            a = b;
            b = t;
        }

        public static void Swap(ref float a, ref float b)
        {
            float t;

            t = a;
            a = b;
            b = t;
        }

        public static void Swap(ref Vector2 a, ref Vector2 b)
        {
            Vector2 t;

            t = a;
            a = b;
            b = t;
        }

        public static void Swap(ref Vector3 a, ref Vector3 b)
        {
            Vector3 t;

            t = a;
            a = b;
            b = t;
        }

        /// <summary>
        /// Convert a rotation matrix (Column Major) to 3 Euler angles (Radians)
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public static Vector3 ToEulerAngles(this Matrix m)
        {
            // Store the Euler angles in radians
            Vector3 pitchYawRoll = new Vector3();

            // Check for Gimbal lock, giving a small tolerance for numerical imprecision
            if (m.M12 > 0.998f)
            {
                // Gimble lock at north pole
                pitchYawRoll.Y = (float)Math.Atan2(m.M31, m.M33);   // Yaw
                pitchYawRoll.X = PI * 0.5f;                         // Pitch
                pitchYawRoll.Z = 0f;                                // Roll

                pitchYawRoll.Y = Clamp(pitchYawRoll.Y, -PI, PI);
            }
            else if (m.M12 < -0.998f)
            {
                // Gimble lock at south pole
                pitchYawRoll.Y = (float)Math.Atan2(m.M31, m.M33);   // Yaw
                pitchYawRoll.X = -PI * 0.5f;                        // Pitch
                pitchYawRoll.Z = 0f;                                // Roll

                pitchYawRoll.Y = Clamp(pitchYawRoll.Y, -PI, PI);
            }
            else
            {
                pitchYawRoll.Y = (float)Math.Atan2(-m.M13, m.M11);  // Yaw               
                pitchYawRoll.X = (float)Math.Asin(m.M12);           // Pitch
                pitchYawRoll.Z = (float)Math.Atan2(-m.M32, m.M22);  // Roll

                pitchYawRoll.X = Clamp(pitchYawRoll.X, -PI, PI);
                pitchYawRoll.Y = Clamp(pitchYawRoll.Y, -PI, PI);
                pitchYawRoll.Z = Clamp(pitchYawRoll.Z, -PI, PI);
            }

            return pitchYawRoll;
        }

        public static float SinF(float angle)
        {
            float angleSqr = angle * angle;

            float result = -2.39e-08f;
            result *= angleSqr;
            result += 2.7526e-06f;
            result *= angleSqr;
            result -= 1.98409e-04f;
            result *= angleSqr;
            result += 8.3333315e-03f;
            result *= angleSqr;
            result -= 1.666666664e-01f;
            result *= angleSqr;
            result += 1.0f;
            result *= angle;

            return result;
        }

        public static Vector3 ToEulerAngles(this Quaternion q)
        {
            q.Normalize();

            // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

            // Store the Euler angles in radians
            Vector3 pitchYawRoll = new Vector3();

            double sqx = q.X * q.X;
            double sqy = q.Y * q.Y;
            double sqz = q.Z * q.Z;
            double sqw = q.W * q.W;

            // If quaternion is normalised the unit is one, otherwise it is the correction factor
            double unit = sqx + sqy + sqz + sqw;

            double test = q.X * q.Y + q.Z * q.W;
            //double test = q.X * q.Z - q.W * q.Y;

            if (test > 0.4999f * unit)                              // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                pitchYawRoll.Y = 2f * (float)Math.Atan2(q.X, q.W);  // Yaw
                pitchYawRoll.X = PIOVER2;                           // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else if (test < -0.4999f * unit)                        // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                pitchYawRoll.Y = -2f * (float)Math.Atan2(q.X, q.W); // Yaw
                pitchYawRoll.X = -PIOVER2;                          // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else
            {
                pitchYawRoll.Y = (float)Math.Atan2(2f * q.Y * q.W - 2f * q.X * q.Z, sqx - sqy - sqz + sqw);       // Yaw
                pitchYawRoll.X = (float)Math.Asin(2f * test / unit);                                              // Pitch
                pitchYawRoll.Z = (float)Math.Atan2(2f * q.X * q.W - 2f * q.Y * q.Z, -sqx + sqy - sqz + sqw);      // Roll

                //pitchYawRoll.Y = (float)Math.Atan2(2f * q.X * q.W + 2f * q.Y * q.Z, 1 - 2f * (sqz + sqw));      // Yaw 
                //pitchYawRoll.X = (float)Math.Asin(2f * (q.X * q.Z - q.W * q.Y));                                // Pitch 
                //pitchYawRoll.Z = (float)Math.Atan2(2f * q.X * q.Y + 2f * q.Z * q.W, 1 - 2f * (sqy + sqz));      // Roll 
            }

            return pitchYawRoll;
        }

        /// <summary>
        /// Clamps the length of a given vector to 'maxLength'
        /// </summary>
        /// <param name="v">Vector to clamp</param>
        /// <param name="maxLength">Length to which to clamp the vector to</param>
        /// <returns>Clamped vector</returns>
        public static Vector3 TruncateLength(this Vector3 v, float maxLength)
        {
            float maxLengthSquared = maxLength * maxLength;
            float vLengthSquared = v.LengthSquared();

            return vLengthSquared <= maxLengthSquared ? v : v * (maxLength / (float)Math.Sqrt(vLengthSquared));
        }

        public static float UnsignedAngleBetweenVectors(Vector2 a, Vector2 b)
        {
            return Abs(SignedAngleBetweenVectors(a, b));
        }

        public static float UnsignedAngleBetweenVectors(Vector3 a, Vector3 b)
        {
            Vector3 c = Vector3.Cross(a, b);

            return (float)Math.Atan2(c.Length(), Vector3.Dot(a, b));
        }

        public static float Vector4DotVector3(Vector4 vector4, Vector3 vector3)
        {
            return (vector4.X * vector3.X + vector4.Y * vector3.Y + vector4.Z * vector3.Z + vector4.W);
        }

        public static float VectorToDegrees(Vector2 vector)
        {
            float degrees = (float)Math.Atan2(vector.X, vector.Y) * ONEEIGHTYOVERPI;
            return degrees < 0 ? 360 + degrees : degrees;
        }

        public static float VectorToRadians(Vector2 vector)
        {
            float radians = (float)Math.Atan2(vector.X, vector.Y);
            return radians < 0 ? TWOPI + radians : radians;
        }

        public static void ViewportSpaceToScreenSpace(float viewportWidth, float viewportHeight, ref Vector3[] vertices)
        {
            // [Viewport To Screen/Clip Space]

            // X, Y coordinatates go from (-1, 1) [Top Left] to (1, -1) [Bottom Right]

            //  (-1,1)-----------(1,1)  [x]: -1 -----► 1 
            //      |               |
            //      |               |   [y]:    1
            //      |     (0,0)     |           ▲
            //      |               |           |
            //      |               |           |
            //  (-1,-1)---------(1,-1)         -1

            float ooViewportWidth = 1f / viewportWidth;
            float ooViewportHeight = 1f / viewportHeight;

            ooViewportWidth *= 2f;
            ooViewportHeight *= -2f;

            int numVertices = vertices.Length;

            for (int i = 0; i < numVertices; ++i)
            {
                vertices[i].X *= ooViewportWidth;
                vertices[i].X -= 1f;
                vertices[i].Y *= ooViewportHeight;
                vertices[i].Y += 1f;
            }
        }

        // [Clip Space Coordinates] 
        // • X, Y  go from (-1, 1) [Top Left] to (1, -1) [Bottom Right]
        //
        //  (-1,1)-----------(1,1)  [x]: -1 -----► 1 
        //      |               |
        //      |               |   [y]:  1
        //      |     (0,0)     |         ▲
        //      |               |         |
        //      |               |         |
        //  (-1,-1)---------(1,-1)       -1

        public static Vector2 WorldToClipSpaceVector2(Vector3 worldPos, Matrix viewProjection)
        {
            // Convert object position (pivot point) to normalised device (clip space) coordinates
            Vector4 worldPos4 = new Vector4();
            worldPos4.X = worldPos.X;
            worldPos4.Y = worldPos.Y;
            worldPos4.Z = worldPos.Z;
            worldPos4.W = 1.0f;

            Vector4 postProjectivePosition = Vector4.Transform(worldPos4, viewProjection);

            // Homogeneous divide
            float ooW = 1.0f / postProjectivePosition.W;

            Vector2 clipSpacePos = new Vector2();
            clipSpacePos.X = postProjectivePosition.X * ooW;
            clipSpacePos.Y = postProjectivePosition.Y * ooW;

            return clipSpacePos;
        }

        public static Vector3 WorldToClipSpaceVector3(Vector3 worldPos, Matrix viewProjection)
        {
            // Convert object position (pivot point) to normalised device (clip space) coordinates
            Vector4 worldPos4 = new Vector4();
            worldPos4.X = worldPos.X;
            worldPos4.Y = worldPos.Y;
            worldPos4.Z = worldPos.Z;
            worldPos4.W = 1.0f;

            Vector4 postProjectivePosition = Vector4.Transform(worldPos4, viewProjection);

            // Homogeneous divide
            float ooW = 1.0f / postProjectivePosition.W;

            Vector3 clipSpacePos = new Vector3();
            clipSpacePos.X = postProjectivePosition.X * ooW;
            clipSpacePos.Y = postProjectivePosition.Y * ooW;
            clipSpacePos.Z = postProjectivePosition.Z * ooW;

            return clipSpacePos;
        }

        public static Vector4 WorldToClipSpaceVector4(Vector3 worldPos, Matrix viewProjection)
        {
            // Convert object position (pivot point) to normalised device (clip space) coordinates
            Vector4 worldPos4 = new Vector4();
            worldPos4.X = worldPos.X;
            worldPos4.Y = worldPos.Y;
            worldPos4.Z = worldPos.Z;
            worldPos4.W = 1.0f;

            Vector4 postProjectivePosition = Vector4.Transform(worldPos4, viewProjection);

            // Homogeneous divide
            float ooW = 1.0f / postProjectivePosition.W;

            Vector4 clipSpacePos = new Vector4();
            clipSpacePos.X = postProjectivePosition.X * ooW;
            clipSpacePos.Y = postProjectivePosition.Y * ooW;
            clipSpacePos.Z = postProjectivePosition.Z * ooW;
            clipSpacePos.W = 1.0f;

            return clipSpacePos;
        }

        public static float WorldToGrid(float worldCoordinate, float cellSize)
        {
            float offset = cellSize * 0.5f;
            return (worldCoordinate + offset) / cellSize;
        }

        /// <summary>
        /// Wraps a floating point number around a min/max range [min, max)
        /// <para>'(' and ')' = Exclusive '[' and ']' = Inclusive</para>
        /// </summary>
        /// <param name="value">Number to wrap</param>
        /// <param name="min">Minimum number in wrap range</param>
        /// <param name="max">Maximum number in wrap range</param>
        /// <returns></returns>
        public static float Wrap(float value, float min, float max)
        {
            if (max < min)
                Swap(ref min, ref max);

            float t = (value - min) % (max - min);

            // (min, max]
            //if (result == min) { result = hi; }

            // [min, max]
            // wrap to lo if (x <= lo), and wrap to hi if (x >= hi)

            return t < 0 ? t + max : t + min;
        }

        /// <summary>
        /// Wraps an integer around a min/max range
        /// </summary>
        /// <param name="value">Integer to wrap</param>
        /// <param name="min">Minimum number in wrap range</param>
        /// <param name="max">Maximum number in wrap range</param>
        /// <returns></returns>
        public static int Wrap(int value, int min, int max)
        {
            if (max < min)
                Swap(ref min, ref max);

            int count = max - min + 1;
            int offset = value < min ? value - min + 1 : value - min;
            int origin = value < min ? max : min;

            return (offset % count) + origin;
        }

        /// <summary>
        /// Wraps a floating point number around a min/max range (only works within range either side of min and max)
        /// <para>Faster than using 'Wrap' method</para>
        /// </summary>
        /// <param name="value">Number to wrap</param>
        /// <param name="min">Minimum number in wrap range</param>
        /// <param name="max">Maximum number in wrap range</param>
        /// <returns></returns>
        public static float WrapBasic(float value, float min, float max)
        {
            if (value > max)
                return (value - max) + min;

            if (value < min)
                return max - (min - value);

            return value;
        }

        /// <summary>
        /// Wraps an integer around a min/max range (only works within range either side of min and max)
        /// <para>Faster than using 'Wrap' method</para>
        /// </summary>
        /// <param name="value">Number to wrap</param>
        /// <param name="min">Minimum number in wrap range</param>
        /// <param name="max">Maximum number in wrap range</param>
        /// <returns></returns>
        public static int WrapBasic(int value, int min, int max)
        {
            if (value > max)
                return (value - max) + min;

            if (value < min)
                return max - (min - value);

            return value;
        }
    }
}