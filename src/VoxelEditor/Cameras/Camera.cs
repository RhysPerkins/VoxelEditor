using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using System.Xml.Serialization;

namespace VoxelMeshEditor.Cameras
{
    // Matrix Order
    // ============
    //
    // Row Major
    // ---------
    // - XNA Default -> matrixView.Right -> M11 M12 M13
    //   xAxis.X = matrix_View.M11;    
    //   xAxis.Y = matrix_View.M12;    
    //   xAxis.Z = matrix_View.M13;
    //
    // Column Major
    // ------------
    // - NOT XNA Default -> M11 M21 M31
    //   xAxis.X = matrix_View.M11;    
    //   xAxis.Y = matrix_View.M21;    
    //   xAxis.Z = matrix_View.M31;
    //
    // Pixel Aspect Ratio
    // ==================
    // - Tells us the ratio of a pixel's height to its width
    // 
    //   xPhysicalPixelSize   xPhysicalHeightMonitor   xMonitorResolution
    //   ------------------ = ---------------------- • ------------------
    //   yPhysicalPixelSize   yPhysicalHeightMonitor   yMonitorResolution
    //
    // Projection Matrix
    // =================
    // - The projection matrix maps the camera-space coordinates into the screen space 
    //   where "left of the screen" is -1, and "right of the screen" is 1, and same for top and bottom 
    // - Using an orthographic projection removes all perspective from the camera's view 
    //   This is mostly useful for making isometric or 2D games
    // - For an orthographic width, you want the X coordinate to be -1 at "0" and 1 at "width" 
    //   This means that the total length of the (x,y,z) vector in the first column should be 2 / width 
    // - For example, if you want 500 to be the center of X, then you should make the last element -500 
    //   (because 500 - 500 == 0, which is the center of the projected screen)

    public enum MatrixOrder : byte
    {
        ColumnMajor,
        RowMajor
    }

    [XmlInclude(typeof(FreeLookCameraSettings))]
    public abstract class CameraSettings { }

    /// <summary>
    /// Abstract base class for all camera types
    /// </summary>
    public abstract class Camera
    {
        protected CollisionDetection.ThreeDimensional.BoundingFrustum boundingFrustum = new CollisionDetection.ThreeDimensional.BoundingFrustum();   // Bounding frustum                                                                               // Entity to be used for camera collision detection
        protected float farClipPlaneZ;
        protected float fov;
        protected VoxelMeshEditorMain game;
        protected bool isAlive = false;
        protected bool isIgnoreInput = false;                                                                       // Ignores all input from HID devices
        protected bool isLookInverted = false;                                                                      // Invert look for y-axis
        protected float lookSensitivity;                                                                            // HID (Mouse/Gamepad) look sensitivity
        protected MatrixOrder matrixOrder = MatrixOrder.RowMajor;                                                   // Set matrix order to XNA default
        protected Matrix matrix_Projection;                                                                         // Projection Matrix
        protected Matrix matrix_View;                                                                               // View Matrix
        protected Matrix matrix_ViewProj;
        protected float nearClipPlaneZ;
        protected Quaternion orientation;                                                                           // Orientation/rotation 
        protected float pitchAccum;                                                                                 // Accumulated degrees for pitch (Looking "Up" and "Down") 
        protected Vector3 position;                                                                                 // Current position      
        protected float speed;
        protected Vector3 velocity;
        protected float yawAccum;                                                                                   // Accumulated degrees for yaw (Looking "Left" and "Right")  

        protected abstract void HandleInput();                                                     // Handles all HID input
        public abstract void LookAt(Vector3 camPosition, Vector3 camTarget);                                        // Tells the camera to immediately look at a target from a position
        public abstract void LookAt(Vector3 camPosition, Vector3 camTarget, float interpTime, GameTime gameTime);   // Tells the camera to interpolate towards a target from a position
        public abstract void Move(Vector3 direction);
        public abstract void Read(CameraSettings settings);
        public abstract CameraSettings Write();
        public abstract void Spawn(CameraSettings settings);
        public abstract void Update(GameTime gameTime);                                                             // Updates camera (view matrix, etc.)
        protected abstract void UpdatePerspective();                                                                // Updates the projection matrix
        protected abstract void UpdateViewMatrix();                                                                 // Updates the view matrix
        public abstract void Viewport(Viewport viewport);

        public CollisionDetection.ThreeDimensional.BoundingFrustum BoundingFrustum
        {
            get { return boundingFrustum; }
        }

        public float FarClip
        {
            get { return farClipPlaneZ; }
            set
            {
                farClipPlaneZ = value;
                UpdatePerspective();
            }
        }

        public Vector3 Forward
        {
            get
            {
                Vector3 forward = new Vector3();

                switch (matrixOrder)
                {
                    case MatrixOrder.ColumnMajor:
                        {
                            forward.X = -matrix_View.M13;      // -ve due to -Z going into the screen
                            forward.Y = -matrix_View.M23;
                            forward.Z = -matrix_View.M33;
                            break;
                        }
                    case MatrixOrder.RowMajor:
                        {
                            forward.X = -matrix_View.M31;
                            forward.Y = -matrix_View.M32;
                            forward.Z = -matrix_View.M33;
                            break;
                        }
                }

                return forward;
            }
        }

        /// <summary>
        /// Camera's field of view in radians (x axis)
        /// </summary>
        public float FOV
        {
            get { return fov; }
            set
            {
                fov = value;
                UpdatePerspective();
            }
        }

        public bool IsIgnoreInput
        {
            get { return isIgnoreInput; }
            set { isIgnoreInput = value; }
        }

        public bool IsLookInverted
        {
            get { return isLookInverted; }
            set { isLookInverted = value; }
        }

        public float LookSensitivity
        {
            get { return lookSensitivity; }
            set { lookSensitivity = value; }
        }

        public float NearClip
        {
            get { return nearClipPlaneZ; }
            set
            {
                nearClipPlaneZ = value;
                UpdatePerspective();
            }
        }

        public Quaternion Orientation
        {
            get { return orientation; }
            set
            {
                orientation = value;

                // TODO: Handle IsInterpolating
                UpdateViewMatrix();
            }
        }

        public Vector3 Position
        {
            get { return position; }
            set { position = value; }
        }

        public Matrix ProjectionMatrix
        {
            get { return matrix_Projection; }
        }

        public Vector3 Right
        {
            get
            {
                Vector3 right = new Vector3();

                switch (matrixOrder)
                {
                    case MatrixOrder.ColumnMajor:
                        {
                            right.X = matrix_View.M11;
                            right.Y = matrix_View.M21;
                            right.Z = matrix_View.M31;
                            break;
                        }
                    case MatrixOrder.RowMajor:
                        {
                            right.X = matrix_View.M11;
                            right.Y = matrix_View.M12;
                            right.Z = matrix_View.M13;
                            break;
                        }
                }

                return right;
            }
        }

        public Vector3 Up
        {
            get
            {
                Vector3 up = new Vector3();

                switch (matrixOrder)
                {
                    case MatrixOrder.ColumnMajor:
                        {
                            up.X = matrix_View.M12;
                            up.Y = matrix_View.M22;
                            up.Z = matrix_View.M32;
                            break;
                        }
                    case MatrixOrder.RowMajor:
                        {
                            up.X = matrix_View.M21;
                            up.Y = matrix_View.M22;
                            up.Z = matrix_View.M23;
                            break;
                        }
                }

                return up;
            }
        }

        public Matrix ViewMatrix
        {
            get { return matrix_View; }
        }

        public Matrix ViewProjectionMatrix
        {
            get { return matrix_ViewProj; }
        }
    }
}