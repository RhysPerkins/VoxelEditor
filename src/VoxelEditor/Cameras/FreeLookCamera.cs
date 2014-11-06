using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using VoxelEditor.Mathematics;

namespace VoxelEditor.Cameras
{
    public class FreeLookCameraSettings : CameraSettings
    {
        public float FarClip = 1100.0f;
        public float FOV = ToolBox.PIOVER4;       // 45 degrees
        public bool IsIgnoreInput = false;
        public float NearClip = 0.1f;
        public Quaternion Orientation = Quaternion.Identity;
        public Vector3 Position = Vector3.Zero;
        public float Sensitivity = 0.3f;
        public float Speed = 10f;
    }

    public class FreeLookCamera : Camera
    {
        float interpolationStart;           // Interpolation start time
        float interpolationTime;            // Time period to perform interpolation over
        bool isInterpolating = false;       // Is the camera interpolating between orientations?     
        Quaternion orientationEnd;          // End orientation for interpolation
        Quaternion orientationStart;        // Starting orientation for interpolation   
        float yaw;                          // Current camera yaw                      

        public FreeLookCamera(VoxelMeshEditorMain game)
            : this(game, null) { }

        public FreeLookCamera(VoxelMeshEditorMain game, FreeLookCameraSettings settings)
        {
            this.game = game;
            matrixOrder = MatrixOrder.ColumnMajor;

            Read(settings);
        }

        protected override void HandleInput()
        {
            if (isIgnoreInput) //|| !game.IsActiveApplication)
                return;

            HandleKeyboard();
            HandleMouse();
        }

        private void HandleKeyboard()
        {
            Vector3 direction = Vector3.Zero;

            if (game.HID.IsKeyDown(PlayerIndex.One, Keys.A))
                direction += -Right;

            if (game.HID.IsKeyDown(PlayerIndex.One, Keys.D))
                direction += Right;

            if (game.HID.IsKeyDown(PlayerIndex.One, Keys.S))
                direction += -Forward;

            if (game.HID.IsKeyDown(PlayerIndex.One, Keys.W))
                direction += Forward;

            Move(direction);
        }

        private void HandleMouse()
        {
            RotateSmoothly(game.HID.MouseMovement.X, game.HID.MouseMovement.Y);
        }

        public override void LookAt(Vector3 camPosition, Vector3 camTarget)
        {
            LookAt(camPosition, camTarget, 0, null);
        }

        public override void LookAt(Vector3 camPosition, Vector3 camTarget, float interpTime, GameTime gameTime)
        {
            if (isInterpolating)
                return;

            // Set the camera position to the new position
            position = camPosition;

            // zAxis
            Vector3 v = position - camTarget;

            // Euler angles from vector v (spherical angles)
            //float l = (float)Math.Sqrt(v.X * v.X + v.Z * v.Z); 
            float l = v.X * v.X + v.Z * v.Z;

            if (l > ToolBox.EPSILON * ToolBox.EPSILON)
            {
                // Calculate current yaw based on camera forward (zAxis)
                yaw = (float)Math.Atan2(v.X, v.Z);
            }

            float pitch = (float)-Math.Atan2(v.Y, l);

            Matrix view = Matrix.CreateFromYawPitchRoll(yaw, pitch, 0);
            view.Translation = position;
            view = Matrix.Invert(view);

            if (interpTime <= ToolBox.EPSILON)
            {
                matrix_View = view;

                // Accumulated pitch and yaw degrees need to be updated with current look at Matrix values 
                // (E.G. Look at target might be elevated by 80 degrees or 20 degrees to the left)
                pitchAccum = MathHelper.ToDegrees(-pitch);
                yawAccum = MathHelper.ToDegrees(yaw);

                Quaternion.CreateFromRotationMatrix(ref matrix_View, out orientation);

                // Create the combined view-projection matrix
                Matrix.Multiply(ref matrix_View, ref matrix_Projection, out matrix_ViewProj);

                // Update the bounding frustum
                boundingFrustum.SetMatrix(matrix_ViewProj);
            }
            else
            {
                orientationStart = orientation;
                Quaternion.CreateFromRotationMatrix(ref view, out orientationEnd);
                interpolationStart = (float)gameTime.TotalGameTime.TotalSeconds;
                interpolationTime = interpTime;
                isInterpolating = true;
            }
        }

        public override void Move(Vector3 direction)
        {
            float dtSeconds = (float)game.GameTime.ElapsedGameTime.TotalSeconds;

            velocity = speed * direction;
            position += velocity * dtSeconds;
        }

        /// <summary>
        /// Set up Camera's Projection Matrix
        /// </summary>
        /// <param name="aspect_Ratio">Screen Resolution Aspect Ratio</param>
        /// <param name="z_NearPlane">Near Clipping Plane</param>
        /// <param name="z_FarPlane">Far Clipping Plane</param>
        private void Perspective(float aspect_Ratio, float z_NearClipPlane, float z_FarClipPlane)
        {
            nearClipPlaneZ = z_NearClipPlane;
            farClipPlaneZ = z_FarClipPlane;

            float yZoom = 1f / (float)Math.Tan(fov * 0.5f);
            float xZoom = yZoom / aspect_Ratio;

            matrix_Projection.M11 = xZoom;
            matrix_Projection.M12 = 0f;
            matrix_Projection.M13 = 0f;
            matrix_Projection.M14 = 0f;

            matrix_Projection.M21 = 0f;
            matrix_Projection.M22 = yZoom;
            matrix_Projection.M23 = 0f;
            matrix_Projection.M24 = 0f;

            matrix_Projection.M31 = 0f;
            matrix_Projection.M32 = 0f;
            matrix_Projection.M33 = z_FarClipPlane / (nearClipPlaneZ - farClipPlaneZ);
            matrix_Projection.M34 = -1f;

            matrix_Projection.M41 = 0f;
            matrix_Projection.M42 = 0f;
            matrix_Projection.M43 = (nearClipPlaneZ * farClipPlaneZ) / (nearClipPlaneZ - farClipPlaneZ);
            matrix_Projection.M44 = 0f;
        }

        private void Rotate(float yawDegrees, float pitchDegrees)
        {
            yawDegrees = -yawDegrees;

            if (!isLookInverted)
                pitchDegrees = -pitchDegrees;

            pitchAccum += pitchDegrees;

            // Stops the Player/Camera from looking any higher than 90 degrees [Mouse Y Movement]
            if (pitchAccum > 90.0f)
            {
                pitchDegrees = 90.0f - (pitchAccum - pitchDegrees);
                pitchAccum = 90.0f;
            }

            // Stops the Player/Camera from looking any lower than 90 degrees [Mouse Y Movement]
            if (pitchAccum < -90.0f)
            {
                pitchDegrees = -90.0f - (pitchAccum - pitchDegrees);
                pitchAccum = -90.0f;
            }

            yawAccum += yawDegrees;

            if (yawAccum > 360.0f)
                yawAccum -= 360.0f;

            if (yawAccum < -360.0f)
                yawAccum += 360.0f;

            float yaw = MathHelper.ToRadians(yawAccum);
            float pitch = MathHelper.ToRadians(pitchAccum);

            orientation =
                Quaternion.CreateFromAxisAngle(Vector3.UnitX, pitch) *
                Quaternion.CreateFromAxisAngle(Vector3.UnitY, yaw);
        }

        /// <summary>
        /// Dampens the rotation by applying the rotation speed to it
        /// </summary>
        /// <param name="yawDegrees">Y axis rotation in degrees. [Mouse X Movement]</param>
        /// <param name="pitchDegrees">X axis rotation in degrees. [Mouse Y Movement]</param>
        public void RotateSmoothly(float yawDegrees, float pitchDegrees)
        {
            yawDegrees *= lookSensitivity;
            pitchDegrees *= lookSensitivity;

            Rotate(yawDegrees, pitchDegrees);
        }

        public override CameraSettings Write()
        {
            FreeLookCameraSettings settings = new FreeLookCameraSettings();

            // Assign current camera settings
            settings.FarClip = farClipPlaneZ;
            settings.IsIgnoreInput = isIgnoreInput;
            settings.NearClip = nearClipPlaneZ;
            settings.Orientation = orientation;
            settings.Position = position;
            settings.Sensitivity = lookSensitivity;
            settings.Speed = speed;

            return settings;
        }

        public override void Read(CameraSettings _settings)
        {
            FreeLookCameraSettings settings = _settings as FreeLookCameraSettings;

            if (settings == null)
                settings = new FreeLookCameraSettings();

            farClipPlaneZ = settings.FarClip;
            fov = settings.FOV;
            isIgnoreInput = settings.IsIgnoreInput;
            lookSensitivity = settings.Sensitivity;
            nearClipPlaneZ = settings.NearClip;
            orientation = settings.Orientation;
            position = settings.Position;
            speed = settings.Speed;

            // Create Projection Matrix
            Perspective(game.GraphicsDevice.Viewport.AspectRatio, nearClipPlaneZ, farClipPlaneZ);

            // Update View Matrix
            UpdateViewMatrix();

            // Accumulated yaw and pitch will need updating because orientation and view matrix have changed
            pitchAccum = MathHelper.ToDegrees((float)Math.Asin(matrix_View.M23));
            yawAccum = MathHelper.ToDegrees((float)Math.Atan2(matrix_View.M13, matrix_View.M33));
        }

        public override void Spawn(CameraSettings _settings)
        {
            Read(_settings);

            // Create Projection Matrix
            Perspective(game.GraphicsDevice.Viewport.AspectRatio, nearClipPlaneZ, farClipPlaneZ);

            // Update View Matrix
            UpdateViewMatrix();

            // Accumulated yaw and pitch will need updating because orientation and view matrix have changed
            pitchAccum = MathHelper.ToDegrees((float)Math.Asin(matrix_View.M23));
            yawAccum = MathHelper.ToDegrees((float)Math.Atan2(matrix_View.M13, matrix_View.M33));
        }

        public override void Update(GameTime gameTime)
        {
            // Input is only handled when the camera is not interpolating
            if (!isInterpolating)
            {
                HandleInput();
            }
            else
            {
                float dt = (float)gameTime.TotalGameTime.TotalSeconds - interpolationStart;

                // Scale dt to the range of [0, 1] for slerping
                float delta = dt / interpolationTime;

                if (delta < 1f)
                {
                    orientation = Quaternion.Slerp(orientationStart, orientationEnd, delta);
                }
                else
                {
                    // Update accumulated pitch and yaw once interpolation has ended
                    pitchAccum = MathHelper.ToDegrees((float)Math.Asin(matrix_View.M23));
                    yawAccum = MathHelper.ToDegrees((float)Math.Atan2(matrix_View.M13, matrix_View.M33));

                    // Flag interpolating as false
                    isInterpolating = false;
                }
            }

            UpdateViewMatrix();
        }

        protected override void UpdatePerspective()
        {
            Perspective(game.GraphicsDevice.Viewport.AspectRatio, nearClipPlaneZ, farClipPlaneZ);
        }

        protected override void UpdateViewMatrix()
        {
            // Make our view matrix
            Matrix.CreateFromQuaternion(ref orientation, out matrix_View);

            matrix_View.M41 = -Vector3.Dot(Right, position);
            matrix_View.M42 = -Vector3.Dot(Up, position);
            matrix_View.M43 = Vector3.Dot(Forward, position);
            matrix_View.M44 = 1f;

            // Create the combined view-projection matrix
            Matrix.Multiply(ref matrix_View, ref matrix_Projection, out matrix_ViewProj);

            // Update the bounding frustum
            boundingFrustum.SetMatrix(matrix_ViewProj);
        }

        public override void Viewport(Viewport viewport)
        {
            // Create Projection Matrix
            Perspective(viewport.AspectRatio, nearClipPlaneZ, farClipPlaneZ);

            // Create the combined view-projection matrix
            Matrix.Multiply(ref matrix_View, ref matrix_Projection, out matrix_ViewProj);

            // Update the bounding frustum
            boundingFrustum.SetMatrix(matrix_ViewProj);
        }
    }
}