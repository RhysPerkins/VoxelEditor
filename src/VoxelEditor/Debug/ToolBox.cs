using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Text;

namespace VoxelEditor.Debug
{
    public enum SwitchMode : byte
    {
        Zero = 0,
        One,
        Two,
        Three,
        Four
    }

    public class ToolBox : DrawableGameComponent
    {
        public static bool Switch = false;

        public float Value = 0.0f;
        public float ValueMultiplier = 1.0f;

        const string MODE_MSG = "Press 'shift' + 'm' to change parameter display";
        const string TOGGLE_MSG = "Press 'F1' to toggle debug info";

        Dictionary<string, Func<object>> debugParams = new Dictionary<string, Func<object>>(15);
        SpriteFont font;
        int frameCount = 0;
        VoxelMeshEditorMain game;
        bool isLogging = false;                     // Enable/Disable StreamWriter logging (writes to Debug.txt file each frame)
        int modeIndex = 0;
        SpriteBatch spriteBatch;
        Stopwatch stopwatch;                        // Accurately measure elapsed time.  Useful for determining how fast a method is. [StopWatchStart() -> Execute method -> StopWatchStop()]
        StreamWriter streamWriter;
        SwitchMode switchMode = SwitchMode.Zero;
        bool wireframe = false;

        public ToolBox(VoxelMeshEditorMain game)
            : base(game)
        {
            if (game == null)
                throw new ArgumentNullException("game");

            this.game = game;

            AddParameter(this, "Value", () => Value);
            Visible = false;
        }

        public void AddParameter(object classType, string paramName, Func<object> debugParam)
        {
            paramName = string.Concat("[" + classType.GetType().Name + "] ", paramName);

            if (!debugParams.ContainsKey(paramName))
                debugParams.Add(paramName, debugParam);
            else
                debugParams[paramName] = debugParam;
        }

        public void AddParameter(object classType, int id, string paramName, Func<object> debugParam)
        {
            paramName = string.Concat("[" + classType.GetType().Name + "]" + "[ID: " + id + "] ", paramName);

            if (!debugParams.ContainsKey(paramName))
                debugParams.Add(paramName, debugParam);
            else
                debugParams[paramName] = debugParam;
        }

        public void AddParameter(string paramName, Func<object> debugParam)
        {
            if (!debugParams.ContainsKey(paramName))
                debugParams.Add(paramName, debugParam);
            else
                debugParams[paramName] = debugParam;
        }

        protected override void Dispose(bool disposing)
        {
            if (streamWriter != null)
                streamWriter.Dispose();
        }

        protected override void LoadContent()
        {
            // Load display font
            font = Game.Content.Load<SpriteFont>("Fonts\\CourierNew_10");

            // Load sprite batch for drawing font
            spriteBatch = new SpriteBatch(GraphicsDevice);

            // Create stopwatch
            stopwatch = new Stopwatch();

            // Create stream writer
            streamWriter = new StreamWriter("DEBUG.txt", false);
        }

        private void HandleInput(GameTime gameTime)
        {
            //if (!game.IsActiveApplication)
            //    return;

            HID hid = game.HID;

            bool isCtrlDown = hid.IsKeyDown(PlayerIndex.One, Keys.LeftControl) || hid.IsKeyDown(PlayerIndex.One, Keys.RightControl);
            bool isShiftDown = hid.IsKeyDown(PlayerIndex.One, Keys.LeftShift) || hid.IsKeyDown(PlayerIndex.One, Keys.RightShift);

            // Toggle visibilty of the debug parameters
            if (hid.IsKeyHit(PlayerIndex.One, Keys.F1))
                Visible = Visible ? false : true;

            // Toggle wireframe mode on/off
            if (hid.IsKeyHit(PlayerIndex.One, Keys.F2))
            {
                wireframe = wireframe ? false : true;
                GraphicsDevice.RasterizerState =
                    GraphicsDevice.RasterizerState == CustomRasterState.WireFrame ?
                    CustomRasterState.SolidCCWCull :
                    CustomRasterState.WireFrame;
            }

            // Toggle debug switch on/off
            if (hid.IsKeyHit(PlayerIndex.One, Keys.Space))
            {
                int index = (int)switchMode;
                index = ++index == Enum.GetValues(typeof(SwitchMode)).Length ? 0 : index;
                switchMode = (SwitchMode)index;

                Switch = Switch ? false : true;
            }

            if (hid.IsKeyDown(PlayerIndex.One, Keys.LeftShift) && hid.IsKeyHit(PlayerIndex.One, Keys.M))
                modeIndex = ++modeIndex == 2 ? 0 : modeIndex;

            // Course control
            if (isShiftDown && hid.IsKeyDown(PlayerIndex.One, Keys.OemMinus))
                Value -= 1 * ValueMultiplier;
            else if (isShiftDown && hid.IsKeyDown(PlayerIndex.One, Keys.OemPlus))
                Value += 1 * ValueMultiplier;

            // Fine control
            if (isCtrlDown && hid.IsKeyHit(PlayerIndex.One, Keys.OemMinus))
                Value -= 1 * ValueMultiplier;
            else if (isCtrlDown && hid.IsKeyHit(PlayerIndex.One, Keys.OemPlus))
                Value += 1 * ValueMultiplier;
        }

        public static void LogException(Exception e)
        {
#if (!XBOX && !XBOX_FAKE)
            string an = Assembly.GetEntryAssembly().Location;
            Assembly asm = Assembly.GetAssembly(typeof(VoxelMeshEditorMain));
            string path = Path.GetDirectoryName(an);
            string fn = path + "\\" + Path.GetFileNameWithoutExtension(asm.Location) + ".log";

            File.AppendAllText(
                fn,
                "////////////////////////////////////////////////////////////////\n" +
                "    Date: " + DateTime.Now.ToString() + "\n" +
                "Assembly: " + Path.GetFileName(asm.Location) + "\n" +
                " Version: " + asm.GetName().Version.ToString() + "\n" +
                " Message: " + e.Message + "\n" +
                "////////////////////////////////////////////////////////////////\n" +
                e.StackTrace + "\n" +
                "////////////////////////////////////////////////////////////////\n\n",
                Encoding.Default
                );
#endif
        }

        public static Func<object> MakeSafeFunc(object o, Func<object> func)
        {
            return () => o == null ? null : func.Invoke();
        }

        public override void Update(GameTime gameTime)
        {
            if (isLogging)
            {
                streamWriter.WriteLine("===[Frame {0}]===================================", frameCount);
                streamWriter.WriteLine();
            }

            HandleInput(gameTime);
            frameCount++;
        }

        public override void Draw(GameTime gameTime)
        {
            float width = GraphicsDevice.Viewport.Width;
            float height = GraphicsDevice.Viewport.Height;

            // Begin Sprite Batch
            spriteBatch.Begin(SpriteSortMode.Immediate, BlendState.AlphaBlend, SamplerState.PointClamp, DepthStencilState.None, CustomRasterState.SolidCCWCull);

            // Visibility toggle info
            spriteBatch.DrawString(font, TOGGLE_MSG, new Vector2(width - 252, height - 16), Color.Black);
            spriteBatch.DrawString(font, TOGGLE_MSG, new Vector2(width - 251, height - 16), Color.White);

            spriteBatch.DrawString(font, MODE_MSG, new Vector2(width - 381, 0), Color.Black);
            spriteBatch.DrawString(font, MODE_MSG, new Vector2(width - 380, 0), Color.White);

            // Debug switch status
            string string_Debug_Switch = string.Format("Debug Switch Mode: {0}", switchMode);
            spriteBatch.DrawString(font, string_Debug_Switch, new Vector2(0, height - 16), Color.Black);
            spriteBatch.DrawString(font, string_Debug_Switch, new Vector2(1, height - 16), Color.White);

            Vector2 position = new Vector2();
            position.Y = 50;

            switch (modeIndex)
            {
                case 0:
                    {
                        string stringCurrent;

                        foreach (KeyValuePair<string, Func<object>> kvp in debugParams)
                        {
                            stringCurrent = string.Format(kvp.Key + ": {0}", kvp.Value());
                            spriteBatch.DrawString(font, stringCurrent, position, Color.White);
                            position.Y += 15;
                        }

                        break;
                    }
                case 1: break;
            }

            // End Sprite Batch
            spriteBatch.End();
        }

        public void PrintAllPropertyValues(object obj)
        {
            foreach (PropertyDescriptor descriptor in TypeDescriptor.GetProperties(obj))
            {
                string name = descriptor.Name;
                object value = descriptor.GetValue(obj);
                Console.WriteLine("{0}={1}", name, value);
            }

            Console.WriteLine("=============================================");
        }

        public void SaveTextureAsPng(Texture2D texture, string name)
        {
            System.IO.FileStream s = new System.IO.FileStream(System.Windows.Forms.Application.StartupPath + name + ".png", System.IO.FileMode.Create);
            texture.SaveAsPng(s, texture.Width, texture.Height);
            s.Close();
        }

        public void SpeedComparison()
        {
            // [Method 1]
            StopwatchStart();

            for (int i = 0; i < 10000; ++i)
            {

            }

            StopwatchStop();

            // [Method 2]
            StopwatchStart();

            for (int i = 0; i < 10000; ++i)
            {

            }

            StopwatchStop();

            System.Diagnostics.Debugger.Break();
        }

        public void StopwatchStart()
        {
            stopwatch.Start();
        }

        public void StopwatchStop()
        {
            stopwatch.Stop();

            // Format and display the TimeSpan value.
            // Get the elapsed time as a TimeSpan value.
            TimeSpan ts = stopwatch.Elapsed;

            // Format and display the TimeSpan value.
            string elapsedTime = String.Format(
                "{0:00}:{1:00}:{2:00}.{3:00}.{4:00}",
                ts.Hours,
                ts.Minutes,
                ts.Seconds,
                ts.Milliseconds / 10,
                ts.Ticks
                );

            Console.WriteLine("Stopwatch Time: {0}", elapsedTime);
        }

        public void WritePixelData(Texture2D texture)
        {
            switch (texture.Format)
            {
                case SurfaceFormat.Alpha8: WritePixelData<float>(texture); break;
                case SurfaceFormat.Bgr565: WritePixelData<Vector3>(texture); break;
                case SurfaceFormat.Bgra4444: WritePixelData<Vector4>(texture); break;
                case SurfaceFormat.Bgra5551: WritePixelData<Vector4>(texture); break;
                case SurfaceFormat.Color: WritePixelData<Vector4>(texture); break;
                case SurfaceFormat.Dxt1: break;
                case SurfaceFormat.Dxt3: break;
                case SurfaceFormat.Dxt5: break;
                case SurfaceFormat.HalfSingle: WritePixelData<float>(texture); break;
                case SurfaceFormat.HalfVector2: WritePixelData<Vector2>(texture); break;
                case SurfaceFormat.HalfVector4: WritePixelData<Vector4>(texture); break;
                case SurfaceFormat.HdrBlendable: break;
                case SurfaceFormat.NormalizedByte2: break;
                case SurfaceFormat.NormalizedByte4: break;
                case SurfaceFormat.Rg32: WritePixelData<float>(texture); break;
                case SurfaceFormat.Rgba1010102: WritePixelData<Vector4>(texture); break;
                case SurfaceFormat.Rgba64: WritePixelData<Vector4>(texture); break;
                case SurfaceFormat.Single: WritePixelData<float>(texture); break;
                case SurfaceFormat.Vector2: WritePixelData<Vector2>(texture); break;
                case SurfaceFormat.Vector4: WritePixelData<Vector4>(texture); break;
            }
        }

        private void WritePixelData<T>(Texture2D texture) where T : struct
        {
            streamWriter.WriteLine("//=============================================");
            streamWriter.WriteLine("//---[Render Target]---------------------------");
            streamWriter.WriteLine("//=============================================");

            int h = texture.Height;
            int w = texture.Width;

            T[] pixelData = new T[w * h];
            texture.GetData<T>(pixelData);

            for (int i = 0; i < h; ++i)
            {
                for (int ii = 0; ii < w; ++ii)
                {
                    streamWriter.Write(pixelData[i * w + ii]);
                }

                streamWriter.Write(System.Environment.NewLine);
            }
        }

        public int FrameCount
        {
            get { return frameCount; }
        }

        /// <summary>
        /// Allows for values to be written to a text file [Creates text file called debug.txt, found in C:\]
        /// <para>
        /// How to use: Type STREAM_WRITER.Writeline() [Uses same syntax as Console.Writeline()]
        /// </para>
        /// </summary>
        /// Set to true to append file, false to overwrite
        public StreamWriter StreamWriter
        {
            get { return streamWriter; }
        }

        public SwitchMode SwitchMode
        {
            get { return switchMode; }
            set { switchMode = value; }
        }
    }
}