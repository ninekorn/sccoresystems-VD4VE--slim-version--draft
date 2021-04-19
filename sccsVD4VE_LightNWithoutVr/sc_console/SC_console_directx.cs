using SharpDX;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System;
using System.Runtime.InteropServices;
using DSystemConfiguration = sccsVD4VE_LightNWithoutVr.sc_core.sc_system_configuration;

using System.Threading;
using System.Threading.Tasks;
using SC_message_object = sc_message_object.sc_message_object;
using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;

using ISCCS_Jitter_Interface = Jitter.ISCCS_Jitter_Interface;
using Jitter;
using Jitter.LinearMath;
using System.Diagnostics;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public abstract class SC_console_directx: ISCCS_Jitter_Interface
    {
        public jitter_sc sc_create_jitter_instances(sc_jitter_data _sc_jitter_data)
        {
            return Instance;
        }

        static jitter_sc instance = null;

        public static jitter_sc Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new jitter_sc();

                }
                return instance;
            }
        }
        public jitter_sc[] create_jitter_instances(jitter_sc[] sc_jitter_physics, sc_jitter_data _sc_jitter_data)
        {
            for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
            {
                for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                {
                    for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                    {
                        var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);
                        sc_jitter_physics[indexer00] = sc_create_jitter_instances(_sc_jitter_data);
                    }
                }
            }

            return sc_jitter_physics;
        }

        public enum BodyTag
        {

            DrawMe,
            DontDrawMe,
            Terrain,
            pseudoCloth,
   

            PlayerHandLeft,
            PlayerHandRight,
            PlayerShoulderLeft,
            PlayerShoulderRight,
            PlayerTorso,
            PlayerPelvis,
            PlayerUpperArmLeft,
            PlayerLowerArmLeft,
            PlayerUpperArmRight,
            PlayerLowerArmRight,
            PlayerUpperLegLeft,
            PlayerLowerLegLeft,
            PlayerUpperLegRight,
            PlayerLowerLegRight,
            PlayerFootRight,
            PlayerFootLeft,
            PlayerHead,
            PlayerLeftElbowTarget,
            PlayerRightHandGrabTarget,
            PlayerLeftHandGrabTarget,

            PlayerRightElbowTarget,
            PlayerLeftElbowTargettwo,
            PlayerRightElbowTargettwo,
            PlayerLeftTargetKnee,
            PlayerRightTargetKnee,
            PlayerLeftTargettwoKnee,
            PlayerRightTargettwoKnee,


            sc_containment_grid,
            sc_grid,
      
            Screen,
            sc_jitter_cloth,
            //someothertest,
            //testChunkCloth,
            //cloth_cube,
            //screen_corners,
            //screen_pointer_touch,
            //screen_pointer_HMD,
            _terrain_tiles,
            _terrain,
            _floor,
            //_icosphere,
            //_sphere,
            _spectrum,
            //_physics_cube_group_b,
            _screen_assets,


            physicsInstancedCube,
            physicsInstancedCone,
            physicsInstancedCylinder,
            physicsInstancedCapsule,
            physicsInstancedSphere,

            sc_perko_voxel,
            physicsInstancedScreen,
        }


        Thread main_thread_update;

        public Matrix OrthoMatrix { get; private set; }
        public SharpDX.Direct3D11.Device device { get; set; }

        [DllImport("User32.dll", CharSet = CharSet.Unicode)]
        public static extern int MessageBox(IntPtr h, string m, string c, int type);

        //OCULUS RIFT
        public bool _useOculusRift = true;
        public int SurfaceWidth;
        public int SurfaceHeight;
        public DateTime startTime;
        //public OculusWrapVirtualRealityProvider _oculusRiftVirtualRealityProvider;
        //public static Ab3d.DirectX.DXDevice _dxDevice;
        private RenderTargetView _renderTargetView;
        SharpDX.Direct3D11.Texture2D depthBuffer;
        private DepthStencilView _depthStencilView;
        protected DepthStencilView DepthStencilView => _depthStencilView;
        SharpDX.Direct3D11.DepthStencilState depthStencilState;
        //MirrorTexture mirrorTexture = null;
        Guid textureInterfaceId = new Guid("6f15aaf2-d208-4e89-9ab4-489535d34f9c"); // Interface ID of the Direct3D Texture2D interface.

        // Properties.
        public bool VerticalSyncEnabled { get; set; }
        public int VideoCardMemory { get; private set; }
        public string VideoCardDescription { get; private set; }
        public SwapChain SwapChain { get; set; }
        public SharpDX.Direct3D11.Device Device { get; private set; }
        public DeviceContext DeviceContext { get; private set; }
        public Texture2D DepthStencilBuffer { get; set; }
        public DepthStencilState _depthStencilState { get; set; }
        public RasterizerState RasterState { get; set; }
        public Matrix ProjectionMatrix { get; private set; }
        //public OvrWrap OVR;
        //public HmdDesc hmdDesc;
        public IntPtr sessionPtr;
        public Result result;
        //public LayerEyeFov layerEyeFov;
        //public EyeTexture[] eyeTextures;
        public Texture2D BackBuffer;
        public SharpDX.Direct3D11.Texture2D mirrorTextureD3D;

        //public ControllerType controllerTypeRTouch;
        //public ControllerType controllerTypeLTouch;
        //public Ab3d.OculusWrap.InputState inputStateLTouch;
        //public Ab3d.OculusWrap.InputState inputStateRTouch;


        public static SC_console_directx D3D;

        public DepthStencilState DepthDisabledStencilState { get; private set; }
        public BlendState AlphaEnableBlendingState { get; private set; }
        public BlendState AlphaDisableBlendingState { get; private set; }
        public DepthStencilState DepthStencilState { get; private set; }


        // Constructor
        /*public SC_console_directx()
        {

        }*/

        protected SC_console_directx() //DSystemConfiguration configuration, IntPtr Hwnd, sc_console.sc_console_writer _writer
        {
            D3D = this;
            //Update();
            SC_Init_DirectX(); //configuration, Hwnd, _writer 
        }




        protected virtual void SC_Init_DirectX() //DSystemConfiguration configuration, IntPtr Hwnd, sc_console.sc_console_writer _writer
        {
            //MessageBox((IntPtr)0,"" + MainWindow.handler, "sccsVD4VE_LightNWithoutVr Error", 0);

            //sc_graphics_sec _graphics_sec
            // Methods
            //public bool Initialize(DSystemConfiguration configuration, IntPtr Hwnd,sc_console.sc_console_writer _writer)
            //{
            try
            {
                startTime = DateTime.Now;
                //var dpiScale = GetDpiScale();

                using (var _factory = new Factory1())
                {
                    var _adapter = _factory.GetAdapter1(0);

                    using (var _output = _adapter.GetOutput(0))
                    {
                        SurfaceWidth = ((SharpDX.Rectangle)_output.Description.DesktopBounds).Width;
                        SurfaceHeight = ((SharpDX.Rectangle)_output.Description.DesktopBounds).Height;
                    }

                }
                //return true;
            }
            catch
            {
                //return false;
            }








            /*
            // Initialize and set up the description of the depth buffer.
            var depthBufferDesc = new Texture2DDescription()
            {
                Width = MainWindow.config.Width,
                Height = MainWindow.config.Height,
                MipLevels = 1,
                ArraySize = 1,
                Format = Format.D24_UNorm_S8_UInt,
                SampleDescription = new SampleDescription(1, 0),
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.DepthStencil,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None
            };

            // Create the texture for the depth buffer using the filled out description.
            DepthStencilBuffer = new Texture2D(device, depthBufferDesc);


            
            
            
            
            // Initialize and set up the description of the stencil state.
            var depthStencilDesc = new DepthStencilStateDescription()
            {
                IsDepthEnabled = true,
                DepthWriteMask = DepthWriteMask.All,
                DepthComparison = Comparison.Less,
                IsStencilEnabled = true,
                StencilReadMask = 0xFF,
                StencilWriteMask = 0xFF,
                // Stencil operation if pixel front-facing.
                FrontFace = new DepthStencilOperationDescription()
                {
                    FailOperation = StencilOperation.Keep,
                    DepthFailOperation = StencilOperation.Increment,
                    PassOperation = StencilOperation.Keep,
                    Comparison = Comparison.Always
                },
                // Stencil operation if pixel is back-facing.
                BackFace = new DepthStencilOperationDescription()
                {
                    FailOperation = StencilOperation.Keep,
                    DepthFailOperation = StencilOperation.Decrement,
                    PassOperation = StencilOperation.Keep,
                    Comparison = Comparison.Always
                }
            };

            // Create the depth stencil state.
            DepthStencilState = new DepthStencilState(Device, depthStencilDesc);








            //STRAIGHT COPY PASTE FROM C# RASTERTEK DAN6040. ALL CREDITS TO HIM. WOW HE IS SUCH A GOOD SCRIPTER. I AM MISSING TIME.

            // Create an orthographic projection matrix for 2D rendering.
            OrthoMatrix = Matrix.OrthoLH(MainWindow.config.Width, MainWindow.config.Height, DSystemConfiguration.ScreenNear, DSystemConfiguration.ScreenDepth);



            // Now create a second depth stencil state which turns off the Z buffer for 2D rendering. Added in Tutorial 11
            // The difference is that DepthEnable is set to false.
            // All other parameters are the same as the other depth stencil state.
            var depthDisabledStencilDesc = new DepthStencilStateDescription()
            {
                IsDepthEnabled = false,
                DepthWriteMask = DepthWriteMask.All,
                DepthComparison = Comparison.Less,
                IsStencilEnabled = true,
                StencilReadMask = 0xFF,
                StencilWriteMask = 0xFF,
                // Stencil operation if pixel front-facing.
                FrontFace = new DepthStencilOperationDescription()
                {
                    FailOperation = StencilOperation.Keep,
                    DepthFailOperation = StencilOperation.Increment,
                    PassOperation = StencilOperation.Keep,
                    Comparison = Comparison.Always
                },
                // Stencil operation if pixel is back-facing.
                BackFace = new DepthStencilOperationDescription()
                {
                    FailOperation = StencilOperation.Keep,
                    DepthFailOperation = StencilOperation.Decrement,
                    PassOperation = StencilOperation.Keep,
                    Comparison = Comparison.Always
                }
            };

            // Create the depth stencil state.
            DepthDisabledStencilState = new DepthStencilState(Device, depthDisabledStencilDesc);














            // Create an alpha enabled blend state description.
            var blendStateDesc = new BlendStateDescription();
            blendStateDesc.RenderTarget[0].IsBlendEnabled = true;
            blendStateDesc.RenderTarget[0].SourceBlend = BlendOption.SourceAlpha;
            blendStateDesc.RenderTarget[0].DestinationBlend = BlendOption.InverseSourceAlpha;
            blendStateDesc.RenderTarget[0].BlendOperation = BlendOperation.Add;
            blendStateDesc.RenderTarget[0].SourceAlphaBlend = BlendOption.One;
            blendStateDesc.RenderTarget[0].DestinationAlphaBlend = BlendOption.Zero;
            blendStateDesc.RenderTarget[0].AlphaBlendOperation = BlendOperation.Add;
            blendStateDesc.RenderTarget[0].RenderTargetWriteMask = ColorWriteMaskFlags.All;

            // Create the blend state using the description.
            AlphaEnableBlendingState = new BlendState(device, blendStateDesc);

            // Modify the description to create an disabled blend state description.
            blendStateDesc.RenderTarget[0].IsBlendEnabled = false;

            // Create the blend state using the description.
            AlphaDisableBlendingState = new BlendState(device, blendStateDesc);
            */
























            try
            {
                main_thread_update = new Thread(() =>
                {
                    jitter_sc[] jitter_sc = new jitter_sc[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z];
                    SC_message_object_jitter[][] _sc_jitter_tasks = new SC_message_object_jitter[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][];

                    sc_jitter_data _sc_jitter_data = new sc_jitter_data();
                    _sc_jitter_data.alloweddeactivation = MainWindow._allow_deactivation;
                    _sc_jitter_data.allowedpenetration = MainWindow._world_allowed_penetration;
                    _sc_jitter_data.width = MainWindow.world_width;
                    _sc_jitter_data.height = MainWindow.world_height;
                    _sc_jitter_data.depth = MainWindow.world_depth;
                    _sc_jitter_data.gravity = MainWindow._world_gravity; 
                    _sc_jitter_data.smalliterations = MainWindow._world_small_iterations;
                    _sc_jitter_data.iterations = MainWindow._world_iterations;


                    for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
                    {
                        for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                        {
                            for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                            {
                                var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);
                                //_jitter_physics[indexer00] = DoSpecialThing();
                                _sc_jitter_tasks[indexer00] = new SC_message_object_jitter[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth];

                                for (int x = 0; x < MainWindow.world_width; x++)
                                {
                                    for (int y = 0; y < MainWindow.world_height; y++)
                                    {
                                        for (int z = 0; z < MainWindow.world_depth; z++)
                                        {
                                            var indexer01 = x + MainWindow.world_width * (y + MainWindow.world_height * z);
                                            _sc_jitter_tasks[indexer00][indexer01] = new SC_message_object_jitter();
                                        }
                                    }
                                }
                            }
                        }
                    }



                    jitter_sc = create_jitter_instances(jitter_sc, _sc_jitter_data);

                    for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
                    {
                        for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                        {
                            for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                            {
                                var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);

                                jitter_sc[indexer00]._sc_create_jitter_world(_sc_jitter_data);

                                for (int x = 0; x < MainWindow.world_width; x++)
                                {
                                    for (int y = 0; y < MainWindow.world_height; y++)
                                    {
                                        for (int z = 0; z < MainWindow.world_depth; z++)
                                        {

                                            var indexer1 = x + MainWindow.world_width * (y + MainWindow.world_height * z);

                                            var world = jitter_sc[indexer00].return_world(indexer1);

                                            if (world == null)
                                            {
                                                Console.WriteLine("null");
                                            }
                                            else
                                            {
                                                //Console.WriteLine("!null");

                                                _sc_jitter_tasks[indexer00][indexer1]._world_data = new object[2];
                                                _sc_jitter_tasks[indexer00][indexer1]._work_index = -1;
                                                _sc_jitter_tasks[indexer00][indexer1]._world_data[0] = world;
                                                //Console.WriteLine("index: " + indexer1);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    _sc_jitter_tasks = init_update_variables(_sc_jitter_tasks, MainWindow.config, MainWindow.consoleHandle, MainWindow.SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER);

                _thread_looper:

                    try
                    {
                        _sc_jitter_tasks = Update(jitter_sc, _sc_jitter_tasks);
                    }
                    catch (Exception ex)
                    {

                    }
                    Thread.Sleep(0);
                    goto _thread_looper;

                    //ShutDown();
                    //ShutDownGraphics();

                }, 0);

                main_thread_update.IsBackground = true;
                main_thread_update.SetApartmentState(ApartmentState.STA);
                main_thread_update.Start();

            }
            catch
            {

            }
            /*finally
            {

            }*/
        }

        public void TurnOnAlphaBlending()
        {
            // Setup the blend factor.
            var blendFactor = new Color4(0, 0, 0, 0);

            // Turn on the alpha blending.
            DeviceContext.OutputMerger.SetBlendState(AlphaEnableBlendingState, blendFactor, -1);
        }

        public void TurnOffAlphaBlending()
        {
            // Setup the blend factor.
            var blendFactor = new Color4(0, 0, 0, 0);

            // Turn on the alpha blending.
            DeviceContext.OutputMerger.SetBlendState(AlphaDisableBlendingState, blendFactor, -1);
        }

        public void TurnZBufferOn()
        {
            DeviceContext.OutputMerger.SetDepthStencilState(DepthStencilState, 1);
        }

        public void TurnZBufferOff()
        {
            DeviceContext.OutputMerger.SetDepthStencilState(DepthDisabledStencilState, 1);
        }









        protected abstract SC_message_object_jitter[][] init_update_variables(SC_message_object_jitter[][] _sc_jitter_tasks, sccsVD4VE_LightNWithoutVr.sc_core.sc_system_configuration configuration, IntPtr hwnd, sc_console.sc_console_writer _writer); //void Update();
        protected abstract SC_message_object_jitter[][] Update(jitter_sc[] jitter_sc, SC_message_object_jitter[][] _sc_jitter_tasks); //void Update();
        protected abstract void ShutDownGraphics();


        public void ShutDown()
        {
            // Before shutting down set to windowed mode or when you release the swap chain it will throw an exception.   
            SwapChain?.SetFullscreenState(false, null);
            RasterState?.Dispose();
            RasterState = null;
            depthStencilState?.Dispose();
            depthStencilState = null;
            DepthStencilBuffer?.Dispose();
            DepthStencilBuffer = null;
            _depthStencilView?.Dispose();
            _depthStencilView = null;
            _renderTargetView?.Dispose();
            _renderTargetView = null;
            DeviceContext?.Dispose();
            Device?.Dispose();
            SwapChain?.Dispose();



            AlphaEnableBlendingState?.Dispose();
            AlphaEnableBlendingState = null;
            AlphaDisableBlendingState?.Dispose();
            AlphaDisableBlendingState = null;
            DepthDisabledStencilState?.Dispose();
            DepthDisabledStencilState = null;
            //DepthStencilView?.Dispose();
            //DepthStencilView = null;
            DepthStencilState?.Dispose();
            DepthStencilState = null;
            DepthStencilBuffer?.Dispose();
            DepthStencilBuffer = null;


            if (main_thread_update != null)
            {
                //main_thread_update.Suspend();
                main_thread_update = null;
            }
            ShutDownGraphics();
        }





        /*
        public void WriteErrorDetails(OvrWrap OVR, Ab3d.OculusWrap.Result result, string message)
        {
            if (result >= Ab3d.OculusWrap.Result.Success)
                return;

            ErrorInfo errorInformation = OVR.GetLastErrorInfo();

            string formattedMessage = string.Format("{0}. \nMessage: {1} (Error code={2})", message, errorInformation.ErrorString, errorInformation.Result);
            //Trace.WriteLine(formattedMessage);
            //System.Windows.Forms.MessageBox.Show(formattedMessage, message);

            throw new Exception(formattedMessage);
        }*/
    }
}