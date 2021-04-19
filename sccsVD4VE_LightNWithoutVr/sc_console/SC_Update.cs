using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Linq;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

/*using Ab3d.DXEngine;
using Ab3d.OculusWrap;
using Ab3d.DXEngine.OculusWrap;
using Ab3d.OculusWrap.DemoDX11;*/

using SharpDX;
using SharpDX.DXGI;
using SharpDX.Direct3D11;
using SharpDX.DirectInput;

using sccsVD4VE_LightNWithoutVr.SC_Graphics;
using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_ShaderManager;

using Jitter;
using Jitter.Dynamics;
using Jitter.Collision;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Forces;

using System.Collections.Generic;
using System.Collections;
using System.Runtime;
using System.Runtime.CompilerServices;

using System.ComponentModel;
using SharpDX.D3DCompiler;

using SC_message_object = sc_message_object.sc_message_object;
using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;

using ISCCS_Jitter_Interface = Jitter.ISCCS_Jitter_Interface;
using Jitter;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public class SC_Update : SC_console_directx
    {

        public SC_Update()
        {

        }



        public static Matrix hmdmatrixRot = Matrix.Identity;

        BackgroundWorker BackgroundWorker_00;


        protected override void ShutDownGraphics()
        {
            if (BackgroundWorker_00 != null)
            {
                BackgroundWorker_00.Dispose();
            }
            /*if (_Keyboard != null)
            {
                _Keyboard.Dispose();
            }
            if (_KeyboardState != null)
            {
                _KeyboardState = null;
            }*/

            if (_desktopFrame._ShaderResource != null)
            {
                _desktopFrame._ShaderResource = null;
            }

            if (_desktopDupe._lastShaderResourceView != null)
            {
                _desktopDupe._lastShaderResourceView = null;
            }
            if (_desktopDupe != null)
            {
                _desktopDupe = null;
            }
        }


        /*protected override void SC_Init_DirectX()
        {

            base.SC_Init_DirectX();
        }*/
        /*public override SC_message_object_jitter[][] sc_write_to_buffer(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }

        public override SC_message_object_jitter[][] loop_worlds(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }
        public override SC_message_object_jitter[][] workOnSomething(SC_message_object_jitter[][] _sc_jitter_tasks, Matrix viewMatrix, Matrix projectionMatrix, Vector3 OFFSETPOS, Matrix originRot, Matrix rotatingMatrix, Matrix rotatingMatrixForPelvis, Matrix _rightTouchMatrix, Matrix _leftTouchMatrix, Posef handPoseRight, Posef handPoseLeft)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }
        public override SC_message_object_jitter[][] _sc_create_world_objects(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }*/

        public static SharpDX.Vector3 originPos = new SharpDX.Vector3(0, 1, 3);
        public static SharpDX.Vector3 originPosScreen = new SharpDX.Vector3(0, 1, -0.25f);

        float disco_sphere_rot_speed = 0.5f;

        float speedRot = 0.15f;
        float speedPos = 0.05f;

        int _start_background_worker_01 = 0;
        sc_graphics_main _the_graphics;

        float _delta_timer_frame = 0;
        float _delta_timer_time = 0;
        DateTime time1;
        DateTime time2;
        float deltaTime;
        Stopwatch timeStopWatch00 = new Stopwatch();
        Stopwatch timeStopWatch01 = new Stopwatch();
        int _swtch = 0;
        int _swtch_counter_00 = 0;
        int _swtch_counter_01 = 0;
        int _swtch_counter_02 = 0;

        public void DoWork(int timeOut) //async Task
        {
            float startTime = (float)(timeStopWatch00.ElapsedMilliseconds);
        _threadLoop:

            if (_swtch == 0 || _swtch == 1)
            {
                if (_swtch == 0)
                {
                    if (_swtch_counter_00 >= 0)
                    {
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        //IMPORTANT PHYSICS TIME 
                        timeStopWatch00.Start();
                        time1 = DateTime.Now;
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        _swtch = 1;
                        _swtch_counter_00 = 0;
                    }
                }
                else if (_swtch == 1)
                {
                    if (_swtch_counter_01 >= 0)
                    {
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        timeStopWatch01.Start();
                        time2 = DateTime.Now;
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        _swtch = 2;
                        _swtch_counter_01 = 0;
                    }
                }
                else if (_swtch == 2)
                {

                }
            }

            /*//FRAME DELTATIME
            _delta_timer_frame = (float)Math.Abs((timeStopWatch01.Elapsed.Ticks - timeStopWatch00.Elapsed.Ticks)) * 100000000f;

            time2 = DateTime.Now;
            _delta_timer_time = (time2.Ticks - time1.Ticks) * 100000000f; //100000000f
            //time1 = time2;

            deltaTime = (float)Math.Abs(_delta_timer_time - _delta_timer_frame);
            */

            //FRAME DELTATIME
            _delta_timer_frame = (float)Math.Abs((timeStopWatch01.Elapsed.Ticks - timeStopWatch00.Elapsed.Ticks)); //10000000000f

            time2 = DateTime.Now;
            _delta_timer_time = (time2.Ticks - time1.Ticks); //100000000f //10000000000f
            time1 = time2;

            deltaTime = (float)Math.Abs(_delta_timer_time-_delta_timer_frame);

            //time1 = time2;
            //await Task.Delay(1);
            //Thread.Sleep(timeOut);
            _swtch_counter_00++;
            _swtch_counter_01++;
            _swtch_counter_02++;

            goto _threadLoop;
        }

        /*public SC_Console_GRAPHICS(sc_graphics_main _graphics)
        {
            _the_graphics = _graphics;
        }*/

        public void ShutDown()
        {

        }

        Matrix rotatingMatrixForGrabber = Matrix.Identity;
        int _sec_logic_swtch_grab = 0;

        int _swtch_hasRotated = 0;
        int _has_grabbed_right_swtch = 0;
        public static double RotationY { get; set; }
        public static double RotationX { get; set; }
        public static double RotationZ { get; set; }
        float thumbstickIsRight;
        float thumbstickIsUp;

        int RotationGrabbedSwtch = 0;

        public static double RotationY4Pelvis;
        public static double RotationX4Pelvis;
        public static double RotationZ4Pelvis;

        public static double RotationY4PelvisTwo;
        public static double RotationX4PelvisTwo;
        public static double RotationZ4PelvisTwo;


        public static double RotationGrabbedYOff;
        public static double RotationGrabbedXOff;
        public static double RotationGrabbedZOff;


        public static double RotationGrabbedY;
        public static double RotationGrabbedX;
        public static double RotationGrabbedZ;


        /*//OCULUS TOUCH SETTINGS 
        Ab3d.OculusWrap.Result resultsRight;
        uint buttonPressedOculusTouchRight;
        Vector2f[] thumbStickRight;
        public static float[] handTriggerRight;
        float[] indexTriggerRight;
        Ab3d.OculusWrap.Result resultsLeft;
        uint buttonPressedOculusTouchLeft;
        Vector2f[] thumbStickLeft;
        public static float[] handTriggerLeft;
        public static float[] indexTriggerLeft;
        Posef handPoseLeft;
        SharpDX.Quaternion _leftTouchQuat;
        Posef handPoseRight;
        SharpDX.Quaternion _rightTouchQuat;
        Matrix _leftTouchMatrix = Matrix.Identity;
        Matrix _rightTouchMatrix = Matrix.Identity;
        //OCULUS TOUCH SETTINGS */

        float offsetPosX = 0.0f;
        float offsetPosY = 0.0f;
        float offsetPosZ = 0.0f;

        /*double displayMidpoint;
        TrackingState trackingState;
        Posef[] eyePoses;
        EyeType eye;
        EyeTexture eyeTexture;
        bool latencyMark = false;
        TrackingState trackState;
        PoseStatef poseStatefer;
        Posef hmdPose;
        Quaternionf hmdRot;
        Vector3 _hmdPoser;
        Quaternion _hmdRoter;*/

        Vector3 intersectPointRight;
        Vector3 intersectPointLeft;
        Matrix final_hand_pos_right_locked;
        Matrix final_hand_pos_left_locked;

        _sc_camera Camera;

        int _failed_screen_capture = 0;
        public static SC_ShaderManager _shaderManager;
        public  int _can_work_physics_objects = 0;

        public static IntPtr HWND;
        sccsVD4VE_LightNWithoutVr.sc_core.sc_system_configuration _configuration;
        sc_console.sc_console_writer _currentWriter;
        public static SC_SharpDX_ScreenFrame _desktopFrame;

        float xq;//= otherQuat.X;
        float yq;//= otherQuat.Y;
        float zq;//= otherQuat.Z;
        float wq;//= otherQuat.W;
        float pitcha;//= (float) Math.Atan2(2 * yq* wq - 2 * xq* zq, 1 - 2 * yq* yq - 2 * zq* zq); //(float)(180 / Math.PI)
        float yawa;//= (float) Math.Atan2(2 * yq* wq - 2 * xq* zq, 1 - 2 * yq* yq - 2 * zq* zq); //(float)(180 / Math.PI) *
        float rolla;// = (float) Math.Atan2(2 * yq* wq - 2 * xq* zq, 1 - 2 * yq* yq - 2 * zq* zq); // (float)(180 / Math.PI) *
        float hyp;// = diffNormPosY / Math.Cos(pitcha);

        int textureIndex;
        SharpDX.Vector3 eyePos;
        SharpDX.Matrix eyeQuaternionMatrix;
        SharpDX.Matrix finalRotationMatrix;
        Vector3 lookUp;
        Vector3 lookAt;
        Vector3 viewPosition;
        Matrix viewMatrix;
        Matrix _projectionMatrix;
        public static Vector3 OFFSETPOS;

        public static SharpDX.Vector3 movePos = new SharpDX.Vector3(0, 0, 0);
        public static SharpDX.Matrix originRot = SharpDX.Matrix.Identity;

        public static SharpDX.Matrix rotatingMatrixForPelvis = SharpDX.Matrix.Identity;
        public static SharpDX.Matrix rotatingMatrix = SharpDX.Matrix.Identity;
        float r = 0;
        float g = 0;
        float b = 0;
        float a = 1;

        Matrix WorldMatrix = Matrix.Identity;
        public static SC_SharpDX_ScreenCapture _desktopDupe;


        public static DateTime startTime;

        protected override SC_message_object_jitter[][] init_update_variables(SC_message_object_jitter[][] _sc_jitter_tasks, sccsVD4VE_LightNWithoutVr.sc_core.sc_system_configuration configuration, IntPtr hwnd, sc_console.sc_console_writer _writer)
        {
            try
            {
                startTime = DateTime.Now;


                HWND = hwnd;

                _configuration = configuration;

                _currentWriter = _writer;


                //ReadKeyboard();
                //Camera = new DCamera();

                //swtch_for_last_pos = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][];

                //RAYCAST STUFF
                //_some_reset_for_applying_force = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][];
                //_some_last_frame_vector = new JVector[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
                //_some_last_frame_rigibodies = new RigidBody[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];


      
                Thread main_thread_update = new Thread(() =>
                {
                 
                _thread_looper:

                    try
                    {
                        DoWork(0);
                    }
                    catch (Exception ex)
                    {

                    }
                    Thread.Sleep(1);
                    goto _thread_looper;

                    //ShutDown();
                    //ShutDownGraphics();

                }, 0);

                main_thread_update.IsBackground = true;
                main_thread_update.SetApartmentState(ApartmentState.STA);
                main_thread_update.Start();

                Camera = new _sc_camera();

                _shaderManager = new SC_ShaderManager();
                _shaderManager.Initialize(D3D.Device, HWND);

                _desktopDupe = new SC_SharpDX_ScreenCapture(0, 0, D3D.device);

                _graphics_sec = new sc_graphics_sec();
                _sc_jitter_tasks = _graphics_sec._sc_create_world_objects(_sc_jitter_tasks);
            }
            catch
            {

            }
            return _sc_jitter_tasks;
        }

        sc_graphics_sec _graphics_sec;


        /*protected override void Update()
        {

        }*/



        protected override SC_message_object_jitter[][] Update(jitter_sc[] jitter_sc, SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            if (_shaderManager != null)
            {
                // Render the graphics scene.
                try
                {
                    _sc_jitter_tasks = _FrameVRTWO(jitter_sc, _sc_jitter_tasks);
                }
                catch (Exception ex)
                {
                    MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sc core systems message", 0);
                }
            }
            else
            {
                //MessageBox((IntPtr)0, "" + ex.ToString(), "sc core systems message", 0);
            }
            return _sc_jitter_tasks;
        }

        int _last_frame_init = 0;

        private unsafe SC_message_object_jitter[][] _FrameVRTWO(jitter_sc[] jitter_sc, SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            for (int i = 0; i < 3;)
            {
                _failed_screen_capture = 0;
                try
                {
                    _desktopFrame = _desktopDupe.ScreenCapture(0);
                }
                catch (Exception ex)
                {
                    _desktopDupe = new SC_SharpDX_ScreenCapture(0, 0, D3D.device);
                    _failed_screen_capture = 1;
                }
                i++;
                if (_failed_screen_capture == 0)
                {
                    break;
                }
            }
            

            /*Matrix tempmat = hmd_matrix * rotatingMatrixForPelvis * hmdmatrixRot;

            Quaternion otherQuat;
            Quaternion.RotationMatrix(ref tempmat, out otherQuat);


            Vector3 direction_feet_forward;
            Vector3 direction_feet_right;
            Vector3 direction_feet_up;

            direction_feet_forward = sc_maths._getDirection(Vector3.ForwardRH, otherQuat);
            direction_feet_right = sc_maths._getDirection(Vector3.Right, otherQuat);
            direction_feet_up = sc_maths._getDirection(Vector3.Up, otherQuat);

            if (thumbStickLeft[0].X > 0.15f)
            {
                movePos += direction_feet_right * speedPos * thumbStickLeft[0].X;
            }
            else if (thumbStickLeft[0].X < -0.15f)
            {
                movePos -= direction_feet_right * speedPos * -thumbStickLeft[0].X;
            }

            if (thumbStickLeft[0].Y > 0.15f)
            {
                movePos += direction_feet_forward * speedPos * thumbStickLeft[0].Y;
            }
            else if (thumbStickLeft[0].Y < -0.15f)
            {
                movePos -= direction_feet_forward * speedPos * -thumbStickLeft[0].Y;
            }

            //Vector3 resulter;
            //Vector3.TransformCoordinate(ref _hmdPoser, ref WorldMatrix, out resulter);
            //var someMatrix = hmd_matrix * finalRotationMatrix;

            OFFSETPOS = originPos + movePos;// + _hmdPoser; //_hmdPoser
            //OFFSETPOS.Y += _hmdPoser.Y;*/






            //START OF PHYSICS ENGINE STEPS
            if (_start_background_worker_01 == 0)
            {
                if (_can_work_physics == 1)
                {
                    var main_thread_update = new Thread(() =>
                    {
                        try
                        {
                            //MainWindow.MessageBox((IntPtr)0, "threadstart succes", "sc core systems message", 0);
                            Stopwatch _this_thread_ticks_01 = new Stopwatch();
                            _this_thread_ticks_01.Start();

                        _thread_looper:

                            for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
                            {
                                for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                                {
                                    for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                                    {
                                        var indexer0 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);

                                        try
                                        {
                                            for (int x = 0; x < MainWindow.world_width; x++)
                                            {
                                                for (int y = 0; y < MainWindow.world_height; y++)
                                                {
                                                    for (int z = 0; z < MainWindow.world_depth; z++)
                                                    {
                                                        var indexer1 = x + MainWindow.world_width * (y + MainWindow.world_height * z);
                                                        object _some_data_00 = (object)_sc_jitter_tasks[indexer0][indexer1]._world_data[0];
                                                        World _jitter_world = (World)_some_data_00;

                                                        if (_jitter_world != null)
                                                        {
                                                            /*if (deltaTime > 1.0f * 0.0075f)
                                                            {
                                                                deltaTime = 1.0f * 0.0075f;
                                                            }
                                                            else if (deltaTime < 1.0f * 0.005f)
                                                            {
                                                                deltaTime = 1.0f * 0.005f;
                                                            }*/
                                                            if (deltaTime > 1.0f * 0.01f)
                                                            {
                                                                deltaTime = 1.0f * 0.01f;
                                                            }

                                                            _jitter_world.Step(deltaTime, true);
                                                        }

                                                    }
                                                }
                                            }
                                        }
                                        catch
                                        {

                                        }
                                    }
                                }
                            }
                            //MessageBox((IntPtr)0, _this_thread_ticks.ElapsedTicks + "", "sc core systems message", 0);
                            //Console.WriteLine("ticks: " + _this_thread_ticks.ElapsedTicks);
                            Thread.Sleep(1);
                            goto _thread_looper;
                        }
                        catch (Exception ex)
                        {

                        }

                    }, 0);

                    main_thread_update.IsBackground = true;
                    //main_thread_update.SetApartmentState(ApartmentState.STA);
                    main_thread_update.Start();








                    /*
                    var tasker = new Task(() =>
                    {

                    });
                    tasker.Start();*/









                    /*BackgroundWorker_00 = new BackgroundWorker();
                    BackgroundWorker_00.DoWork += (object sender, DoWorkEventArgs argers) =>
                    {
                        
                    };

                    BackgroundWorker_00.RunWorkerCompleted += delegate (object sender, RunWorkerCompletedEventArgs argers)
                    {

                    };

                    BackgroundWorker_00.RunWorkerAsync();*/







                    BackgroundWorker_00 = new BackgroundWorker();
                    BackgroundWorker_00.DoWork += (object sender, DoWorkEventArgs argers) =>
                    {
                        //MainWindow.MessageBox((IntPtr)0, "threadstart succes", "sc core systems message", 0);
                        Stopwatch _this_thread_ticks_01 = new Stopwatch();
                        _this_thread_ticks_01.Start();

                    _thread_looper:

                        //_sc_jitter_tasks = _graphics_sec.loop_worlds(_sc_jitter_tasks, originRot, rotatingMatrix, hmdmatrixRot, hmd_matrix, rotatingMatrixForPelvis, _rightTouchMatrix, _leftTouchMatrix);
                        _sc_jitter_tasks = _graphics_sec.loop_worlds(_sc_jitter_tasks, originRot, rotatingMatrix, hmdmatrixRot, Matrix.Identity, rotatingMatrixForPelvis, Matrix.Identity, Matrix.Identity);

                        //_ticks_watch.Restart();

                        //Console.WriteLine(_ticks_watch.Elapsed.Ticks);
                        Thread.Sleep(1);
                        goto _thread_looper;
                    };

                    BackgroundWorker_00.RunWorkerCompleted += delegate (object sender, RunWorkerCompletedEventArgs argers)
                    {

                    };

                    BackgroundWorker_00.RunWorkerAsync();



                    _start_background_worker_01 = 1;
                }
            }
            //END OF PHYSICS ENGINE STEPS



            try
            {
                if (_can_work_physics == 1)
                {

                    _sc_jitter_tasks = _graphics_sec.sc_write_to_buffer(_sc_jitter_tasks);
                }
            }
            catch (Exception ex)
            {
                MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sc core systems message", 0);
            }




            /*if (writetobuffer == 0)
            {
                if (_can_work_physics == 1)
                {
                    
                }

                writetobuffer = 1;
            }*/


            /*
            D3D.result = D3D.OVR.SubmitFrame(D3D.sessionPtr, 0L, IntPtr.Zero, ref D3D.layerEyeFov);
            D3D.WriteErrorDetails(D3D.OVR, D3D.result, "Failed to submit the frame of the current layers.");
            D3D.DeviceContext.CopyResource(D3D.mirrorTextureD3D, D3D.BackBuffer);
            //D3D.SwapChain.Present(0, PresentFlags.None); //crap visuals returning to only spheroids.
            */













            //###SC start physics on frame 1 instead of 0
            _can_work_physics = 1;
            _can_work_physics_objects = 1;
            //###SC start physics on frame 1 instead of 0
            return _sc_jitter_tasks;
        }

        int writetobuffer = 0;







        Stopwatch _ticks_watch = new Stopwatch();
        public int _can_work_physics = 0;

        /*public KeyboardState _KeyboardState;
        public SharpDX.DirectInput.Keyboard _Keyboard;
        DirectInput directInput;*/

        /*private bool ReadKeyboard()
        {
            directInput = new DirectInput();

            _Keyboard = new Keyboard(directInput);

            //Acquire the joystick
            _Keyboard.Properties.BufferSize = 128;

            var resultCode = SharpDX.DirectInput.ResultCode.Ok;
            _KeyboardState = new KeyboardState();

            try
            {
                // Read the keyboard device.
                _Keyboard.GetCurrentState(ref _KeyboardState);
            }
            catch (SharpDX.SharpDXException ex)
            {
                resultCode = ex.Descriptor; // ex.ResultCode;
            }
            catch (Exception)
            {
                return false;
            }

            // If the mouse lost focus or was not acquired then try to get control back.
            if (resultCode == SharpDX.DirectInput.ResultCode.InputLost || resultCode == SharpDX.DirectInput.ResultCode.NotAcquired)
            {
                try
                {
                    _Keyboard.Acquire();

                }
                catch
                { 
                
                }
                return true;
            }

            if (resultCode == SharpDX.DirectInput.ResultCode.Ok)
            {
                return true;
            }

            return false;
        }*/
    }
}

