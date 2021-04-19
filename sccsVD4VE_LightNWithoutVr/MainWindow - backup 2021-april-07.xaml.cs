using System;
using System.IO;
using System.Xml;
using System.Xml.Linq;
using System.Text;
using System.Linq;
using System.Windows;
using System.Threading;
using System.Reflection;
using System.Diagnostics;
using System.ComponentModel;
using System.Threading.Tasks;
using System.Speech;
using System.Speech.Recognition;
using System.Speech.Synthesis;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System.Runtime.InteropServices;
using System.Globalization;

using SharpDX;
using SharpDX.DXGI;
using SharpDX.Direct3D11;
using SharpDX.DirectInput;
using SharpDX.Multimedia;
using SharpDX.IO;
using SharpDX.XAudio2;

using System.Windows.Forms;
using System.Drawing.Imaging;

using _messager = sccsVD4VE_LightNWithoutVr.sc_console._messager;
using SC_message_object = sc_message_object.sc_message_object;
using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;

using sccsVD4VE_LightNWithoutVr.sound;



using Jitter;
using Jitter.LinearMath;
using Jitter.Dynamics;
using Jitter.Collision;
using Jitter.Collision.Shapes;
using Jitter.Forces;

/*using sccsVD4VE_LightNWithoutVr.sc_core;
using sccsVD4VE_LightNWithoutVr.sc_console;
using _messager = sccsVD4VE_LightNWithoutVr.sc_console._messager;
using SC_message_object = sc_message_object.sc_message_object;
using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;*/

//using System.Windows.Input;
///using Microsoft.VisualBasic;
//using System.Windows.Forms;
//using Microsoft.VisualBasic.Devices;
//using System.Speech.Recognition;
//using System.Speech.Synthesis; ///For program to speak back
//using AI.Perceptron;
//using System.Threading.Tasks;
//using System.Speech;
//using System.Speech.Recognition;
//using _messager = sccsVD4VE_LightNWithoutVr.sc_console._messager;
//using SC_message_object = sc_message_object.sc_message_object;
//using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;
//using System.Windows.Threading;
//using System.Linq;
//using System.Windows;
//using System.Windows.Input;
//using System.IO;
//using Microsoft.VisualBasic;
//using System.Reflection;
//using System.Text.RegularExpressions;
//using System.Xml;
//using sccsVD4VE_LightNWithoutVr.sc_console;
//using System.ComponentModel;
//using Jitter.Dynamics;
//using Jitter.Collision;
//using Jitter.Collision.Shapes;
//using Jitter.Forces;
//using System.Windows.Controls;
//using System.Windows.Data;
//using System.Windows.Documents;
//using System.Windows.Media;
//using System.Windows.Media.Imaging;
//using System.Windows.Navigation;
//using System.Windows.Shapes;
//using Microsoft.VisualBasic.Devices;
//using AI.Perceptron;
//using System.Threading.Tasks;
//using System.Speech;
//using System.Speech.Recognition;
//using System.Speech.Recognition;
//using System.Speech.Synthesis; ///For program to speak back
//using System.Windows.Forms;
//using System.Windows.Controls;
//using System.Windows.Data;
//using System.Windows.Documents;
//using System.Windows.Media;
//using System.Windows.Media.Imaging;
//using System.Windows.Navigation;
//using System.Windows.Shapes;

using sccsVD4VE_LightNWithoutVr.sc_core;
using sccsVD4VE_LightNWithoutVr.sc_console;

using WindowsInput;

using sccsVD4VE_LightNWithoutVr.SC_Graphics;

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml;
using System.Drawing.Imaging;
using System.Drawing.Drawing2D;
using System.Runtime.InteropServices;
using System.Reflection;
using Microsoft.Win32.SafeHandles;
using System.Diagnostics;

using Matrix = SharpDX.Matrix;



namespace sccsVD4VE_LightNWithoutVr
{
    public partial class MainWindow : Window
    //internal class Program
    {
        static int synthesizerVolume = 5;
        int scriptMultiply = 0;




        Matrix[][][] worldMatrix_instances_spectrum;

        sc_spectrum[][] _world_spectrum_list;
        Matrix _object_worldmatrix = Matrix.Identity;
        float offsetPosX = 0;
        float offsetPosY = 0;
        float offsetPosZ = 0;
        Matrix WorldMatrix = Matrix.Identity;
        float r = 0;
        float g = 0;
        float b = 0;
        float a = 0;

        //SPECTRUM
        //SPECTRUM
        //SPECTRUM
        const int _inst_spectrum_x = 420; // 36 // 210 //75
        const int _inst_spectrum_y = 1;
        const int _inst_spectrum_z = 210; // 36 // 210 //75 //5625
        float _spectrum_size_x = 0.0015f; //0.001115f
        float _spectrum_size_y = 0.0015f;
        float _spectrum_size_z = 0.0015f;
        byte[] _sound_byte_array = new byte[_inst_spectrum_x * _inst_spectrum_z]; //44100
        byte[] _sound_byte_array_instant = new byte[_inst_spectrum_x * _inst_spectrum_z]; //44100 //176400
        int has_spoken_main = 0;
        int has_spoken_sec = 0;
        int has_spoken_tier = 0;
        int has_spoken_quart = 0;
        string last_xml_filepath = "";
        string last_wave_filepath = "";
        DateTime _time_of_recording_start = DateTime.Now;
        DateTime _time_of_recording_end = DateTime.Now;
        int sc_can_start_rec_counter = 0;
        int sc_can_start_rec_counter_before_add_index = 0;
        int sc_play_file = 0;
        int sc_play_file_counter = 0;
        int sc_save_file = 0;
        int sc_save_file_counter = 0;
        int sc_start_recording = 0;
        int sc_start_recording_counter = 0;
        string short_path = "";
        string instant_short_path = "";
        float spectrum_noise_value = 0;
        SoundPlayer _sound_player = new SoundPlayer();
        Matrix spectrum_mat = Matrix.Identity;
        static XmlTextWriter writer = new XmlTextWriter(Console.Out);
        string path;
        int _records_counter = 0;
        int _records_instant_counter = 0;
        int _frames_to_access_counter = 0;
        int _spectrum_work = 0;
        int _spectrum_work_counter = 0;
        int _has_recorded = 0;

        public static int GetSoundLength(string fileName)
        {
            StringBuilder lengthBuf = new StringBuilder(32);
            mciSendString(string.Format("open \"{0}\" type waveaudio alias wave", fileName), null, 0, IntPtr.Zero);
            mciSendString("status wave length", lengthBuf, lengthBuf.Capacity, IntPtr.Zero);
            mciSendString("close wave", null, 0, IntPtr.Zero);
            int length = 0;
            int.TryParse(lengthBuf.ToString(), out length);
            return length;
        }
        int swtchinstantsound = -1;
        //END OF
        //END OF
        //END OF

        [DllImport("winmm.dll")]
        private static extern long mciSendString(string strCommand, StringBuilder strReturn, int iReturnLength, IntPtr hwndCallback);




        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern bool EnumChildWindows(IntPtr hwndParent, EnumWindowsProc lpEnumFunc, IntPtr lParam);

        private List<IntPtr> GetChildWindows(IntPtr parent)
        {
            List<IntPtr> result = new List<IntPtr>();
            GCHandle listHandle = GCHandle.Alloc(result);
            try
            {
                EnumWindowsProc childProc = new EnumWindowsProc(EnumWindow);
                EnumChildWindows(parent, childProc, GCHandle.ToIntPtr(listHandle));
            }
            finally
            {
                if (listHandle.IsAllocated)
                    listHandle.Free();
            }
            return result;
        }

        private bool EnumWindow(IntPtr hWnd, IntPtr lParam)
        {
            GCHandle gcChildhandlesList = GCHandle.FromIntPtr(lParam);

            if (gcChildhandlesList == null || gcChildhandlesList.Target == null)
            {
                return false;
            }

            List<IntPtr> childHandles = gcChildhandlesList.Target as List<IntPtr>;
            childHandles.Add(hWnd);

            return true;
        }




        private delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

        [DllImport("user32.dll", EntryPoint = "FindWindow", SetLastError = true)]
        static extern IntPtr FindWindowByCaption(IntPtr ZeroOnly, string lpWindowName);

        [DllImport("user32.dll", SetLastError = true)]
        static extern IntPtr FindWindowEx(IntPtr hwndParent, IntPtr hwndChildAfter, string lpszClass, string lpszWindow);

        [DllImport("User32.dll")]
        public static extern Boolean EnumChildWindows(int hWndParent, Delegate lpEnumFunc, int lParam);

        [DllImport("user32.dll", EntryPoint = "GetDesktopWindow")]
        public static extern int GetDesktopWindow();


        public delegate int Callback(int hWnd, int lParam);
        static Callback myCallBack = new Callback(EnumChildGetValue);

        [DllImport("user32.dll", SetLastError = true, CharSet = CharSet.Auto)]
        static extern int GetClassName(IntPtr hWnd, StringBuilder lpClassName, int nMaxCount);

        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern bool EnableWindow(IntPtr hWnd, bool bEnable);

        public static int EnumChildGetValue(int hWnd, int lParam)
        {
            StringBuilder formDetails = new StringBuilder(256);
            string editText = "";
            StringBuilder ClassName = new StringBuilder(256);
            var nRet = GetClassName(new IntPtr(hWnd), ClassName, ClassName.Capacity);
            Console.WriteLine("Control Caption : " + editText + " hWnd : " + hWnd.ToString("X") + " Class Name : " + ClassName);
            Trace.WriteLine("Class Name : " + ClassName);
            Console.ReadLine();

            if (ClassName.ToString().Equals("Edit"))
            {
                Console.WriteLine("Edit Control Found");
                Console.WriteLine("Current Control : " + hWnd.ToString("X"));
                Console.WriteLine("Disabling Notepad Edit Component");
                EnableWindow(new IntPtr(hWnd), true);
            }
            return 1;
        }

        public static InputSimulator inputsim;
        public static KeyboardSimulator keyboardsim;
        public static MouseSimulator mousesim;

        //const int WM_SETTEXT = 0x000C;
        //const int WM_KEYDOWN = 0x0100;
        //const int VK_RETURN = 0x0D;
        [DllImport("User32.dll")]
        public static extern Int32 SendMessage(int hWnd, int Msg, int wParam, string lParam);

        //[DllImport("User32.Dll")]
        //public static extern Int32 PostMessage(int hWnd, int msg, int wParam, int lParam);

        [DllImport("user32.dll")]
        public static extern IntPtr FindWindow(string lpClassName, string lpWindowName);

        [DllImport("user32.dll")]
        static extern bool PostMessage(IntPtr hWnd, UInt32 Msg, int wParam, int lParam);
        //static extern bool PostMessage(IntPtr hWnd, UInt32 Msg, IntPtr wParam, IntPtr lParam);


        [DllImport("user32.dll", CharSet = CharSet.Auto, CallingConvention = CallingConvention.StdCall)]
        public static extern void keybd_event(uint bVk, uint bScan, uint dwFlags, uint dwExtraInfo);

        private const int VK_LEFT = 0x25;
        private const int VK_4 = 0x34;
        private const int VK_5 = 0x35;

        public static int is_wpf = 1;

        public static string _MainWindow_name = "";
        //static SpeechRecognitionEngine recEngine;// = new SpeechRecognitionEngine();
        //public static SC_Update sc_update;
        static Task _console_worker_task;
        static Task _console_reader_task;
        static Task _console_writer_task;
        static Thread _mainTasker00;

        public static sccsVD4VE_LightNWithoutVr.sc_core.sc_globals_accessor SC_GLOBALS_ACCESSORS;
        public static sc_system_configuration config;

        public static JVector _world_gravity = new JVector(0, -9.81f, 0); //-9.81f base
        public static int _world_iterations = 3; // as high as possible normally for higher precision
        public static int _world_small_iterations = 3; // as high as possible normally for higher precision
        public static float _world_allowed_penetration = 0.00123f; //0.00123f  _world_gravity = new JVector(0, -9.81f, 0);
        public static bool _allow_deactivation = true;

        public static int _physics_engine_instance_x = 1; //4
        public static int _physics_engine_instance_y = 1; //1
        public static int _physics_engine_instance_z = 1; //4

        public static int world_width = 1;
        public static int world_height = 1;
        public static int world_depth = 1;

        public static float _delta_timer_frame = 0;
        public static float _delta_timer_time = 0;
        public static DateTime time1;
        public static DateTime time2;
        public static Stopwatch timeStopWatch00 = new Stopwatch();
        public static Stopwatch timeStopWatch01 = new Stopwatch();
        public static int _swtch = 0;
        public static int _swtch_counter_00 = 0;
        public static int _swtch_counter_01 = 0;
        public static int _swtch_counter_02 = 0;

        static float deltaTime = 0.0f;
        static float[] _array_stop_watch_tick;

        int ailanguagepriorphysicsengine = 0;


        public static async Task sc_deltatime(int timeOut, int _taskID)
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
            //FRAME DELTATIME
            var _delta_timer_frame = (float)Math.Abs((timeStopWatch01.Elapsed.Ticks - timeStopWatch00.Elapsed.Ticks)) * 0.0000000001f;

            time2 = DateTime.Now;
            var _delta_timer_time = (time2.Ticks - time1.Ticks); //100000000f
            //time1 = time2;

            deltaTime = (float)Math.Abs(_delta_timer_frame - _delta_timer_time) * 0.0000000001f;

            _array_stop_watch_tick[0] = deltaTime;

            //time1 = time2;
            await Task.Delay(1);
            Thread.Sleep(timeOut);
            _swtch_counter_00++;
            _swtch_counter_01++;
            _swtch_counter_02++;

            goto _threadLoop;
        }

        /*typeof(int),
                typeof(int),
                typeof(float),
                typeof(bool),
                typeof(JVector),
                typeof(int),
                typeof(int),
                typeof(int),*/

        static int _initX0 = 0;
        static int _initY0 = 0;

        public static IntPtr consoleHandle;
        public static IntPtr GameHandle;

        //public static SC_Console_GRAPHICS consoleGraphics = null;
        //public static sc_graphics_sec _graphics_sec = null;
        //public static SC_console_directx D3D;// = new SC_console_directx();

        //EXTERNACCESSORS
        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static extern bool GetCursorPos(ref Win32Point pt);


        //MAYBE TO READD
        //MAYBE TO READD        
        //MAYBE TO READD
        //[DllImport("winmm.dll")]
        //private static extern long mciSendString(string strCommand, StringBuilder strReturn, int iReturnLength, IntPtr hwndCallback);
        //MAYBE TO READD
        //MAYBE TO READD        
        //MAYBE TO READD


        [DllImport("User32.dll", CharSet = CharSet.Unicode)]
        public static extern int MessageBox(IntPtr h, string m, string c, int type);

        [StructLayout(LayoutKind.Sequential)]
        internal struct Win32Point
        {
            public Int32 X;
            public Int32 Y;
        };
        /*public static Point GetMousePosition()
        {
            Win32Point w32Mouse = new Win32Point();
            GetCursorPos(ref w32Mouse);
            return new Point(w32Mouse.X, w32Mouse.Y);
        }*/

        //MAINPROGRAMCARIABLES
        //public static MainWindow _mainWindow;
        //static bool _isComponents;
        public static int _processorCount = 0;
        public static int _workerThreadsTotal;
        public static int _portThreadsTotal;
        //static Thread _thread;
        public static int _totalThreads = 0;
        public static bool _mainFrameStarterItemz = true;

        //static int _swtch4consoleborders = 0;
        public static int _init_main = 1;
        //static int _counter_reset_console_borders = 0;

        public static SharpDX.DirectInput.KeyboardState keyboardstate;
        public static sc_console.sc_console_keyboard_input _keyboard_input;
        static int MaxSizeMainObject = 16;
        public static int MaxSizeMessageObject = 32;

        static sc_message_object.sc_message_object[] _main_received_messages;//
        static _messager[] _sec_received_messages;
        public static string _program_name = "SC Core Systems";
        static sc_message_object.sc_message_object _data00_IN;
        static sc_message_object.sc_message_object _data00_OUT;
        public static int init_directX_main_swtch =2;
        public static int init_vr_main_swtch = -1; //2
        static int has_init_directx = 0;
        static sc_console.sc_console_reader._console_reader_data _console_reader_string;

        static IntPtr vewindowsfoundedz;// = IntPtr.Zero; // = FindWindow(null, "voidexpanse");
        static IntPtr edwindowsfoundedz;

        static int edisrunning = -1;
        static int veisrunning = 0;

        Window _mainWindow; //        //static void Main(string[] args)

    
        public MainWindow()
        {
            InitializeComponent();

            //vewindowsfoundedz = FindWindow(null, "voidexpanse");
            //edwindowsfoundedz = FindWindow(null, "elitedangerous64");


            _mainWindow = this;


            /*_mainWindow = this;
            //INITIALIZING COMPONENTS//
            for (int j = 0; j < 1; j++)
            {
                try
                {
                    //InitializeComponent();
                    _isComponents = true;
                }
                catch
                {
                    _isComponents = false;
                    break;
                }
            }*/
            //INITIALIZING COMPONENTS//

            //CREATING SC_Console//
            for (int j = 0; j < 1; j++)
            {
                try
                {
                    //SC_Console.createConsole();
                    //SC_GCGollect collector = new SC_GCGollect();
                }
                catch
                {
                    break;
                }
            }
            //CREATING SC_Console//

            for (int j = 0; j < 1; j++)
            {
                try
                {
                    _processorCount = Environment.ProcessorCount;// SC_SystemInfoSeeker.getSystemProcessorCount();
                }
                catch //(Exception ex)
                {
                    break;
                }
            }

            for (int j = 0; j < 1; j++)
            {
                try
                {
                    ThreadPool.GetMaxThreads(out _workerThreadsTotal, out _portThreadsTotal);
                    ThreadPool.GetAvailableThreads(out _workerThreadsTotal, out _portThreadsTotal);
                }
                catch
                {
                    break;
                }
            }

            //_totalThreads = (int)(_portThreadsTotal * 0.01f);
            _totalThreads = 1;

            //SC_Console.consoleMessageQueue messageQueue1 = new SC_Console.consoleMessageQueue(_totalThreads + "", 0, 0);

            //ContentFrame2.Source = new Uri("Customizations/SC_AI_VR.xaml", UriKind.Relative);
            //ContentFrame1.Source = new Uri("Customizations/SC_desktopDupe.xaml", UriKind.Relative);


            int _last_console_width = Console.WindowWidth;
            int _last_console_height = Console.WindowHeight;




            //SC_GLOBALS_ACCESSORS AND GLOBALS CREATOR
            SC_GLOBALS_ACCESSORS = new sccsVD4VE_LightNWithoutVr.sc_core.sc_globals_accessor(_main_received_messages);
            if (SC_GLOBALS_ACCESSORS == null)
            {
                //System.Windows.MessageBox.Show("SC_GLOBALS_ACCESSORS NULL", "Console");
            }
            else
            {
                //System.Windows.MessageBox.Show("SC_GLOBALS_ACCESSORS !NULL", "Console");
            }
            //borderlessconsole console_ = new borderlessconsole();
            //SC_GLOBALS_ACCESSORS AND GLOBALS CREATOR

            ////////////////////
            ///KEYBOARD INPUT///
            ////////////////////
            for (int j = 0; j < 1; j++)
            {
                try
                {
                    _keyboard_input = new sc_console.sc_console_keyboard_input();
                    //keyboardstate = _keyboard_input._KeyboardState;



                    inputsim = new InputSimulator();
                    keyboardsim = new KeyboardSimulator(inputsim);
                    mousesim = new MouseSimulator(inputsim);
                }
                catch (Exception ex)
                {
                    MessageBox((IntPtr)0, "cannot get keyboard info main 00: " + ex.ToString() + "", "_sc_core_systems error", 0);
                    //something is wrong, todo something else later. but not implemented yet. maybe get raw input instead from SharpDX i dont know
                    break;
                }
            }
            for (int j = 0; j < 1; j++)
            {
                try
                {
                    if (_keyboard_input != null)
                    {
                        _keyboard_input._InitializeKeyboard();
                        _keyboard_input._KeyboardState = new SharpDX.DirectInput.KeyboardState();
                    }
                    else
                    {
                        MessageBox((IntPtr)0, "cannot get keyboard info main 01: ", "_sc_core_systems error", 0);
                        //something is wrong, todo something else later. but not implemented yet. maybe get raw input instead from SharpDX i dont know
                    }
                }
                catch (Exception ex)
                {
                    MessageBox((IntPtr)0, "cannot get keyboard info main 02: " + ex.ToString() + "", "_sc_core_systems error", 0);
                    //something is wrong, todo something else later. but not implemented yet. maybe get raw input instead from SharpDX i dont know
                    break;
                }
            }

            /*if (_keyboard_input._KeyboardState != null)
            {
                MessageBox((IntPtr)0, "could get keyboard info main 03: ", "_sc_core_systems error", 0);
            }
            else
            {
                MessageBox((IntPtr)0, "cannot get keyboard info main 04: ", "_sc_core_systems error", 0);
                //something is wrong, todo something else later. but not implemented yet. maybe get raw input instead from SharpDX i dont know
            }*/

            ////////////////////
            ///KEYBOARD INPUT///
            ////////////////////

            /////////////////////////////// : But i am not totally sure. I certainly can use this struct and send it to an "async" task and retrieve the data
            ///////////////////////////////   right away even if i sometimes or always "modify" the data before sending it to a task and then right after the task
            ///////////////////////////////   finishes, i am retrieving the data from that async task. But i am not sure yet if my shit works "completely". as i am
            ///////////////////////////////   not "declaring" the task as "async" BUT i am only starting that task once and let it run whatever it has to run in a loop.
            ///message_thread_safe_kinda///   everybody does that it's for sure anyway, i didn't invent anything here. It's just that this struct is very important
            ///////////////////////////////   for the rest of the program to run and this first struct will be able to communicate with all of my program in INTs
            ///////////////////////////////   used as binary code "kinda" for every option that i want to implement in order to alleviate the load of menu mecanics.
            ///////////////////////////////   the namespace to get it really can really be simplified. i mean really. wtheck am i doing.
            ///////////////////////////////
            _main_received_messages = new sc_message_object.sc_message_object[MaxSizeMainObject];


            for (int i = 0; i < _main_received_messages.Length; i++)
            {
                _main_received_messages[i] = new sc_message_object.sc_message_object();
                _main_received_messages[i]._received_switch_in = -1;
                _main_received_messages[i]._received_switch_out = -1;
                _main_received_messages[i]._sending_switch_in = -1;
                _main_received_messages[i]._sending_switch_out = -1;
                _main_received_messages[i]._timeOut0 = -1;
                _main_received_messages[i]._ParentTaskThreadID0 = -1;
                _main_received_messages[i]._main_cpu_count = _processorCount;
                _main_received_messages[i]._passTest = "";
                _main_received_messages[i]._welcomePackage = -1;
                _main_received_messages[i]._work_done = -1;
                _main_received_messages[i]._current_menu = -1;
                _main_received_messages[i]._last_current_menu = -1;
                _main_received_messages[i]._main_menu = -1;
                _main_received_messages[i]._menuOption = "";
                _main_received_messages[i]._voRecSwtc = -1;
                _main_received_messages[i]._voRecMsg = "";
                _main_received_messages[i]._someData = new object[MaxSizeMainObject];

                for (int j = 0; j < _main_received_messages[i]._someData.Length; j++)
                {
                    _main_received_messages[i]._someData[j] = new object();
                }

                //VOICE RECOGNITION. NOT IMPLEMENTED YET.
                /*if (i == MaxSizeMainObject - 1)
                {
                    _main_received_messages[i]._someData[0] = _keyboard_input._KeyboardState;
                    _main_received_messages[i]._voRecSwtc = 1;
                }*/
            }
            ///////////////////////////////
            ///////////////////////////////   
            ///////////////////////////////   
            ///////////////////////////////  
            ///message_thread_safe_kinda///   
            ///////////////////////////////   
            ///////////////////////////////   
            ///////////////////////////////







            object _console_reader_object;
            _sec_received_messages = new _messager[MaxSizeMessageObject];



            for (int i = 0; i < _sec_received_messages.Length; i++)
            {
                _sec_received_messages[i]._swtch0 = -1;
            }



            _sec_received_messages[4]._messager_list = new _messager[MaxSizeMainObject];
            _sec_received_messages[4]._message = "";
            _sec_received_messages[4]._originalMsg = "";
            _sec_received_messages[4]._messageCut = "";
            _sec_received_messages[4]._specialMessage = -1;
            _sec_received_messages[4]._specialMessageLineX = 0;
            _sec_received_messages[4]._specialMessageLineY = 0;
            _sec_received_messages[4]._orilineX = 0;
            _sec_received_messages[4]._orilineY = 0;
            _sec_received_messages[4]._lineX = 0;
            _sec_received_messages[4]._lineY = 0;
            _sec_received_messages[4]._lastOrilineX = 0;
            _sec_received_messages[4]._lastOrilineY = 0;
            _sec_received_messages[4]._count = 0;
            _sec_received_messages[4]._swtch0 = 1;
            _sec_received_messages[4]._swtch1 = 1;
            _sec_received_messages[4]._delay = 50;
            _sec_received_messages[4]._looping = 1;

            _console_reader_string._has_message_to_display = 0;
            _console_reader_string._console_reader_message = "";
            _console_reader_string._has_init = 0;
            _console_reader_object = _console_reader_string;


            //FOR SPEECH RECOGNITION
            //_sec_received_messages[15]._swtch0 = 1;





            //Console.WriteLine("test");

            _sec_received_messages[12]._message = msgdisabled;
            _sec_received_messages[12]._originalMsg = msgdisabled;
            _sec_received_messages[12]._messageCut = msgdisabled;
            _sec_received_messages[12]._specialMessage = 3;
            _sec_received_messages[12]._specialMessageLineX = 0;
            _sec_received_messages[12]._specialMessageLineY = 0;
            _sec_received_messages[12]._orilineX = 0;// Console.WindowWidth - 3 - msgdisabled.Length;
            _sec_received_messages[12]._orilineY = 1;
            _sec_received_messages[12]._lineX = _sec_received_messages[12]._orilineX;
            _sec_received_messages[12]._lineY = _sec_received_messages[12]._orilineY;
            _sec_received_messages[12]._count = 0;
            _sec_received_messages[12]._swtch0 = 1;
            _sec_received_messages[12]._swtch1 = 1;
            _sec_received_messages[12]._delay = 11;
            _sec_received_messages[12]._looping = 1;


            //Console.WriteLine("test");

            _sec_received_messages[13]._message = msgenabled;
            _sec_received_messages[13]._originalMsg = msgenabled;
            _sec_received_messages[13]._messageCut = msgenabled;
            _sec_received_messages[13]._specialMessage = 3;
            _sec_received_messages[13]._specialMessageLineX = 0;
            _sec_received_messages[13]._specialMessageLineY = 0;
            _sec_received_messages[13]._orilineX = 0;// Console.WindowWidth - 3 - msgdisabled.Length;
            _sec_received_messages[13]._orilineY = 0;
            _sec_received_messages[13]._lineX = _sec_received_messages[13]._orilineX;
            _sec_received_messages[13]._lineY = _sec_received_messages[13]._orilineY;
            _sec_received_messages[13]._count = 0;
            _sec_received_messages[13]._swtch0 = 1;
            _sec_received_messages[13]._swtch1 = 1;
            _sec_received_messages[13]._delay = 11;
            _sec_received_messages[13]._looping = 1;




            _sec_received_messages[10]._message = speechcaptured;
            _sec_received_messages[10]._originalMsg = speechcaptured;
            _sec_received_messages[10]._messageCut = speechcaptured;
            _sec_received_messages[10]._specialMessage = 3;
            _sec_received_messages[10]._specialMessageLineX = 0;
            _sec_received_messages[10]._specialMessageLineY = 0;
            _sec_received_messages[10]._orilineX = 0; //Console.WindowWidth - 3 - msgdisabled.Length
            _sec_received_messages[10]._orilineY = 2;
            _sec_received_messages[10]._lineX = _sec_received_messages[10]._orilineX;
            _sec_received_messages[10]._lineY = _sec_received_messages[10]._orilineY;
            _sec_received_messages[10]._count = 0;
            _sec_received_messages[10]._swtch0 = 1;
            _sec_received_messages[10]._swtch1 = 1;
            _sec_received_messages[10]._delay = 11;
            _sec_received_messages[10]._looping = 1;














            //int _init_main_thread_in_out = 0;
            //int _init_main = 1;
            int _worker_000_has_init = 0;
            int _start_thread_console_writer = 1;

            int _console_reader_canWork = 1;
            int startThread = 0;


            int _counter_reset_console_borders = 0;

            int _lastMenu = -2;
            string _lastMenuOption = "";
            string _lastUsername = "";
            int _some_other_swtch = 0;

            int loop_main_thread = 0;

            try
            {

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }

            /*if (recEngine == null)
            {
                Console.WriteLine("null rec engine");
            }
            else
            {
                //Console.WriteLine("!null rec engine");

                //threadOneGrammarLoad();
                var updateMainUITitle = new Action(() =>
                {
                    if (_UIStarterItemz)
                    {
                        threadOneGrammarLoad();
                        _UIStarterItemz = false;
                    }

                    _mainUpdateThread();
                });

                Dispatcher.Invoke(updateMainUITitle);




            /*foreach (RecognizerInfo ri in SpeechRecognitionEngine.InstalledRecognizers())
            {
                //System.Diagnostics.Debug.WriteLine(ri.Culture.Name);

                Console.WriteLine(ri.Culture.Name);

            }
            }*/





            var updateMainUITitle = new Action(() =>
            {
                if (_UIStarterItemz == 1)
                {
                    threadOneGrammarLoad();
                    _UIStarterItemz = 0;
                }

                //_mainUpdateThread();
            });

            Dispatcher.Invoke(updateMainUITitle);


            //vewindowsfoundedz = FindWindow(null, "voidexpanse");

            Process[] processlist = Process.GetProcesses();

            foreach (Process process in processlist)
            {
                if (process.ProcessName.ToLower() == "voidexpanse")
                {
                    //Console.WriteLine("Process: {0} ID: {1} Window title: {2}", process.ProcessName, process.Id, process.MainWindowTitle);
                    //MessageBox((IntPtr)0, "ED" + " " + process.MainWindowHandle, "sccsVD4VE_LightNWithoutVr0", 0);
                    vewindowsfoundedz = process.MainWindowHandle;
                }
            }



            //MessageBox((IntPtr)0, "" + vewindowsfoundedz, "", 0);



            /*var startSomeThread = new Action(() =>
            {
                //threadOneGrammarLoad();
                Thread _threadForMainUpdatesUI;
                _threadForMainUpdatesUI = new Thread(() => _mainThreadStarter());
                _threadForMainUpdatesUI.IsBackground = true;
                _threadForMainUpdatesUI.SetApartmentState(ApartmentState.STA);// =;
                _threadForMainUpdatesUI.Start();
            });

            System.Windows.Application.Current.Dispatcher.Invoke(startSomeThread);*/







            Thread _thread;
            _thread = new Thread(() => _mainThreadStarter());
            _thread.IsBackground = false;
            _thread.SetApartmentState(ApartmentState.STA);// =;
            _thread.Start();


            //check later in a loop if synth is null or not... and restart the synth later.
            //SpeechSynthesizer synth = new SpeechSynthesizer();


            _world_spectrum_list = new sc_spectrum[1][];
            worldMatrix_instances_spectrum = new Matrix[1][][];










            if (scriptMultiply==1){
                sccsScriptAutoCorrectNMultiply();
            }



            //














            for (int j = 0; j < 1; j++)
            {
                try
                {
                    if (_mainFrameStarterItemz)
                    {
                        //Creating a thread individually... It has access to the UI by using System.Windows.Application.Current.Dispatcher.Invoke and loops
                        //STRICTLY CURRENTLY RESERVED FOR SPEECH RECOGNITION

                        // sc_ai_language = new SC_AI_Language();

                        //start your multi threaded program here
                        //Process p = new Process();
                        //p.Start();

                        _mainTasker00 = new Thread((tester0000) =>
                        {
                           

                            if (loop_main_thread == 0)
                            {

                                SpeechSynthesizer synth = new SpeechSynthesizer();
                                //_mainFrameStarterItemz = false;
                                // Some biolerplate to react to close window event, CTRL-C, kill, etc
                                _handler += new EventHandler(Handler);
                                SetConsoleCtrlHandler(_handler, true);
                                //sccsScriptAutoCorrectNMultiply();
                            _thread_main_loop:
                                windowsToVoidExpanseKeyboardClicksComs();


                                /*var updateMainUITitleTWo = new Action(() =>
                                {
                                    //if (_UIStarterItemz == 1)
                                    //{
                                    //    threadOneGrammarLoad();
                                    //    _UIStarterItemz = 0;
                                    //}
                                    //windowsToVoidExpanseKeyboardClicksComs();
                                    //sccsScriptAutoCorrectNMultiply();
                                });

                                Dispatcher.Invoke(updateMainUITitleTWo);*/
                                //windowsToVoidExpanseKeyboardClicksComs();
                                /*if (!_thread.IsAlive)
                                {
                                    MessageBox((IntPtr)0, "_thread alive not", "sccsVD4VE_LightNWithoutVr", 0);
                                }*/

                                //hold the console so it doesn’t run off the end
                                if (exitSystem)
                                {
                                    //MessageBox((IntPtr)0,"shutting off", "sccsVD4VE_LightNWithoutVr",0);
                                    //Thread.Sleep(500);
                                    //loop_main_thread = 1;
                                    _mainFrameStarterItemz = false;
                                }

                                /*_thread = new Thread(() => _mainThreadStarter());
                                _thread.IsBackground = true;
                                _thread.Start();*/

                                /*if (_main_received_messages[0]._welcomePackage == 998 || _main_received_messages[0]._welcomePackage == 999)
                                {

                                }*/

                                //////READ KEYBOARD=>
                                _keyboard_input.ReadKeyboard();
                                //////READ KEYBOARD<=

                                //Console.SetCursorPosition(1, 1);
                                //Console.WriteLine("test");

                                /*if (Console.WindowWidth != _last_console_width || Console.WindowHeight != _last_console_height)
                                {
                                    if (_counter_reset_console_borders > 50)
                                    {
                                        SC_GLOBALS_ACCESSORS.SC_CONSOLE_CORE._reset_console_borders();
                                        _last_console_width = Console.WindowWidth;
                                        _last_console_height = Console.WindowHeight;
                                        _counter_reset_console_borders = 0;
                                    }
                                    _counter_reset_console_borders++;
                                }*/

                                if (_start_thread_console_writer == 1)
                                {
                                    _console_writer_task = Task<object[]>.Factory.StartNew((tester0001) =>
                                    {
                                        var _initX = (Console.WindowWidth / 2) - (_program_name.Length / 2);
                                        var _initY = (Console.WindowHeight / 2);

                                        _sec_received_messages[0]._message = _program_name;
                                        _sec_received_messages[0]._originalMsg = _program_name;
                                        _sec_received_messages[0]._messageCut = _program_name;
                                        _sec_received_messages[0]._specialMessage = 0;
                                        _sec_received_messages[0]._specialMessageLineX = 0;
                                        _sec_received_messages[0]._specialMessageLineY = 0;
                                        _sec_received_messages[0]._orilineX = _initX;
                                        _sec_received_messages[0]._orilineY = _initY;
                                        _sec_received_messages[0]._lineX = _initX;
                                        _sec_received_messages[0]._lineY = _initY;
                                        _sec_received_messages[0]._lastOrilineX = _initX;
                                        _sec_received_messages[0]._lastOrilineY = _initY;
                                        _sec_received_messages[0]._count = 0;
                                        _sec_received_messages[0]._swtch0 = 1;
                                        _sec_received_messages[0]._swtch1 = 1;
                                        _sec_received_messages[0]._delay = 5;
                                        _sec_received_messages[0]._looping = 1;

                                        _worker_000_has_init = 1;

                                    //////CONSOLE WRITER=>
                                    _thread_loop_console:

                                        _sec_received_messages = SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER._console_writer(_sec_received_messages);

                                        Thread.Sleep(1);

                                        goto _thread_loop_console;
                                        //////CONSOLE WRITER <=
                                    }, _main_received_messages);
                                    _start_thread_console_writer = 2;
                                }

                                //CONFIRM CONSOLE WRITER IS WORKING=>
                                if (_worker_000_has_init == 1)
                                {
                                    if (SC_GLOBALS_ACCESSORS != null)
                                    {
                                        if (SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER != null)
                                        {
                                            /*_sec_received_messages[1]._message = "C-WR" + " ENABLED";
                                            _sec_received_messages[1]._originalMsg = "C-WR" + " ENABLED";
                                            _sec_received_messages[1]._messageCut = "C-WR" + " ENABLED";
                                            _sec_received_messages[1]._specialMessage = 2;
                                            _sec_received_messages[1]._specialMessageLineX = 0;
                                            _sec_received_messages[1]._specialMessageLineY = 0;
                                            _sec_received_messages[1]._orilineX = 1;
                                            _sec_received_messages[1]._orilineY = Console.WindowHeight - 2;
                                            _sec_received_messages[1]._lineX = 1;
                                            _sec_received_messages[1]._lineY = Console.WindowHeight - 2;
                                            _sec_received_messages[1]._lastOrilineX = _sec_received_messages[1]._lineX;
                                            _sec_received_messages[1]._lastOrilineY = _sec_received_messages[1]._lineY;
                                            _sec_received_messages[1]._count = 0;
                                            _sec_received_messages[1]._swtch0 = 1;
                                            _sec_received_messages[1]._swtch1 = 0;
                                            _sec_received_messages[1]._delay = 10;
                                            _sec_received_messages[1]._looping = 0;*/

                                            _worker_000_has_init = 2;
                                        }
                                    }
                                }
                                //CONFIRM CONSOLE WRITER IS WORKING<=

                                //CONFIRM CONSOLE READER IS WORKING=>
                                if (_worker_000_has_init == 2)
                                {
                                    if (SC_GLOBALS_ACCESSORS != null)
                                    {
                                        if (SC_GLOBALS_ACCESSORS.SC_CONSOLE_READER != null)
                                        {
                                            /*_sec_received_messages[2]._message = "core C-RE" + " ENABLED";
                                            _sec_received_messages[2]._originalMsg = "core C-RE" + " ENABLED";
                                            _sec_received_messages[2]._messageCut = "core C-RE" + " ENABLED";
                                            _sec_received_messages[2]._specialMessage = 2;
                                            _sec_received_messages[2]._specialMessageLineX = 0;
                                            _sec_received_messages[2]._specialMessageLineY = 1;
                                            _sec_received_messages[2]._orilineX = _sec_received_messages[2]._message.Length + 3;
                                            _sec_received_messages[2]._orilineY = Console.WindowHeight - 2;
                                            _sec_received_messages[2]._lineX = _sec_received_messages[2]._message.Length + 3;
                                            _sec_received_messages[2]._lineY = Console.WindowHeight - 2;
                                            _sec_received_messages[2]._count = 0;
                                            _sec_received_messages[2]._swtch0 = 1;
                                            _sec_received_messages[2]._swtch1 = 0;
                                            _sec_received_messages[2]._delay = 10;
                                            _sec_received_messages[2]._looping = 0;*/

                                            _worker_000_has_init = 3;
                                        }
                                    }
                                }
                                //CONFIRM CONSOLE READER IS WORKING<=





                                if (_worker_000_has_init == 3)
                                {
                                    if (SC_GLOBALS_ACCESSORS != null)
                                    {
                                        if (SC_GLOBALS_ACCESSORS.SC_CONSOLE_READER != null)
                                        {
                                            var _program_name0 = "Press Enter";
                                            _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                            _initY0 = (Console.WindowHeight / 2) + 1;

                                            _sec_received_messages[3]._message = _program_name0;
                                            _sec_received_messages[3]._originalMsg = _program_name0;
                                            _sec_received_messages[3]._messageCut = _program_name0;
                                            _sec_received_messages[3]._specialMessage = 2;
                                            _sec_received_messages[3]._specialMessageLineX = 0;
                                            _sec_received_messages[3]._specialMessageLineY = 0;
                                            _sec_received_messages[3]._orilineX = _initX0;
                                            _sec_received_messages[3]._orilineY = _initY0;
                                            _sec_received_messages[3]._lineX = _initX0;
                                            _sec_received_messages[3]._lineY = _initY0;
                                            _sec_received_messages[3]._count = 0;
                                            _sec_received_messages[3]._swtch0 = 1;
                                            _sec_received_messages[3]._swtch1 = 1;
                                            _sec_received_messages[3]._delay = 100;
                                            _sec_received_messages[3]._looping = 1;
                                            _worker_000_has_init = 4;
                                        }
                                    }
                                }



                                if (_worker_000_has_init == 4)
                                {

                                    _console_reader_task = Task<object[]>.Factory.StartNew((tester0001) =>
                                    {
                                        while (true)
                                        {
                                            if (_console_reader_canWork == 1)
                                            {
                                                _console_reader_string = SC_GLOBALS_ACCESSORS.SC_CONSOLE_READER._console_reader(_console_reader_object);
                                            }

                                            if (SC_GLOBALS_ACCESSORS.SC_CONSOLE_READER._main_has_init == 1)
                                            {
                                                _console_reader_string._console_reader_message = "";
                                                _console_reader_string._has_message_to_display = 0;

                                                var _program_name0 = "WELCOME";
                                                _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                _initY0 = (Console.WindowHeight / 2) - 1;

                                                _sec_received_messages[4]._message = _program_name0;
                                                _sec_received_messages[4]._originalMsg = _program_name0;
                                                _sec_received_messages[4]._messageCut = _program_name0;
                                                _sec_received_messages[4]._specialMessage = 2;
                                                _sec_received_messages[4]._specialMessageLineX = 0;
                                                _sec_received_messages[4]._specialMessageLineY = 0;
                                                _sec_received_messages[4]._orilineX = _initX0;
                                                _sec_received_messages[4]._orilineY = _initY0;
                                                _sec_received_messages[4]._lineX = _initX0;
                                                _sec_received_messages[4]._lineY = _initY0;
                                                _sec_received_messages[4]._count = 0;
                                                _sec_received_messages[4]._swtch0 = 1;
                                                _sec_received_messages[4]._swtch1 = 0;
                                                _sec_received_messages[4]._delay = 200;
                                                _sec_received_messages[4]._looping = 0;

                                                _sec_received_messages[0]._swtch0 = 0;
                                                _sec_received_messages[0]._swtch1 = 0;
                                                _sec_received_messages[3]._swtch0 = 0;
                                                _sec_received_messages[3]._swtch1 = 0;

                                                _program_name0 = "Please Enter your Username: ";
                                                _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                _initY0 = (Console.WindowHeight / 2) + 2;

                                                _sec_received_messages[6]._message = _program_name0;
                                                _sec_received_messages[6]._originalMsg = _program_name0;
                                                _sec_received_messages[6]._messageCut = _program_name0;
                                                _sec_received_messages[6]._specialMessage = 2;
                                                _sec_received_messages[6]._specialMessageLineX = 0;
                                                _sec_received_messages[6]._specialMessageLineY = 0;
                                                _sec_received_messages[6]._orilineX = _initX0;
                                                _sec_received_messages[6]._orilineY = _initY0;
                                                _sec_received_messages[6]._lineX = _initX0;
                                                _sec_received_messages[6]._lineY = _initY0;
                                                _sec_received_messages[6]._count = 0;
                                                _sec_received_messages[6]._swtch0 = 1;
                                                _sec_received_messages[6]._swtch1 = 1;
                                                _sec_received_messages[6]._delay = 50;
                                                _sec_received_messages[6]._looping = 1;

                                                Console.SetCursorPosition(_initX0 + _program_name0.Length, _initY0);

                                                startThread = 3;
                                                SC_GLOBALS_ACCESSORS.SC_CONSOLE_READER._main_has_init = 2;
                                            }

                                            if (startThread == 3 && _console_reader_string._has_message_to_display == 1)
                                            {

                                                if (_console_reader_string._console_reader_message.ToLower() == "nine" || _console_reader_string._console_reader_message.ToLower() == "ninekorn" || _console_reader_string._console_reader_message.ToLower() == "9")
                                                {

                                                    var _program_name0 = "Access Authorized";
                                                    _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                    _initY0 = (Console.WindowHeight / 2) + 2;

                                                    _sec_received_messages[6]._message = _program_name0;
                                                    _sec_received_messages[6]._originalMsg = _program_name0;
                                                    _sec_received_messages[6]._messageCut = _program_name0;
                                                    _sec_received_messages[6]._specialMessage = 2;
                                                    _sec_received_messages[6]._specialMessageLineX = 0;
                                                    _sec_received_messages[6]._specialMessageLineY = 0;
                                                    _sec_received_messages[6]._lineX = _initX0;
                                                    _sec_received_messages[6]._lineY = _initY0;
                                                    _sec_received_messages[6]._count = 0;
                                                    _sec_received_messages[6]._swtch0 = 1;
                                                    _sec_received_messages[6]._swtch1 = 0;
                                                    _sec_received_messages[6]._delay = 50;
                                                    _sec_received_messages[6]._looping = 0;

                                                    for (int L0_IN = 0; L0_IN < _main_received_messages.Length; L0_IN++)
                                                    {
                                                        _main_received_messages[L0_IN]._passTest = _console_reader_string._console_reader_message.ToLower();
                                                    }
                                                    Console.SetCursorPosition(_initX0, _initY0 + 1);
                                                    _lastUsername = _console_reader_string._console_reader_message;
                                                    _console_reader_string._console_reader_message = "";
                                                    startThread = 4;
                                                }
                                                else if (_console_reader_string._console_reader_message.ToLower() != " " || _console_reader_string._console_reader_message.ToLower() != "")
                                                {

                                                    var _program_name0 = "Access Denied";
                                                    _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                    _initY0 = (Console.WindowHeight / 2) + 2;

                                                    _sec_received_messages[6]._message = _program_name0;
                                                    _sec_received_messages[6]._originalMsg = _program_name0;
                                                    _sec_received_messages[6]._messageCut = _program_name0;
                                                    _sec_received_messages[6]._specialMessage = 2;
                                                    _sec_received_messages[6]._specialMessageLineX = 0;
                                                    _sec_received_messages[6]._specialMessageLineY = 0;
                                                    _sec_received_messages[6]._lineX = _initX0;
                                                    _sec_received_messages[6]._lineY = _initY0;
                                                    _sec_received_messages[6]._count = 0;
                                                    _sec_received_messages[6]._swtch0 = 1;
                                                    _sec_received_messages[6]._swtch1 = 0;
                                                    _sec_received_messages[6]._delay = 50;
                                                    _sec_received_messages[6]._looping = 0;

                                                    _lastUsername = "";
                                                    _console_reader_string._console_reader_message = "";
                                                    Console.SetCursorPosition(_initX0, _initY0);
                                                    startThread = 3;
                                                }
                                            }
                                            else if (startThread == 4)
                                            {
                                                if (_console_reader_string._console_reader_message.ToLower() == "vr" ||
                                                     _console_reader_string._console_reader_message.ToLower() == "standard" ||
                                                      _console_reader_string._console_reader_message.ToLower() == "std")
                                                {
                                                    if (_console_reader_string._console_reader_message.ToLower() == "vr")
                                                    {

                                                        var _program_name0 = "creating VR mecanics";
                                                        _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                        _initY0 = (Console.WindowHeight / 2) + 2;

                                                        _sec_received_messages[6]._message = _program_name0;
                                                        _sec_received_messages[6]._originalMsg = _program_name0;
                                                        _sec_received_messages[6]._messageCut = _program_name0;
                                                        _sec_received_messages[6]._specialMessage = 2;
                                                        _sec_received_messages[6]._specialMessageLineX = 0;
                                                        _sec_received_messages[6]._specialMessageLineY = 0;
                                                        _sec_received_messages[6]._lineX = _initX0;
                                                        _sec_received_messages[6]._lineY = _initY0;
                                                        _sec_received_messages[6]._count = 0;
                                                        _sec_received_messages[6]._swtch0 = 1;
                                                        _sec_received_messages[6]._swtch1 = 0;
                                                        _sec_received_messages[6]._delay = 50;
                                                        _sec_received_messages[6]._looping = 0;


                                                        _lastMenuOption = _console_reader_string._console_reader_message.ToLower();
                                                        _console_reader_string._console_reader_message = "";

                                                        _main_received_messages[0]._received_switch_in = 1;
                                                        _main_received_messages[0]._received_switch_out = 1;
                                                        _main_received_messages[0]._sending_switch_in = 1;
                                                        _main_received_messages[0]._sending_switch_out = 1;
                                                        _main_received_messages[0]._welcomePackage = 999;

                                                        _main_received_messages = sc_console_menu.sc_console_menu_00._console_menu(_main_received_messages);
                                                        Console.SetCursorPosition(_initX0, _initY0 + 1);
                                                        _some_other_swtch = 1;
                                                    }
                                                    else if (_console_reader_string._console_reader_message.ToLower() == "standard" ||
                                                            _console_reader_string._console_reader_message.ToLower() == "std")
                                                    {

                                                        _lastMenuOption = _console_reader_string._console_reader_message.ToLower();
                                                        _console_reader_string._console_reader_message = "";

                                                        _main_received_messages[0]._received_switch_in = 1;
                                                        _main_received_messages[0]._received_switch_out = 1;
                                                        _main_received_messages[0]._sending_switch_in = 1;
                                                        _main_received_messages[0]._sending_switch_out = 1;
                                                        _main_received_messages[0]._welcomePackage = 998;
                                                        _main_received_messages = sc_console_menu.sc_console_menu_00._console_menu(_main_received_messages);
                                                        Console.SetCursorPosition(_initX0, _initY0 + 1);
                                                        _some_other_swtch = 1;
                                                    }
                                                }
                                                else
                                                {

                                                    var _program_name0 = "Option Not Implemented";
                                                    _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                    _initY0 = (Console.WindowHeight / 2) + 2;

                                                    _sec_received_messages[6]._message = _program_name0;
                                                    _sec_received_messages[6]._originalMsg = _program_name0;
                                                    _sec_received_messages[6]._messageCut = _program_name0;
                                                    _sec_received_messages[6]._specialMessage = 2;
                                                    _sec_received_messages[6]._specialMessageLineX = 0;
                                                    _sec_received_messages[6]._specialMessageLineY = 0;
                                                    _sec_received_messages[6]._lineX = _initX0;
                                                    _sec_received_messages[6]._lineY = _initY0;
                                                    _sec_received_messages[6]._count = 0;
                                                    _sec_received_messages[6]._swtch0 = 1;
                                                    _sec_received_messages[6]._swtch1 = 0;
                                                    _sec_received_messages[6]._delay = 50;
                                                    _sec_received_messages[6]._looping = 0;

                                                    _lastMenuOption = "";
                                                    _console_reader_string._console_reader_message = "";

                                                    Console.SetCursorPosition(_initX0, _initY0);
                                                }
                                            }

                                            Thread.Sleep(1);
                                        }
                                    }, _main_received_messages);
                                    _worker_000_has_init = 5;
                                }

                                if (_worker_000_has_init == 5)
                                {
                                    _console_worker_task = Task<object[]>.Factory.StartNew((tester0001) =>
                                    {
                                        while (true)
                                        {
                                            if (_worker_000_has_init == 2)
                                            {
                                                int _welcomePackage00 = _main_received_messages[0]._welcomePackage;

                                                if (_welcomePackage00 == 0)
                                                {
                                                    _main_received_messages = sc_console_menu.sc_console_menu_00._console_menu(_main_received_messages);
                                                }
                                                else if (_welcomePackage00 == 1)
                                                {
                                                    int _current_menu00 = _data00_OUT._current_menu;

                                                    if (_lastMenu != _current_menu00)
                                                    {
                                                        var _program_name0 = _current_menu00 + "";
                                                        _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                        _initY0 = (Console.WindowHeight / 2) + 9;
                                                        _sec_received_messages[5]._message = _program_name0;
                                                        _sec_received_messages[5]._originalMsg = _program_name0;
                                                        _sec_received_messages[5]._messageCut = _program_name0;
                                                        _sec_received_messages[5]._specialMessage = 2;
                                                        _sec_received_messages[5]._specialMessageLineX = 0;
                                                        _sec_received_messages[5]._specialMessageLineY = 0;
                                                        _sec_received_messages[5]._lineX = _initX0;
                                                        _sec_received_messages[5]._lineY = _initY0;
                                                        _sec_received_messages[5]._count = 0;
                                                        _sec_received_messages[5]._swtch0 = 1;
                                                        _sec_received_messages[5]._swtch1 = 0;
                                                        _sec_received_messages[5]._delay = 50;
                                                        _sec_received_messages[5]._looping = 0;
                                                    }

                                                    if (_current_menu00 == -1)
                                                    {
                                                        _data00_IN._received_switch_in = 0;
                                                        _data00_IN._received_switch_out = 0;
                                                        _data00_IN._sending_switch_in = 0;
                                                        _data00_IN._sending_switch_out = 0;

                                                        _data00_IN._current_menu = _data00_OUT._current_menu;
                                                        _data00_IN._menuOption = _lastMenuOption;

                                                        var objecterer = _data00_IN;
                                                        _data00_OUT = sc_console_menu.sc_console_menu_01._console_menu(objecterer);

                                                        _lastMenu = _data00_OUT._current_menu;
                                                        _lastMenuOption = "";
                                                    }
                                                    else if (_current_menu00 == 0)
                                                    {
                                                        var _program_name0 = _current_menu00 + "";
                                                        _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                        _initY0 = (Console.WindowHeight / 2) + 9;
                                                        _sec_received_messages[5]._message = _program_name0;
                                                        _sec_received_messages[5]._originalMsg = _program_name0;
                                                        _sec_received_messages[5]._messageCut = _program_name0;
                                                        _sec_received_messages[5]._specialMessage = 2;
                                                        _sec_received_messages[5]._specialMessageLineX = 0;
                                                        _sec_received_messages[5]._specialMessageLineY = 0;
                                                        _sec_received_messages[5]._lineX = _initX0;
                                                        _sec_received_messages[5]._lineY = _initY0;
                                                        _sec_received_messages[5]._count = 0;
                                                        _sec_received_messages[5]._swtch0 = 1;
                                                        _sec_received_messages[5]._swtch1 = 0;
                                                        _sec_received_messages[5]._delay = 50;
                                                        _sec_received_messages[5]._looping = 0;

                                                        _data00_IN._received_switch_in = 0;
                                                        _data00_IN._received_switch_out = 0;
                                                        _data00_IN._sending_switch_in = 0;
                                                        _data00_IN._sending_switch_out = 0;

                                                        _data00_IN._current_menu = _data00_OUT._current_menu;
                                                        _data00_IN._menuOption = _lastMenuOption;

                                                        var objecterer = _data00_IN;
                                                        _data00_OUT = sc_console_menu.sc_console_menu_01._console_menu(objecterer);
                                                        _lastMenu = _data00_OUT._current_menu;
                                                        _lastMenuOption = "";
                                                    }
                                                    else if (_current_menu00 == 1)
                                                    {
                                                        var _program_name0 = _current_menu00 + "";
                                                        _initX0 = (Console.WindowWidth / 2) - (_program_name0.Length / 2);
                                                        _initY0 = (Console.WindowHeight / 2) + 9;
                                                        _sec_received_messages[5]._message = _program_name0;
                                                        _sec_received_messages[5]._originalMsg = _program_name0;
                                                        _sec_received_messages[5]._messageCut = _program_name0;
                                                        _sec_received_messages[5]._specialMessage = 2;
                                                        _sec_received_messages[5]._specialMessageLineX = 0;
                                                        _sec_received_messages[5]._specialMessageLineY = 0;
                                                        _sec_received_messages[5]._lineX = _initX0;
                                                        _sec_received_messages[5]._lineY = _initY0;
                                                        _sec_received_messages[5]._count = 0;
                                                        _sec_received_messages[5]._swtch0 = 1;
                                                        _sec_received_messages[5]._swtch1 = 0;
                                                        _sec_received_messages[5]._delay = 50;
                                                        _sec_received_messages[5]._looping = 0;

                                                        _data00_IN._received_switch_in = 0;
                                                        _data00_IN._received_switch_out = 0;
                                                        _data00_IN._sending_switch_in = 0;
                                                        _data00_IN._sending_switch_out = 0;

                                                        _data00_IN._current_menu = _data00_OUT._current_menu;
                                                        _data00_IN._menuOption = _lastMenuOption;

                                                        var objecterer = _data00_IN;
                                                        _data00_OUT = sc_console_menu.sc_console_menu_01._console_menu(objecterer);
                                                        _lastMenu = _data00_OUT._current_menu;
                                                        _lastMenuOption = "";
                                                    }
                                                }
                                            }
                                            Thread.Sleep(1);
                                        }
                                    }, _main_received_messages);
                                    _worker_000_has_init = 6;
                                }






                                if (_some_other_swtch == 1)  //_some_other_swtch == 1
                                {
                                    if (has_init_directx == 0)
                                    {
                                        if (init_directX_main_swtch == 2 || init_vr_main_swtch == 2)
                                        {
                                            if (init_directX_main_swtch == 2)
                                            {
                                                consoleHandle = sc_console_core.handle;
                                                /*config = new sc_system_configuration("sc core systems", 1920, 1080, false, false);

                                                handler = sc_console_core.handle;// SC_GLOBALS_ACCESSORS.SC_CONSOLE_CORE.handle;

                                                if (handler == IntPtr.Zero)
                                                {
                                                    //MessageBox((IntPtr)0, "null console ", "_sc_core_systems error", 0);
                                                }
                                                else
                                                {
                                                    //MessageBox((IntPtr)0, "!null console ", "_sc_core_systems error", 0);
                                                }
                                                sc_update = new SC_Update();*/

                                                /*for (int x = 0; x < Console.WindowWidth; x++)
                                                {
                                                    for (int y = 0; y < Console.WindowWidth; y++)
                                                    {
                                                        SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER.Draw(x, y, " ");
                                                    }
                                                }
                                                sc_update = new SC_Update();*/
                                                init_directX_main_swtch = 3;
                                            }

                                            if (init_vr_main_swtch == 2)
                                            {
                                                //config = new sc_system_configuration("sc core systems", 1920, 1080, false, false);

                                                consoleHandle = sc_console_core.handle;

                                                /*Process[] processlist = Process.GetProcesses();

                                                foreach (Process process in processlist)
                                                {
                                                    if (process.ProcessName == "EliteDangerous64")
                                                    {
                                                        //Console.WriteLine("Process: {0} ID: {1} Window title: {2}", process.ProcessName, process.Id, process.MainWindowTitle);
                                                        //MessageBox((IntPtr)0, "ED" + " " + process.MainWindowHandle, "sccsVD4VE_LightNWithoutVr0", 0);
                                                        GameHandle = process.MainWindowHandle;
                                                    }
                                                }*/




                                                //var tester = sc_console_core.handle;
                                                /*consoleHandle = sc_console_core.handle;// SC_GLOBALS_ACCESSORS.SC_CONSOLE_CORE.handle;

                                                if (consoleHandle == IntPtr.Zero)
                                                {
                                                    //MessageBox((IntPtr)0, "null console ", "_sc_core_systems error", 0);
                                                }
                                                else
                                                {
                                                    //MessageBox((IntPtr)0, "!null console ", "_sc_core_systems error", 0);
                                                }*/
                                                //sc_update = new SC_Update();

                                                /*handler = SC_GLOBALS_ACCESSORS.SC_CONSOLE_CORE.handle;

                                                if (handler == IntPtr.Zero)
                                                {
                                                    MessageBox((IntPtr)0, "test 00: ", "_sc_core_systems error", 0);
                                                }

                                                for (int x = 0; x < Console.WindowWidth; x++)
                                                {
                                                    for (int y = 0; y < Console.WindowWidth; y++)
                                                    {
                                                        SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER.Draw(x, y, " ");
                                                    }
                                                }

                                                sc_update = new SC_Update();*/















                                                /*var updateMainUITitle = new Action(() =>
                                                {
                                                    threadOneGrammarLoad();
                                                });

                                                System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);*/
                                                init_vr_main_swtch = 3;
                                                //init_directX_main_swtch = 3;
                                            }

                                            has_init_directx = 1;
                                        }
                                    }



                                    if (init_directX_main_swtch == 3)
                                    {


                                        if (has_spoken_sec == 1)
                                        {
                                            //MainWindow.MessageBox((IntPtr)0, "" + "0", "sccsVD4VE_LightNWithoutVr", 0);
                                            //_sound_byte_array.OrderBy(x => String.Join(String.Empty, x));
                                            //_sound_byte_array.OrderBy(x > x).FirstOrDefault();
                                            //_sound_byte_array = _sound_byte_array.ToList().OrderBy(x => x).ToArray(); // new List<byte>();
                                            //_sound_byte_array = list.OrderBy(x => x).ToArray();
                                            //var test = _sound_byte_array.OrderBy(x => x, new ByteListComparer());

                                            int count_spec = 0;

                                            var widthL = (int)(_inst_spectrum_x / 2);
                                            var widthR = (int)((_inst_spectrum_x / 2) - 1);

                                            var heightL = _inst_spectrum_y;
                                            var heightR = _inst_spectrum_y;

                                            var depthL = (int)(_inst_spectrum_z / 2);
                                            var depthR = (int)((_inst_spectrum_z / 2) - 1);

                                            //var depthL = _inst_spectrum_z;// (int)(_inst_spectrum_z / 2);
                                            //var depthR = _inst_spectrum_z;// (int)((_inst_spectrum_z / 2) - 1);

                                            // wL5 and <=wR4  = -5 to 5 ... 210/2 = 105 and 105 so 105-1 = 104 for wL105 and <=wR104   

                                            for (int x = -widthL; x <= widthR; x++)
                                            {
                                                for (int y = -heightL; y <= heightR; y++)
                                                {
                                                    for (int z = -depthL; z <= depthR; z++)
                                                    {
                                                        //MainWindow.MessageBox((IntPtr)0, "" + "0", "sccsVD4VE_LightNWithoutVr", 0);

                                                        //the higher the closer to center middle.

                                                        float posX = (x);
                                                        float posY = (y);
                                                        float posZ = (z);

                                                        var xx = x;
                                                        var yy = y;
                                                        var zz = z;

                                                        if (xx < 0)
                                                        {
                                                            xx *= -1;
                                                            xx = widthR + xx;
                                                        }
                                                        if (yy < 0)
                                                        {
                                                            yy *= -1;
                                                            yy = heightR + yy;
                                                        }
                                                        if (zz < 0)
                                                        {
                                                            zz *= -1;
                                                            zz = depthR + zz;
                                                        }

                                                        var _index = xx + _inst_spectrum_x * (yy + _inst_spectrum_y * zz);

                                                        Vector3 pos = new Vector3(posX, posY, posZ);

                                                        if (_index < _sound_byte_array.Length)
                                                        {
                                                            //MainWindow.MessageBox((IntPtr)0, "" + _index, "sccsVD4VE_LightNWithoutVr", 0);
                                                        }
                                                        else
                                                        {
                                                            //MainWindow.MessageBox((IntPtr)0, "" + _index, "sccsVD4VE_LightNWithoutVr", 0);
                                                        }

                                                        //var dist = sc_maths.sc_check_distance_node_3d_geometry(Vector3.Zero, pos, 9, 9, 9, 9, 9, 9);

                                                        try
                                                        {




                                                            if (_index < _world_spectrum_list[0][0]._arrayOfInstances.Length && _index < _sound_byte_array.Length) //_inst_spectrum_x * _inst_spectrum_y * _inst_spectrum_z)
                                                            {
                                                                if (_sound_byte_array != null)
                                                                {
                                                                    if (_sound_byte_array.Length > 0)
                                                                    {
                                                                        var _left_touch_pos = new Vector3(WorldMatrix.M41, WorldMatrix.M42, WorldMatrix.M43);
                                                                        /*
                                                                        if (count_spec < _sound_byte_array.Length && count_spec < _world_spectrum_list[0]._arrayOfInstances.Length) // 
                                                                        {

                                                                            /*if (_has_recorded == 1)
                                                                            {
                                                                                var planeSize = 0.01f;
                                                                                var detailscale = 10;
                                                                                var heightmul = 20;
                                                                                var seed = 3420;
                                                                                var fastNoise = new FastNoise(seed);
                                                                                float noise = fastNoise.GetPerlin((((spectrum_mat.M41 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M42 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M43 * planeSize) + seed) / detailscale) * heightmul);
                                                                                _has_recorded = 2;
                                                                            }
                                                                            else
                                                                            {
                                                                                //spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M41;
                                                                                //spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M42;
                                                                                //spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M43;
                                                                            }
                                                                        }
                                                                        else
                                                                        {
                                                                            spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M41;
                                                                            spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value; //0.0015f // + (_sound_byte_array[count_spec] * 0.0005f)
                                                                            spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M43;
                                                                            //count_spec = 0;
                                                                        }*/
                                                                        spectrum_mat.M41 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M41;
                                                                        spectrum_mat.M42 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value + (_sound_byte_array[_index] * 0.005f); //0.0015f
                                                                        spectrum_mat.M43 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M43;

                                                                        worldMatrix_instances_spectrum[0][0][_index] = spectrum_mat;
                                                                        count_spec++;

                                                                        /*else
                                                                        {
                                                                            var _left_touch_pos = new Vector3(WorldMatrix.M41, WorldMatrix.M42, WorldMatrix.M43);

                                                                            spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M41;
                                                                            spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value + (_sound_byte_array[0] * 0.0015f);
                                                                            spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M43;

                                                                            if (_has_recorded == 1)
                                                                            {
                                                                                var planeSize = 0.01f;
                                                                                var detailscale = 100;
                                                                                var heightmul = 20;
                                                                                var seed = 3420;
                                                                                var fastNoise = new FastNoise(3420);
                                                                                float noise = fastNoise.GetPerlin((((spectrum_mat.M41 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M42 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M43 * planeSize) + seed) / detailscale) * heightmul);
                                                                                _has_recorded = 2;
                                                                            }
                                                                            else
                                                                            {
                                                                                //spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M41;
                                                                                //spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M42;
                                                                                //spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M43;
                                                                            }
                                                                            worldMatrix_instances_spectrum[0][_index] = spectrum_mat;
                                                                        }*/
                                                                    }
                                                                }
                                                            }
                                                        }
                                                        catch (Exception ex)
                                                        {
                                                            MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sccsVD4VE_LightNWithoutVr", 0);
                                                        }
                                                        //for (int sba = 0; sba < _sound_byte_array.Length; sba++)
                                                        //{
                                                        //
                                                        //}
                                                    }
                                                }
                                            }

                                            //_sound_byte_array = list.OrderBy(x => x).ThenBy().ToArray();

                                            /*for (int s = 0; s < worldMatrix_instances_spectrum[0].Length; s++)
                                            {

                                            }*/
                                            has_spoken_sec = 0;
                                            has_spoken_main = 0;
                                        }

                                        //Console.WriteLine("test0");

                                        //RECORD SOUND
                                        if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.R))
                                        {
                                            if (sc_start_recording == 0)
                                            {
                                                _records_instant_counter = 0;
                                                _time_of_recording_start = DateTime.Now;

                                                mciSendString("open new Type waveaudio Alias recsound", null, 0, IntPtr.Zero);
                                                mciSendString("record recsound", null, 0, IntPtr.Zero);

                                                sc_start_recording = 1;
                                                has_spoken_main = 1;
                                                //has_spoken_sec = 1;
                                            }
                                        }

                                        if (swtchinstantsound == 1)
                                        {
                                            if (has_spoken_tier == 1 && has_spoken_main == 1)
                                            {
                                                instant_short_path = "wave_instant_audio_" + _records_instant_counter;
                                                var filename = @"C:\Users\ninekorn\Desktop\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";
                                                //var filename = @"D:\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";

                                                mciSendString("save recinstantsound " + filename, null, 0, IntPtr.Zero);
                                                mciSendString("close recinstantsound", null, 0, IntPtr.Zero);

                                                /*Process p = new Process();
                                                p.StartInfo = new ProcessStartInfo()
                                                {
                                                    FileName = short_path
                                                };
                                                p.Start();
                                                p.Refresh();
                                                p.Close();*/

                                                DirectoryInfo dir = new DirectoryInfo(filename);
                                                dir.Refresh();
                                                if (!File.Exists(filename))
                                                {
                                                    FileInfo filinfo = new FileInfo(filename);
                                                    filinfo.Refresh();
                                                }

                                                var nativeFileStream = new NativeFileStream(filename, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                                                SoundStream soundStream = new SoundStream(nativeFileStream);
                                                MemoryStream ms = new MemoryStream();

                                                soundStream.CopyTo(ms);
                                                _sound_byte_array_instant = ms.ToArray();

                                                soundStream.Dispose();

                                                ms.Dispose();
                                                dir.Refresh();

                                                if (File.Exists(filename))
                                                {
                                                    File.Delete(filename);
                                                }

                                                _records_instant_counter++;
                                                has_spoken_tier = 2;
                                            }


                                            if (has_spoken_main == 1)
                                            {
                                                mciSendString("open new Type waveaudio Alias recinstantsound", null, 0, IntPtr.Zero);
                                                mciSendString("record recinstantsound", null, 0, IntPtr.Zero);
                                                //has_spoken_main = 1;
                                            }
                                        }

                                        if (sc_start_recording == 2)
                                        {
                                            if (sc_start_recording_counter >= 50)
                                            {
                                                sc_save_file = 0;
                                                //MessageBox((IntPtr)0, "" + sc_start_recording_counter + "", "Oculus Message", 0);
                                                sc_start_recording = 0;
                                                sc_start_recording_counter = 0;
                                            }
                                            sc_start_recording_counter++;
                                        }

                                        if (_frames_to_access_counter >= 0)
                                        {
                                            if (_records_counter > 0)
                                            {

                                            }
                                            //if (_frames_to_access == 0)
                                            //{
                                            //
                                            //    _frames_to_access = 1;
                                            //}

                                            _frames_to_access_counter = 0;
                                        }


                                        //SAVE SOUND
                                        if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.S))
                                        {
                                            if (sc_start_recording == 1)
                                            {
                                                if (sc_save_file == 0)
                                                {

                                                    short_path = "wave_audio_" + _records_counter;
                                                    var filename = @"C:\Users\ninekorn\Desktop\#RECSOUND\" + "wave_audio_" + _records_counter + ".wav";
                                                    mciSendString("save recsound " + filename, null, 0, IntPtr.Zero);
                                                    mciSendString("close recsound", null, 0, IntPtr.Zero);
                                                    _has_recorded = 1;
                                                    last_wave_filepath = filename;

                                                    string audiotype = "wave_audio_";

                                                    var stringtemp = short_path;
                                                    //"wave_audio_"
                                                    stringtemp.Substring(11, 1);
                                                    var info = new FileInfo(last_wave_filepath);
                                                    info.Refresh();



                                                    System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
                                                    customCulture.NumberFormat.NumberDecimalSeparator = ".";
                                                    System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

                                                    //short_path = "wave_audio_" + _records_counter;
                                                    //path = "c:\\Users\\ninekorn\\Desktop\\testXML\\" + 0 + ".xml";
                                                    path = @"C:\Users\ninekorn\Desktop\#RECSOUND\" + "wave_audio_" + _records_counter + ".xml";
                                                    last_xml_filepath = path;
                                                    //sc_can_start_rec_counter_before_add_index = sc_can_start_rec_counter;
                                                    //doc = new XmlDocument();
                                                    //writer = new XmlTextWriter(Console.Out);
                                                    writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
                                                    //sc_can_start_rec_counter_before_add_index = sc_can_start_rec_counter;

                                                    //root = doc.DocumentElement;

                                                    writer.WriteProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");

                                                    writer.Formatting = Formatting.Indented;
                                                    writer.Indentation = 2;

                                                    writer.WriteStartElement("root"); // open 0
                                                    writer.WriteStartElement("wave"); // open 1
                                                    writer.WriteStartElement("data"); // open 2



                                                    writer.WriteStartElement("StartTime"); //open 3
                                                    writer.WriteString("" + _time_of_recording_start);
                                                    writer.WriteEndElement(); //close 3

                                                    _time_of_recording_end = DateTime.Now;
                                                    writer.WriteStartElement("EndTime"); //open 3
                                                    writer.WriteString("" + _time_of_recording_end);
                                                    writer.WriteEndElement(); //close 3

                                                    writer.WriteStartElement("size"); //open 3
                                                    long size = new System.IO.FileInfo(last_wave_filepath).Length;
                                                    writer.WriteString("" + size);
                                                    writer.WriteEndElement(); //close 3

                                                    writer.WriteStartElement("length"); //open 4
                                                    int length = GetSoundLength(last_wave_filepath);
                                                    writer.WriteString("" + length);
                                                    writer.WriteEndElement(); //close 4


                                                    writer.WriteEndElement(); //close 0
                                                    writer.WriteEndElement(); //close 1
                                                    writer.WriteEndElement(); //close 2

                                                    writer.Close();
                                                    //root.WriteTo(writer);

                                                    //doc.Save(path);

                                                    sc_can_start_rec_counter++;
                                                    _records_counter++;
                                                    //sc_save_file = 0;

                                                    var lastAccess = info.LastAccessTime;

                                                    //_index = sc_can_start_rec_counter_before_add_index + "";
                                                    _sound_player.AddWave(_records_counter - 1 + "", last_wave_filepath);

                                                    /*var nativeFileStream = new NativeFileStream(last_wave_filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                                                    SoundStream soundStream = new SoundStream(nativeFileStream);
                                                    MemoryStream ms = new MemoryStream();

                                                    soundStream.CopyTo(ms);
                                                    _sound_byte_array = ms.ToArray();*/

                                                    var nativeFileStream = new NativeFileStream(last_wave_filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                                                    SoundStream soundStream = new SoundStream(nativeFileStream);
                                                    MemoryStream ms = new MemoryStream();

                                                    soundStream.CopyTo(ms);
                                                    _sound_byte_array = ms.ToArray();

                                                    soundStream.Dispose();
                                                    ms.Dispose();

                                                    _sound_player.Play((_records_counter - 1) + "");
                                                    sc_start_recording = 2;
                                                    sc_save_file = 1;
                                                    has_spoken_main = 1;
                                                    has_spoken_sec = 1;
                                                    //has_spoken_tier = 0;
                                                    //has_spoken_quart = 0;
                                                }
                                            }
                                        }

                                        //PLAY SOUND AT INDEX
                                        if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.P))
                                        {
                                            if (sc_play_file == 0)
                                            {
                                                if (File.Exists(last_wave_filepath))
                                                {
                                                    _sound_player.Play((_records_counter - 1) + "");

                                                    //sc_can_start_rec_counter_before_add_index++;

                                                    //MessageBox((IntPtr)0, "" + stringtemp + "", "Oculus Message", 0);

                                                    //File.Delete(last_wave_filepath);
                                                    /*Process p = new Process();
                                                    p.StartInfo = new ProcessStartInfo()
                                                    {
                                                        FileName = last_wave_filepath
                                                    };
                                                    p.Start();
                                                    p.Refresh();
                                                    p.Close();*/

                                                    //int can_play_index= 0;

                                                    //Array.Sort(_sound_byte_array);
                                                    //_sound_byte_array.OrderBy(x > x).FirstOrDefault();
                                                    //lowestX = point3DCollection.OrderBy(x => x.X).FirstOrDefault();
                                                    //highestX = point3DCollection.OrderBy(x => x.X).Last();

                                                    //for (int s = 0; s < _sound_byte_array.Length;s++)
                                                    //{
                                                    //    if (_sound_byte_array[s] > 100)
                                                    //    {
                                                    //        MessageBox((IntPtr)0, "" + "has sound", "Oculus Message", 0);
                                                    //    }
                                                    //    //MessageBox((IntPtr)0, "" + "has recorded", "Oculus Error", 0);
                                                    //}
                                                    //_main_received_object[0]._someData[0] = _sound_byte_array;// new object[1];

                                                    ///MessageBox((IntPtr)0, "" + _sound_byte_array.Length, "_sc_core_systems error", 0);
                                                    //using (var soundStream = new SoundStream(nativeFileStream))
                                                    //{
                                                    //    using (MemoryStream ms = new MemoryStream())
                                                    //    {
                                                    //        soundStream.CopyTo(ms);
                                                    //        _sound_byte_array = ms.ToArray();
                                                    //        _main_received_messages[0]._someData[0] = _sound_byte_array;// new object[1];
                                                    //    }
                                                    //}

                                                    sc_play_file = 1;
                                                }
                                                else
                                                {
                                                    //create blank file and start listening instead lol. kidding. nerd joke.
                                                }
                                            }
                                        }

                                        if (sc_play_file == 1)
                                        {
                                            if (sc_play_file_counter >= 50)
                                            {
                                                sc_play_file = 0;
                                                sc_play_file_counter = 0;
                                            }
                                            sc_play_file_counter++;
                                        }







                                    }











                                    /*if (init_vr_main_swtch == 3)
                                    {

                                        if (_keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.A))
                                        {
                                            if (hasresetedspeechrecog == 0)
                                            {
                                                var updateMainUITitle = new Action(() =>
                                                {
                                                    threadOneGrammarLoad();
                                                });

                                                System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);
                                                hasresetedspeechrecog = 1;
                                            }
                                        }
                                        else
                                        {
                                            if (hasresetedspeechrecog == 0)
                                            {
                                                var updateMainUITitle = new Action(() =>
                                                {
                                                    threadOneGrammarLoad();
                                                });

                                                System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);
                                                hasresetedspeechrecog = 1;
                                            }
                                        }
                                        /*if (sc_update._can_work_physics == 1)
                                        {
                                            var updateMainUITitle = new Action(() =>
                                            {
                                                threadOneGrammarLoad();
                                            });

                                            System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);
                                            has_init_directx = 2;
                                        }
                                    }*/
                                }



                                //////////////TO READD
                                //////////////TO READD
                                //////////////TO READD
                                /*if (has_init_directx != 1 && init_vr_main_swtch != 3)
                                {
                                    if (ailanguagepriorphysicsengine == 0)
                                    {
                                        if (hasresetedspeechrecog == 1)
                                        {
                                            //var updateMainUITitle = new Action(() =>
                                            //{
                                            //    threadOneGrammarLoad();
                                            //});




                                            //this.Dispatcher.BeginInvoke(updateMainUITitle);
                                            //System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);
                                            //Console.WriteLine("ONE starting or failing to start speech recognition");
                                            
                                            hasresetedspeechrecog = 2;
                                            ailanguagepriorphysicsengine = 1;
                                        }
                                    }
                                }
                                else if (has_init_directx == 1 || init_vr_main_swtch == 3)
                                {
                                    if (sc_update != null)
                                    {
                                        if (sc_update._can_work_physics == 1)
                                        {
                                            if (hasresetedspeechrecog == 3)
                                            {
                                                //var updateMainUITitle = new Action(() =>
                                                //{
                                                //    threadOneGrammarLoad();
                                                //});
                                                //System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);

                                                //Console.WriteLine("TWO starting or failing to start speech recognition");
                                                hasresetedspeechrecog = 4;
                                            }

                                            if (_keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.A))
                                            {
                                                if (hasresetedspeechrecog == 4)
                                                {
                                                    //threadOneGrammarLoad();

                                                    //var updateMainUITitle = new Action(() =>
                                                    //{
                                                    //    threadOneGrammarLoad();
                                                    //});

                                                    //System.Windows.Application.Current.Dispatcher.Invoke(updateMainUITitle);
                                                    hasresetedspeechrecog = 5;
                                                }
                                            }
                                        }
                                    }
                                }

                                if (hasresetedspeechrecog == 0)
                                {
                                    if (hasresetedspeechrecogcounter >= 150)
                                    {
                                        hasresetedspeechrecog = 1;
                                        hasresetedspeechrecogcounter = 0;
                                    }
                                }
                                else if (hasresetedspeechrecog == 2)
                                {
                                    if (hasresetedspeechrecogcounter >= 250)
                                    {
                                        hasresetedspeechrecog = 3;
                                        hasresetedspeechrecogcounter = 0;
                                    }
                                }
                                else if (hasresetedspeechrecog == 5)
                                {
                                    if (hasresetedspeechrecogcounter >= 50)
                                    {
                                        hasresetedspeechrecog = 4;
                                        hasresetedspeechrecogcounter = 0;
                                    }
                                }*/
                                
                                if (recEnginer == null)
                                {
                                    if (_sec_received_messages[14]._swtch0 == -1)
                                    {
                                        string msg = " ";
                                        _sec_received_messages[11]._message = msg;
                                        _sec_received_messages[11]._originalMsg = msg;
                                        _sec_received_messages[11]._messageCut = msg;
                                        _sec_received_messages[11]._specialMessage = 3;
                                        _sec_received_messages[11]._specialMessageLineX = 0;
                                        _sec_received_messages[11]._specialMessageLineY = 0;
                                        _sec_received_messages[11]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[11]._orilineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[11]._lineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[11]._lineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[11]._count = 0;
                                        _sec_received_messages[11]._swtch0 = 1;
                                        _sec_received_messages[11]._swtch1 = 1;
                                        _sec_received_messages[11]._delay = 50;
                                        _sec_received_messages[11]._looping = 0;

                                         msg = " ";
                                        _sec_received_messages[14]._message = msg;
                                        _sec_received_messages[14]._originalMsg = msg;
                                        _sec_received_messages[14]._messageCut = msg;
                                        _sec_received_messages[14]._specialMessage = 3;
                                        _sec_received_messages[14]._specialMessageLineX = 0;
                                        _sec_received_messages[14]._specialMessageLineY = 0;
                                        _sec_received_messages[14]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[14]._orilineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[14]._lineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[14]._lineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[14]._count = 0;
                                        _sec_received_messages[14]._swtch0 = 1;
                                        _sec_received_messages[14]._swtch1 = 1;
                                        _sec_received_messages[14]._delay = 50;
                                        _sec_received_messages[14]._looping = 1;

                                        //Console.WriteLine("test");
                                        //msg = "Speech Recognition Engine disabled";
                                        msg = "#";
                                        _sec_received_messages[14]._message = msg;
                                        _sec_received_messages[14]._originalMsg = msg;
                                        _sec_received_messages[14]._messageCut = msg;
                                        _sec_received_messages[14]._specialMessage = 2;
                                        _sec_received_messages[14]._specialMessageLineX = 0;
                                        _sec_received_messages[14]._specialMessageLineY = 0;
                                        _sec_received_messages[14]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[14]._orilineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[14]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[14]._orilineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[14]._count = 0;
                                        _sec_received_messages[14]._swtch0 = 1;
                                        _sec_received_messages[14]._swtch1 = 1;
                                        _sec_received_messages[14]._delay = 50;
                                        _sec_received_messages[14]._looping = 1;
                                    }
                                }
                                else
                                {
                                    if (_sec_received_messages[15]._swtch0 == -1)
                                    {
                                        string msg = " ";
                                        _sec_received_messages[11]._message = msg;
                                        _sec_received_messages[11]._originalMsg = msg;
                                        _sec_received_messages[11]._messageCut = msg;
                                        _sec_received_messages[11]._specialMessage = 3;
                                        _sec_received_messages[11]._specialMessageLineX = 0;
                                        _sec_received_messages[11]._specialMessageLineY = 0;
                                        _sec_received_messages[11]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[11]._orilineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[11]._lineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[11]._lineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[11]._count = 0;
                                        _sec_received_messages[11]._swtch0 = 1;
                                        _sec_received_messages[11]._swtch1 = 1;
                                        _sec_received_messages[11]._delay = 50;
                                        _sec_received_messages[11]._looping = 1;

                                         msg = " ";
                                        _sec_received_messages[14]._message = msg;
                                        _sec_received_messages[14]._originalMsg = msg;
                                        _sec_received_messages[14]._messageCut = msg;
                                        _sec_received_messages[14]._specialMessage = 3;
                                        _sec_received_messages[14]._specialMessageLineX = 0;
                                        _sec_received_messages[14]._specialMessageLineY = 0;
                                        _sec_received_messages[14]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[14]._orilineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[14]._lineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[14]._lineY = _sec_received_messages[12]._orilineY;
                                        _sec_received_messages[14]._count = 0;
                                        _sec_received_messages[14]._swtch0 = 1;
                                        _sec_received_messages[14]._swtch1 = 1;
                                        _sec_received_messages[14]._delay = 50;
                                        _sec_received_messages[14]._looping = 1;

                                        //msg = "Speech Recognition Engine enabled";
                                        msg = "#";
                                        _sec_received_messages[15]._message = msg;
                                        _sec_received_messages[15]._originalMsg = msg;
                                        _sec_received_messages[15]._messageCut = msg;
                                        _sec_received_messages[15]._specialMessage = 2;
                                        _sec_received_messages[15]._specialMessageLineX = 0;
                                        _sec_received_messages[15]._specialMessageLineY = 0;
                                        _sec_received_messages[15]._orilineX = _sec_received_messages[13]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[15]._orilineY = _sec_received_messages[13]._orilineY;
                                        _sec_received_messages[15]._lineX = _sec_received_messages[13]._orilineX + msgdisabled.Length + 1;
                                        _sec_received_messages[15]._lineY = _sec_received_messages[13]._orilineY;
                                        _sec_received_messages[15]._count = 0;
                                        _sec_received_messages[15]._swtch0 = 1;
                                        _sec_received_messages[15]._swtch1 = 1;
                                        _sec_received_messages[15]._delay = 50;
                                        _sec_received_messages[15]._looping = 1;
                                    }
                                }

                                hasresetedspeechrecogcounter++;

                                /*var DesktopHandle = GetDesktopWindow();
                                //edwindowsfoundedz = FindWindowEx(new IntPtr(DesktopHandle), IntPtr.Zero, "elite-dangerous-64", "");
                                edwindowsfoundedz = FindWindow(null, "elitedangerous64");
                                if (edwindowsfoundedz != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test2", "sccsVD4VE_LightNWithoutVr0", 0);
                                }

                                vewindowsfoundedz = FindWindow(null, "voidexpanse");
                                if (vewindowsfoundedz != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "VE", "sccsVD4VE_LightNWithoutVr0", 0);
                                }*/



                                /*Process[] processlist = Process.GetProcesses();

                                foreach (Process process in processlist)
                                {
                                    if (process.ProcessName == "EliteDangerous64")
                                    {
                                        //Console.WriteLine("Process: {0} ID: {1} Window title: {2}", process.ProcessName, process.Id, process.MainWindowTitle);
                                        MessageBox((IntPtr)0, "ED" + " " + process.MainWindowHandle, "sccsVD4VE_LightNWithoutVr0", 0);

                                    }
                                }*/










                                /*var filename = @"C:\Users\ninekorn\Desktop\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";
                                DirectoryInfo dir = new DirectoryInfo(filename);
                                dir.Refresh();
                                if (!File.Exists(filename))
                                {
                                    FileInfo filinfo = new FileInfo(filename);
                                    filinfo.Refresh();
                                }

                                var nativeFileStream = new NativeFileStream(filename, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                                SoundStream soundStream = new SoundStream(nativeFileStream);
                                MemoryStream ms = new MemoryStream();

                                soundStream.CopyTo(ms);
                                _sound_byte_array_instant = ms.ToArray();

                                soundStream.Dispose();

                                ms.Dispose();
                                dir.Refresh();

                                if (File.Exists(filename))
                                {
                                    File.Delete(filename);
                                }

                                _records_instant_counter++;
                                has_spoken_tier = 2;*/













                                //IS VOID EXPANSE RUNNING
                                //IS VOID EXPANSE RUNNING
                                //IS VOID EXPANSE RUNNING
                                //vewindowsfoundedz = FindWindow(null, "voidexpanse");
                                //edwindowsfoundedz = FindWindow(null, "elitedangerous64");

                                if (veisrunning == 0)
                                {
                                    //edwindowsfoundedz = FindWindow(null, "EliteDangerous64");
                                    vewindowsfoundedz = FindWindow(null, "voidexpanse");

                                    if (vewindowsfoundedz != IntPtr.Zero)
                                    {
                                        //SpeechSynthesizer synth = new SpeechSynthesizer();
                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Volume = synthesizerVolume;
                                        synth.Speak("Void Expanse is running");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        veisrunning = 1;
                                    }
                                    else
                                    {
                                        //SpeechSynthesizer synth = new SpeechSynthesizer();
                                        synth.SetOutputToDefaultAudioDevice();
                                        synth.Volume = synthesizerVolume;
                                        synth.Speak("Void Expanse has stopped running");
                                        Console.WriteLine("synth not working");
                                        veisrunning = 2;
                                    }

                                    var programGameMonitoringTitle = "Game Running: ";
                                    //var msg = "void expanse is not running.";
                                    _sec_received_messages[17]._message = programGameMonitoringTitle;
                                    _sec_received_messages[17]._originalMsg = programGameMonitoringTitle;
                                    _sec_received_messages[17]._messageCut = programGameMonitoringTitle;
                                    _sec_received_messages[17]._specialMessage = 3;
                                    _sec_received_messages[17]._specialMessageLineX = 0;
                                    _sec_received_messages[17]._specialMessageLineY = 0;
                                    _sec_received_messages[17]._orilineX = 0;// _sec_received_messages[17]._orilineX + msgVoidExpanseDisabled.Length + 1;
                                    _sec_received_messages[17]._orilineY = 3;// _sec_received_messages[17]._orilineY;
                                    _sec_received_messages[17]._lineX = _sec_received_messages[17]._orilineX; //+ msgVoidExpanseDisabled.Length + 1
                                    _sec_received_messages[17]._lineY = _sec_received_messages[17]._orilineY;
                                    _sec_received_messages[17]._count = 0;
                                    _sec_received_messages[17]._swtch0 = 1;
                                    _sec_received_messages[17]._swtch1 = 1;
                                    _sec_received_messages[17]._delay = 7;
                                    _sec_received_messages[17]._looping = 1;


                                    _sec_received_messages[16]._message = msgVoidExpanseEnabled;
                                    _sec_received_messages[16]._originalMsg =  msgVoidExpanseEnabled;
                                    _sec_received_messages[16]._messageCut = msgVoidExpanseEnabled;
                                    _sec_received_messages[16]._specialMessage = 2;
                                    _sec_received_messages[16]._specialMessageLineX = 0;
                                    _sec_received_messages[16]._specialMessageLineY = 0;
                                    _sec_received_messages[16]._orilineX = programGameMonitoringTitle.Length + 0;// _sec_received_messages[16]._orilineX + msgVoidExpanseEnabled.Length + 1;
                                    _sec_received_messages[16]._orilineY = 3;// _sec_received_messages[16]._orilineY;
                                    _sec_received_messages[16]._lineX = _sec_received_messages[16]._orilineX; // + msgVoidExpanseEnabled.Length + 1
                                    _sec_received_messages[16]._lineY = _sec_received_messages[16]._orilineY;
                                    _sec_received_messages[16]._count = 0;
                                    _sec_received_messages[16]._swtch0 = 1;
                                    _sec_received_messages[16]._swtch1 = 1;
                                    _sec_received_messages[16]._delay = 50;
                                    _sec_received_messages[16]._looping = 1;
                                }
                                else if (veisrunning == 2)
                                {
                                    var programGameMonitoringTitle = "Game !Running: ";
                                    //var msg = "void expanse is not running.";
                                    _sec_received_messages[17]._message = programGameMonitoringTitle;
                                    _sec_received_messages[17]._originalMsg = programGameMonitoringTitle;
                                    _sec_received_messages[17]._messageCut = programGameMonitoringTitle;
                                    _sec_received_messages[17]._specialMessage = 3;
                                    _sec_received_messages[17]._specialMessageLineX = 0;
                                    _sec_received_messages[17]._specialMessageLineY = 0;
                                    _sec_received_messages[17]._orilineX = 0;// _sec_received_messages[17]._orilineX + msgVoidExpanseDisabled.Length + 1;
                                    _sec_received_messages[17]._orilineY = 3;// _sec_received_messages[17]._orilineY;
                                    _sec_received_messages[17]._lineX = _sec_received_messages[17]._orilineX; //+ msgVoidExpanseDisabled.Length + 1
                                    _sec_received_messages[17]._lineY = _sec_received_messages[17]._orilineY;
                                    _sec_received_messages[17]._count = 0;
                                    _sec_received_messages[17]._swtch0 = 1;
                                    _sec_received_messages[17]._swtch1 = 1;
                                    _sec_received_messages[17]._delay = 7;
                                    _sec_received_messages[17]._looping = 1;

                                    //var msg = "void expanse is not running.";
                                    _sec_received_messages[16]._message = msgVoidExpanseDisabled;
                                    _sec_received_messages[16]._originalMsg = msgVoidExpanseDisabled;
                                    _sec_received_messages[16]._messageCut = msgVoidExpanseDisabled;
                                    _sec_received_messages[16]._specialMessage = 2;
                                    _sec_received_messages[16]._specialMessageLineX = 0;
                                    _sec_received_messages[16]._specialMessageLineY = 0;
                                    _sec_received_messages[16]._orilineX = programGameMonitoringTitle.Length + 0 ;// _sec_received_messages[16]._orilineX + msgVoidExpanseDisabled.Length + 1;
                                    _sec_received_messages[16]._orilineY = 3;// _sec_received_messages[16]._orilineY;
                                    _sec_received_messages[16]._lineX = _sec_received_messages[16]._orilineX; //+ msgVoidExpanseDisabled.Length + 1
                                    _sec_received_messages[16]._lineY = _sec_received_messages[16]._orilineY;
                                    _sec_received_messages[16]._count = 0;
                                    _sec_received_messages[16]._swtch0 = 1;
                                    _sec_received_messages[16]._swtch1 = 1;
                                    _sec_received_messages[16]._delay = 50;
                                    _sec_received_messages[16]._looping = 1;

                                    veisrunning = 3;
                                }
                                //IS VOID EXPANSE RUNNING
                                //IS VOID EXPANSE RUNNING
                                //IS VOID EXPANSE RUNNING



                                /*edwindowsfoundedz = FindWindow(null, "EDLaunch.exe");
                                if (edwindowsfoundedz != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test0", "sccsVD4VE_LightNWithoutVr0", 0);
                                    edisrunning = 1;
                                }
                                else
                                {
                                    //Console.WriteLine("synth not working");
                                }

                                IntPtr hWnd = FindWindowByCaption(IntPtr.Zero, "EDLaunch.exe");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test0", "sccsVD4VE_LightNWithoutVr0", 0);
                                }

                                hWnd = FindWindowByCaption(IntPtr.Zero, "EDLaunch");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test1", "sccsVD4VE_LightNWithoutVr0", 0);
                                }

                                hWnd = FindWindowByCaption(IntPtr.Zero, "edlaunch");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test2", "sccsVD4VE_LightNWithoutVr0", 0);
                                }



                                hWnd = FindWindowByCaption(IntPtr.Zero, "edlaunch.exe");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test3", "sccsVD4VE_LightNWithoutVr0", 0);
                                }



                                hWnd = FindWindowByCaption(IntPtr.Zero, "elitedangerous");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test3", "sccsVD4VE_LightNWithoutVr0", 0);
                                }


                                hWnd = FindWindowByCaption(IntPtr.Zero, "elitedangerous64");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test3", "sccsVD4VE_LightNWithoutVr0", 0);
                                }



                                hWnd = FindWindowByCaption(IntPtr.Zero, "elitedangerous64.exe");

                                if (hWnd != IntPtr.Zero)
                                {
                                    MessageBox((IntPtr)0, "test3", "sccsVD4VE_LightNWithoutVr0", 0);
                                }*/










                                if (edisrunning != 1)
                                {


                                    /*var DesktopHandle = GetDesktopWindow();
                                    Console.WriteLine("Desktop Handle : " + new IntPtr(DesktopHandle).ToString("X"));
                                    //var result = FindWindow(null, "Untitled - Notepad");
                                    //Trace.WriteLine("Current result is : " + result.ToString("X"));
                                    //IntPtr notepadIntPtr = new IntPtr(result);            
                                    //EnumChildWindows(result, myCallBack, 0);
                                    EnumChildWindows(DesktopHandle, myCallBack, 0);*/
                                    //Console.ReadLine();













                                    /*edwindowsfoundedz = FindWindow(null, "elitedangerous");
                                    if (edwindowsfoundedz != IntPtr.Zero)
                                    {
                                        MessageBox((IntPtr)0, "test0", "sccsVD4VE_LightNWithoutVr0", 0);
                                        /*SpeechSynthesizer synth = new SpeechSynthesizer();

                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Speak("Elite Dangerous is running zero");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        edisrunning = 1;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("synth not working");
                                    }


                                    edwindowsfoundedz = FindWindow(null, "elitedangerous64");
                                    if (edwindowsfoundedz != IntPtr.Zero)
                                    {
                                        MessageBox((IntPtr)0, "test1", "sccsVD4VE_LightNWithoutVr0", 0);

                                        /*SpeechSynthesizer synth = new SpeechSynthesizer();

                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Speak("Elite Dangerous is running one");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        edisrunning = 1;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("synth not working");
                                    }


                                    edwindowsfoundedz = FindWindow(null, "elitedangerous64.exe");
                                    if (edwindowsfoundedz != IntPtr.Zero)
                                    {
                                        MessageBox((IntPtr)0, "test2", "sccsVD4VE_LightNWithoutVr0", 0);

                                        /*SpeechSynthesizer synth = new SpeechSynthesizer();

                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Speak("Elite Dangerous is running two");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        edisrunning = 1;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("synth not working");
                                    }


                                    edwindowsfoundedz = FindWindow(null, "EliteDangerous");
                                    if (edwindowsfoundedz != IntPtr.Zero)
                                    {
                                        MessageBox((IntPtr)0, "test3", "sccsVD4VE_LightNWithoutVr0", 0);

                                        /*SpeechSynthesizer synth = new SpeechSynthesizer();

                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Speak("Elite Dangerous is running three");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        edisrunning = 1;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("synth not working");
                                    }

                                    edwindowsfoundedz = FindWindow(null, "EliteDangerous64");
                                    if (edwindowsfoundedz != IntPtr.Zero)
                                    {
                                        MessageBox((IntPtr)0, "test4", "sccsVD4VE_LightNWithoutVr0", 0);

                                        /*SpeechSynthesizer synth = new SpeechSynthesizer();

                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Speak("Elite Dangerous is running four");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        edisrunning = 1;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("synth not working");
                                    }
                                    edwindowsfoundedz = FindWindow(null, "EliteDangerous64.exe");
                                    if (edwindowsfoundedz != IntPtr.Zero)
                                    {
                                        MessageBox((IntPtr)0, "test5", "sccsVD4VE_LightNWithoutVr0", 0);
                                        /*SpeechSynthesizer synth = new SpeechSynthesizer();

                                        // Configure the audio output.   
                                        synth.SetOutputToDefaultAudioDevice();

                                        // Speak a string.  
                                        //synth.Speak("This example demonstrates a basic use of Speech Synthesizer");
                                        synth.Speak("Elite Dangerous is running five");
                                        //Console.WriteLine();
                                        //Console.WriteLine("Press any key to exit...");
                                        //Console.ReadKey();

                                        edisrunning = 1;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("synth not working");
                                    }*/
                                }














                                /*
                                if (haspressedkey4VESWTCH == 1)
                                {

                                    keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                                    //MessageBox((IntPtr)0, "test", "sccsVD4VE_LightNWithoutVr0", 0);
                                    //SendMessage(vewindowsfoundedz, 0, WM_KEYUP, 1);
                                    //MessageBox((IntPtr)0, "test", "sccsVD4VE_LightNWithoutVr1", 0);
                                    //SendMessage(vewindowsfoundedz, 0, WM_KEYUP, 1);
                                    //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 1);
                                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);

                                    //PostMessage(vewindowsfoundedz, WM_KEYUP | KEYEVENTF_EXTENDEDKEY, VK_4, 10);
                                    haspressedkey4VESWTCH = 0;
                                }

                                if (haspressedkey4VE == 1)
                                {
                                    keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);

                                    //MessageBox((IntPtr)0, "test", "sccsVD4VE_LightNWithoutVr00", 0);
                                    //SendMessage(vewindowsfoundedz, 0, WM_KEYDOWN, 1);
                                    //SendMessage(vewindowsfoundedz, 0, WM_KEYUP, 1);
                                    //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 1);
                                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);

                                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN | KEYEVENTF_EXTENDEDKEY, VK_4, 10);

                                    haspressedkey4VESWTCH = 1;
                                    haspressedkey4VE = 0;
                                }*/

                                Thread.Sleep(1);
                                goto _thread_main_loop;
                            }
                            //_thread_start:
                        }, 0); //100000 //999999999

                        _mainTasker00.IsBackground = true;
                        _mainTasker00.SetApartmentState(ApartmentState.STA);
                        _mainTasker00.Start();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                    //SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue(ex.ToString(), 0, 20);
                }
                finally
                {

                    if (_console_worker_task != null)
                    {
                        _console_worker_task.Dispose();
                        _console_worker_task = null;
                    }
                    if (_console_reader_task != null)
                    {
                        _console_reader_task.Dispose();
                        _console_reader_task = null;
                    }
                    if (_console_writer_task != null)
                    {
                        _console_writer_task.Dispose();
                        _console_writer_task = null;
                    }
                    if (_mainTasker00 != null)
                    {
                        //_mainTasker00.Suspend();
                        //_mainTasker00.Abort();
                        _mainTasker00 = null;
                    }
                }
            }

            //ARE COMPONENTS LOADED???
            /*if (_isComponents == false)
            {
                //the main documents/page wpf cannot be loaded. // we continue the program anyway. Later on, my program will start from DOS... fuck it.
            }
            else
            {
                //the main documents/page wpf is loaded. // 
            }*/
        }



        int haspressedkey4VESWTCH = 0;

        int hasresetedspeechrecog = 0;
        int hasresetedspeechrecogcounter = 0;



        //https://stackoverflow.com/questions/1119841/net-console-application-exit-event
        static bool exitSystem = false;
        //#region Trap application termination
        [DllImport("Kernel32")]
        private static extern bool SetConsoleCtrlHandler(EventHandler handler, bool add);
        private delegate bool EventHandler(CtrlType sig);
        static EventHandler _handler;
        enum CtrlType
        {
            CTRL_C_EVENT = 0,
            CTRL_BREAK_EVENT = 1,
            CTRL_CLOSE_EVENT = 2,
            CTRL_LOGOFF_EVENT = 5,
            CTRL_SHUTDOWN_EVENT = 6
        }

        private static bool Handler(CtrlType sig)
        {
            //Console.WriteLine("Exiting system due to external CTRL-C, or process kill, or shutdown");

            //do your cleanup here
            //Thread.Sleep(5000); //simulate some cleanup delay

            //Console.WriteLine("Cleanup complete");

            //allow main to run off
            /*if (sc_update != null)
            {
                sc_update.ShutDown();
                sc_update = null;
            }*/

            //MessageBox((IntPtr)0, "shutting off", "sccsVD4VE_LightNWithoutVr", 0);
            exitSystem = true;

            //shutdown right away so there are no lingering threads
            Environment.Exit(-1);




            return true;
        }
        //#endregion










        public static bool _mainThreadStarterItemsBool = true;
        public static int _mainThreadFrameCounter = 0;

        public static workerThreads _workThread;

        public static int someInitItems = 0;

        public void _mainThreadStarter()
        {
            try
            {
                //windowsToVoidExpanseKeyboardClicksComs();
                //TO READD // NEW SCRIPT MULTIPLY FOR VOID EXPANSE LIGHT AI LIBRARIES TO BE, AS THEY AREN'T LIGHT RIGHT NOW.
                var updateMainUITitle = new Action(() =>
                {
                    if (_UIStarterItemz == 1)
                    {
                        threadOneGrammarLoad();
                        _UIStarterItemz = 0;
                    }
                    //windowsToVoidExpanseKeyboardClicksComs();
                    //sccsScriptAutoCorrectNMultiply();
                });

                Dispatcher.Invoke(updateMainUITitle);
                //TO READD


                



                //_mainUpdateThread();
                while (true)
                {

                    windowsToVoidExpanseKeyboardClicksComs();

                    /*Console.WriteLine("_mainThreadStarter");
                    if (someInitItems == 0)
                    {
                        var updateMainUITitle = new Action(() =>
                        {


                            XmlDocument doc = new XmlDocument();
                            doc.Load("");

                            string pathToDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);


                            string folderName = "#VEscriptgen";
                            DirectoryInfo directoryFolder = new DirectoryInfo(folderName);
                            directoryFolder.Refresh();

                            if (!Directory.Exists(pathToDesktop + @"\" + folderName))
                            {
                                Console.WriteLine("the directory !exists");
                                Directory.CreateDirectory(pathToDesktop + @"\" + folderName);
                            }
                            else
                            {
                                Console.WriteLine("the directory exists");
                            }


                            _mainUpdateThread();
                        });

                        Dispatcher.Invoke(updateMainUITitle);

                        /*if (!File.Exists(folderName))
                        {
                            FileInfo filinfo = new FileInfo(folderName);
                            filinfo.Refresh();
                        }

                        //threadOneGrammarLoad();
                        someInitItems = 3;
                    }*/



                    /*var updateMainUITitle = new Action(() =>
                    {
                        if (_UIStarterItemz == 1)
                        {
                            threadOneGrammarLoad();
                            _UIStarterItemz = 0;
                        }

                        _mainUpdateThread();
                    });

                    Dispatcher.Invoke(updateMainUITitle);*/ //System.Windows.Application.Current.

                    /*if (_mainThreadStarterItemsBool)
                    {
                        var _threadID = 0;
                        //threadOneGrammarLoad();

                        //Creating multiple threads. Can be sent with invoke or without it seems. If created inside of another thread, I think it would be best.
                        /*for (int i = 0; i < _totalThreads; i++)
                        {
                            _workThread = new workerThreads(i);
                        }

                        var updateMainUITitle = new Action(() =>
                        {
                            /*if (_UIStarterItemz == 1)
                            {
                                threadOneGrammarLoad();
                                _UIStarterItemz = 0;
                            }

                            _mainUpdateThread();
                        });

                        Dispatcher.Invoke(updateMainUITitle);






                        _mainThreadStarterItemsBool = false;
                    }*/
                    //windowsToVoidExpanseKeyboardClicksComs();
                    _mainThreadFrameCounter++;

                    //Console.WriteLine("test");
                    Thread.Sleep(1);
                }
            }
            catch
            {

            }

        }

        public static int _UIStarterItemz = 1;


        static int hasSpoken = 0;
        static int keyboardclickscounter = 0;
        static int keyboardclicksend = 0;
        static int maxkeyframe = 0;



        static int isDispatching = 0;


























        //UI THREAD TEST
        //////////////////////////////////
        //////////////////////////////////
        public static void sccsScriptAutoCorrectNMultiply()
        {



            //Console.WriteLine("0_mainThreadStarter");
            if (someInitItems == 0)
            {
                //Console.WriteLine("1_mainThreadStarter");
                var updateMainUITitle2 = new Action(() =>
                {
                    


                    //_mainUpdateThread();
                });
               // XmlDocument doc = new XmlDocument();
                //doc.Load("");

                string pathToDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

                //Console.WriteLine("2_mainThreadStarter");
                string folderName = "#VEscriptgen";
                DirectoryInfo directoryFolder = new DirectoryInfo(folderName);
                directoryFolder.Refresh();
                string pathToDesktopFolder = pathToDesktop + @"\" + folderName;

                //Console.WriteLine("2_mainThreadStarter");
                if (!Directory.Exists(pathToDesktopFolder))
                {
                    Console.WriteLine("0the directory !exists");
                    Directory.CreateDirectory(pathToDesktopFolder);
                }
                else
                {
                    Console.WriteLine("0the directory exists");
                }



                string folderNameOriginal =   "original";
                DirectoryInfo dirnameOriginal = new DirectoryInfo(folderNameOriginal);
                dirnameOriginal.Refresh();
                //Console.WriteLine("2_mainThreadStarter");
                if (!Directory.Exists(pathToDesktop + @"\" + folderName))
                {
                    Console.WriteLine("1the directory !exists");
                    Directory.CreateDirectory(folderNameOriginal);
                }
                else
                {
                    Console.WriteLine("1the directory exists");
                }

                string finalPathOriginal = pathToDesktop + @"\" + folderName + @"\" + folderNameOriginal;
                Console.WriteLine(finalPathOriginal);




                string folderNameMul = "multiplied";
                string mulPath = pathToDesktop + @"\" + folderName + @"\" + folderNameMul;
                if (!Directory.Exists(mulPath))
                {
                    Console.WriteLine("2the directory !exists");
                    Directory.CreateDirectory(mulPath);
                }
                else
                {
                    Console.WriteLine("2the directory exists");
                }


                var arrayOfFiles = Directory.GetFiles(finalPathOriginal);


                if (arrayOfFiles.Length > 0)
                {
                    Console.WriteLine("Got Files");
                }
                else
                {
                    //Console.WriteLine("!Got Files");
                }

                DirectoryInfo dirnameMulNewFiles = new DirectoryInfo(mulPath);

                string[] arrayOfOnlyfileNamesFormationOne = new string[arrayOfFiles.Length];
                string[] arrayOfOnlyfileNamesFormationORIGINAL = new string[arrayOfFiles.Length];

                var indexToChange = 0;
                var mainIterator = 0;

                var formationLength = 5;


                for (int i = 0; i < arrayOfOnlyfileNamesFormationORIGINAL.Length; i++)
                {
                    arrayOfOnlyfileNamesFormationORIGINAL[i] = arrayOfFiles[i];
                }


                for (int f = 0; f < formationLength; f++)
                {
                
                    var indexMain = f + 1;
                    for (int i = 0; i < arrayOfFiles.Length; i++)
                    {
                        arrayOfFiles[i] = arrayOfOnlyfileNamesFormationORIGINAL[i];
                        /*if (mainIterator >= arrayOfFiles.Length)
                        {
                            mainIterator = 0;
                            indexToChange += 1;
                        }*/
                        //Console.WriteLine("3the directory exists");

                        var filenewPath = folderNameMul + @"\" + arrayOfFiles[i];

                        var someString = pathToDesktop + @"\" + folderName + @"\" + folderNameOriginal + @"\";
                        //var test = arrayOfFiles[i];
                        var newstring = arrayOfFiles[i].Substring(someString.Length);
                        arrayOfOnlyfileNamesFormationOne[i] = newstring;

                        //var removedExtension = arrayOfFiles[i].Substring(someString.Length);

                        var someOtherString = Path.ChangeExtension(newstring, "");
                        //Console.WriteLine(newstring);

                        var somePathOri = Path.Combine(pathToDesktop + @"\" + folderName + @"\" + folderNameOriginal, newstring);
                        var somePathMul = Path.Combine(pathToDesktop + @"\" + folderName + @"\" + folderNameMul, newstring);

                        var someStringOri = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\";
                        var somestuff = Path.ChangeExtension(somePathMul, ".xml");

                        somestuff = somestuff.Substring(someStringOri.Length);

                        var arrayOfFilesToMod = somestuff;
                        string st = "_" + (indexMain);
                        //arrayOfFilesToMod.Replace("_1", st);

                        /*Regex regex = new Regex(@"cc_");
                        var input = arrayOfFilesToMod;
                        var output = regex.Replace(input, "Mining_");*/

                        Regex regex2 = new Regex(@"_1");
                        var input2 = arrayOfFilesToMod; //output
                        var output2 = regex2.Replace(input2, "_" + indexMain);

                        /*foreach (var test in arrayOfFilesToMod.Where(x => x == "_1"))
                        {
                            
                        }*/

                        //File.ReadAllText(arrayOfFilesToMod).Replace("_1", st);
                        //Console.WriteLine(output);

                        var lastPath = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\" + output2;
                        File.Copy(arrayOfFiles[i], Path.ChangeExtension(lastPath, ".xml"), true);
                    }
                }

                dirnameMulNewFiles.Refresh();
      
                arrayOfOnlyfileNamesFormationOne = Directory.GetFiles(mulPath);

                string[] arrayOfFilesNames = new string[arrayOfOnlyfileNamesFormationOne.Length];
                for (int i = 0; i < arrayOfOnlyfileNamesFormationOne.Length; i++)
                {
                    arrayOfFilesNames[i] = arrayOfOnlyfileNamesFormationOne[i];// Replace("1", "2");
                }

                /*string[] arrayOfOnlyfileNamesFormationTwo = new string[arrayOfOnlyfileNamesFormationOne.Length];
                string[] arrayOfOnlyfileNamesFormationThree = new string[arrayOfOnlyfileNamesFormationOne.Length];
                string[] arrayOfOnlyfileNamesFormationFour = new string[arrayOfOnlyfileNamesFormationOne.Length];
                string[] arrayOfOnlyfileNamesFormationFive = new string[arrayOfOnlyfileNamesFormationOne.Length];

                for (int i = 0; i < arrayOfOnlyfileNamesFormationOne.Length; i++)
                {
                    arrayOfOnlyfileNamesFormationTwo[i] = arrayOfOnlyfileNamesFormationOne[i];// Replace("1", "2");
                    arrayOfOnlyfileNamesFormationThree[i] = arrayOfOnlyfileNamesFormationOne[i];// .Replace("2", "3");
                    arrayOfOnlyfileNamesFormationFour[i] = arrayOfOnlyfileNamesFormationOne[i];// .Replace("3", "4");
                    arrayOfOnlyfileNamesFormationFive[i] = arrayOfOnlyfileNamesFormationOne[i];// .Replace("4", "5");
                }*/




                for (int f = 0; f < formationLength; f++)
                {
                    var indexMain = f + 1;

                    var currentStringIndexMain = "_" + indexMain;

                    for (int i = 0; i < arrayOfOnlyfileNamesFormationOne.Length; i++)
                    {
                        if (arrayOfOnlyfileNamesFormationOne[i].Contains(currentStringIndexMain))
                        {
                            var someString = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\";
                            var newstring = arrayOfOnlyfileNamesFormationOne[i].Substring(someString.Length);
                            var someLastString = someString + newstring;

                            Console.WriteLine(someLastString);


                            var menuOption = 1;

                            if (menuOption == 0)
                            {

                                var lastString = "Mining_";
                                var lastString2 = "_" + indexMain;
                                File.WriteAllText(someLastString, File.ReadAllText(someLastString).Replace("Combat_cc_", lastString).Replace("_1", lastString2));



                                if (File.Exists(someLastString))
                                {
                                    //Console.WriteLine("02File.Exists");
                                    FileInfo filinfo = new FileInfo(someLastString);
                                    filinfo.Refresh();
                                }


                                //var newString = File.ReadAllText(someLastString);

                                lastString = "Mining_";
                                lastString2 = "_" + indexMain;
                                File.WriteAllText(someLastString, File.ReadAllText(someLastString).Replace("cc_", lastString).Replace("_1", lastString2));

                            }
                            else
                            {
                                /*var lastString = "Mining_";
                                var lastString2 = "_" + indexMain;
                                File.WriteAllText(someLastString, File.ReadAllText(someLastString).Replace("Combat_cc_", lastString).Replace("_1", lastString2));*/



                                if (File.Exists(someLastString))
                                {
                                    //Console.WriteLine("02File.Exists");
                                    FileInfo filinfo = new FileInfo(someLastString);
                                    filinfo.Refresh();
                                }


                                //var newString = File.ReadAllText(someLastString);

                                //lastString = "Mining_";
                                var lastString2 = "_" + indexMain;
                                File.WriteAllText(someLastString, File.ReadAllText(someLastString).Replace("_1", lastString2));
                            }














                            //Console.WriteLine(someLastString);

                            //File.WriteAllText(someLastString, File.ReadAllText(someLastString).Replace("_1", lastString));

                            //Regex regex2 = new Regex(@"_1");
                            // var input2 = arrayOfFilesToMod;
                            //var output2 = regex2.Replace(input2, "_" + indexMain);


                            //Regex regex = new Regex(@"cc_1");
                            //var input = someLastString;
                            //var output = regex.Replace(input, "Mining_" + indexMain);

                            //File.WriteAllText(someLastString, output);

                            File.Copy(someLastString, Path.ChangeExtension(someLastString, ".js"), true);
                        }
                    }
                }

                dirnameMulNewFiles.Refresh();

                //arrayOfOnlyfileNamesFormationOne = Directory.GetFiles(mulPath);



                for (int i = 0; i < arrayOfFilesNames.Length; i++)
                {
                    //Console.WriteLine(arrayOfFilesNames[i]);
                    //Directory.Fil(mulPath + @"\" + arrayOfOnlyfileNamesFormationOne[i]);
                    if (File.Exists(arrayOfOnlyfileNamesFormationOne[i]))
                    {
                        //Console.WriteLine("02File.Exists");
                        FileInfo filinfo = new FileInfo(arrayOfOnlyfileNamesFormationOne[i]);
                        filinfo.Delete();
                    }

                }



                dirnameMulNewFiles.Refresh();





                Console.WriteLine("ended scripts multiplication");

                //var someString = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\";
                //var test = arrayOfFiles[i];
                //var newstring = arrayOfFiles[i].Substring(someString.Length);
                //var someTest = newstring; // arrayOfOnlyfileNamesFormationOne[i] 



                /*for (int i = 0; i < arrayOfOnlyfileNamesFormationOne.Length; i++)
                {
                    if (arrayOfOnlyfileNamesFormationOne[i].Contains(""))
                    {

                    }
                    //Console.WriteLine(arrayOfOnlyfileNamesFormationFive[i]);

                    if (File.Exists(arrayOfOnlyfileNamesFormationOne[i]))
                    {
                        //Console.WriteLine("02File.Exists");
                        FileInfo filinfo = new FileInfo(arrayOfOnlyfileNamesFormationOne[i]);
                        filinfo.Refresh();
                    }
                    var filenewPath = folderNameMul + @"\" + arrayOfOnlyfileNamesFormationOne[i];

                    var someString = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\";
                    //var test = arrayOfOnlyfileNamesFormationOne[i];
                    var newstring = arrayOfOnlyfileNamesFormationOne[i].Substring(someString.Length);

                    //newstring = newstring+ ".xml";

                    //var somePathMul = Path.Combine(pathToDesktop + @"\" + folderName + @"\" + folderNameMul, newstring);

                    var someLastString = someString + newstring;

                    //Console.WriteLine(someLastString);

                    if (File.Exists(someLastString))
                    {
                        //Console.WriteLine("02File.Exists");
                        FileInfo filinfo = new FileInfo(someLastString);
                        filinfo.Refresh();

                        var someResults = File.ReadAllText(someLastString);

                        someResults.Replace("_1", "_5");

                        //File.WriteAllText(someLastString, someResults);

                        File.WriteAllText(someLastString, File.ReadAllText(someLastString).Replace("_1", "_5"));
                    }
                    else
                    {
                       // Console.WriteLine("02File.!Exists");
                    }
                }*/

























                //arrayOfOnlyfileNames[i]
                //Console.WriteLine(someResults);








                /*using (XmlReader xmlreader = new XmlTextReader(someLastString))
                {
                    //Console.WriteLine("element: ");
                    while (xmlreader.Read())
                    {
                        Console.WriteLine("element: ");
                        //var element = xmlreader.Name;
                        //Console.WriteLine("element: " + element);
                    }
                }*/
                /*XDocument doc = XDocument.Load(someLastString);

                using (var reader = XmlReader.Create(someLastString))
                {
                    reader.Dispose();
                    //doc = XDocument.Load(lastPath2);
                }*/



                //XDocument doc = new XDocument();
                //string xmlString = "<book><title>Oberon's Legacy</title></book>";
                //doc.Load(new StringReader(someLastString));
                //var lastPath2 = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\" + someOtherString + ".xml";
                //XmlDocument doc = new XmlDocument();
                //doc.Load(lastPath);










                //File.Move(arrayOfFiles[i], Path.ChangeExtension(lastPath, ".xml"));
                //dirnameMulNewFiles.Refresh();

                // Console.WriteLine("4the directory exists " + arrayOfFiles[i]);

                /*if (!File.Exists(lastPath))
                {
                    Console.WriteLine("00!File.Exists");
                    FileInfo filinfo = new FileInfo(lastPath);
                    filinfo.Refresh();
                }
                else
                {
                    Console.WriteLine("01File.Exists");
                }*/
                //var result = Path.ChangeExtension(lastPath, ".xml");


                /*var lastPath2 = pathToDesktop + @"\" + folderName + @"\" + folderNameMul + @"\" + someOtherString + ".xml";
                //XmlDocument doc = new XmlDocument();
                //doc.Load(lastPath);


                if (File.Exists(lastPath2))
                {
                    Console.WriteLine("02File.Exists");
                    FileInfo filinfo = new FileInfo(lastPath2);
                    filinfo.Refresh();
                }
                else
                {
                    Console.WriteLine("02File.!Exists");
                }*/
                //XDocument doc = XDocument.Load(lastPath2);
                //doc.Save(lastPath2);


                /*using (XmlTextReader textReader = new XmlTextReader(lastPath2))
                {
                    while (textReader.Read())
                    {
                        var element = textReader.Name;
                        //Console.WriteLine(element + "");

                    }
                    textReader.Close();
                }*/


                //XmlDocument doc = new XmlDocument();
                //doc.Load(lastPath2);


                //                   System.Text.Encoding.UTF8







                //textReader.Read();




                /*XDocument doc = XDocument.Load(lastPath2);

                using (var reader = XmlReader.Create(lastPath2))
                {
                    reader.Dispose();
                    //doc = XDocument.Load(lastPath2);
                }*/



                //XDocument doc = XDocument.Load(lastPath);
                //doc.Save(lastPath2);




                //System.Xml.XmlText

                //XmlReader xmlreader = new XmlReader();

                //XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
                /*using (XmlTextReader xmlreader = new XmlTextReader(lastPath2))
                {
                    while (xmlreader.Read())
                    {

                    }
                }*/
                //new StringReader(lastPath2))

                //File.Move();

                /*if (File.Exists(arrayOfFiles[i]))
                {
                    FileInfo filinfo = new FileInfo(arrayOfFiles[i]);
                    filinfo.Refresh();
                }*/

                /*var result = Path.ChangeExtension(lastPath,".xml");
                Console.WriteLine(result);
                if (File.Exists(lastPath))
                {
                    FileInfo filinfo = new FileInfo(lastPath);
                    filinfo.Refresh();
                }*/




                /*for (int i = 0; i < arrayOfOnlyfileNamesFormationOne.Length; i++)
                {
                    arrayOfOnlyfileNamesFormationTwo[i].Replace("1", "2");
                    arrayOfOnlyfileNamesFormationThree[i].Replace("1", "3");
                    arrayOfOnlyfileNamesFormationFour[i].Replace("1", "4");
                    arrayOfOnlyfileNamesFormationFive[i].Replace("1", "5");
                }*/






                //XmlDocument doc = new XmlDocument();
                //doc.Load("");










                //Console.WriteLine("3_mainThreadStarter");
                //System.Windows.Threading.Dispatcher.Invoke(updateMainUITitle2);

                /*if (!File.Exists(folderName))
                {
                    FileInfo filinfo = new FileInfo(folderName);
                    filinfo.Refresh();
                }*/

                //threadOneGrammarLoad();
                someInitItems = 3;
            }






            //Console.Title = "" + _mainThreadFrameCounter.ToString();

            //SC_GCGollect.GCCollectUtility(100);
        }
        //////////////////////////////////
        //////////////////////////////////



        static int frameCounterMAX = 35;

        static int dispatchMSGCounter = 0;
        //UI THREAD TEST
        //////////////////////////////////
        //////////////////////////////////
        public static void windowsToVoidExpanseKeyboardClicksComs()
        {


            if (hasSpoken == 1)
            {
                if (lastWord == "computer" && isDispatching == 0)
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //have the keyboardsim send the input to the void expanse window instead of wherever on the desktop
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        string msg = " ";
                        _sec_received_messages[19]._message = msg;
                        _sec_received_messages[19]._originalMsg = msg;
                        _sec_received_messages[19]._messageCut = msg;
                        _sec_received_messages[19]._specialMessage = 3;
                        _sec_received_messages[19]._specialMessageLineX = 0;
                        _sec_received_messages[19]._specialMessageLineY = 0;
                        _sec_received_messages[19]._orilineX = 0;
                        _sec_received_messages[19]._orilineY = 5;
                        _sec_received_messages[19]._lineX = 0;
                        _sec_received_messages[19]._lineY = 5;
                        _sec_received_messages[19]._count = 0;
                        _sec_received_messages[19]._swtch0 = 1;
                        _sec_received_messages[19]._swtch1 = 1;
                        _sec_received_messages[19]._delay = 7;
                        _sec_received_messages[19]._looping = 0;

                        //msg = "Speech Recognition Engine enabled";
                        msg = lastWord + " " + keyboardclicksend;
                        _sec_received_messages[19]._message = msg;
                        _sec_received_messages[19]._originalMsg = msg;
                        _sec_received_messages[19]._messageCut = msg;
                        _sec_received_messages[19]._specialMessage = 2;
                        _sec_received_messages[19]._specialMessageLineX = 0;
                        _sec_received_messages[19]._specialMessageLineY = 0;
                        _sec_received_messages[19]._orilineX = 0;
                        _sec_received_messages[19]._orilineY = 5;
                        _sec_received_messages[19]._lineX = 0;
                        _sec_received_messages[19]._lineY = 5;
                        _sec_received_messages[19]._count = 0;
                        _sec_received_messages[19]._swtch0 = 1;
                        _sec_received_messages[19]._swtch1 = 1;
                        _sec_received_messages[19]._delay = 7;
                        _sec_received_messages[19]._looping = 1;

                        //Console.WriteLine("test0");
                        //dispatchMSGCounter++;
                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }
                    if (keyboardclicksend >= 2)
                    {
                        isDispatching = 1;
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }




                    

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    /**/

                    /*if (keyboardclickscounter >= 0)
                    {
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);
                        Console.WriteLine("test0");
                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    if (keyboardclicksend >= 1)
                    {
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }*/
                    //Console.WriteLine("test0");
                }
                else if (lastWord == "market")
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //SendMessage(vewindowsfoundedz, WM, VK_4, 0);
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                        //Console.WriteLine("test1");

                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    if (keyboardclicksend >= 1)
                    {
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }
                }



                if (lastWord == "release" && isDispatching == 1)
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //SendMessage(vewindowsfoundedz, WM, VK_4, 0);
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                        //Console.WriteLine("test1");

                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    if (keyboardclicksend >= 2)
                    {
                        isDispatching = 2;
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }
                }
                else if (lastWord == "retrieve" && isDispatching == 1)
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //SendMessage(vewindowsfoundedz, WM, VK_4, 0);
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                        //Console.WriteLine("test1");

                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    if (keyboardclicksend >= 3)
                    {
                        isDispatching = 2;
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }
                }



                if (lastWord == "combat" && isDispatching == 2)
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //SendMessage(vewindowsfoundedz, WM, VK_4, 0);
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                        //Console.WriteLine("test1");

                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    if (keyboardclicksend >= 2)
                    {
                        isDispatching = 0;
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }
                }
                else if (lastWord == "repair" && isDispatching == 2)
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //SendMessage(vewindowsfoundedz, WM, VK_4, 0);
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                        //Console.WriteLine("test1");

                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    if (keyboardclicksend >= 3)
                    {
                        isDispatching = 0;
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }
                }
                else if (lastWord == "mining" && isDispatching == 2)
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    if (keyboardclickscounter >= frameCounterMAX)
                    {
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //SendMessage(vewindowsfoundedz, WM, VK_4, 0);
                        keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                        keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                        //Console.WriteLine("test1");

                        keyboardclicksend++;
                        keyboardclickscounter = 0;
                    }

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardclicksend++;

                    if (keyboardclicksend >= 4)
                    {
                        isDispatching = 0;
                        keyboardclicksend = 0;
                        hasSpoken = 0;
                    }
                }



                if (lastWord == "start")
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    //veisrunning = 0;
                    if (recEnginer == null)
                    {
                        try
                        {
                            //gb = new GrammarBuilder();
                            //gb.Append(mychoices);
                            //mygrammar = new Grammar(gb);
                            //recEnginer.LoadGrammarAsync(mygrammar);

                            recEnginer.SpeechRecognized -= new EventHandler<SpeechRecognizedEventArgs>(recEngine_SpeechRecognized);

                            //recEnginer.SetInputToDefaultAudioDevice();
                            //recEnginer.RecognizeAsync(RecognizeMode.Multiple);
                        }
                        catch (Exception ex)
                        {
                            MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sc core systems message", 0);
                        }
                    }

                    hasSpoken = 0;
                }
                else if (lastWord == "stop")
                {
                    resetVoidExpanserecog.Restart();
                    counterForResetingVoidExpanseMenu = 0;
                    try
                    {
                        //gb = new GrammarBuilder();
                        //gb.Append(mychoices);
                        //mygrammar = new Grammar(gb);
                        //recEnginer.LoadGrammarAsync(mygrammar);

                        recEnginer.SpeechRecognized -= new EventHandler<SpeechRecognizedEventArgs>(recEngine_SpeechRecognized);

                        //recEnginer.SetInputToDefaultAudioDevice();
                        //recEnginer.RecognizeAsync(RecognizeMode.Multiple);
                    }
                    catch (Exception ex)
                    {
                        MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sc core systems message", 0);
                    }

                    hasSpoken = 0;
                }

                keyboardclickscounter++;
            }

            if (resetVoidExpanserecog.Elapsed.TotalSeconds > 5) // 5 seconds for 120 frames per seconds in void expanse.
            {
                
                /*if (counterForResetingVoidExpanseMenu >= maxcounterForResetingVoidExpanseMenu)
                {
                    MainWindow.MessageBox((IntPtr)0, "counter over", "sc core systems message", 0);

                    isDispatching = 0;
                    counterForResetingVoidExpanseMenu = 0;
                }*/

                //MainWindow.MessageBox((IntPtr)0, "counter over", "sc core systems message", 0);

                _sec_received_messages[17]._message = "reseted comms";
                _sec_received_messages[17]._originalMsg = "reseted comms";
                _sec_received_messages[17]._messageCut = "reseted comms";
                _sec_received_messages[17]._specialMessage = 2;
                _sec_received_messages[17]._specialMessageLineX = 0;
                _sec_received_messages[17]._specialMessageLineY = 0;
                _sec_received_messages[17]._orilineX = 1;// _sec_received_messages[17]._orilineX + msgVoidExpanseDisabled.Length + 1;
                _sec_received_messages[17]._orilineY = 11;// _sec_received_messages[17]._orilineY;
                _sec_received_messages[17]._lineX = 1;// _sec_received_messages[17]._orilineX + msgVoidExpanseDisabled.Length + 1;
                _sec_received_messages[17]._lineY = 11;//_sec_received_messages[17]._orilineY;
                _sec_received_messages[17]._count = 0;
                _sec_received_messages[17]._swtch0 = 1;
                _sec_received_messages[17]._swtch1 = 1;
                _sec_received_messages[17]._delay = 25;
                _sec_received_messages[17]._looping = 0;


                isDispatching = 0;
                counterForResetingVoidExpanseMenu = 0;
                counterForResetingVoidExpanseMenu++;
                resetVoidExpanserecog.Stop();
                resetVoidExpanserecog.Reset();
            }




            //Console.Title = "" + _mainThreadFrameCounter.ToString();

            //SC_GCGollect.GCCollectUtility(100);
        }


        static int counterForResetingVoidExpanseMenu = 0;
        static int maxcounterForResetingVoidExpanseMenu = 350;

        static Stopwatch resetVoidExpanserecog = new Stopwatch();

        public class workerThreads
        {
            public int _mainFrameCounterThreadOne = 0;
            public int _availableThreads = 0;
            public ThreadStart _threadStart;
            public Thread[] _listOfThreads;
            public int _threadID;
            public Thread _thread;


            public workerThreads(int threadID)
            {

                this._threadID = threadID;

                _threadStart = new ThreadStart(() =>
                {
                    threadOneMainDispatcherUpdate(_threadID);
                });

                _thread = new Thread(_threadStart);
                _thread.IsBackground = true;
                _thread.SetApartmentState(ApartmentState.STA);
                _thread.Start();
            }

            public static int _whosFirst = 0;
            public static int _threadCreationCounter = 0;
            public static bool _canLoadStarterItems = true;

            //[STAThread]
            void threadOneMainDispatcherUpdate(int threadIndex)
            {
                int _threadIndex = threadIndex;
                int _workerFrame = 0;

                try
                {
                    //SC_Console.consoleMessageQueue messageQueue00 = new SC_Console.consoleMessageQueue("SC_AvailableThreads#" + _totalThreads, 0, 0);
                    //SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("#" + _threadIndex + "", 0, 3 + _threadIndex);


                    while (true)
                    {
                        //List of functions that the multithreaded app will start every frames.
                        ////////////////////
                        //_threadUpdateTest();
                        ////////////////////
                        if (_canLoadStarterItems)
                        {
                            //Console.WriteLine("grammar load");
                            //threadOneGrammarLoad();
                            _canLoadStarterItems = false;
                        }


                        if (_workerFrame > -1)
                        {
                            _threadCreationCounter++;
                            _workerFrame = 0;
                        }

                        _workerFrame++;
                        Thread.Sleep(1);
                    }
                }
                //#pragma warning disable CS0168 // The variable 'ex' is declared but never used
                catch (Exception ex)
                //#pragma warning restore CS0168 // The variable 'ex' is declared but never used
                {
                    SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue(ex.ToString(), 0, 20);
                }
            }
        }


        //static SpeechRecognitionEngine recEngine = new SpeechRecognitionEngine();

        /*static bool mathSwitch = false;
        static string[] stringOfMathOps;
        static bool firstInput = false;
        static bool secondInput = false;*/


        /*static int functionCounter00 = 1;
        static int functionCounter01 = 1;
        static int functionCounter02 = 1;*/


        //static string totalCombination = "";

        //static string currentWord = "";


        static List<string> currentListOfCommands = new List<string>();
        static int counterTotalWords = 0;
        static int functionCounter03 = 1;
        static string lastWord = "";
        static int frameCounterForVoiceRecognitionRecognizedWords = 0;

        //static int frameCounterForVoiceRecognition = 0;
        //https://docs.microsoft.com/en-us/dotnet/api/system.speech.recognition.speechrecognitionengine.recognizeasync?view=netframework-4.8
        static bool completed;


        //private static SpeechRecognitionEngine recEnginer =new SpeechRecognitionEngine("en-GB");
        static GrammarBuilder gb;
        static Grammar mygrammar;
        static Choices mychoices;// = new Choices();

        static SpeechRecognitionEngine recEnginer;
        public void threadOneGrammarLoad()
        {
            var enUS = new System.Globalization.CultureInfo("en-US");
            //SpeechRecognitionEngine rec = new SpeechRecognitionEngine(enUS);


            recEnginer = new SpeechRecognitionEngine(new CultureInfo("en-US"));
            //Console.WriteLine("test0");

            mychoices = new Choices();
            //mychoices.Add(new string[] { "Ok", "Test", "Hello" });
            //mychoices.Add(new string[] { "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z" });
            //mychoices.Add(new string[] { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z" });
            //mychoices.Add(new string[] { "Ok", "Play", "Stop", "Cancel", "Repeat", "Select", "Backward", "Forward" }); //, "Attack"
            mychoices.Add(new string[] { "start", "stop",
                                         "computer", "market", "release", "retrieve","combat", "repair", "mining",
                                         "attack",});



            try
            {
                gb = new GrammarBuilder();
                gb.Append(mychoices);
                mygrammar = new Grammar(gb);
                recEnginer.LoadGrammarAsync(mygrammar);

                recEnginer.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(recEngine_SpeechRecognized);

                recEnginer.SetInputToDefaultAudioDevice();
                recEnginer.RecognizeAsync(RecognizeMode.Multiple);
            }
            catch (Exception ex)
            {
                MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sc core systems message", 0);
            }














            /*
            using (SpeechRecognitionEngine recognizer = new SpeechRecognitionEngine(new CultureInfo("en-US")))
            {
                // Create a grammar for choosing cities for a flight.  
                Choices cities = new Choices(new string[]
                { "Los Angeles", "New York", "Chicago", "San Francisco", "Miami", "Dallas" });

                GrammarBuilder gramb = new GrammarBuilder();
                gramb.Append("I want to fly from");
                gramb.Append(cities);
                gramb.Append("to");
                gramb.Append(cities);

                // Construct a Grammar object and load it to the recognizer.  
                Grammar cityChooser = new Grammar(gramb);
                cityChooser.Name = ("City Chooser");
                recognizer.LoadGrammarAsync(cityChooser);

                // Attach event handlers.  
                recognizer.SpeechDetected +=
                  new EventHandler<SpeechDetectedEventArgs>(
                    SpeechDetectedHandler);
                recognizer.SpeechHypothesized +=
                  new EventHandler<SpeechHypothesizedEventArgs>(
                    SpeechHypothesizedHandler);
                recognizer.SpeechRecognitionRejected +=
                  new EventHandler<SpeechRecognitionRejectedEventArgs>(
                    SpeechRecognitionRejectedHandler);
                recognizer.SpeechRecognized +=
                  new EventHandler<SpeechRecognizedEventArgs>(
                    SpeechRecognizedHandler);
                recognizer.RecognizeCompleted +=
                  new EventHandler<RecognizeCompletedEventArgs>(
                    RecognizeCompletedHandler);

                // Assign input to the recognizer and start an asynchronous  
                // recognition operation.  
                recognizer.SetInputToDefaultAudioDevice();

                completed = false;
                Console.WriteLine("Starting asynchronous recognition...");
                recognizer.RecognizeAsync();

                // Wait for the operation to complete.  
                /*while (!completed)
                {
                    Thread.Sleep(333);
                }
                Console.WriteLine("Done.");
            }*/

            /* Console.WriteLine();
             Console.WriteLine("Press any key to exit...");
             Console.ReadKey();*/

            /*SpeechRecognitionEngine recEngine = new SpeechRecognitionEngine(new CultureInfo("en-US"));

            recEngine = new SpeechRecognitionEngine();
            Choices commands = new Choices();

            commands.Add(new string[] { "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z" });
            //commands.Add(new string[] { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z" });

            //commands.Add(new string[] { "attack" });
            //commands.Add(new string[] { "stop" });

            //"letter", "letters", "alphabet", 

            //commands.Add(new string[] { "number", "numbers", "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety" });
            //commands.Add(new string[] { "hundred", "thousand", "million", "billion", "trillion" });
            //commands.Add(new string[] { "math", "maths", "plus", "minus", "divide", "multiply", "multiplied", "equal", "square", "square root", "cos", "sin", "tan" });
            //commands.Add(new string[] { "console", "program", "directory", "start", "plus" });

            ////////////////COMMANDS FOR CALCULATOR/////////////////////
            ////////////////////////////////////////////////////////////
            //commands.Add(new string[] { "calculations", "calculator" });
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            //List<string> words = System.IO.File.ReadAllText("MyWords.txt").Split(new string[] { Environment.NewLine }).ToList();
            ///ok wow. i dont need to build my own spectrum crap and how many days/weeks/months/years would it take me to code it... but damn i learned a ton
            ///in the process anyway. Next step. How do we load a c# library in Unity and use it? OMG this stuff is powerfull.
            GrammarBuilder gBuilder = new GrammarBuilder();
            gBuilder.Append(commands);
            Grammar grammar = new Grammar(gBuilder);

            recEngine.LoadGrammarAsync(grammar);

            recEngine.SetInputToDefaultAudioDevice();
            recEngine.RecognizeAsync(RecognizeMode.Multiple);
            recEngine.SpeechRecognized += recEngine_SpeechRecognized;*/
        }


        // Handle the SpeechDetected event.  
        static void SpeechDetectedHandler(object sender, SpeechDetectedEventArgs e)
        {
            Console.WriteLine(" In SpeechDetectedHandler:");
            Console.WriteLine(" - AudioPosition = {0}", e.AudioPosition);
        }

        // Handle the SpeechHypothesized event.  
        static void SpeechHypothesizedHandler(object sender, SpeechHypothesizedEventArgs e)
        {
            Console.WriteLine(" In SpeechHypothesizedHandler:");

            string grammarName = "<not available>";
            string resultText = "<not available>";
            if (e.Result != null)
            {
                if (e.Result.Grammar != null)
                {
                    grammarName = e.Result.Grammar.Name;
                }
                resultText = e.Result.Text;
            }

            Console.WriteLine(" - Grammar Name = {0}; Result Text = {1}",
              grammarName, resultText);
        }

        // Handle the SpeechRecognitionRejected event.  
        static void SpeechRecognitionRejectedHandler(
          object sender, SpeechRecognitionRejectedEventArgs e)
        {
            Console.WriteLine(" In SpeechRecognitionRejectedHandler:");

            string grammarName = "<not available>";
            string resultText = "<not available>";
            if (e.Result != null)
            {
                if (e.Result.Grammar != null)
                {
                    grammarName = e.Result.Grammar.Name;
                }
                resultText = e.Result.Text;
            }

            Console.WriteLine(" - Grammar Name = {0}; Result Text = {1}",
              grammarName, resultText);
        }

        // Handle the SpeechRecognized event.  
        static void SpeechRecognizedHandler(
          object sender, SpeechRecognizedEventArgs e)
        {
            Console.WriteLine(" In SpeechRecognizedHandler.");

            string grammarName = "<not available>";
            string resultText = "<not available>";
            if (e.Result != null)
            {
                if (e.Result.Grammar != null)
                {
                    grammarName = e.Result.Grammar.Name;
                }
                resultText = e.Result.Text;
            }

            Console.WriteLine(" - Grammar Name = {0}; Result Text = {1}",
              grammarName, resultText);
        }

        // Handle the RecognizeCompleted event.  
        static void RecognizeCompletedHandler(
          object sender, RecognizeCompletedEventArgs e)
        {
            Console.WriteLine(" In RecognizeCompletedHandler.");

            if (e.Error != null)
            {
                Console.WriteLine(
                  " - Error occurred during recognition: {0}", e.Error);
                return;
            }
            if (e.InitialSilenceTimeout || e.BabbleTimeout)
            {
                Console.WriteLine(
                  " - BabbleTimeout = {0}; InitialSilenceTimeout = {1}",
                  e.BabbleTimeout, e.InitialSilenceTimeout);
                return;
            }
            if (e.InputStreamEnded)
            {
                Console.WriteLine(
                  " - AudioPosition = {0}; InputStreamEnded = {1}",
                  e.AudioPosition, e.InputStreamEnded);
            }
            if (e.Result != null)
            {
                Console.WriteLine(
                  " - Grammar = {0}; Text = {1}; Confidence = {2}",
                  e.Result.Grammar.Name, e.Result.Text, e.Result.Confidence);
                Console.WriteLine(" - AudioPosition = {0}", e.AudioPosition);
            }
            else
            {
                Console.WriteLine(" - No result.");
            }

            completed = true;
        }
        private static int _hitCounter = 0;


        static int haspressedkey4VE = 0;
        const int WM_SYSKEYDOWN = 0x0104;
        const int WM_SYSKEYUP = 0x0105;
        const int VK_F5 = 0x74;
        const int WM_SYSCOMMAND = 0x112;
        const int SC_MAXIMIZE = 0xf030;
        const int WM_KEYDOWN = 0x0100;
        const int WM_KEYUP = 0x0101;
        const int KEYEVENTF_EXTENDEDKEY = 0x0001;


        //const uint WM_KEYDOWN = 0x100; // 0x0100
        //const uint WM_KEYUP = 0x101; //0x0101
        const int KeyDown = 0x0001;
        const int KeyUp = 0x0002;
        const byte VK_SPACE = 0x20;
        const byte SpaceSC = 39;

        [DllImport("user32.dll", SetLastError = true)]
        static extern void keybd_event(byte bVk, byte bScan, uint dwFlags, int dwExtraInfo);


        [DllImport("user32.dll", SetLastError = true)]
        public static extern int SendMessage(IntPtr hWnd, int Msg, int wParam, int lParam);


        static string speechcaptured = "Speech Captured";
        static string msgenabled = "Speech Recognit ON";
        static string msgdisabled = "Speech Recognit OFF";


        static string msgVoidExpanseDisabled = "Void Expanse";
        static string msgVoidExpanseEnabled = "Void Expanse";


        static int recognitwordswtch = 0;










        static void recEngine_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            recognitwordswtch = 0;
            lastWord = "";

            //currentWord = e.Result.Text;
            //Console.WriteLine("test" + _hitCounter);

            //MainWindow.MessageBox((IntPtr)0,"test","sccsVD4VE_LightNWithoutVr",0);

            _hitCounter++;

            /*string msg = "*";
            _sec_received_messages[11]._message = msg;
            _sec_received_messages[11]._originalMsg = msg;
            _sec_received_messages[11]._messageCut = msg;
            _sec_received_messages[11]._specialMessage = 2;
            _sec_received_messages[11]._specialMessageLineX = 0;
            _sec_received_messages[11]._specialMessageLineY = 0;
            _sec_received_messages[11]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
            _sec_received_messages[11]._orilineY = _sec_received_messages[12]._orilineY;
            _sec_received_messages[11]._lineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
            _sec_received_messages[11]._lineY = _sec_received_messages[12]._orilineY;
            _sec_received_messages[11]._count = 0;
            _sec_received_messages[11]._swtch0 = 1;
            _sec_received_messages[11]._swtch1 = 1;
            _sec_received_messages[11]._delay = 11;
            _sec_received_messages[11]._looping = 0;*/


           


            switch (e.Result.Text.ToLower())
            {
                case "market":
                    lastWord = "market";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;

                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                    //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                    //SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                    //SendMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                    //SendMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                    //SendMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);

                    //_keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.NumberPad4);
                    hasSpoken = 1;


                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);

                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                    //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                    //SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);


                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                    //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 1);


                    /*SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                    PostMessage(vewindowsfoundedz, WM_SYSKEYDOWN, VK_4, 0);
                    PostMessage(vewindowsfoundedz, WM_KEYDOWN | WM_KEYUP, VK_4, 0);
                    SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);*/


                    /*var updateMainUITitle = new Action(() =>
                    {
                        PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        PostMessage(vewindowsfoundedz, WM_SYSKEYUP, VK_4, 0);
                        PostMessage(vewindowsfoundedz, WM_KEYDOWN | WM_KEYUP, VK_4, 0);
                        SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);

                        
                    });

                    Dispatcher.Invoke(updateMainUITitle);*/

                    break;
                case "start":
                    lastWord = "start";
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    hasSpoken = 1;
                    break;
                case "stop":
                    lastWord = "stop";
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    hasSpoken = 1;
                    break;
                case "mining":
                    lastWord = "mining";
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    hasSpoken = 1;
                    break;
                case "combat":
                    lastWord = "combat";
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    hasSpoken = 1;
                    break;
                case "repair":
                    lastWord = "repair";
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    hasSpoken = 1;
                    break;
                case "release":
                    lastWord = "release";
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    hasSpoken = 1;
                    break;
                case "computer":
                    lastWord = "computer";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;

                    //_keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.NumberPad4);
                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_5);               
                    //PostMessage(vewindowsfoundedz, WM_KEYDOWN , VK_4, 1);
                    //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 1);
                    //SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                    //keyboardsim.KeyDown(WindowsInput.Native.VirtualKeyCode.VK_4);
                    //keyboardsim.KeyUp(WindowsInput.Native.VirtualKeyCode.VK_4);
                    hasSpoken = 1;


                    /*SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                    PostMessage(vewindowsfoundedz, WM_SYSKEYDOWN, VK_4, 0);
                    PostMessage(vewindowsfoundedz, WM_KEYDOWN | WM_KEYUP, VK_4, 0);
                    SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);*/


                    /*var updateMainUITitle = new Action(() =>
                    {
                        PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        PostMessage(vewindowsfoundedz, WM_SYSKEYUP, VK_4, 0);
                        PostMessage(vewindowsfoundedz, WM_KEYDOWN | WM_KEYUP, VK_4, 0);
                        SendMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);

                        
                    });

                    Dispatcher.Invoke(updateMainUITitle);*/

                    break;

                case "play":
                    lastWord = "play";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;
                /*case "stop":
                    lastWord = "stop";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;*/
                case "cancel":
                    lastWord = "cancel";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;
                case "repeat":
                    lastWord = "repeat";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;
                case "select": // pick
                    lastWord = "select";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //MainWindow.MessageBox((IntPtr)0, "select", "sccs message", 0);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;
                case "backward": // pick
                    lastWord = "backward";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //MainWindow.MessageBox((IntPtr)0, "backward", "sccs message", 0);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;
                case "forward": // pick
                    lastWord = "forward";
                    //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                    //Console.WriteLine(lastWord);
                    //MainWindow.MessageBox((IntPtr)0, "backward", "sccs message", 0);
                    //totalCombination += lastWord;
                    functionCounter03++;
                    counterTotalWords++;
                    frameCounterForVoiceRecognitionRecognizedWords++;
                    recognitwordswtch = 1;
                    break;
                    //VOID EXPANSE DRONES COMMS - NOT YET IMPLEMENTED
                    /*case "Attack":

                        // System.Windows.Forms.SendKeys.Send(keys);

                        //Process p = (Process)comboBox1.SelectedItem;

                        //keybd_event(VK_SPACE, 0, WM_KEYDOWN, 0);
                        //Thread.Sleep(50);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 0);
                        //keybd_event(VK_SPACE, 0, WM_KEYUP, 0);

                        //keybd_event(ModKey, 0, Key_Down, 0);
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //Thread.Sleep(10);
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 1);

                        //int handle = int.Parse(vewindowsfoundedz + "", NumberStyles.HexNumber);

                        // Now we'll send the WM_SETTEXT message to the window, passing the text
                        // through the lParam parameter.
                        //SendMessage(handle, (int)WM_KEYUP, 0, "http://www.improve.dk");
                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN, VK_4, 1);
                        //Console.WriteLine("Text set!");

                        // Now send a message telling the Edit box that the Return key has been pressed,
                        // resulting in Internet Explorer navigating to the page.
                        //PostMessage(vewindowsfoundedz, WM_KEYUP, VK_4, 1);
                        //Console.WriteLine("Return keypress sent!");

                        // And we're done

                        //keybd_event(ModKey, 0, Key_Up, 0);

                        //PostMessage(vewindowsfoundedz, WM_SYSKEYUP, VK_4, 0);

                        //PostMessage(vewindowsfoundedz, WM_KEYDOWN | WM_KEYUP, VK_4, 0);

                        //SendMessage(vewindowsfoundedz, WM_SYSCOMMAND, SC_MAXIMIZE, IntPtr.Zero);

                        //if (_keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.D4))
                        //{
                        //
                        //}
                        //_keyboard_input._KeyboardState.
                        //keybd_event(VK_4, 0, 0, 0);

                        lastWord = "Attack";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;*/
                    /*case "loop":
                        lastWord = "Loop";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        //Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        recognitwordswtch = 1;
                        break;*/
                    /*case "ok":
                        lastWord = "ok";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        //Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        recognitwordswtch = 1;
                        break;*/
                    //"Ok", "Play", "Stop", "Cancel", "Repeat", "Select", "Backward", "Forward"

                    /*case "A":
                        lastWord = "A";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "B":
                        lastWord = "B";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "C":
                        lastWord = "C";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "D":
                        lastWord = "D";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "E":
                        lastWord = "E";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "F":
                        lastWord = "F";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "G":
                        lastWord = "G";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "H":
                        lastWord = "H";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "I":
                        lastWord = "I";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "J":
                        lastWord = "J";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "K":
                        lastWord = "K";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "L":
                        lastWord = "L";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "M":
                        lastWord = "M";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "N":
                        lastWord = "N";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "O":
                        lastWord = "O";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "P":
                        lastWord = "P";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "Q":
                        lastWord = "Q";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "R":
                        lastWord = "R";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "S":
                        lastWord = "S";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "T":
                        lastWord = "T";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "U":
                        lastWord = "U";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "V":
                        lastWord = "V";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "W":
                        lastWord = "W";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "X":
                        lastWord = "X";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "Y":
                        lastWord = "Y";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;
                    case "Z":
                        lastWord = "Z";
                        //SC_Console.WriteAt(functionCounter00 + "", 0, 14, false);
                        Console.WriteLine(lastWord);
                        //totalCombination += lastWord;
                        functionCounter03++;
                        counterTotalWords++;
                        frameCounterForVoiceRecognitionRecognizedWords++;
                        break;*/
            }

            if (recognitwordswtch == 1)
            {
                string msg = " ";
                _sec_received_messages[11]._message = msg;
                _sec_received_messages[11]._originalMsg = msg;
                _sec_received_messages[11]._messageCut = msg;
                _sec_received_messages[11]._specialMessage = 3;
                _sec_received_messages[11]._specialMessageLineX = 0;
                _sec_received_messages[11]._specialMessageLineY = 0;
                _sec_received_messages[11]._orilineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                _sec_received_messages[11]._orilineY = _sec_received_messages[12]._orilineY;
                _sec_received_messages[11]._lineX = _sec_received_messages[12]._orilineX + msgdisabled.Length + 1;
                _sec_received_messages[11]._lineY = _sec_received_messages[12]._orilineY;
                _sec_received_messages[11]._count = 0;
                _sec_received_messages[11]._swtch0 = 1;
                _sec_received_messages[11]._swtch1 = 1;
                _sec_received_messages[11]._delay = 7;
                _sec_received_messages[11]._looping = 0;

                //msg = "Speech Recognition Engine enabled";
                msg = "#";
                _sec_received_messages[11]._message = msg;
                _sec_received_messages[11]._originalMsg = msg;
                _sec_received_messages[11]._messageCut = msg;
                _sec_received_messages[11]._specialMessage = 2;
                _sec_received_messages[11]._specialMessageLineX = 0;
                _sec_received_messages[11]._specialMessageLineY = 0;
                _sec_received_messages[11]._orilineX = _sec_received_messages[13]._orilineX + msgdisabled.Length + 1;
                _sec_received_messages[11]._orilineY = _sec_received_messages[13]._orilineY;
                _sec_received_messages[11]._lineX = _sec_received_messages[13]._orilineX + msgdisabled.Length + 1;
                _sec_received_messages[11]._lineY = _sec_received_messages[13]._orilineY;
                _sec_received_messages[11]._count = 0;
                _sec_received_messages[11]._swtch0 = 1;
                _sec_received_messages[11]._swtch1 = 1;
                _sec_received_messages[11]._delay = 7;
                _sec_received_messages[11]._looping = 1;

                string msgspeechrecognit = lastWord;
                _sec_received_messages[9]._message = msgspeechrecognit;
                _sec_received_messages[9]._originalMsg = msgspeechrecognit;
                _sec_received_messages[9]._messageCut = msgspeechrecognit;
                _sec_received_messages[9]._specialMessage = 2;
                _sec_received_messages[9]._specialMessageLineX = 0;
                _sec_received_messages[9]._specialMessageLineY = 0;
                //_sec_received_messages[9]._orilineX = _sec_received_messages[12]._orilineX;
                //_sec_received_messages[9]._orilineY = _sec_received_messages[12]._orilineY + 2;
                //_sec_received_messages[9]._lineX = _sec_received_messages[12]._orilineX;
                //_sec_received_messages[9]._lineY = _sec_received_messages[12]._orilineY + 2;

                _sec_received_messages[9]._orilineX = _sec_received_messages[13]._orilineX + msgdisabled.Length + 3;
                _sec_received_messages[9]._orilineY = _sec_received_messages[13]._orilineY;
                _sec_received_messages[9]._lineX = _sec_received_messages[13]._orilineX + msgdisabled.Length + 3;
                _sec_received_messages[9]._lineY = _sec_received_messages[13]._orilineY;
                _sec_received_messages[9]._count = 0;
                _sec_received_messages[9]._swtch0 = 1;
                _sec_received_messages[9]._swtch1 = 1;
                _sec_received_messages[9]._delay = 50;
                _sec_received_messages[9]._looping = 0;

                SpeechSynthesizer synth = new SpeechSynthesizer();
                synth.SetOutputToDefaultAudioDevice();

                synth.Volume = synthesizerVolume;
                synth.Speak(lastWord);
                recognitwordswtch = 0;
            }
        }

















        //SC_GCGollect _GCollector;

        public static void _threadUpdateTest()
        {



            //lookForKeyPress();
            //SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("V" + "", 0, 1);
        }













        //MAYBE TO READD
        //MAYBE TO READD        
        //MAYBE TO READD
        //KEY PRESS AND VOICE RECORDER/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //********************************
        //********************************
        //********************************
        //[DllImport("winmm.dll", EntryPoint = "mciSendStringA", CharSet = CharSet.Ansi, SetLastError = true, ExactSpelling = true)]
        //private static extern int mciSendString(string lpstrCommand, string lpstrReturnString, int uReturnLength, int hwndCallback);
        //MAYBE TO READD
        //MAYBE TO READD        
        //MAYBE TO READD














        //static bool mainCaptureSwitch = true;
        //static Computer c = new Computer();
        //static Computer a = new Computer();
        /*static string fullSubFolderPath = "";
        static string folderName = "";
        static int lastFrame = -1;
        static Process p;
        static int numberOfStartedRecordings = 0;

        static bool _pressOnce = true;
        static int _iterations = 0;*/

        /*
        public static void lookForKeyPress()
        {

            for (int i = 0; i < 1; i++)
            {

                if (System.Windows.Input.Keyboard.IsKeyDown(Key.R))
                {
                    if (_iterations > 0)
                    {
                        break;
                    }
                    SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("test" + _iterations, 0, 15 + _iterations);
                    //SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("#", 0, 15);


                    // training set for an NAND function
                    TrainingItem[] trainingSet =
                    {
                          new TrainingItem(true, 1, 0, 0),
                          new TrainingItem(true, 1, 0, 1),
                          new TrainingItem(true, 1, 1, 0),
                          new TrainingItem(false, 1, 1, 1)
                    };
                    // create a perceptron with 3 inputs
                    var perceptron = new BinaryPerceptron(3);

                    int attemptCount = 0;
                    // teach the neural network until all the inputs are correctly clasified
                    int _numberOfIterations = 0;

                    int errorCount = 0;

                    /*for (int j = 0; j < trainingSet.Length; j++)
                    {
                        var output = perceptron.Learn(trainingSet[j].Output, trainingSet[j].Inputs);
                        SC_Console.consoleMessageQueue messageQueue00 = new SC_Console.consoleMessageQueue("Fail\t {0} & {1} & {2} != {3}" + trainingSet[j].Inputs[0] + trainingSet[j].Inputs[1] + trainingSet[j].Inputs[2] + output, 0, 15 + errorCount + _numberOfIterations);
                        errorCount++;
                    }




                    _sometest:

                }             
            }


           





















            /*if (System.Windows.Input.Keyboard.IsKeyDown(Key.Enter))
            {
                GC.Collect(0);
                GC.Collect(1);
                GC.Collect(2);
                GC.Collect(10);
            }*/














        /*if (_iterations > 0)
        {
            _pressOnce = true;
        }*/



        /*if (System.Windows.Input.Keyboard.IsKeyDown(Key.R))
        {
            if (_pressOnce)
            {
                //SC_Console.WriteAt("r", 1, 1, false, 0);
                SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("#", 0, 1);

                mciSendString("open new Type waveaudio Alias recsound", "", 0, 0);
                mciSendString("record recsound", "", 0, 0);
                numberOfStartedRecordings++;
                _pressOnce = false;
            }
        }


        else if (System.Windows.Input.Keyboard.IsKeyDown(Key.Y))
        {
            if (mainCaptureSwitch)
            {
                string strWorkingDirectory = Directory.GetCurrentDirectory();

                string soundDirectory = "\\SC_AI_AudioSaves";

                fullSubFolderPath = strWorkingDirectory + soundDirectory;

                //Directory.GetFiles(fullSubFolderPath);

                if (!Directory.Exists(fullSubFolderPath))
                {
                    Directory.CreateDirectory(fullSubFolderPath);
                }


                //SC_Console.WriteAt("y", 1, 1, false, 0);
                SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("y", 0, 2);
                folderName = fullSubFolderPath + "\\" + "SC_AI_AudioSaves" + numberOfStartedRecordings + ".wav";

                mciSendString("save recsound" + " " + folderName, "", 0, 0);
                mciSendString("close recsound ", "", 0, 0);
                //Computer a = new Computer();
                c.Audio.Stop();
                lastFrame = numberOfStartedRecordings;

                p = new Process();
                p.StartInfo = new ProcessStartInfo()
                {
                    FileName = fullSubFolderPath //"c:\\"
                };

                p.Start();
                p.Refresh();
                _pressOnce = true;
                mainCaptureSwitch = false;
            }
        }

        else if (System.Windows.Input.Keyboard.IsKeyDown(Key.P))
        {
            //SC_Console.WriteAt("p", 1, 1, false, 0);
            SC_Console.consoleMessageQueue messageQueue0 = new SC_Console.consoleMessageQueue("p", 0, 3);

            if (Directory.Exists(fullSubFolderPath))
            {
                //string[] dirs = Directory.GetFiles(@"c:\", "c*");
                //string[] dirs = Directory.GetFiles(fullSubFolderPath, "*SC_AI_AudioSaves" + frameRate + "*");
                //string[] dirs = Directory.GetFiles(fullSubFolderPath, "*SC_AI_AudioSaves" + frameRate+".wav");
                string[] dirs = Directory.GetFiles(fullSubFolderPath, "*" + lastFrame + "*" + ".wav");
                SC_Console.WriteAt(dirs.Length.ToString(), 0, 4, false, 0);
                //Computer aa = new Computer();
                c.Audio.Play(dirs[0]);

            }
            mainCaptureSwitch = true;
        }
    }*/
        //********************************
        //********************************
        //********************************
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /*private void ContentFrame_OnNavigated(object sender, NavigationEventArgs e)
        {
            // Prevent navigation (for example clicking back button) because our ListBox is not updated when this navigation occurs
            // We prevent navigation with clearing the navigation history each time navigation item changes
            //ContentFrame0.NavigationService.RemoveBackEntry();
            //ContentFrame1.NavigationService.RemoveBackEntry();
        }*/

    }
}









//D3D.ini
//_graphics_sec = new sc_graphics_sec();
/*if (!D3D.Initialize(config, handler, SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER))
{

}*/


//_graphics_sec = new sc_graphics_sec();
//consoleGraphics = new SC_Console_GRAPHICS(_graphics_sec);
//_sc_jitter_task = _graphics_sec._sc_create_world_objects(_sc_jitter_task);
//_sc_jitter_task = consoleGraphics.Initialize(_sc_jitter_task,config, handler, SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER, _graphics_sec);


//consoleGraphics = new SC_Console_GRAPHICS(_graphics_sec);

/*_sc_jitter_physics = consoleGraphics.create_jitter_instances(_sc_jitter_physics, _sc_jitter_data);

for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {
            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

            _sc_jitter_physics[indexer00]._sc_create_jitter_world(_sc_jitter_data[indexer00]);


            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {

                        var indexer1 = x + world_width * (y + world_height * z);

                        var world = _sc_jitter_physics[indexer00].return_world(indexer1);

                        if (world == null)
                        {
                            Console.WriteLine("null world");
                        }
                        else
                        {
                            Console.WriteLine("!null world");
                            _sc_jitter_task[indexer00][indexer1]._world_data = new object[2];
                            _sc_jitter_task[indexer00][indexer1]._work_index = -1;
                            _sc_jitter_task[indexer00][indexer1]._world_data[0] = world;
                        }
                    }
                }
            }
        }
    }
}


_sc_jitter_task = _graphics_sec._sc_create_world_objects(_sc_jitter_task);
_sc_jitter_task = consoleGraphics.Initialize(_sc_jitter_task, config, handler, SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER, _graphics_sec);*/



















/*
_sc_jitter_physics = consoleGraphics.create_jitter_instances(_sc_jitter_physics, _sc_jitter_data);

for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {
            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

            _sc_jitter_physics[indexer00]._sc_create_jitter_world(_sc_jitter_data);


            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {

                        var indexer1 = x + world_width * (y + world_height * z);

                        var world = _sc_jitter_physics[indexer00].return_world(indexer1);

                        if (world == null)
                        {
                            Console.WriteLine("null");
                        }
                        else
                        {
                            //Console.WriteLine("!null");

                            _sc_jitter_task[indexer00][indexer1]._world_data = new object[2];
                            _sc_jitter_task[indexer00][indexer1]._work_index = -1;
                            _sc_jitter_task[indexer00][indexer1]._world_data[0] = world;
                            //Console.WriteLine("index: " + indexer1);
                        }
                    }
                }
            }
        }
    }
}

_sc_jitter_task = _graphics_sec._sc_create_world_objects(_sc_jitter_task);
_sc_jitter_task = consoleGraphics.Initialize(_sc_jitter_task, config, handler, SC_GLOBALS_ACCESSORS.SC_CONSOLE_WRITER, _graphics_sec);
*/



/*
var thread = new Thread(() =>
 {
 _thread_looper:

     try
     {
         for (int xx = 0; xx < _physics_engine_instance_x; xx++)
         {
             for (int yy = 0; yy < _physics_engine_instance_y; yy++)
             {
                 for (int zz = 0; zz < _physics_engine_instance_z; zz++)
                 {
                     var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

                     _sc_jitter_physics[indexer00]._sc_jitter_world_input();

                     for (int x = 0; x < world_width; x++)
                     {
                         for (int y = 0; y < world_height; y++)
                         {
                             for (int z = 0; z < world_depth; z++)
                             {
                                 var indexer1 = x + world_width * (y + world_height * z);
                                 _sc_jitter_data[indexer00].worlds[indexer1].Step(_array_stop_watch_tick[0], true);
                                 //var world = _sc_jitter_physics[indexer00].return_world(indexer1);
                                 _sc_jitter_task[indexer00][indexer1]._world_data[0] = _sc_jitter_data[indexer00].worlds[indexer1];
                             }
                         }
                     }
                 }
             }
         }*/

/*for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {
            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

            //_sc_jitter_physics[indexer00]._sc_create_jitter_world(_sc_jitter_data);

            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {

                        var indexer1 = x + world_width * (y + world_height * z);

                        var world = _sc_jitter_physics[indexer00].return_world(indexer1);

                        if (world == null)
                        {
                            Console.WriteLine("null");
                        }
                        else
                        {
                            //Console.WriteLine("!null");
                            _sc_jitter_task[indexer00][indexer1]._world_data[0] = world;
                        }
                    }
                }
            }
        }
    }
}

}
catch (Exception ex)
{

}
Thread.Sleep(0);
goto _thread_looper;
},0);

thread.IsBackground = true;
thread.SetApartmentState(ApartmentState.STA);
thread.Start();*/



/*
BackgroundWorker threaders = new BackgroundWorker();
threaders.DoWork += (object sender, DoWorkEventArgs argers) =>
{
    //object[] parametors = args.Argument as object[];
    //var workor = (_task_worker)parametors[0];
    //var parameters[] = _array_stop_watch_tick[_index];
    //int _indexer = workor._worker_task_id;
    int _start_once = 0;

_thread_looper:
    try
    {
        try
        {

            for (int xx = 0; xx < _physics_engine_instance_x; xx++)
            {
                for (int yy = 0; yy < _physics_engine_instance_y; yy++)
                {
                    for (int zz = 0; zz < _physics_engine_instance_z; zz++)
                    {
                        var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

                        _sc_jitter_physics[indexer00]._sc_jitter_world_input();

                        for (int x = 0; x < world_width; x++)
                        {
                            for (int y = 0; y < world_height; y++)
                            {
                                for (int z = 0; z < world_depth; z++)
                                {
                                    var indexer1 = x + world_width * (y + world_height * z);
                                    _sc_jitter_data[indexer00].worlds[indexer1].Step(_array_stop_watch_tick[0],true);
                                    //var world = _sc_jitter_physics[indexer00].return_world(indexer1);
                                    _sc_jitter_task[indexer00][indexer1]._world_data[0] = _sc_jitter_data[indexer00].worlds[indexer1];
                                }
                            }
                        }
                    }
                }
            }



        }
        catch (Exception ex)
        {
            MessageBox((IntPtr)0, "" + ex.ToString(), "Oculus Error", 0);
        }
    }
    catch (Exception ex)
    {

    }
    Thread.Sleep(0);
    goto _thread_looper;
};

threaders.RunWorkerCompleted += delegate (object sender, RunWorkerCompletedEventArgs argers)
{
    MessageBox((IntPtr)0, "worker has completed", "Oculus Error", 0);
};

threaders.RunWorkerAsync();*/




/*
 * 
                                        try
                                        {

                                        }
                                        catch (Exception ex)
                                        {
                                            MessageBox((IntPtr)0, "CONSOLE GRAPHICS" + ex.ToString(), "_sc_core_systems error", 0);
                                        }
*/



//SC_message_object_jitter[][] _sc_jitter_task = new SC_message_object_jitter[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z][];
//sc_jitter_data _sc_jitter_data = new sc_jitter_data();

/*try
{
    var pather = @"C:\Users\ninekorn\Desktop\Jitter Physics 0.1.7.0\Jitter Physics 0.1.7.0\sc_core_systems\sc_core_systems\_dll_sc\jitter.dll";
    var DLL = Assembly.LoadFile(pather);

    _sc_jitter_data.alloweddeactivation = true;
    _sc_jitter_data.allowedpenetration = 0.00123f;
    _sc_jitter_data.depth = 1;
    _sc_jitter_data.width = 1;
    _sc_jitter_data.height = 1;
    _sc_jitter_data.gravity = new JVector(0, -9.81f, 0);
    _sc_jitter_data.smalliterations = 10;
    _sc_jitter_data.iterations = 10;

    Type[] typeMainMethod = new Type[]
    {
    typeof(int),
    typeof(int),
    typeof(float),
    typeof(bool),
    typeof(JVector),
    typeof(int),
    typeof(int),
    typeof(int),
    };

    Type t = DLL.GetType(pather + "_sc_jitter_physics");
    var methodInfo = t.GetMethod("_sc_create_jitter_world", typeMainMethod);
}
catch (Exception ex)
{

}*/






//var o = Activator.CreateInstance(Type.GetType(pather, true));
//Type t = DLL.GetType(pather + "_sc_jitter_physics");
//var o = Activator.CreateInstance(t);

/*var methodInfo = o.GetMethod("_sc_create_jitter_world", new Type[]
  {
        typeof(int),
        typeof(int),
        typeof(float),
        typeof(bool),
        typeof(JVector),
        typeof(int),
        typeof(int),
        typeof(int),
  });
object[] somedata = new object[1];
somedata[0] = _sc_jitter_data;
var result = methodInfo.Invoke(o, somedata);*/



/*foreach (Type type in DLL.GetExportedTypes())
{
    //Console.WriteLine(type);
    var o = Activator.CreateInstance(type);
    object[] somedata = new object[1];
    somedata[0] = _sc_jitter_data;
    var result = methodInfo.Invoke(o, somedata);
}*/







/*for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {
            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

            _sc_jitter_task[indexer00] = new SC_message_object_jitter[world_width * world_height * world_depth];

            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {
                        var indexer01 = x + world_width * (y + world_height * z);

                    }
                }
            }
        }
    }
}*/












/*
Type t = DLL.GetType(pather + "_sc_jitter_physics");  //Classname is class to be instantiated
var methodInfo = t.GetMethod("_jitter_hook", new Type[]
{
    typeof(int),
    typeof(int),
    typeof(float),
    typeof(bool),
    typeof(JVector),
    typeof(int),
    typeof(int),
    typeof(int),
});//myMethod is method to be instantiated*/


/*object obj = Activator.CreateInstance(Type.GetType(pather, true));

foreach (Type type in DLL.GetExportedTypes())
{
    //dynamic c = Activator.CreateInstance(type);
    //c.Output(@"Hello");
}*/
//object obj = Activator.CreateInstance(Type.GetType("DALL.LoadClass, DALL", true));


/*_sc_jitter_physics[] _sc_jitter_physics = new _sc_jitter_physics[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z];
SC_message_object_jitter[][] _sc_jitter_task = new SC_message_object_jitter[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z][];
sc_jitter_data[] _sc_jitter_data = new sc_jitter_data[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z];


CollisionSystemPersistentSAP[] _collisionSAP = new CollisionSystemPersistentSAP[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z];
World[][] worlds_ = new World[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z][];

_array_stop_watch_tick = new float[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z];

sc_deltatime(0, 0);

for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {

            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);
            //_jitter_physics[indexer00] = DoSpecialThing();
            _sc_jitter_data[indexer00] = new sc_jitter_data();
            _sc_jitter_task[indexer00] = new SC_message_object_jitter[world_width * world_height * world_depth];
            worlds_[indexer00] = new World[world_width * world_height * world_depth];
            //_sc_jitter_physics[indexer00] = new _sc_jitter_physics();

            _collisionSAP[indexer00] = new CollisionSystemPersistentSAP();


            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {
                        var indexer01 = x + world_width * (y + world_height * z);
                        _sc_jitter_task[indexer00][indexer01] = new SC_message_object_jitter();
                        worlds_[indexer00][indexer01] = new World(_collisionSAP[indexer00]);
                        worlds_[indexer00][indexer01].AllowDeactivation = false;
                        worlds_[indexer00][indexer01].Gravity = _world_gravity;
                        worlds_[indexer00][indexer01].SetIterations(_world_iterations, _world_small_iterations);
                        worlds_[indexer00][indexer01].ContactSettings.AllowedPenetration = _world_allowed_penetration;
                    }
                }
            }

            _sc_jitter_data[indexer00]._collisionSAP = _collisionSAP;
            _sc_jitter_data[indexer00].worlds = worlds_[indexer00];

        }
    }
}*/

/*sc_jitter_data _sc_jitter_data = new sc_jitter_data();
/*_sc_jitter_data.alloweddeactivation = true;
_sc_jitter_data.allowedpenetration = 0.00123f;
_sc_jitter_data.width = world_width;
_sc_jitter_data.height = world_height;
_sc_jitter_data.depth = world_depth;
_sc_jitter_data.gravity = new JVector(0, 0, 0);
_sc_jitter_data.smalliterations = 3;
_sc_jitter_data.iterations = 3;*/








/*
_sc_jitter_physics[] _sc_jitter_physics = new _sc_jitter_physics[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z];
SC_message_object_jitter[][] _sc_jitter_task = new SC_message_object_jitter[_physics_engine_instance_x * _physics_engine_instance_y * _physics_engine_instance_z][];

sc_jitter_data _sc_jitter_data = new sc_jitter_data();
_sc_jitter_data.alloweddeactivation = false;
_sc_jitter_data.allowedpenetration = 0.00123f;
_sc_jitter_data.width = world_width;
_sc_jitter_data.height = world_height;
_sc_jitter_data.depth = world_depth;
_sc_jitter_data.gravity = new JVector(0, 0, 0);
_sc_jitter_data.smalliterations = 10;
_sc_jitter_data.iterations = 10;


for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {
            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);
            //_jitter_physics[indexer00] = DoSpecialThing();
            _sc_jitter_task[indexer00] = new SC_message_object_jitter[world_width * world_height * world_depth];

            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {
                        var indexer01 = x + world_width * (y + world_height * z);
                        _sc_jitter_task[indexer00][indexer01] = new SC_message_object_jitter();


                    }
                }
            }
        }
    }
}*/















/*
for (int xx = 0; xx < _physics_engine_instance_x; xx++)
{
    for (int yy = 0; yy < _physics_engine_instance_y; yy++)
    {
        for (int zz = 0; zz < _physics_engine_instance_z; zz++)
        {
            var indexer00 = xx + _physics_engine_instance_x * (yy + _physics_engine_instance_y * zz);

            _sc_jitter_task[indexer00] = new SC_message_object_jitter[world_width * world_height * world_depth];

            for (int x = 0; x < world_width; x++)
            {
                for (int y = 0; y < world_height; y++)
                {
                    for (int z = 0; z < world_depth; z++)
                    {
                        var indexer01 = x + world_width * (y + world_height * z);

                        //JITTER PHYSICS
                        var collision = new CollisionSystemPersistentSAP();
                        var World = new World(collision);
                        World.AllowDeactivation = true;

                        World.Gravity = _world_gravity;
                        World.SetIterations(_world_iterations, _world_small_iterations);
                        World.ContactSettings.AllowedPenetration = _world_allowed_penetration;

                        //_sc_jitter_physics[indexer00][indexer01] = World;

                        _sc_jitter_task[indexer00][indexer01]._world_data = new object[2];
                        _sc_jitter_task[indexer00][indexer01]._work_index = -1;
                        _sc_jitter_task[indexer00][indexer01]._world_data[0] = World;
                    }
                }
            }                           
        }
    }
}*/
