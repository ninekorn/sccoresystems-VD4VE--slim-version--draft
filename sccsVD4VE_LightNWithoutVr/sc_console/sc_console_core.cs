using System;
using System.Collections.Generic;
using System.Threading.Tasks;
//using SharpDX.Direct3D11;
using System.Runtime.InteropServices;
using System.Threading;
using System.ComponentModel;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public class sc_console_core// : sc_icomponent
    {
        int _init_main = 1;

        /*public static sc_icomponent _SC_ICOMPONENT;
        sc_globals sc_icomponent.SC_Globals
        {
            get => SC_GLOB;
        }*/
        public static sc_globals SC_GLOB;
        public int _console_hasINIT = 1;

        [DllImport("user32.dll")]
        static extern bool ShowScrollBar(IntPtr hWnd, int wBar, bool bShow);

        [DllImport("user32.dll")]
        public static extern int DeleteMenu(IntPtr hMenu, int nPosition, int wFlags);

        [DllImport("user32.dll")]
        private static extern IntPtr GetSystemMenu(IntPtr hWnd, bool bRevert);

        [DllImport("kernel32.dll", ExactSpelling = true)]
        private static extern IntPtr GetConsoleWindow();
        [DllImport(@"kernel32.dll", SetLastError = true)]
        static extern bool AllocConsole();

        [DllImport("kernel32.dll")]
        static extern bool GetConsoleMode(IntPtr hConsoleHandle, out uint lpMode);
        [DllImport("kernel32.dll")]
        static extern bool SetConsoleMode(IntPtr hConsoleHandle, uint dwMode);

        [DllImport(@"user32.dll")]
        static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern IntPtr GetStdHandle(int nStdHandle);

        [DllImport("user32")]
        static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int x, int y, int cx, int cy, int flags);

        const int SWP_NOZORDER = 0x4;
        const int SWP_NOACTIVATE = 0x10;
        public static void SetWindowPosition(IntPtr Handle, int x, int y, int width, int height)
        {
            SetWindowPos(Handle, IntPtr.Zero, x, y, width, height, SWP_NOZORDER | SWP_NOACTIVATE);
        }

        const int SwHide = 0;
        const int SwShow = 5;

        const int ENABLE_MOUSE_INPUT = 0x0010;

        const int STD_INPUT_HANDLE = -10;
        const uint ENABLE_QUICK_EDIT = 0x0040;

        public uint _originalConsoleModeWithMouseInput;
        public uint _originalConsoleModeWithoutMouseInput;
        public uint _modifiedConsoleMode;


        public sc_console_core(sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object[] tester)
        {
            _createConsole(tester);
        }


        public static IntPtr handle;

        private enum ScrollBarDirection
        {
            SB_HORZ = 0,
            SB_VERT = 1,
            SB_CTL = 2,
            SB_BOTH = 3
        }

        public IntPtr hMod;

        public void _reset_console_borders()
        {
            //IntPtr sysMenu = GetSystemMenu(handle, false);
            if (handle != IntPtr.Zero)
            {
                //DeleteMenu(sysMenu, SC_CLOSE, MF_BYCOMMAND);
                //DeleteMenu(sysMenu, SC_MINIMIZE, MF_BYCOMMAND);
                //DeleteMenu(sysMenu, SC_MAXIMIZE, MF_BYCOMMAND);
                //DeleteMenu(sysMenu, SC_SIZE, MF_BYCOMMAND);
                ShowScrollBar(handle, (int)ScrollBarDirection.SB_VERT, false);
                ShowScrollBar(handle, (int)ScrollBarDirection.SB_HORZ, false);
                ShowScrollBar(handle, (int)ScrollBarDirection.SB_BOTH, false);

            }
        }





















        [DllImport("USER32.DLL")]
        public static extern int SetWindowLong(IntPtr hWnd, int nIndex, int dwNewLong);

        [DllImport("user32.dll", EntryPoint = "FindWindow", SetLastError = true)]
        public static extern IntPtr FindWindowByCaption(IntPtr ZeroOnly, string lpWindowName);

        [DllImport("user32.dll")]
        public static extern bool DrawMenuBar(IntPtr hWnd);

        [DllImport("user32.dll", EntryPoint = "SetWindowPos")]
        public static extern IntPtr SetWindowPos(IntPtr hWnd, int hWndInsertAfter, int x, int Y, int cx, int cy, int wFlags);

        [DllImport("user32.dll", SetLastError = true)]
        public static extern bool GetWindowRect(IntPtr hwnd, out RECT lpRect);

        [DllImport("user32", ExactSpelling = true, SetLastError = true)]
        internal static extern int MapWindowPoints(IntPtr hWndFrom, IntPtr hWndTo, [In, Out] ref RECT rect, [MarshalAs(UnmanagedType.U4)] int cPoints);

        [DllImport("user32.dll", CharSet = CharSet.Auto, ExactSpelling = true)]
        public static extern IntPtr GetDesktopWindow();

        [StructLayout(LayoutKind.Sequential)]
        public struct RECT
        {
            public int left, top, bottom, right;
        }

        public static readonly string WINDOW_NAME = "SC Core Systems";  //name of the window
        public const int GWL_STYLE = -16;              //hex constant for style changing
        public const int WS_BORDER = 0x00800000;       //window with border
        public const int WS_CAPTION = 0x00C00000;      //window with a title bar
        public const int WS_SYSMENU = 0x00080000;      //window with no borders etc.
        public const int WS_MINIMIZEBOX = 0x00020000;  //window with minimizebox

        public static IntPtr window;
        public static RECT rect;

        //static int _init_main = 1;
        public static IntPtr HWND_DESKTOP;




        public void _createConsole(sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object[] tester)
        {
            handle = GetConsoleWindow();
            if (handle == IntPtr.Zero)
            {
                AllocConsole();
                //consoleMessageQueue _consoleMessageQueue = new consoleMessageQueue("IntPtr.Zero", 0, 0);
            }
            else
            {
                ShowWindow(handle, SwShow);
                //consoleMessageQueue _consoleMessageQueue = new consoleMessageQueue("!IntPtr.Zero", 0, 0);
            }

            IntPtr consoleHandle = GetStdHandle(STD_INPUT_HANDLE);

            if (!GetConsoleMode(consoleHandle, out _originalConsoleModeWithMouseInput))
            {
                // ERROR: Unable to get console mode.
                //return false;
                //consoleMessageQueue _consoleMessageQueue = new consoleMessageQueue("null GetConsoleMode", 0, 0);
            }
            else
            {
                //consoleMessageQueue _consoleMessageQueue = new consoleMessageQueue("not null GetConsoleMode", 0, 0);
            }      

            _modifiedConsoleMode = _originalConsoleModeWithMouseInput;

            _modifiedConsoleMode &= ~ENABLE_QUICK_EDIT;

            _originalConsoleModeWithoutMouseInput = _modifiedConsoleMode;

            // set the new mode
            if (!SetConsoleMode(consoleHandle, _modifiedConsoleMode))
            {
                // ERROR: Unable to set console mode
                //consoleMessageQueue _consoleMessageQueue = new consoleMessageQueue("null SetConsoleMode", 0, 0);
            }
            else
            {
                //consoleMessageQueue _consoleMessageQueue = new consoleMessageQueue("not null SetConsoleMode", 0, 0);
            }
            //Console.SetBufferSize(Console.BufferWidth, Console.BufferHeight);

            IntPtr sysMenu = GetSystemMenu(handle, false);

            if (handle != IntPtr.Zero)
            {
                //DeleteMenu(sysMenu, SC_CLOSE, MF_BYCOMMAND);
                //DeleteMenu(sysMenu, SC_MINIMIZE, MF_BYCOMMAND);
                //DeleteMenu(sysMenu, SC_MAXIMIZE, MF_BYCOMMAND);
                //DeleteMenu(sysMenu, SC_SIZE, MF_BYCOMMAND);
            }

            Console.OutputEncoding = System.Text.Encoding.GetEncoding(28591); //System.Text.Encoding.UTF8;//  65001
            Console.SetCursorPosition(0, 0);

            int origWidth = Console.WindowWidth;
            int origHeight = Console.WindowHeight;
            int width = origWidth;
            int height = origHeight;
            Console.SetWindowSize(width, height);
            Console.SetBufferSize(width, height);

            ShowScrollBar(handle, (int)ScrollBarDirection.SB_VERT, false);
            ShowScrollBar(handle, (int)ScrollBarDirection.SB_HORZ, false);
            ShowScrollBar(handle, (int)ScrollBarDirection.SB_BOTH, false);
  
            _console_hasINIT = 1;

     



            /*int x = 100;
            int y = 75;

            // Get the handle of self
            //IntPtr window = FindWindowByCaption(IntPtr.Zero, WINDOW_NAME);
            // Get the rectangle of self (Size)
            GetWindowRect(handle, out rect);
            //RECT recter = new RECT();
            //recter.left = 0;
            //recter.right = Console.WindowWidth - 10;
            //recter.bottom = 0 + 1000;
            //recter.top = Console.WindowHeight - 1000;
            // Get the handle of the desktop
            HWND_DESKTOP = GetDesktopWindow();
            // Attempt to get the location of self compared to desktop
            MapWindowPoints(HWND_DESKTOP, handle, ref rect, 2);
            // update self
            SetWindowLong(window, GWL_STYLE, WS_SYSMENU);
            //SetWindowLong(handle, WS_SYSMENU, WS_SYSMENU);
            // rect.left rect.top should work but they're returning negative values for me. I probably messed up
            SetWindowPos(handle, -2, (int)x, (int)y, rect.bottom, rect.right, 0x0040); // -2,100,75
            //
            DrawMenuBar(handle);*/




















            /////////////////////////////////////////////////////////
            ////////////////////CONSOLE CREATED//////////////////////
            /////////////////////////////////////////////////////////   






            /////////////////////////////////////////////////////////
            ////////////////////CONSOLE MAIN LOOP////////////////////
            /////////////////////////////////////////////////////////   
            /*_main_thread_Loop_x00:

                if (_init_main == 1)
                {
                    try
                    {
                        Thread _mainTasker00 = new Thread((tester0000) =>
                        {
                        _thread_main_loop:

                            //Console.SetCursorPosition(0, 1);
                            //Console.WriteLine("is alive");

                            Thread.Sleep(1);
                            goto _thread_main_loop;
                        }, 999999999); // 100000 //999999999 // dependant on ram. 100000 is 40-50 megs of ram and 999999999 is 1 gig.

                        _mainTasker00.IsBackground = false;
                        _mainTasker00.SetApartmentState(ApartmentState.STA);
                        _mainTasker00.Start();
                    }
                    catch
                    {
                        MainWindow.MessageBox((IntPtr)0, "stack overflow possible. no clue what it is anyway ", "Oculus Error", 0);
                        //temp fix, go back to creating the main thread. later try isbackground true and STA or not etc, switch to task or backgroundworker.
                        goto _main_thread_Loop_x00;
                    }

                    _init_main = 2;
                }

                if (_init_main == 2)
                {
                    //found with a google search that when using this option it at least makes you able to have a "prompt" or something in order for the console 
                    //to have a looping "main" thread so i just added a goto loop to "catch" this first frame and to loop it.
                    if (System.Diagnostics.Debugger.IsAttached)
                    {
                        goto _main_thread_Loop_x00;
                    }
                    else
                    {
                        goto _main_thread_Loop_x00;
                    }
                }
                else
                {
                    //System.Windows.MessageBox.Show("lOOp", "CONSOLE");
                    goto _main_thread_Loop_x00;
                }
                Console.WriteLine("nope... no console loop !");*/
            /////////////////////////////////////////////////////////
            ////////////////////CONSOLE MAIN LOOP////////////////////
            /////////////////////////////////////////////////////////   
        }





        static sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object _data00_IN;

        private const int MF_BYCOMMAND = 0x00000000;
        public const int SC_CLOSE = 0xF060;
        public const int SC_MINIMIZE = 0xF020;
        public const int SC_MAXIMIZE = 0xF030;
        public const int SC_SIZE = 0xF000;
        //private const int WM_SIZE = 0x0005;






            /*
        public  void HideConsoleWindow()
        {

        }

        public  void starterItems(string main, string secondary)
        {
            string currentTime = DateTime.Now.ToString("h:mm:ss tt");
            string currentDay = DateTime.Today.ToString();
        }

        public  void clearConsole()
        {
            Console.Clear();
        }

        public  void writeToConsole(string test)
        {
            Console.Write(test);
        }

        public  void writeLineToConsole(string test)
        {
            Console.WriteLine(test);
        }
        public  int cursorLeft()
        {
            int left = Console.CursorLeft;
            return left;
        }
        public  int cursorTop()
        {
            int top = Console.CursorTop;
            return top;
        }
        public  int consoleWidth()
        {
            int width = Console.WindowWidth;
            return width;
        }
        public  int consoleHeight()
        {
            int height = Console.WindowHeight;
            return height;
        }

        public  void consoleSwitchLine()
        {
            Console.Write("\n");
        }

        public  void consoleResetCursor()
        {
            int top = Console.CursorTop;
            int left = Console.CursorLeft;
            Console.SetCursorPosition(left, top);
        }
        public  void ClearCurrentConsoleLine(int x, int y)
        {
            Console.SetCursorPosition(0, y);
            for (int i = 0; i < Console.WindowWidth; i++)
            {
                Console.Write(new string(' ', Console.WindowWidth));
            }
        }
        public  void setCursorPosition(int x, int y)
        {
            Console.SetCursorPosition(x, y);
        }

        public  void WriteAt(string s, int x, int y, bool canPassLine, int linePassNumber)
        {
            string numberOfLines = "";
            for (int i = 0; i < linePassNumber; i++)
            {
                if (canPassLine)
                {
                    numberOfLines += "\n";
                }
            }

            Console.SetCursorPosition(x, y);
            Console.Write(numberOfLines + s);
        }*/
    }
}