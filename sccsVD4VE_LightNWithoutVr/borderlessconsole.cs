//https://stackoverflow.com/questions/12705038/borderless-console-application
//https://stackoverflow.com/questions/8364758/get-handle-to-desktop-shell-window
//https://stackoverflow.com/questions/18034975/how-do-i-find-position-of-a-win32-control-window-relative-to-its-parent-window

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

using System.Threading;
using System.Threading.Tasks;

namespace SCCoreSystems
{
    public class borderlessconsole
    {
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

        public borderlessconsole()
        {
            Console.Title = WINDOW_NAME;
            makeBorderless(100,75);

            //Console.WriteLine("SC Core Systems");
            //Console.ReadLine();

        _main_thread_Loop_x00:

            if (_init_main == 1)
            {
                try
                {
                    Thread _mainTasker00 = new Thread((tester0000) =>
                    {
                    _thread_main_loop:

                        //makeBorderless(0,0);
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
            Console.WriteLine("nope... no world 01!");
            
        }

        static int _init_main = 1;
        public static IntPtr HWND_DESKTOP;

        public static void makeBorderless(int x, int y)
        {
            // Get the handle of self
            IntPtr window = FindWindowByCaption(IntPtr.Zero, WINDOW_NAME);      
            // Get the rectangle of self (Size)
            GetWindowRect(window, out rect);            
            /*RECT recter = new RECT();
            recter.left = 0;
            recter.right = Console.WindowWidth - 10;
            recter.bottom = 0 + 1000;
            recter.top = Console.WindowHeight - 1000;*/
            // Get the handle of the desktop
            HWND_DESKTOP = GetDesktopWindow();
            // Attempt to get the location of self compared to desktop
            MapWindowPoints(HWND_DESKTOP, window, ref rect, 2);
            // update self
            SetWindowLong(window, GWL_STYLE, WS_SYSMENU);

            // rect.left rect.top should work but they're returning negative values for me. I probably messed up
            SetWindowPos(window, -2, (int)x, (int)y, rect.bottom, rect.right, 0x0040); // -2,100,75
            //
            DrawMenuBar(window);
        }
    }
}