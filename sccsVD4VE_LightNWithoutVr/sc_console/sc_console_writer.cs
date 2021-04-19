using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Microsoft.Win32.SafeHandles;
using System.IO;
using System.Runtime.InteropServices;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public struct _messager
    {
        public _messager[] _messager_list;
        public int _specialMessage;
        public int _specialMessageLineX;
        public int _specialMessageLineY;
        public string _message;
        public string _messageCut;
        public string _originalMsg;
        public int _lineX;
        public int _lineY;
        public int _orilineX;
        public int _orilineY;
        public int _lastOrilineX;
        public int _lastOrilineY;
        public int _delay;
        public int _swtch0;
        public int _swtch1;
        public int _count;
        public int _looping;
        public int vRecSwtc;
    }



    public class sc_console_writer
    {
        [DllImport("User32.dll", CharSet = CharSet.Unicode)]
        public static extern int MessageBox(IntPtr h, string m, string c, int type);

        static _messager[] _message_to_pass = new _messager[MainWindow.MaxSizeMessageObject];
        public static List<_messager> _message_to_pass_list = new List<_messager>();

        _messager _dummyMessage = new _messager();

        int _console_width = 0;
        int _console_height = 0;

        static int[] _map_array_last;
        static int[] _map_array;
        static int[] _map_array_dirty;

        int _original_width = 0;
        int _original_height = 0;

        string _program_name = "skYaRk";
        public static sc_console_writer _CONSOLE_WRITER;
        public List<object[]> _TASK_00_WR_QUEUE = new List<object[]>();
        public object _ResultsOfTasks0;
        public int _Task00_init_console = 1;
        public Task _TASK_00_WR;
        public int _console_is_alive_00_WR = 0;
        public int _console_ERROR = -1;
        public int _console_hasINIT = 0;
        public int _xCurrentCursorPos;
        public int _yCurrentCursorPos;

        string _lastConsoleMessage = "";

        int mainMessageCursorPosSwitchCounter = 0;



        int currentWidthLast = 0;
        int currentHeightLast = 0;


        public sc_console_writer(sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object[] tester)// : base(tester)
        {
            _console_width = Console.WindowWidth;
            _console_height = Console.WindowHeight;
            _original_width = _console_width;
            _original_height = _console_height;

            _initX = (_console_width / 2) - (_program_name.Length / 2);
            _initY = (_console_height / 2);

            _map_array = new int[_console_width * _console_height];
            _map_array_last = new int[_console_width * _console_height];
            _map_array_dirty = new int[_console_width * _console_height];

            //_fastNoise = new FastNoise();

            for (int x = 0; x < _console_width; x++)
            {
                for (int y = 0; y < _console_height; y++)
                {
                    if (x == 0 && y > 0 && y < _original_height - 1)
                    {
                        try
                        {
                            //Draw(x, y, "│");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (y == 0 && x > 0 && x < _original_width - 1)
                    {
                        try
                        {
                            //Draw(x, y, "─");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (x == _original_width - 1 && y > 0 && y < _original_height - 1)
                    {
                        try
                        {
                            //Draw(x, y, "│");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (y == _original_height - 1 && x > 0 && x < _original_width - 1)
                    {
                        try
                        {
                            //Draw(x, y, "─");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (y == 0 && x == 0)
                    {
                        try
                        {
                            //Draw(x, y, "┌");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (y == _original_height - 1 && x == 0)
                    {
                        try
                        {
                            //Draw(x, y, "└");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (y == 0 && x == _original_width - 1)
                    {
                        try
                        {
                            //Draw(x, y, "┐");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else if (y == _original_height - 1 && x == _original_width - 1)
                    {
                        try
                        {
                            //Draw(x, y, "┘");
                            _map_array[y * _console_width + x] = 2;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    else
                    {
                        try
                        {
                            //Draw(x, y, " ");
                            _map_array[y * _console_width + x] = 0;
                        }
                        catch (Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                }
            }

            for (int i = 0; i < _map_array.Length; i++)
            {
                _map_array_last[i] = _map_array[i];
                _map_array_dirty[i] = _map_array[i];
            }

            currentWidthLast = _console_width;
            currentHeightLast = _console_height;

        }



        [DllImport("Kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
        static extern SafeFileHandle CreateFile(
        string fileName,
        [MarshalAs(UnmanagedType.U4)] uint fileAccess,
        [MarshalAs(UnmanagedType.U4)] uint fileShare,
        IntPtr securityAttributes,
        [MarshalAs(UnmanagedType.U4)] FileMode creationDisposition,
        [MarshalAs(UnmanagedType.U4)] int flags,
        IntPtr template);

        [StructLayout(LayoutKind.Sequential)]
        public struct Coord
        {
            public short X;
            public short Y;

            public Coord(short X, short Y)
            {
                this.X = X;
                this.Y = Y;
            }
        };

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool WriteConsoleOutputCharacter(
            SafeFileHandle hConsoleOutput,
            string lpCharacter,
            int nLength,
            Coord dwWriteCoord,
            ref int lpumberOfCharsWritten);

        public void Draw(int x, int y, string renderingChar)
        {
            // The handle to the output buffer of the console
            SafeFileHandle consoleHandle = CreateFile("CONOUT$", 0x40000000, 2, IntPtr.Zero, FileMode.Open, 0, IntPtr.Zero);

            // Draw with this native method because this method does NOT move the cursor.
            int n = 0;
            WriteConsoleOutputCharacter(consoleHandle, renderingChar, 1, new Coord((short)x, (short)y), ref n);
        }


        int _initX = 0;
        int _initY = 0;
        string _char;
        string _cut_msg;
        int _targetLineX;
        int _targetLineY;

        int _last_console_width = 0;
        int _last_console_height = 0;
        int _counter_reset_console_borders = 0;

        //int _current_message_posX = 0;

        public _messager[] _console_writer(_messager[] _main_object)  //object[]
        {
            //int currentWidth = Console.WindowWidth;
            //int currentHeight = Console.WindowHeight;

            _xCurrentCursorPos = Console.CursorLeft;
            _yCurrentCursorPos = Console.CursorTop;

            mainMessageCursorPosSwitchCounter = 0;





            if (Console.WindowWidth != _last_console_width || Console.WindowHeight != _last_console_height)
            {
                if (_counter_reset_console_borders > 50)
                {
                    _last_console_width = Console.WindowWidth;
                    _last_console_height = Console.WindowHeight;
                    _counter_reset_console_borders = 0;
                }
                _counter_reset_console_borders++;
            }


            for (int i = 0; i < _main_object.Length; i++)
            {
                _message_to_pass[i] = (_messager)_main_object[i];

                if (_message_to_pass[i]._specialMessage == 0)
                {
                    if (_message_to_pass[i]._swtch0 == 1)
                    {
                        if (_message_to_pass[i]._count >= _message_to_pass[i]._delay)
                        {
                            if (_message_to_pass[i]._messageCut != "")
                            {
                                _char = _message_to_pass[i]._messageCut.Substring(0, 1);
                                _cut_msg = _message_to_pass[i]._messageCut.Substring(1, _message_to_pass[i]._messageCut.Length - 1);
                                _message_to_pass[i]._messageCut = _cut_msg;
                                _message_to_pass[i]._message = _char;

                                _targetLineX = (int)_message_to_pass[i]._lineX;
                                _targetLineY = (int)_message_to_pass[i]._lineY;

                                Draw(_targetLineX, _targetLineY, _char);
                                _map_array[_targetLineY * _console_width + _targetLineX] = 2;
                                _map_array_dirty[_targetLineY * _console_width + _targetLineX] = _message_to_pass[i]._delay * 10;

                                _lastConsoleMessage = _char.ToString();

                                _message_to_pass[i]._count = 0;

                                _targetLineX = (int)_message_to_pass[i]._lineX + 1;
                                _targetLineY = (int)_message_to_pass[i]._lineY;
                                _message_to_pass[i]._lineX = _targetLineX;
                                _message_to_pass[i]._lineY = _targetLineY;
                            }
                            else
                            {
                                _message_to_pass[i]._message = "";
                                _message_to_pass[i]._originalMsg = "";
                                _message_to_pass[i]._messageCut = "";
                                _message_to_pass[i]._specialMessage = -1;
                                _message_to_pass[i]._specialMessageLineX = 0;
                                _message_to_pass[i]._specialMessageLineY = 0;
                                _message_to_pass[i]._lineX = 0;
                                _message_to_pass[i]._lineY = 0;
                                _message_to_pass[i]._count = 0;
                                _message_to_pass[i]._swtch0 = 0;
                            }
                        }
                        _message_to_pass[i]._count = _message_to_pass[i]._count + 1;
                        mainMessageCursorPosSwitchCounter++;
                    }
                    else
                    {
                        mainMessageCursorPosSwitchCounter--;
                    }
                }
                else if (_message_to_pass[i]._specialMessage == 2)
                {
                    if (_message_to_pass[i]._swtch0 == 1)
                    {
                        _message_to_pass[i]._count = _message_to_pass[i]._delay;
                        _message_to_pass[i]._swtch0 = 2;
                    }
                    else if (_message_to_pass[i]._swtch0 == 2)
                    {
                        if (_message_to_pass[i]._messageCut != "")
                        {
                            if (_message_to_pass[i]._count <= 0)
                            {
                                _char = _message_to_pass[i]._messageCut.Substring(0, 1);

                                _cut_msg = _message_to_pass[i]._messageCut.Substring(1, _message_to_pass[i]._messageCut.Length - 1);
                                _message_to_pass[i]._messageCut = _cut_msg;
                                _message_to_pass[i]._message = _char;

                                _targetLineX = (int)_message_to_pass[i]._lineX;
                                _targetLineY = (int)_message_to_pass[i]._lineY;

                                Draw(_targetLineX, _targetLineY, _char);
                                _map_array[_targetLineY * _console_width + _targetLineX] = 1;
                                _map_array_dirty[_targetLineY * _console_width + _targetLineX] = _message_to_pass[i]._delay * 10;

                                _lastConsoleMessage = _char.ToString();

                                _message_to_pass[i]._count = 0;

                                _targetLineX = (int)_message_to_pass[i]._lineX + 1;
                                _targetLineY = (int)_message_to_pass[i]._lineY;
                                _message_to_pass[i]._lineX = _targetLineX;
                                _message_to_pass[i]._lineY = _targetLineY;
                                _message_to_pass[i]._count = _message_to_pass[i]._delay;
                            }
                            else
                            {
                                _message_to_pass[i]._count--;
                            }
                        }
                        else
                        {
                            if (_message_to_pass[i]._swtch1 == 1)
                            {
                                _message_to_pass[i]._count = _message_to_pass[i]._delay * 15;

                                _message_to_pass[i]._swtch1 = 2;
                            }
                            else if (_message_to_pass[i]._swtch1 == 2)
                            {
                                if (_message_to_pass[i]._count <= 0)
                                {
                                    if (_message_to_pass[i]._looping == 1)
                                    {
                                        _message_to_pass[i]._message = _message_to_pass[i]._originalMsg;
                                        _message_to_pass[i]._originalMsg = _message_to_pass[i]._originalMsg;
                                        _message_to_pass[i]._messageCut = _message_to_pass[i]._originalMsg;
                                        _message_to_pass[i]._specialMessage = 2;
                                        _message_to_pass[i]._specialMessageLineX = 0;
                                        _message_to_pass[i]._specialMessageLineY = 0;
                                        _message_to_pass[i]._lineX = _message_to_pass[i]._orilineX;
                                        _message_to_pass[i]._lineY = _message_to_pass[i]._orilineY;
                                        _message_to_pass[i]._count = 0;
                                        _message_to_pass[i]._swtch0 = 1;
                                        _message_to_pass[i]._swtch1 = 1;
                                    }
                                    else
                                    {
                                        _message_to_pass[i]._message = _message_to_pass[i]._originalMsg;
                                        _message_to_pass[i]._originalMsg = _message_to_pass[i]._originalMsg;
                                        _message_to_pass[i]._messageCut = _message_to_pass[i]._originalMsg;
                                        _message_to_pass[i]._specialMessage = -1;
                                        _message_to_pass[i]._specialMessageLineX = 0;
                                        _message_to_pass[i]._specialMessageLineY = 0;
                                        _message_to_pass[i]._lineX = 0;
                                        _message_to_pass[i]._lineY = 0;
                                        _message_to_pass[i]._count = 0;
                                        _message_to_pass[i]._swtch0 = 0;
                                        _message_to_pass[i]._swtch1 = 0;
                                    }
                                }
                                else
                                {
                                    _message_to_pass[i]._count--;
                                }
                            }
                        }
                    }
                }
                else if (_message_to_pass[i]._specialMessage == 3)
                {
                    if (_message_to_pass[i]._swtch0 == 1)
                    {
                        if (_message_to_pass[i]._count >= _message_to_pass[i]._delay)
                        {
                            if (_message_to_pass[i]._messageCut != "")
                            {
                                _char = _message_to_pass[i]._messageCut.Substring(0, 1);
                                _cut_msg = _message_to_pass[i]._messageCut.Substring(1, _message_to_pass[i]._messageCut.Length - 1);
                                _message_to_pass[i]._messageCut = _cut_msg;
                                _message_to_pass[i]._message = _char;

                                _targetLineX = (int)_message_to_pass[i]._lineX;
                                _targetLineY = (int)_message_to_pass[i]._lineY;

                                Draw(_targetLineX, _targetLineY, _char);
                                //_map_array[_targetLineY * _console_width + _targetLineX] = 2;
                                //_map_array_dirty[_targetLineY * _console_width + _targetLineX] = _message_to_pass[i]._delay * 10;

                                _lastConsoleMessage = _char.ToString();

                                _message_to_pass[i]._count = 0;

                                _targetLineX = (int)_message_to_pass[i]._lineX + 1;
                                _targetLineY = (int)_message_to_pass[i]._lineY;
                                _message_to_pass[i]._lineX = _targetLineX;
                                _message_to_pass[i]._lineY = _targetLineY;
                            }
                            else
                            {
                                _message_to_pass[i]._message = "";
                                _message_to_pass[i]._originalMsg = "";
                                _message_to_pass[i]._messageCut = "";
                                _message_to_pass[i]._specialMessage = -1;
                                _message_to_pass[i]._specialMessageLineX = 0;
                                _message_to_pass[i]._specialMessageLineY = 0;
                                _message_to_pass[i]._lineX = 0;
                                _message_to_pass[i]._lineY = 0;
                                _message_to_pass[i]._count = 0;
                                _message_to_pass[i]._swtch0 = 0;
                            }
                        }
                        _message_to_pass[i]._count = _message_to_pass[i]._count + 1;
                        mainMessageCursorPosSwitchCounter++;
                    }
                    else
                    {
                        mainMessageCursorPosSwitchCounter--;
                    }
                }
                else if (_message_to_pass[i]._specialMessage == 4)
                {
                    if (_message_to_pass[i]._swtch0 == 1)
                    {
                        if (_message_to_pass[i]._count >= _message_to_pass[i]._delay)
                        {
                            if (_message_to_pass[i]._messageCut != "")
                            {
                                _char = _message_to_pass[i]._messageCut.Substring(0, 1);
                                _cut_msg = _message_to_pass[i]._messageCut.Substring(1, _message_to_pass[i]._messageCut.Length - 1);
                                _message_to_pass[i]._messageCut = _cut_msg;
                                _message_to_pass[i]._message = _char;

                                _targetLineX = (int)_message_to_pass[i]._lineX;
                                _targetLineY = (int)_message_to_pass[i]._lineY;

                                Draw(_targetLineX, _targetLineY, _char);
                                _map_array[_targetLineY * _console_width + _targetLineX] = 2;
                                _map_array_dirty[_targetLineY * _console_width + _targetLineX] = _message_to_pass[i]._delay * 10;

                                _lastConsoleMessage = _char.ToString();

                                _message_to_pass[i]._count = 0;

                                _targetLineX = (int)_message_to_pass[i]._lineX + 1;
                                _targetLineY = (int)_message_to_pass[i]._lineY;
                                _message_to_pass[i]._lineX = _targetLineX;
                                _message_to_pass[i]._lineY = _targetLineY;
                            }
                            else
                            {
                                _message_to_pass[i]._message = "";
                                _message_to_pass[i]._originalMsg = "";
                                _message_to_pass[i]._messageCut = "";
                                _message_to_pass[i]._specialMessage = -1;
                                _message_to_pass[i]._specialMessageLineX = 0;
                                _message_to_pass[i]._specialMessageLineY = 0;
                                _message_to_pass[i]._lineX = 0;
                                _message_to_pass[i]._lineY = 0;
                                _message_to_pass[i]._count = 0;
                                _message_to_pass[i]._swtch0 = 0;
                            }
                        }
                        _message_to_pass[i]._count = _message_to_pass[i]._count + 1;
                        mainMessageCursorPosSwitchCounter++;
                    }
                    else
                    {
                        mainMessageCursorPosSwitchCounter--;
                    }
                }
                /*if (_message_to_pass[i]._messager_list != null)
                {

                    for (int c = 0; c < _message_to_pass[i]._messager_list.Length; c++)
                    {
                        if (_message_to_pass[i]._messager_list[c]._specialMessage == 0)
                        {
                            if (_message_to_pass[i]._messager_list[c]._swtch0 == 1)
                            {
                                if (_message_to_pass[i]._messager_list[c]._count >= _message_to_pass[i]._messager_list[c]._delay)
                                {
                                    if (_message_to_pass[i]._messager_list[c]._messageCut != "")
                                    {
                                        _char = _message_to_pass[i]._messager_list[c]._messageCut.Substring(0, 1);

                                        _cut_msg = _message_to_pass[i]._messager_list[c]._messageCut.Substring(1, _message_to_pass[i]._messager_list[c]._messageCut.Length - 1);
                                        _message_to_pass[i]._messager_list[c]._messageCut = _cut_msg;
                                        _message_to_pass[i]._messager_list[c]._message = _char;

                                        _targetLineX = (int)_message_to_pass[i]._messager_list[c]._lineX;
                                        _targetLineY = (int)_message_to_pass[i]._messager_list[c]._lineY;

                                        Draw(_targetLineX, _targetLineY, _char);
                                        _map_array[_targetLineY * _console_width + _targetLineX] = 2;
                                        _map_array_dirty[_targetLineY * _console_width + _targetLineX] = _message_to_pass[i]._messager_list[c]._delay * 10;

                                        _lastConsoleMessage = _char.ToString();

                                        _message_to_pass[i]._messager_list[c]._count = 0;

                                        _targetLineX = (int)_message_to_pass[i]._messager_list[c]._lineX + 1;
                                        _targetLineY = (int)_message_to_pass[i]._messager_list[c]._lineY;
                                        _message_to_pass[i]._messager_list[c]._lineX = _targetLineX;
                                        _message_to_pass[i]._messager_list[c]._lineY = _targetLineY;
                                    }
                                    else
                                    {
                                        _message_to_pass[i]._messager_list[c]._message = "";
                                        _message_to_pass[i]._messager_list[c]._originalMsg = "";
                                        _message_to_pass[i]._messager_list[c]._messageCut = "";
                                        _message_to_pass[i]._messager_list[c]._specialMessage = -1;
                                        _message_to_pass[i]._messager_list[c]._specialMessageLineX = 0;
                                        _message_to_pass[i]._messager_list[c]._specialMessageLineY = 0;
                                        _message_to_pass[i]._messager_list[c]._lineX = 0;
                                        _message_to_pass[i]._messager_list[c]._lineY = 0;
                                        _message_to_pass[i]._messager_list[c]._count = 0;
                                        _message_to_pass[i]._messager_list[c]._swtch0 = 0;
                                    }
                                }
                                _message_to_pass[i]._messager_list[c]._count = _message_to_pass[i]._messager_list[c]._count + 1;
                                mainMessageCursorPosSwitchCounter++;
                            }
                            else
                            {
                                mainMessageCursorPosSwitchCounter--;
                            }
                        }
                        else if (_message_to_pass[i]._messager_list[c]._specialMessage == 2)
                        {
                            if (_message_to_pass[i]._messager_list[c]._swtch0 == 1)
                            {
                                _message_to_pass[i]._messager_list[c]._count = _message_to_pass[i]._messager_list[c]._delay;
                                _message_to_pass[i]._messager_list[c]._swtch0 = 2;
                            }
                            else if (_message_to_pass[i]._messager_list[c]._swtch0 == 2)
                            {
                                if (_message_to_pass[i]._messager_list[c]._messageCut != "")
                                {
                                    if (_message_to_pass[i]._messager_list[c]._count <= 0)
                                    {
                                        _char = _message_to_pass[i]._messager_list[c]._messageCut.Substring(0, 1);

                                        _cut_msg = _message_to_pass[i]._messager_list[c]._messageCut.Substring(1, _message_to_pass[i]._messager_list[c]._messageCut.Length - 1);
                                        _message_to_pass[i]._messager_list[c]._messageCut = _cut_msg;
                                        _message_to_pass[i]._messager_list[c]._message = _char;

                                        _targetLineX = (int)_message_to_pass[i]._messager_list[c]._lineX;
                                        _targetLineY = (int)_message_to_pass[i]._messager_list[c]._lineY;

                                        Draw(_targetLineX, _targetLineY, _char);
                                        _map_array[_targetLineY * _console_width + _targetLineX] = 1;
                                        _map_array_dirty[_targetLineY * _console_width + _targetLineX] = _message_to_pass[i]._messager_list[c]._delay * 10;

                                        _lastConsoleMessage = _char.ToString();

                                        _message_to_pass[i]._messager_list[c]._count = 0;

                                        _targetLineX = (int)_message_to_pass[i]._messager_list[c]._lineX + 1;
                                        _targetLineY = (int)_message_to_pass[i]._messager_list[c]._lineY;
                                        _message_to_pass[i]._messager_list[c]._lineX = _targetLineX;
                                        _message_to_pass[i]._messager_list[c]._lineY = _targetLineY;
                                        _message_to_pass[i]._messager_list[c]._count = _message_to_pass[i]._messager_list[c]._delay;
                                    }
                                    else
                                    {
                                        _message_to_pass[i]._messager_list[c]._count--;
                                    }
                                }
                                else
                                {
                                    if (_message_to_pass[i]._messager_list[c]._swtch1 == 1)
                                    {
                                        _message_to_pass[i]._messager_list[c]._count = _message_to_pass[i]._messager_list[c]._delay * 15;

                                        _message_to_pass[i]._messager_list[c]._swtch1 = 2;
                                    }
                                    else if (_message_to_pass[i]._messager_list[c]._swtch1 == 2)
                                    {
                                        if (_message_to_pass[i]._messager_list[c]._count <= 0)
                                        {
                                            if (_message_to_pass[i]._messager_list[c]._looping == 1)
                                            {
                                                _message_to_pass[i]._messager_list[c]._message = _message_to_pass[i]._messager_list[c]._originalMsg;
                                                _message_to_pass[i]._messager_list[c]._originalMsg = _message_to_pass[i]._messager_list[c]._originalMsg;
                                                _message_to_pass[i]._messager_list[c]._messageCut = _message_to_pass[i]._messager_list[c]._originalMsg;
                                                _message_to_pass[i]._messager_list[c]._specialMessage = 2;
                                                _message_to_pass[i]._messager_list[c]._specialMessageLineX = 0;
                                                _message_to_pass[i]._messager_list[c]._specialMessageLineY = 0;
                                                _message_to_pass[i]._messager_list[c]._lineX = _message_to_pass[i]._messager_list[c]._orilineX;
                                                _message_to_pass[i]._messager_list[c]._lineY = _message_to_pass[i]._messager_list[c]._orilineY;
                                                _message_to_pass[i]._messager_list[c]._count = 0;
                                                _message_to_pass[i]._messager_list[c]._swtch0 = 1;
                                                _message_to_pass[i]._messager_list[c]._swtch1 = 1;
                                            }
                                            else
                                            {
                                                _message_to_pass[i]._messager_list[c]._message = _message_to_pass[i]._messager_list[c]._originalMsg;
                                                _message_to_pass[i]._messager_list[c]._originalMsg = _message_to_pass[i]._messager_list[c]._originalMsg;
                                                _message_to_pass[i]._messager_list[c]._messageCut = _message_to_pass[i]._messager_list[c]._originalMsg;
                                                _message_to_pass[i]._messager_list[c]._specialMessage = -1;
                                                _message_to_pass[i]._messager_list[c]._specialMessageLineX = 0;
                                                _message_to_pass[i]._messager_list[c]._specialMessageLineY = 0;
                                                _message_to_pass[i]._messager_list[c]._lineX = 0;
                                                _message_to_pass[i]._messager_list[c]._lineY = 0;
                                                _message_to_pass[i]._messager_list[c]._count = 0;
                                                _message_to_pass[i]._messager_list[c]._swtch0 = 0;
                                                _message_to_pass[i]._messager_list[c]._swtch1 = 0;
                                            }
                                        }
                                        else
                                        {
                                            _message_to_pass[i]._messager_list[c]._count--;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }*/
            }
            lock (_message_to_pass_list)
            {
                if (_message_to_pass_list.Count > 0)
                {
                    _dummyMessage = _message_to_pass_list[0];
                    if (_dummyMessage._count >= _dummyMessage._delay)
                    {
                        if (_dummyMessage._messageCut != "")
                        {
                            string _char = _dummyMessage._messageCut.Substring(0, 1);
                            string _cut_msg = _dummyMessage._messageCut.Substring(1, _dummyMessage._messageCut.Length - 1);
                            _dummyMessage._messageCut = _cut_msg;

                            int _targetLineX = (int)_dummyMessage._lineX;
                            int _targetLineY = (int)_dummyMessage._lineY;
                            Draw(_targetLineX, _targetLineY, _char);

                            //MessageBox((IntPtr)0, _char + "", "Console", 0);

                            _dummyMessage._count = 0;
                            _targetLineX = (int)_dummyMessage._lineX + 1;
                            _targetLineY = (int)_dummyMessage._lineY;
                            _dummyMessage._lineX = _targetLineX;
                            _dummyMessage._lineY = _targetLineY;

                            _message_to_pass_list[0] = _dummyMessage;
                        }
                        else
                        {
                            _message_to_pass_list[0] = _dummyMessage;
                            _message_to_pass_list.Remove(_message_to_pass_list[0]);
                        }
                    }
                    else
                    {
                        _dummyMessage._count = _dummyMessage._count + 1;
                        _message_to_pass_list[0] = _dummyMessage;
                    }
                }
                else
                {
                    //Draw(1, 6, _message_to_pass_list.Count + "");
                }
            }

            for (int x = 0; x < _console_width; x++)
            {
                for (int y = 0; y < _console_height; y++)
                {
                    if (_map_array[y * _console_width + x] == 1)
                    {
                        if (_map_array_dirty[y * _console_width + x] != 0)
                        {
                            _map_array_dirty[y * _console_width + x]--;

                        }
                        else
                        {
                            Draw(x, y, " ");
                            _map_array[(y * _console_width) + x] = 0;
                            _map_array_dirty[(y * _console_width) + x] = 0;
                        }
                    }
                    else
                    {

                    }
                }
            }
            return _message_to_pass;
        }
    }
}
