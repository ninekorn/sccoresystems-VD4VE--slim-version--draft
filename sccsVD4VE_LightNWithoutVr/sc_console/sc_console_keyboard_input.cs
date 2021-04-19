

using System;
using SharpDX.DirectInput;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public class sc_console_keyboard_input
    {
        public SharpDX.DirectInput.Keyboard _Keyboard;
        DirectInput directInput = new DirectInput();
        public KeyboardState _KeyboardState;
        int _InitializeKeyboardAuth = 0;

        public sc_console_keyboard_input()
        {
            _InitializeKeyboard();
            _KeyboardState = new KeyboardState();
        }

        public int _InitializeKeyboard()
        {
            _InitializeKeyboardAuth = 1;
            try
            {
                directInput = new DirectInput();
                _Keyboard = new SharpDX.DirectInput.Keyboard(directInput);
                _Keyboard.Properties.BufferSize = 128;
                _Keyboard.Acquire();
            }
            catch
            {

                _InitializeKeyboardAuth = 0;
            }
            return _InitializeKeyboardAuth;
        }


        public bool ReadKeyboard()
        {
            var resultCode = SharpDX.DirectInput.ResultCode.Ok;

            try
            {
                _Keyboard.GetCurrentState(ref _KeyboardState);
            }
            catch (SharpDX.SharpDXException ex)
            {
                resultCode = ex.Descriptor;
            }
            catch (Exception ex)
            {
                MainWindow.MessageBox((IntPtr)0, "cannot get keyboard info 00: " + ex.ToString() + "", "Oculus error", 0);
                return false;
            }

            if (resultCode == SharpDX.DirectInput.ResultCode.InputLost || resultCode == SharpDX.DirectInput.ResultCode.NotAcquired)
            {
                try
                {
                    _Keyboard.Acquire();
                }
                catch (Exception ex)
                {
                    MainWindow.MessageBox((IntPtr)0, "cannot get keyboard info 01: " + ex.ToString() + "", "Oculus error", 0);
                }

                return true;
            }

            if (resultCode == SharpDX.DirectInput.ResultCode.Ok)
            {
                return true;
            }

            return false;
        }
    }
}
