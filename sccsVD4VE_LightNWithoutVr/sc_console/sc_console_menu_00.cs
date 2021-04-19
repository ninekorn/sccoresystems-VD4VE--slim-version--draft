using System;

namespace sccsVD4VE_LightNWithoutVr.sc_console_menu
{
    public static class sc_console_menu_00
    {
        static sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object _someReceivedObject0 = new sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object();
        static sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object _data00_IN;
        static sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object _toReturnObject;

        public static sc_message_object.sc_message_object[] _console_menu(sc_message_object.sc_message_object[] _main_object)
        {
            try
            {
                for (int L0_IN = 0; L0_IN < _main_object.Length; L0_IN++)
                {
                    _data00_IN = _main_object[L0_IN];

                    int _work_doner = _data00_IN._work_done;
                    int _received_switch_in00 = _data00_IN._received_switch_in;   
                    int _received_switch_out00 = _data00_IN._received_switch_out; 
                    int _sending_switch_in00 = _data00_IN._sending_switch_in;     
                    int _sending_switch_out00 = _data00_IN._sending_switch_out;   

                    int _timeOut00 = _data00_IN._timeOut0;
                    int _ParentTaskThreadID00 = _data00_IN._ParentTaskThreadID0;
                    int _main_cpu_count00 = _data00_IN._main_cpu_count;
                    string _passTest00 = _data00_IN._passTest;
                    int _welcomePackage00 = _data00_IN._welcomePackage;
                    int _current_menu00 = _data00_IN._current_menu;
                    int _last_current_menu00 = _data00_IN._last_current_menu;

                    if (_received_switch_in00 == 0 &&
                        _received_switch_out00 == 0 &&
                        _sending_switch_in00 == 0 &&
                        _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 1 &&
                             _received_switch_out00 == 0 &&
                             _sending_switch_in00 == 0 &&
                             _sending_switch_out00 == 0)
                    {
                        if (_welcomePackage00 == -1)
                        {
                            _main_object[L0_IN]._received_switch_in = 0;
                            _main_object[L0_IN]._received_switch_out = 0;
                            _main_object[L0_IN]._sending_switch_in = 0;
                            _main_object[L0_IN]._sending_switch_out = 0;
                        }
                        else if (_welcomePackage00 == 0)
                        {
                            if (_passTest00.ToLower() == "nine" || _passTest00.ToLower() == "ninekorn" || _passTest00.ToLower() == "9")
                            {
                                _main_object[L0_IN]._received_switch_in = 1;
                                _main_object[L0_IN]._received_switch_out = 1;
                                _main_object[L0_IN]._sending_switch_in = 0;
                                _main_object[L0_IN]._sending_switch_out = 0;
                                _main_object[L0_IN]._passTest = _passTest00.ToLower();
                                _main_object[L0_IN]._welcomePackage = 1;
                                _main_object[L0_IN]._work_done = 1;
                                _main_object[L0_IN]._main_menu = 0;
                            }
                            else
                            {
                                _main_object[L0_IN]._received_switch_in = 1;
                                _main_object[L0_IN]._received_switch_out = 0;
                                _main_object[L0_IN]._sending_switch_in = 0;
                                _main_object[L0_IN]._sending_switch_out = 0;
                                _main_object[L0_IN]._welcomePackage = 0;
                                _main_object[L0_IN]._work_done = 1;

                                _main_object[L0_IN]._passTest = "";
                            }
                        }
                    }
                    else if (_received_switch_in00 == 0 &&
                             _received_switch_out00 == 1 &&
                             _sending_switch_in00 == 0 &&
                             _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 0 &&
                             _received_switch_out00 == 0 &&
                             _sending_switch_in00 == 1 &&
                             _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 0 &&
                           _received_switch_out00 == 0 &&
                           _sending_switch_in00 == 0 &&
                           _sending_switch_out00 == 1)
                    {

                        string _optionCommand = Console.ReadLine();

                        if (_optionCommand.ToLower() == "option" ||
                            _optionCommand.ToLower() == "command" ||
                            _optionCommand.ToLower() == "options" ||
                            _optionCommand.ToLower() == "commands")
                        {

                            _main_object[L0_IN]._received_switch_in = 0;
                            _main_object[L0_IN]._received_switch_out = 0;
                            _main_object[L0_IN]._sending_switch_in = 0;
                            _main_object[L0_IN]._sending_switch_out = 0;
                        }
                        else
                        {
                            _main_object[L0_IN]._received_switch_in = 0;
                            _main_object[L0_IN]._received_switch_out = 0;
                            _main_object[L0_IN]._sending_switch_in = 0;
                            _main_object[L0_IN]._sending_switch_out = 0;
                        }
                    }
                    else if (_received_switch_in00 == 1 &&
                            _received_switch_out00 == 1 &&
                            _sending_switch_in00 == 0 &&
                            _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 1 &&
                         _received_switch_out00 == 0 &&
                         _sending_switch_in00 == 1 &&
                         _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 1 &&
                           _received_switch_out00 == 0 &&
                           _sending_switch_in00 == 0 &&
                           _sending_switch_out00 == 1)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 0 &&
                        _received_switch_out00 == 1 &&
                        _sending_switch_in00 == 1 &&
                        _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 0 &&
                          _received_switch_out00 == 1 &&
                          _sending_switch_in00 == 0 &&
                          _sending_switch_out00 == 1)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 0 &&
                          _received_switch_out00 == 0 &&
                          _sending_switch_in00 == 1 &&
                          _sending_switch_out00 == 1)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 1 &&
                               _received_switch_out00 == 0 &&
                               _sending_switch_in00 == 1 &&
                               _sending_switch_out00 == 1)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 0 &&
                            _received_switch_out00 == 1 &&
                            _sending_switch_in00 == 1 &&
                            _sending_switch_out00 == 1)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }

                    else if (_received_switch_in00 == 1 &&
                            _received_switch_out00 == 1 &&
                            _sending_switch_in00 == 0 &&
                            _sending_switch_out00 == 1)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 1 &&
                            _received_switch_out00 == 1 &&
                            _sending_switch_in00 == 1 &&
                            _sending_switch_out00 == 0)
                    {
                        _main_object[L0_IN]._received_switch_in = 0;
                        _main_object[L0_IN]._received_switch_out = 0;
                        _main_object[L0_IN]._sending_switch_in = 0;
                        _main_object[L0_IN]._sending_switch_out = 0;
                    }
                    else if (_received_switch_in00 == 1 &&
                          _received_switch_out00 == 1 &&
                          _sending_switch_in00 == 1 &&
                          _sending_switch_out00 == 1)
                    {
                        if (_welcomePackage00 == 998)
                        {
                            MainWindow.init_directX_main_swtch = 2;
                        }
                        else if (_welcomePackage00 == 999)
                        {
                            MainWindow.init_vr_main_swtch = 2;
                        }
                    }
                }
            }
            catch (Exception ex)
            {

            }
            return _main_object;
        }
    }
}
