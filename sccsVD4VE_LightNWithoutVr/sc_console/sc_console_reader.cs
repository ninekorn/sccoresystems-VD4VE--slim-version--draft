using System;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public class sc_console_reader
    {
        public sc_console_writer _SC_CONSOLE_WRITER;
        //_console_reader_data _current_console_reader_data;
        public int _main_has_init = 0;

        public sc_console_reader(object tester)
        {
            _SC_CONSOLE_WRITER = sccsVD4VE_LightNWithoutVr.sc_core.sc_globals_accessor.SC_GLOB.SC_CONSOLE_WRITER;
        }

        public _messager[] _console_reader(_messager[] _sec_received_messages)//object _console_reader_object)
        {

            //_current_console_reader_data = (_console_reader_data)_console_reader_object;

            if (_sec_received_messages[0].vRecSwtc == 0 || _sec_received_messages[0].vRecSwtc == 1)
            {
                if (_main_has_init == 0)
                {
                    string tester = Console.ReadLine();
                    //_current_console_reader_data._console_reader_message = "nothing ";
                    //_current_console_reader_data._has_message_to_display = 0;


                    _main_has_init = 1;
                }
                else if (_main_has_init == 1 || _main_has_init == 2)
                {
                    string tester = Console.ReadLine();
                    //_current_console_reader_data._console_reader_message = tester;
                    //_current_console_reader_data._has_message_to_display = 1;
                }
            }
            else
            {
                //_current_console_reader_data._has_message_to_display = 0;
                Console.WriteLine("blocked from writting to the console.");
            }
            //Console.WriteLine("blocked from writting to the console.");
            return _sec_received_messages;
        }

        public struct _console_reader_data
        {
            public int _has_init;
            public int _has_message_to_display;
            public string _console_reader_message;
        }
    }
}
