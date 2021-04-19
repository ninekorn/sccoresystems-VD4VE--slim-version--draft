using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;



namespace sccsVD4VE_LightNWithoutVr.sc_core
{
    public class sc_globals_accessor : sc_icomponent, sc_globals
    {
        public static sc_icomponent SC_ICOMPONENT;
        sc_globals sc_icomponent.SC_Globals
        {
            get => SC_GLOB;
        }
        public static sc_globals SC_GLOB;


        public virtual sccsVD4VE_LightNWithoutVr.sc_console.sc_console_core SC_CONSOLE_CORE { get; set; }
        public virtual sccsVD4VE_LightNWithoutVr.sc_console.sc_console_writer SC_CONSOLE_WRITER { get; set; }
        public virtual sccsVD4VE_LightNWithoutVr.sc_console.sc_console_reader SC_CONSOLE_READER { get; set; }
        public virtual sccsVD4VE_LightNWithoutVr.sc_core.sc_globals_accessor SC_GLOBALS_ACCESSORS { get; set; }

        public virtual int _Activate_Desktop_Image { get; set; }

        private sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object testingInit(sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object _main_object)
        {   
            return _main_object;
        }

        public static int _init_main_Task = 1;

        public sc_globals_accessor(sccsVD4VE_LightNWithoutVr.sc_message_object.sc_message_object[] _main_object)
        {
            SC_GLOB = this;
            SC_ICOMPONENT = this;

            SC_CONSOLE_CORE = new sc_console.sc_console_core(_main_object);
            SC_CONSOLE_WRITER = new sc_console.sc_console_writer(_main_object);
            SC_CONSOLE_READER = new sc_console.sc_console_reader(_main_object);

            SC_GLOB.SC_CONSOLE_CORE = SC_CONSOLE_CORE;
            SC_GLOB.SC_CONSOLE_WRITER = SC_CONSOLE_WRITER;
            SC_GLOB.SC_CONSOLE_READER = SC_CONSOLE_READER;       
        }
    }
}
