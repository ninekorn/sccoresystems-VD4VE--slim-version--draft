using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace sc_message_object
{
    public struct sc_message_object
    {
        public int _received_switch_in;
        public int _received_switch_out;
        public int _sending_switch_in;
        public int _sending_switch_out;
        //public List<int[]> _chain_Of_Tasks0;
        public int _timeOut0;
        public int _ParentTaskThreadID0;
        public int _main_cpu_count;
        public string _passTest;
        public int _welcomePackage;
        //public ManualResetEvent _reset_event;
        public int _work_done;
        public int _current_menu;
        public int _last_current_menu;
        public int _main_menu;
        public string _menuOption;
        public int _voRecSwtc;
        public string _voRecMsg;
        public object[] _world_data;

        //public World _world; //Jitter Physics
    }

    public struct sc_message_object_jitter
    {
        public int _sc_jitter_main;
        public int _sc_jitter_can_work;
        public float _world_step;
        public object[] _world_data;
        public int _work_index;
    }
}

