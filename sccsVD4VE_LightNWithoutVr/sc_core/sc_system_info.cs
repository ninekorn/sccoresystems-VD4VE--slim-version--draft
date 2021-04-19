using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace sccsVD4VE_LightNWithoutVr.sc_core
{
    public static class sc_system_info
    {
        //public static int processorCount = -1;

        public static int getSystemProcessorCount()
        {
            int processorCount = Environment.ProcessorCount;
            return processorCount;
        }
    }
}
