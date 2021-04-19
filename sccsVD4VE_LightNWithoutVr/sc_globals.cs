namespace sccsVD4VE_LightNWithoutVr
{
    public interface sc_globals
    {
        sccsVD4VE_LightNWithoutVr.sc_console.sc_console_core SC_CONSOLE_CORE { get; set; }
        sccsVD4VE_LightNWithoutVr.sc_console.sc_console_writer SC_CONSOLE_WRITER { get; set; }
        sccsVD4VE_LightNWithoutVr.sc_console.sc_console_reader SC_CONSOLE_READER { get; set; }
        sccsVD4VE_LightNWithoutVr.sc_core.sc_globals_accessor SC_GLOBALS_ACCESSORS { get; set; }
        int _Activate_Desktop_Image { get; set; }

    }
}
