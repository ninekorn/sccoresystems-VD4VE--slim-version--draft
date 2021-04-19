using System;
using System.Collections.Generic;
using System.Text;

using SharpDX;

using SC_message_object = sc_message_object.sc_message_object;
using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;

//using Ab3d.OculusWrap;

namespace sccsVD4VE_LightNWithoutVr
{
    public abstract class sc_graphics_main //: SC_Console_GRAPHICS
    {
        public sc_graphics_main()
        {

        }

        public abstract SC_message_object_jitter[][] sc_write_to_buffer(SC_message_object_jitter[][] _sc_jitter_tasks);
        public abstract SC_message_object_jitter[][] loop_worlds(SC_message_object_jitter[][] _sc_jitter_tasks);
        public abstract SC_message_object_jitter[][] workOnSomething(SC_message_object_jitter[][] _sc_jitter_tasks, Matrix viewMatrix, Matrix projectionMatrix, Vector3 OFFSETPOS, Matrix originRot, Matrix rotatingMatrix, Matrix rotatingMatrixForPelvis, Matrix _rightTouchMatrix, Matrix _leftTouchMatrix, Vector3 handPoseRight, Vector3 handPoseLeft);
        public abstract SC_message_object_jitter[][] _sc_create_world_objects(SC_message_object_jitter[][] _sc_jitter_tasks);
    }
}
