using System;
using System.Collections.Generic;
using System.Text;

using SharpDX;
using sccsVD4VE_LightNWithoutVr.SC_Graphics;
using Jitter;
using Jitter.Dynamics;
using Jitter.Collision;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Forces;

using SC_message_object = sc_message_object.sc_message_object;
using SC_message_object_jitter = sc_message_object.sc_message_object_jitter;

using System.Collections;
using System.ComponentModel;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;


using System.Runtime.InteropServices;

using System.IO;


using Jitter.DataStructures;
using SingleBodyConstraints = Jitter.Dynamics.Constraints.SingleBody;
//using Jitter.Dynamics.Constraints.SingleBody;
//using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;
//using Jitter.Dynamics.Joints;
//using Jitter.Forces;
//using DeltaEngine.Datatypes;
//using DeltaEngine.Core;
//using DeltaEngine.Extensions;

using Vector2 = SharpDX.Vector2;
using Vector3 = SharpDX.Vector3;
using Vector4 = SharpDX.Vector4;
using Quaternion = SharpDX.Quaternion;
using Matrix = SharpDX.Matrix;
using Plane = SharpDX.Plane;
using Ray = SharpDX.Ray;

using System.Text;
using System.IO;
using SharpDX.Multimedia;
using SharpDX.IO;
using System.Xml;
using SharpDX.XAudio2;
using System.Linq;

namespace sccsVD4VE_LightNWithoutVr.sc_console
{
    public class sc_graphics_sec //: SC_Update//SC_Intermediate_Update
    {

        float heightmapscale = 0.001f;
        float heightmapscaleMin = 0.0001f;
        float heightmapscaleMax = 100f;



        float totalDiffX = 0;
        float totalDiffY = 0;
        float totalDiffZ = 0;

        public static double touchRXLast = 0;
        public static double touchRYLast = 0;
        public static double touchRZLast = 0;


        public static double touchRX = 0;
        public static double touchRY = 0;
        public static double touchRZ = 0;

        Matrix grabbedBodyMatrix = Matrix.Identity;


        double pitchTouchRer = 0;
        double yawTouchRer = 0;
        double rollTouchRer = 0;

        Matrix rotMatForPelvis = SharpDX.Matrix.Identity;

        _sc_texture_loader _pink_texture;
        _sc_texture_loader _basicTexture;

        Vector3 current_rotation_of_torso_pivot_forward = Vector3.Zero;
        Vector3 current_rotation_of_torso_pivot_right = Vector3.Zero;
        Vector3 current_rotation_of_torso_pivot_up = Vector3.Zero;


        Vector3 rayDirUp = Vector3.Zero;
        Vector3 rayDirRight = Vector3.Zero;
        Vector3 rayDirForward = Vector3.Zero;


        double grabrotX = 0;
        double grabrotY = 0;
        double grabrotZ = 0;


        double grabrotDiffx = 0;
        double grabrotDiffy = 0;
        double grabrotDiffz = 0;


        /*protected override void SC_Init_DirectX() //DSystemConfiguration configuration, IntPtr Hwnd, sc_console.sc_console_writer _writer
        {
            base.SC_Init_DirectX(); //configuration, Hwnd, _writer
        }*/

        /*public override SC_message_object_jitter[][] sc_write_to_buffer(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }

        public override SC_message_object_jitter[][] loop_worlds(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }
        public override SC_message_object_jitter[][] workOnSomething(SC_message_object_jitter[][] _sc_jitter_tasks, Matrix viewMatrix, Matrix projectionMatrix, Vector3 OFFSETPOS, Matrix originRot, Matrix rotatingMatrix, Matrix rotatingMatrixForPelvis, Matrix _rightTouchMatrix, Matrix _leftTouchMatrix, Posef handPoseRight, Posef handPoseLeft)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }
        public override SC_message_object_jitter[][] _sc_create_world_objects(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            return _sc_jitter_tasks;
            //base.sc_write_to_buffer(_sc_jitter_tasks);
        }


        protected override SC_message_object_jitter[][] init_update_variables(SC_message_object_jitter[][] _sc_jitter_tasks, sccsVD4VE_LightNWithoutVr.sc_core.sc_system_configuration configuration, IntPtr hwnd, sc_console.sc_console_writer _writer)
        {

        }
        protected override SC_message_object_jitter[][] Update(_sc_jitter_physics[] _sc_jitter_physics, SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            base.Update(_sc_jitter_physics, _sc_jitter_tasks);
        }*/


        Vector3 direction_feet_forward_ori = Vector3.Zero;
        Vector3 direction_feet_right_ori = Vector3.Zero;
        Vector3 direction_feet_up_ori = Vector3.Zero;



        int sc_menu_scroller = 0;
        int sc_menu_scroller_counter = 0;

        Vector4 ambientColor = new Vector4(0.45f, 0.45f, 0.45f, 1.0f);
        Vector4 diffuseColour = new Vector4(1, 1, 1, 1);
        Vector3 lightDirection = new Vector3(0, -1, -1);
        Vector3 dirLight = new Vector3(0, -1, 0);
        Vector3 lightpos = new Vector3(0, 10, 0);



        SharpDX.Matrix _oculusR_Cursor_matrix = SharpDX.Matrix.Identity;
        Stopwatch _updateFunctionStopwatchRightThumbstickGoRight = new Stopwatch();
        Stopwatch _updateFunctionStopwatchRightThumbstickGoLeft = new Stopwatch();
        Stopwatch _updateFunctionStopwatchLeftThumbstick = new Stopwatch();
        Stopwatch _updateFunctionStopwatchRight = new Stopwatch();

        int frame_counter_4_buttonY = 45;
        int display_grid_type = 0;

        int gravity_swtch_counter = 45;
        int gravity_swtch = 0;

        Matrix hmd_matrix_current = Matrix.Identity;

        SharpDX.Matrix _intersectTouchRightMatrix = SharpDX.Matrix.Identity;
        SharpDX.Matrix _intersectTouchLeftMatrix = SharpDX.Matrix.Identity;

        Matrix final_hand_pos_right_locked;
        Matrix final_hand_pos_left_locked;
        Matrix tempMatrix = Matrix.Identity;// tempMatrix
        Matrix _last_screen_pos = Matrix.Identity;
        int had_locked_screen = -1;
        int _tier_logic_swtch_lock_screen = 0;
        Matrix _current_screen_pos = Matrix.Identity;

        

        float disco_sphere_rot_speed = 0.15f;
        float force_4_voxel = 0.0015f;
        float force_4_cubes = 0.0015f;
        float force_4_screen = 0.0015f;
        int _has_locked_screen_pos = 0;
        int _has_locked_screen_pos_counter = 0;
        Matrix _direction_offsetter;
        Matrix _screen_direction_offsetter_two;
        float sizeWidtherer = 0.0f;
        float sizeheighterer = 0.0f;
        Matrix[] worldMatrix_Cloth_instances;

        //PseudoCloth sc_jitter_cloth;
        double RotationScreenY { get; set; }
        double RotationScreenX { get; set; }
        double RotationScreenZ { get; set; }

        Matrix originRotScreen;
        Matrix rotatingMatrixScreen;
        float oriRotationScreenY { get; set; }
        float oriRotationScreenX { get; set; }
        float oriRotationScreenZ { get; set; }

        struct _rigid_data
        {
            public RigidBody _body;
            public Matrix position;
            public Vector3 directionToGrabber;
            public Vector3 rayGrabDir;
            public float rayGrabDirLength;
            public Vector3 grabHitPoint;
            public float grabHitPointLength;
            public float dirDiffX;
            public float dirDiffY;
            public float dirDiffZ;

            public int _index;
            public int _physics_engine_index;
        }

        _rigid_data _grab_rigid_data;
        float _size_screen;
        int[][][] swtch_for_last_pos;
        int tempIndex = 0;
        int _inactive_counter_cubes = 0;
        int _inactive_counter_voxels = 0;

        int _static_counter = 0;
        Quaternion quat_buffers;

        //SCREEN SETTINGS
        int _inst_screen_x = 1;
        int _inst_screen_y = 1;
        int _inst_screen_z = 1;

        float _screen_size_x = 2; //0.0115f //1.5f
        float _screen_size_y = 2; //0.0115f //1.5f
        float _screen_size_z = 0.0035f; //0.0025f

        float mulScreen = 0.85f;

        int _inst_screen_assets_x = 3;
        int _inst_screen_assets_y = 1;
        int _inst_screen_assets_z = 3;

        float _screen_assets_size_x = 0.005f; //0.0115f //1.5f
        float _screen_assets_size_y = 0.005f; //0.0115f //1.5f
        float _screen_assets_size_z = 0.025f;

        bool is_static = false;
        //END OF

        //HUMAN RIG
        const int _human_inst_rig_x = 1;
        const int _human_inst_rig_y = 1;
        const int _human_inst_rig_z = 1;
        const int _addToWorld = 0;

        //the physics engine can run 4000 objects enabled and having angularOrLinear velocities but the voxels lag a bit at that many objects. but loading as many as 475000 cubes
        //having 36 vertices each and 72 triangles each but at that point it will also lag. will try later to improve the performance.
        float _voxel_mass = 100;
        int _inst_voxel_cube_x = 1;
        int _inst_voxel_cube_y = 1;
        int _inst_voxel_cube_z = 1;
        float _voxel_cube_size_x = 0.15f;//0.0115f //restitution
        float _voxel_cube_size_y = 0.15f;//0.0115f //static friction
        float _voxel_cube_size_z = 0.15f;//0.0015f //kinetic friction
        float voxel_general_size = 0.0025f;
        int voxel_type = -1;



        float _voxel_rig_cube_size_x = 0.15f;//0.0115f //restitution
        float _voxel_rig_cube_size_y = 0.15f;//0.0115f //static friction
        float _voxel_rig_cube_size_z = 0.15f;//0.0015f //kinetic friction


        //1,024‬
        //1,024‬
        //1,024‬
        //1,024‬
        //1,024‬
        //1,024‬

        //PHYSICS CUBES
        int _inst_cube_x = 4;
        int _inst_cube_y = 10;
        int _inst_cube_z = 4; 
        float _cube_size_x = 0.025f; //0.0115f //1.5f
        float _cube_size_y = 0.025f; //0.0115f //1.5f
        float _cube_size_z = 0.025f;
        //END OF

        //PHYSICS GRID
        int _inst_grid_x = 1;
        int _inst_grid_y = 1;
        int _inst_grid_z = 1;
        float _grid_size_x = 10; //0.0115f //1.5f
        float _grid_size_y = 1; //0.0115f //1.5f
        float _grid_size_z = 10;
        //END OF

        //float _voxel_cube_size_x = 0.0515f;
        //float _voxel_cube_size_y = 0.0515f;
        //float _voxel_cube_size_z = 0.0515f;


        //SPECTRUM
        //SPECTRUM
        //SPECTRUM
        const int _inst_spectrum_x = 420; // 36 // 210 //75
        const int _inst_spectrum_y = 1;
        const int _inst_spectrum_z = 210; // 36 // 210 //75 //5625
        float _spectrum_size_x = 0.0015f; //0.001115f
        float _spectrum_size_y = 0.0015f;
        float _spectrum_size_z = 0.0015f;
        byte[] _sound_byte_array = new byte[_inst_spectrum_x * _inst_spectrum_z]; //44100
        byte[] _sound_byte_array_instant = new byte[_inst_spectrum_x * _inst_spectrum_z]; //44100 //176400
        int has_spoken_main = 0;
        int has_spoken_sec = 0;
        int has_spoken_tier = 0;
        int has_spoken_quart = 0;
        string last_xml_filepath = "";
        string last_wave_filepath = "";
        DateTime _time_of_recording_start = DateTime.Now;
        DateTime _time_of_recording_end = DateTime.Now;
        int sc_can_start_rec_counter = 0;
        int sc_can_start_rec_counter_before_add_index = 0;
        int sc_play_file = 0;
        int sc_play_file_counter = 0;
        int sc_save_file = 0;
        int sc_save_file_counter = 0;
        int sc_start_recording = 0;
        int sc_start_recording_counter = 0;
        string short_path = "";
        string instant_short_path = "";
        float spectrum_noise_value = 0;
        SoundPlayer _sound_player = new SoundPlayer();
        Matrix spectrum_mat = Matrix.Identity;
        static XmlTextWriter writer = new XmlTextWriter(Console.Out);
        string path;
        int _records_counter = 0;
        int _records_instant_counter = 0;
        int _frames_to_access_counter = 0;
        int _spectrum_work = 0;
        int _spectrum_work_counter = 0;
        int _has_recorded = 0;

        public static int GetSoundLength(string fileName)
        {
            StringBuilder lengthBuf = new StringBuilder(32);
            mciSendString(string.Format("open \"{0}\" type waveaudio alias wave", fileName), null, 0, IntPtr.Zero);
            mciSendString("status wave length", lengthBuf, lengthBuf.Capacity, IntPtr.Zero);
            mciSendString("close wave", null, 0, IntPtr.Zero);
            int length = 0;
            int.TryParse(lengthBuf.ToString(), out length);
            return length;
        }
        int swtchinstantsound = -1;
        //END OF
        //END OF
        //END OF

        [DllImport("winmm.dll")]
        private static extern long mciSendString(string strCommand, StringBuilder strReturn, int iReturnLength, IntPtr hwndCallback);




        //static cubes 
        int _inst_terrain_tile_x = 1;
        int _inst_terrain_tile_y = 1;
        int _inst_terrain_tile_z = 1;
        float _terrain_tile_size_x = 0.015f;
        float _terrain_tile_size_y = 0.05f;
        float _terrain_tile_size_z = 0.015f;

        //main terrain.
        float _terrain_size_x = 3;
        float _terrain_size_y = 0.095f; //0.02f too small objects go through
        float _terrain_size_z = 3;


        //main terrain.
        float _platform_size_x = 3;
        float _platform_size_y = 2; //0.02f too small objects go through
        float _platform_size_z = 3;


        //main terrain.
        float _floor_size_x = 13;
        float _floor_size_y = 13;
        float _floor_size_z = 13;

        //float _size__neg_x = 1.175494351F - 38;
        //float _size__pos_x = 3.402823466F + 38;
        int _type_of_cube = 3;





        Matrix WorldMatrix = Matrix.Identity;
        Matrix _object_worldmatrix = Matrix.Identity;

        Matrix heightMapWorldMatrix = Matrix.Identity;


        //HUMAN RIG
        int _inst_p_upper_l_leg_x = _human_inst_rig_x;
        int _inst_p_upper_l_leg_y = _human_inst_rig_y;
        int _inst_p_upper_l_leg_z = _human_inst_rig_z;
        int _inst_p_upper_r_leg_x = _human_inst_rig_x;
        int _inst_p_upper_r_leg_y = _human_inst_rig_y;
        int _inst_p_upper_r_leg_z = _human_inst_rig_z;
        int _inst_p_lower_l_leg_x = _human_inst_rig_x;
        int _inst_p_lower_l_leg_y = _human_inst_rig_y;
        int _inst_p_lower_l_leg_z = _human_inst_rig_z;
        int _inst_p_lower_r_leg_x = _human_inst_rig_x;
        int _inst_p_lower_r_leg_y = _human_inst_rig_y;
        int _inst_p_lower_r_leg_z = _human_inst_rig_z;
        int _inst_p_l_foot_x = _human_inst_rig_x;
        int _inst_p_l_foot_y = _human_inst_rig_y;
        int _inst_p_l_foot_z = _human_inst_rig_z;
        int _inst_p_r_foot_x = _human_inst_rig_x;
        int _inst_p_r_foot_y = _human_inst_rig_y;
        int _inst_p_r_foot_z = _human_inst_rig_z;
        int _inst_p_torso_x = _human_inst_rig_x;
        int _inst_p_torso_y = _human_inst_rig_y;
        int _inst_p_torso_z = _human_inst_rig_z;
        int _inst_p_pelvis_x = _human_inst_rig_x;
        int _inst_p_pelvis_y = _human_inst_rig_y;
        int _inst_p_pelvis_z = _human_inst_rig_z;
        int _inst_p_r_hand_x = _human_inst_rig_x;
        int _inst_p_r_hand_y = _human_inst_rig_y;
        int _inst_p_r_hand_z = _human_inst_rig_z;
        int _inst_p_l_hand_x = _human_inst_rig_x;
        int _inst_p_l_hand_y = _human_inst_rig_y;
        int _inst_p_l_hand_z = _human_inst_rig_z;
        int _inst_p_r_shoulder_x = _human_inst_rig_x;
        int _inst_p_r_shoulder_y = _human_inst_rig_y;
        int _inst_p_r_shoulder_z = _human_inst_rig_z;
        int _inst_p_l_shoulder_x = _human_inst_rig_x;
        int _inst_p_l_shoulder_y = _human_inst_rig_y;
        int _inst_p_l_shoulder_z = _human_inst_rig_z;
        int _inst_p_l_upperarm_x = _human_inst_rig_x;
        int _inst_p_l_upperarm_y = _human_inst_rig_y;
        int _inst_p_l_upperarm_z = _human_inst_rig_z;
        int _inst_p_r_upperarm_x = _human_inst_rig_x;
        int _inst_p_r_upperarm_y = _human_inst_rig_y;
        int _inst_p_r_upperarm_z = _human_inst_rig_z;
        int _inst_p_l_lowerarm_x = _human_inst_rig_x;
        int _inst_p_l_lowerarm_y = _human_inst_rig_y;
        int _inst_p_l_lowerarm_z = _human_inst_rig_z;
        int _inst_p_r_lowerarm_x = _human_inst_rig_x;
        int _inst_p_r_lowerarm_y = _human_inst_rig_y;
        int _inst_p_r_lowerarm_z = _human_inst_rig_z;
        int _inst_p_head_x = _human_inst_rig_x;
        int _inst_p_head_y = _human_inst_rig_y;
        int _inst_p_head_z = _human_inst_rig_z;

        

        Matrix[] voxel_sometester_r_hand_grab;
        Matrix[] voxel_sometester_l_hand_grab;
        Matrix[] voxel_sometester_r_upper_leg;
        Matrix[] voxel_sometester_l_upper_leg;
        Matrix[] voxel_sometester_r_lower_leg;
        Matrix[] voxel_sometester_l_lower_leg;
        Matrix[] voxel_sometester_r_foot;
        Matrix[] voxel_sometester_l_foot;
        Matrix[] voxel_sometester_r_hnd;
        Matrix[] voxel_sometester_l_hnd;
        Matrix[] voxel_sometester_l_up_arm;
        Matrix[] voxel_sometester_r_up_arm;
        Matrix[] voxel_sometester_l_low_arm;
        Matrix[] voxel_sometester_r_low_arm;
        Matrix[] voxel_sometester_l_shld;
        Matrix[] voxel_sometester_r_shld;
        Matrix[] voxel_sometester_l_targ;
        Matrix[] voxel_sometester_r_targ;
        Matrix[] voxel_sometester_l_targ_two;
        Matrix[] voxel_sometester_r_targ_two;
        Matrix[] voxel_sometester_pelvis;
        Matrix[] voxel_sometester_torso;
        Matrix[] voxel_sometester_l_targ_knee;
        Matrix[] voxel_sometester_r_targ_knee;
        Matrix[] voxel_sometester_l_targ_two_knee;
        Matrix[] voxel_sometester_r_targ_two_knee;
        
        Vector3[][][] point3DCollection;

        Matrix[][][] worldMatrix_instances_l_hand_grab;
        Matrix[][][] worldMatrix_instances_r_hand_grab;
        Matrix[][][] _screenDirMatrix_correct_pos;
        Matrix[][][] worldMatrix_instances_player_ik;
        Matrix[][][] worldMatrix_instances_voxel_cube;
        Matrix[][][] worldMatrix_instances_spectrum;
        Matrix[][][] worldMatrix_instances_DZgrid;
        Matrix[][][] worldMatrix_instances_floor;
        Matrix[][][] worldMatrix_instances_terrain_tiles;
        Matrix[][][] worldMatrix_instances_terrain;
        Matrix[][][] worldMatrix_instances_screen_assets;
        Matrix[][][] _screenDirMatrix;
        Matrix[][][] worldMatrix_instances_screens;
        Matrix[][][] world_last_Matrix_instances_screens;
        Matrix[][][] worldMatrix_instances_cubes;
        Matrix[][][] worldMatrix_instances_r_elbow_target;
        Matrix[][][] worldMatrix_instances_l_elbow_target;
        Matrix[][][] worldMatrix_instances_r_elbow_target_two;
        Matrix[][][] worldMatrix_instances_l_elbow_target_two;
        Matrix[][][] worldMatrix_instances_r_target_knee;
        Matrix[][][] worldMatrix_instances_l_target_knee;
        Matrix[][][] worldMatrix_instances_r_target_two_knee;
        Matrix[][][] worldMatrix_instances_l_target_two_knee;
        Matrix[][][] worldMatrix_instances_r_upper_leg;
        Matrix[][][] worldMatrix_instances_l_upper_leg;
        Matrix[][][] worldMatrix_instances_r_lower_leg;
        Matrix[][][] worldMatrix_instances_l_lower_leg;
        Matrix[][][] worldMatrix_instances_r_foot;
        Matrix[][][] worldMatrix_instances_l_foot;
        Matrix[][][] worldMatrix_instances_head;
        Matrix[][][] worldMatrix_instances_torso;
        Matrix[][][] worldMatrix_instances_pelvis;
        Matrix[][][] worldMatrix_instances_r_hand;
        Matrix[][][] worldMatrix_instances_l_hand;
        Matrix[][][] worldMatrix_instances_r_shoulder;
        Matrix[][][] worldMatrix_instances_l_shoulder;
        Matrix[][][] worldMatrix_instances_r_upperarm;
        Matrix[][][] worldMatrix_instances_l_upperarm;
        Matrix[][][] worldMatrix_instances_r_lowerarm;
        Matrix[][][] worldMatrix_instances_l_lowerarm;
        Matrix[][][] worldMatrix_instances_grid;
        Matrix[][][] worldMatrix_instances_containment_grid_RH;
        Matrix[][][] worldMatrix_instances_containment_grid_LH;
        Matrix[][][] worldMatrix_instances_containment_grid_screen;
        Matrix[][][] worldMatrix_instances_cone;
        Matrix[][][] worldMatrix_instances_cylinder;
        Matrix[][][] worldMatrix_instances_capsule;
        Matrix[][][] worldMatrix_instances_sphere;
        
        sc_spectrum[][] _world_spectrum_list;
        
        Vector4[][] _array_of_colors;
        Vector3[][][] _array_of_last_frame_voxel_pos;
        Vector3[][][] _array_of_last_frame_cube_pos;
        Vector3[][][] _array_of_last_frame_cone_pos;
        Vector3[][][] _array_of_last_frame_cylinder_pos;
        Vector3[][][] _array_of_last_frame_capsule_pos;
        Vector3[][][] _array_of_last_frame_sphere_pos;
        Matrix[] worldMatrix_base;


        int[][][] _some_frame_counter_grab_right_hand_swtch;
        int[][][] _some_frame_counter_grab_right_hand;
        int[][][] _some_frame_counter_raycast_00;
        int[][][] _some_frame_counter_raycast_01;

        float a = 0.0f;
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        float offsetPosX = 0.0f;
        float offsetPosY = 0.0f;
        float offsetPosZ = 0.0f;

        public static Jitter.Forces.Buoyancy _buo;
        Jitter.Forces.Buoyancy[] _buoyancy_area;
        int has_water_buo_effect = -1;
        bool containsCoord;
        JVector rh_attract_force = JVector.Zero;
        JVector lh_attract_force = JVector.Zero;

        //SC_console_directx D3D;
        //IntPtr HWND;


        Matrix translationMatrix = Matrix.Identity;
        JQuaternion quatterer = new JQuaternion(0, 0, 0, 1);
        Quaternion tester = Quaternion.Identity;
        Matrix rigidbody_matrix = Matrix.Identity;
        IEnumerator enumerator;
        RigidBody body;





        //SC_DRGrid _grid;
        //main terrain.
        //SC_VR_IcoSphere _icoSphere;
        int _icoVertexCount = 0;
        

        const int ChunkWidth_L = 3;
        const int ChunkWidth_R = 2;
        const int ChunkHeight_L = 3;
        const int ChunkHeight_R = 2;
        const int ChunkDepth_L = 3;
        const int ChunkDepth_R = 2;
        float[] arrayX = new float[(ChunkWidth_L + ChunkWidth_R + 1) * (ChunkHeight_L + ChunkHeight_R + 1) * (ChunkDepth_L + ChunkDepth_R + 1)];
        float[] arrayY = new float[(ChunkWidth_L + ChunkWidth_R + 1) * (ChunkHeight_L + ChunkHeight_R + 1) * (ChunkDepth_L + ChunkDepth_R + 1)];
        float[] arrayZ = new float[(ChunkWidth_L + ChunkWidth_R + 1) * (ChunkHeight_L + ChunkHeight_R + 1) * (ChunkDepth_L + ChunkDepth_R + 1)];
        int[] draw_dcontainmentgrid = new int[(ChunkWidth_L + ChunkWidth_R + 1) * (ChunkHeight_L + ChunkHeight_R + 1) * (ChunkDepth_L + ChunkDepth_R + 1)];
        public static int _vertexCount = 8;
        

        
        sccsVD4VE_LightNWithoutVr.SC_Graphics.sc_spectrum.DLightBuffer[] _DLightBuffer_spectrum = new sc_spectrum.DLightBuffer[1];
        
        int _start_background_worker_00 = 0;
        int _start_background_worker_01 = 0;
        

        public sc_graphics_sec() //SC_console_directx _SC_console_directx, IntPtr _HWND
        {
            //D3D = _SC_console_directx;
            //HWND = _HWND;

            _screenDirMatrix = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _screenDirMatrix_correct_pos = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            point3DCollection = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
           
            worldMatrix_instances_screens = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            world_last_Matrix_instances_screens = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            worldMatrix_instances_screen_assets = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];

            



            worldMatrix_instances_cubes = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            worldMatrix_instances_voxel_cube = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            worldMatrix_instances_cone = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            worldMatrix_instances_cylinder = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            worldMatrix_instances_capsule = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            worldMatrix_instances_sphere = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];




            worldMatrix_instances_terrain_tiles = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _array_of_colors = new Vector4[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][];
            _array_of_last_frame_cube_pos = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _array_of_last_frame_voxel_pos = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _array_of_last_frame_cone_pos = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _array_of_last_frame_cylinder_pos = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _array_of_last_frame_capsule_pos = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _array_of_last_frame_sphere_pos = new Vector3[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];



            //SINGLE OBJECTS
            //SINGLE OBJECTS
            //SINGLE OBJECTS

            worldMatrix_instances_terrain = new Matrix[1][][];

            _world_spectrum_list = new sc_spectrum[1][];
            worldMatrix_instances_spectrum = new Matrix[1][][]; 

            worldMatrix_instances_DZgrid = new Matrix[1][][];
            worldMatrix_instances_grid = new Matrix[1][][];
            worldMatrix_instances_containment_grid_RH = new Matrix[1][][];
            worldMatrix_instances_containment_grid_LH = new Matrix[1][][];
            worldMatrix_instances_containment_grid_screen = new Matrix[1][][];
            //SINGLE OBJECTS
            //SINGLE OBJECTS
            //SINGLE OBJECTS





            //HUMAN RIG STUFF
            var tempMultiInstancePhysicsTotal = 1;


            worldMatrix_instances_l_hand_grab = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_hand_grab = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_upper_leg = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_upper_leg = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_lower_leg = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_lower_leg = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_foot = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_foot = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_target_knee = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_target_knee = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_target_two_knee = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_target_two_knee = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_elbow_target = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_elbow_target = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_elbow_target_two = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_elbow_target_two = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_head = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_torso = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_pelvis = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_hand = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_hand = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_shoulder = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_shoulder = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_upperarm = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_upperarm = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_lowerarm = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_lowerarm = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_r_foot = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_l_foot = new Matrix[tempMultiInstancePhysicsTotal][][];
            worldMatrix_instances_floor = new Matrix[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];



            _some_frame_counter_grab_right_hand_swtch = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _some_frame_counter_grab_right_hand = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _some_frame_counter_raycast_00 = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            _some_frame_counter_raycast_01 = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];


            _some_frame_counter_grab_right_hand_swtch[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
            _some_frame_counter_grab_right_hand[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
            _some_frame_counter_raycast_00[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
            _some_frame_counter_raycast_01[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];


            _some_frame_counter_grab_right_hand_swtch[0][0] = new int[_human_inst_rig_x * _human_inst_rig_y * _human_inst_rig_z];
            _some_frame_counter_grab_right_hand[0][0] = new int[_human_inst_rig_x * _human_inst_rig_y * _human_inst_rig_z];
            //_some_frame_counter_raycast_00[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
            //_some_frame_counter_raycast_01[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];



            swtch_for_last_pos = new int[MainWindow._physics_engine_instance_x * MainWindow._physics_engine_instance_y * MainWindow._physics_engine_instance_z][][];
            swtch_for_last_pos[0] = new int[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
            swtch_for_last_pos[0][0] = new int[_human_inst_rig_x * _human_inst_rig_y * _human_inst_rig_z];
            swtch_for_last_pos[0][0][0] = 0;

            for (int i = 0; i < _some_frame_counter_grab_right_hand[0][0].Length; i++)
            {
                _some_frame_counter_grab_right_hand[0][0][i] = 0;
                _some_frame_counter_grab_right_hand_swtch[0][0][i] = 0;
            }


            worldMatrix_base = new Matrix[1];
            worldMatrix_base[0] = Matrix.Identity;

            DoWork(0);
        }

        public SC_message_object_jitter[][] _sc_create_world_objects(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            try
            {
                _buoyancy_area = new Jitter.Forces.Buoyancy[1];

                //draw_dcontainmentgrid = new int[6 * 6 * 6];

                for (int i = 0; i < draw_dcontainmentgrid.Length; i++)
                {
                    draw_dcontainmentgrid[i] = 0;
                }

                for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
                {
                    for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                    {
                        for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                        {
                            var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);
                            Vector3 physics_engine_offset_pos = new Vector3(xx * 2, yy * 2, zz * 2);

                            //World[] _jitter_worlds = (World[])_sc_jitter_tasks[indexer00]._world_data;
                            _array_of_last_frame_voxel_pos[indexer00] = new Vector3[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            _array_of_last_frame_cube_pos[indexer00] = new Vector3[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            _array_of_last_frame_cone_pos[indexer00] = new Vector3[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            _array_of_last_frame_cylinder_pos[indexer00] = new Vector3[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            _array_of_last_frame_capsule_pos[indexer00] = new Vector3[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            _array_of_last_frame_sphere_pos[indexer00] = new Vector3[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            
                            worldMatrix_instances_cubes[indexer00] = new Matrix[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            worldMatrix_instances_cone[indexer00] = new Matrix[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            worldMatrix_instances_cylinder[indexer00] = new Matrix[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            worldMatrix_instances_capsule[indexer00] = new Matrix[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];
                            worldMatrix_instances_sphere[indexer00] = new Matrix[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];

                            worldMatrix_instances_voxel_cube[indexer00] = new Matrix[MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth][];

                            float offsetCubeY = 0;
                            float offsetVoxelY = 0;
                            float offsetCapsuleY = 0;
                            float offsetConeY = 0;
                            float offsetCylinderY = 0;
                            float offsetSphereY = 0;

                            try
                            {
                                for (int x = 0; x < MainWindow.world_width; x++)
                                {
                                    for (int y = 0; y < MainWindow.world_height; y++)
                                    {
                                        for (int z = 0; z < MainWindow.world_depth; z++)
                                        {
                                            var indexer01 = x + MainWindow.world_width * (y + MainWindow.world_height * z);

                                            Vector3 world_pos_offset = new Vector3(x * 2, y * 2, z * 2);

                                            object _some_data_00 = (object)_sc_jitter_tasks[indexer00][indexer01]._world_data[0];
                                            //World _jitter_worlds = (World)_some_data_00;
                                            World _jitter_world = (World)_some_data_00;//= _jitter_worlds[0];



                                            
                                        }
                                    }
                                }
                                //END OF LOOP FOR WORLDS
                            }
                            catch
                            {

                            }
                        }
                    }
                }

                //SETTING UP SINGLE WORLD OBJECTS
                //END OF LOOP FOR PHYSICS ENGINE INSTANCES
                object _some_data_0 = (object)_sc_jitter_tasks[0][0]._world_data[0];
                //World[] _jitter_worlds0 = (World[])_some_data_0;
                World _thejitter_world = (World)_some_data_0;

                r = 0.10f;
                g = 0.10f;
                b = 0.10f;
                a = 1.0f;
                
            
                //////////SPECTRUM//////////
                ////////////////////////////
                r = 0.10f;
                g = 0.10f;
                b = 0.10f;
                a = 1.0f;
                /*
                _object_worldmatrix = Matrix.Identity;

                _object_worldmatrix = WorldMatrix;

                _object_worldmatrix.M41 = 0;
                _object_worldmatrix.M42 = 0;
                _object_worldmatrix.M43 = 0;
                _object_worldmatrix.M44 = 1;

                offsetPosX = 0;
                offsetPosY = 0;
                offsetPosZ = 0;*/
                _object_worldmatrix = Matrix.Identity;
                offsetPosX = _spectrum_size_x * 1.15f; //x between each world instance
                offsetPosY = _spectrum_size_y * 1.15f; //y between each world instance
                offsetPosZ = _spectrum_size_z * 1.15f; //z between each world instance
                _object_worldmatrix = WorldMatrix;
                _object_worldmatrix.M41 = 0;// + 0 + physics_engine_offset_pos.X + world_pos_offset.X;
                _object_worldmatrix.M42 = 0.5f;// + 0 + physics_engine_offset_pos.Y + world_pos_offset.Y + offsetCubeY;
                _object_worldmatrix.M43 = 0;// + 0 + physics_engine_offset_pos.Z + world_pos_offset.Z;
                _object_worldmatrix.M44 = 1;


                _world_spectrum_list[0] = new sc_spectrum[1];
                _world_spectrum_list[0][0] = new sc_spectrum();
                _world_spectrum_list[0][0].Initialize(SC_console_directx.D3D, SC_console_directx.D3D.SurfaceWidth, SC_console_directx.D3D.SurfaceHeight, 0.05f, 1, 1, _spectrum_size_x, _spectrum_size_y, _spectrum_size_z, new Vector4(r, g, b, a), _inst_spectrum_x, _inst_spectrum_y, _inst_spectrum_z, SC_Update.HWND, _object_worldmatrix, 2, offsetPosX, offsetPosY, offsetPosZ, _thejitter_world, sccsVD4VE_LightNWithoutVr.sc_console.SC_console_directx.BodyTag._spectrum, true, 0, 10, 0, 0, 0); //, "terrainGrassDirt.bmp" //0.00035f

                worldMatrix_instances_spectrum[0] = new Matrix[1][]; //MainWindow.world_width * MainWindow.world_height * MainWindow.world_depth
                worldMatrix_instances_spectrum[0][0] = new Matrix[_inst_spectrum_x * _inst_spectrum_y * _inst_spectrum_z]; //_inst_terrain_tile_x * _inst_terrain_tile_y * _inst_terrain_tile_z

                for (int i = 0; i < worldMatrix_instances_spectrum[0][0].Length; i++)
                {
                    worldMatrix_instances_spectrum[0][0][i] = _world_spectrum_list[0][0]._arrayOfInstances[i]._POSITION;
                }
                _world_spectrum_list[0][0]._WORLDMATRIXINSTANCES = worldMatrix_instances_spectrum[0][0];

                ////////////////////////////
                //////////SPECTRUM//////////
                ////////////////////////////

                

                _DLightBuffer_spectrum[0] = new sccsVD4VE_LightNWithoutVr.SC_Graphics.sc_spectrum.DLightBuffer()
                {
                    ambientColor = ambientColor,
                    diffuseColor = diffuseColour,
                    lightDirection = dirLight,
                    padding0 = 0,
                    lightPosition = lightpos,
                    padding1 = 100
                };
                







                
            }
            catch (Exception ex)
            {
                MainWindow.MessageBox((IntPtr)0, "TESt" + ex.ToString(), "sc core systems message", 0);
            }

            return _sc_jitter_tasks;
        }

        bool _set_fluid_point(ref JVector test)
        {
            //test = new JVector(5, 5, 5);
            test = new JVector(0, 0, 0);
            return _buoyancy_area[0].FluidBox.Contains(ref test) != JBBox.ContainmentType.Disjoint;
        }
























        RigidBody grabBody;
        JVector hitNormal;

        public SC_message_object_jitter[][] loop_worlds(SC_message_object_jitter[][] _sc_jitter_tasks, Matrix originRoter, Matrix rotatingMatrixer, Matrix hmdmatrixRoter, Matrix hmd_matrixer, Matrix rotatingMatrixForPelviser, Matrix _rightTouchMatrixer, Matrix _leftTouchMatrixer)
        {
            /*
            if (buttonPressedOculusTouchRight != 0)
            {
                if (buttonPressedOculusTouchRight == 4)
                {
                    if (gravity_swtch_counter >= 75)
                    {

                        for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
                        {
                            for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                            {
                                for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                                {
                                    var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);

                                    try
                                    {
                                        for (int x = 0; x < MainWindow.world_width; x++)
                                        {
                                            for (int y = 0; y < MainWindow.world_height; y++)
                                            {
                                                for (int z = 0; z < MainWindow.world_depth; z++)
                                                {
                                                    var indexer01 = x + MainWindow.world_width * (y + MainWindow.world_height * z);

                                                    object _some_data_00 = (object)_sc_jitter_tasks[indexer00][indexer01]._world_data[0];
                                                    World _jitter_world = (World)_some_data_00;
                                                    if (_jitter_world != null)
                                                    {
                                                        if (_jitter_world.RigidBodies.Count > 0)
                                                        {

                                                            if (gravity_swtch == 0 || gravity_swtch == 2)
                                                            {
                                                                _jitter_world.Gravity = new JVector(0, 0, 0);
                                                            }
                                                            else if (gravity_swtch == 1)
                                                            {
                                                                _jitter_world.Gravity = new JVector(0, -9.81f, 0);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    catch (Exception ex)
                                    {

                                    }
                                }
                            }
                        }
                        if (gravity_swtch == 0 || gravity_swtch == 2)
                        {
                            gravity_swtch_counter = 0;
                            gravity_swtch = 1;
                        }
                        else if (gravity_swtch == 1)
                        {
                            gravity_swtch_counter = 0;
                            gravity_swtch = 2;
                        }

                    }
                }
            }*/














            for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
            {
                for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                {
                    for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                    {
                        var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);

                        try
                        {
                            for (int x = 0; x < MainWindow.world_width; x++)
                            {
                                for (int y = 0; y < MainWindow.world_height; y++)
                                {
                                    for (int z = 0; z < MainWindow.world_depth; z++)
                                    {
                                        var indexer01 = x + MainWindow.world_width * (y + MainWindow.world_height * z);



                                        object _some_data_00 = (object)_sc_jitter_tasks[indexer00][indexer01]._world_data[0];
                                        World _jitter_world = (World)_some_data_00;
                                        if (_jitter_world != null)
                                        {
                                            if (_jitter_world.RigidBodies.Count > 0)
                                            {
                                                /*
                                                if (buttonPressedOculusTouchRight != 0)
                                                {
                                                    if (buttonPressedOculusTouchRight == 4)
                                                    {
                                                        if (gravity_swtch_counter >= 75)
                                                        {
                                                            if (gravity_swtch == 0 || gravity_swtch == 2)
                                                            {
                                                                _jitter_world.Gravity = new JVector(0, 0, 0);
                                                                gravity_swtch_counter = 0;
                                                                gravity_swtch = 1;
                                                            }
                                                            else if (gravity_swtch == 1)
                                                            {
                                                                _jitter_world.Gravity = new JVector(0, -9.81f, 0);
                                                                gravity_swtch_counter = 0;
                                                                gravity_swtch = 2;
                                                            }
                                                        }
                                                    }
                                                }*/


                                                _inactive_counter_cubes = 0;
                                                _inactive_counter_voxels = 0;



                                                int _terrain_count = 0;
                                                int _floor_count = 0;
                                                int _voxel_cube_counter = 0;
                                                int _non_voxel_cube_counter = 0;
                                                int _non_voxel_cone_counter = 0;
                                                int _non_voxel_cylinder_counter = 0;
                                                int _non_voxel_capsule_counter = 0;
                                                int _non_voxel_sphere_counter = 0;


                                                int clothCounter = 0;



                                                int p_l_shldr_count = 0;
                                                int p_r_shldr_count = 0;


                                                int p_r_hnd_count = 0;
                                                int p_l_hnd_count = 0;
                                                int p_l_lowerA_count = 0;
                                                int p_r_lowerA_count = 0;
                                                int p_l_upperA_count = 0;
                                                int p_r_upperA_count = 0;
                                                int p_l_target_count = 0;
                                                int p_r_target_count = 0;
                                                int p_l_target_two_count = 0;
                                                int p_r_target_two_count = 0;





                                                int p_r_foot_count = 0;
                                                int p_l_foot_count = 0;
                                                int p_l_lowerL_count = 0;
                                                int p_r_lowerL_count = 0;
                                                int p_l_upperL_count = 0;
                                                int p_r_upperL_count = 0;
                                                int p_l_target_knee_count = 0;
                                                int p_r_target_knee_count = 0;
                                                int p_l_target_knee_two_count = 0;
                                                int p_r_target_knee_two_count = 0;










                                                int p_torso_count = 0;
                                                int p_pelvis_count = 0;
                                                int p_head_count = 0;

                                                int _screen_asset_counter = 0;
                                                int _screen_counter = 0;

                                                has_water_buo_effect = -1;

                                                enumerator = _jitter_world.RigidBodies.GetEnumerator();

                                                while (enumerator.MoveNext())
                                                {
                                                    body = (RigidBody)enumerator.Current;

                                                    if (body != null && body.Tag != null) //&& body != _grab_rigid_data._body
                                                    {
                                                        

                                                    }
                                                }
                                            }
                                            //Console.Title = MainWindow._MainWindow_name + " ### " + " Made by Steve Chassé" + " ### " + " => " + "disabled cubes: " + _inactive_counter_cubes + " disabled voxels: " + _inactive_counter_voxels;
                                            Console.Title = MainWindow._MainWindow_name + " ### " + " Made by ninekorn" + " ### " + " => " + "disabled cubes: " + _inactive_counter_cubes + " disabled voxels: " + _inactive_counter_voxels;

                                        }
                                    }
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine("test0");
                        }
                    }
                }
            }
            gravity_swtch_counter++;
            _some_frame_counter_grab_right_hand[0][0][0]++;
            //END OF



            /*Matrix tempmat = _icoSphere._TEMPPOSITION; //_player_rght_hnd[0]._arrayOfInstances[0].current_pos
            Quaternion otherQuat;
            Quaternion.RotationMatrix(ref tempmat, out otherQuat);
            dirLight = sc_maths._getDirection(Vector3.ForwardRH, otherQuat);
            //Vector3 dirLight = new Vector3(0, -1, 0); //lightDirection;// new Vector3(0, -1, 0);
            //direction_feet_forward = _getDirection(Vector3.ForwardRH, otherQuat);
            //direction_feet_right = _getDirection(Vector3.Right, otherQuat);
            //direction_feet_up = _getDirection(Vector3.Up, otherQuat);
            lightpos = new Vector3(0, _icoSphere._TEMPPOSITION.M42, 0);
            */



            return _sc_jitter_tasks;
        }















        Matrix finalRotationMatrix = Matrix.Identity;







        PointPointDistance _distanceConstraintRight;
        PointPointDistance _distanceConstraintLeft;
        RigidBody _lastRigidGrab;
        float _lastFraction = 0;
        JVector _lastHitPoint;
        //bool _hasGrabbed = false;

        private bool RaycastCallback(RigidBody body, JVector normal, float fraction)
        {
            if (body.IsStatic) return false;
            else return true;
        }



        Matrix hmdmatrixRot_;
        Matrix OriginRot = Matrix.Identity;
        Matrix RotatingMatrix = Matrix.Identity;
        Matrix RotatingMatrixForPelvis = Matrix.Identity;
        Matrix viewMatrix_ = Matrix.Identity;
        public SC_message_object_jitter[][] workOnSomething(SC_message_object_jitter[][] _sc_jitter_tasks, Matrix viewMatrix, Matrix projectionMatrix, Vector3 OFFSETPOS, Matrix originRot, Matrix rotatingMatrix, Matrix hmdrotMatrix, Matrix hmd_matrix, Matrix rotatingMatrixForPelvis, Matrix _rightTouchMatrix, Matrix _leftTouchMatrix, Vector3 handPoseRight, Vector3 handPoseLeft)
        {
            viewMatrix_ = viewMatrix;



            /*
            if (_some_frame_counter_grab_right_hand[0][0][0] > 1)
            {
                Vector3 current_handposR = new Vector3(_player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M41, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M42, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M43);
                var centerPosRighthandposR = new Vector3(final_hand_pos_right_locked.M41, final_hand_pos_right_locked.M42, final_hand_pos_right_locked.M43);
                Quaternion.RotationMatrix(ref final_hand_pos_right_locked, out _rightTouchQuat);
                var rayDirFront = sc_maths._getDirection(Vector3.ForwardRH, _rightTouchQuat);
                someRay = new Ray(centerPosRighthandposR, rayDirFront);


                if ((sccsVD4VE_LightNWithoutVr.sc_console.SC_console_directx.BodyTag)body.Tag == sccsVD4VE_LightNWithoutVr.sc_console.SC_console_directx.BodyTag.physicsInstancedScreen)
                {
                    Quaternion _quat_screen000;
                    Matrix mater = _world_screen_list[0][0]._arrayOfInstances[0].current_pos;
                    Quaternion.RotationMatrix(ref mater, out _quat_screen000);
                    var screenNormal = sc_maths._getDirection(Vector3.ForwardRH, _quat_screen000);
                    screenNormal.Normalize();

                    var planer = new Plane(new Vector3(mater.M41, mater.M42, mater.M43), screenNormal);
                    intersecter = someRay.Intersects(ref planer, out intersectPointRight);

                    var handToScreenNormalDistance = sc_maths.sc_check_distance_node_3d(current_handposR, intersectPointRight, 2, 2, 2, 2, 2, 2, 2, 2, 2);

                    if (handToScreenNormalDistance < 0.25f && intersectPointRight != Vector3.Zero)
                    {
                        MainWindow.MessageBox((IntPtr)0, "" + "resulter", "sc core systems Error", 0);


                        if (buttonPressedOculusTouchRight == 1) // grabObject
                        {
                            Quaternion.RotationMatrix(ref WorldMatrix, out quaterr);

                            var centerPosRight = new SharpDX.Vector3(current_handposR.X, current_handposR.Y, current_handposR.Z);    //Point3D                                                                                                                            //var rayoriginLeft = centerPosLeft;
                            //var rayDirForward = sc_maths._getDirection(SharpDX.Vector3.ForwardRH, quaterr);
                            var _ray = new SharpDX.Ray(centerPosRight, rayDirFront);


                            JVector ray = new JVector(_ray.Direction.X, _ray.Direction.Y, _ray.Direction.Z);
                            //JVector camp = Conversion.ToJitterVector(Camera.Position);
                            ray = JVector.Normalize(ray) * 0.25f;


                            var camp = new JVector(centerPosRight.X, centerPosRight.Y, centerPosRight.Z);

                            bool resulter = _jitter_world.CollisionSystem.Raycast(camp, ray, RaycastCallback, out grabBody, out hitNormal, out fraction);

                            if (resulter)
                            {
                                MainWindow.MessageBox((IntPtr)0, "" + "resulter", "sc core systems Error", 0);
                                var hitPoint = camp + fraction * ray;

                                if (_distanceConstraintRight != null)
                                {
                                    _jitter_world.RemoveConstraint(_distanceConstraintRight);7
                                }


                                JVector lanchor = new JVector(current_handposR.X, current_handposR.Y, current_handposR.Z) - grabBody.Position; //hitPoint
                                lanchor = JVector.Transform(lanchor, JMatrix.Transpose(grabBody.Orientation));

                                _distanceConstraintRight = new PointPointDistance(_player_rght_hnd[0][0]._arrayOfInstances[0].transform.Component.rigidbody, grabBody, camp, hitPoint);

                                _distanceConstraintRight.Softness = 0.0001f;
                                _distanceConstraintRight.BiasFactor = 0.1f;
                                _distanceConstraintRight.Distance = 0.001f;

                                _jitter_world.AddConstraint(_distanceConstraintRight);

                                _lastFraction = fraction;
                                _lastRigidGrab = grabBody;
                                _some_frame_counter_grab_right_hand[0][0][0] = 0;
                                _some_frame_counter_grab_right_hand_swtch[0][0][0] = 1;
                            }
                            else
                            {

                            }
                        }
                    }
                    else
                    {

                    }
                }
                else
                {
                   bool _boundingBoxer = _jitter_world.CollisionSystem.CheckBoundingBoxes(body, _player_rght_hnd[0][0]._arrayOfInstances[0].transform.Component.rigidbody);

                    if (_boundingBoxer)
                    {
                        if (_distanceConstraintRight != null)
                        {
                            _jitter_world.RemoveConstraint(_distanceConstraintRight);
                        }

                        var centerPosRight = new JVector(current_handposR.X, current_handposR.Y, current_handposR.Z);
                        JVector lanchor = new JVector(current_handposR.X, current_handposR.Y, current_handposR.Z) - grabBody.Position; //hitPoint
                        lanchor = JVector.Transform(lanchor, JMatrix.Transpose(grabBody.Orientation));

                        _distanceConstraintRight = new PointPointDistance(_player_rght_hnd[0][0]._arrayOfInstances[0].transform.Component.rigidbody, grabBody, centerPosRight, lanchor);

                        _distanceConstraintRight.Softness = 0.0001f;
                        _distanceConstraintRight.BiasFactor = 0.1f;
                        _distanceConstraintRight.Distance = 0.001f;

                        _jitter_world.AddConstraint(_distanceConstraintRight);

                        _lastFraction = fraction;
                        _lastRigidGrab = grabBody;
                        _some_frame_counter_grab_right_hand[0][0][0] = 0;
                        _some_frame_counter_grab_right_hand_swtch[0][0][0] = 1;

                    }
                    else
                    {

                    }
                }

            }

            if (_some_frame_counter_grab_right_hand_swtch[0][0][0] == 0) //keep counting up when switch is at 0
            {
                _some_frame_counter_grab_right_hand[0][0][0]++;
            }
            else
            {
                if (buttonPressedOculusTouchRight == 2) // release grabbed object
                {
                    if (_distanceConstraintRight != null)
                    {
                        _jitter_world.RemoveConstraint(_distanceConstraintRight);
                    }
                    grabBody = null;
                    _lastRigidGrab = null;
                    _some_frame_counter_grab_right_hand_swtch[0][0][0] = 0;
                }
            }*/




            /*Vector3 current_handposR = new Vector3(_player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M41, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M42, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M43);
            var centerPosRighthandposR = new Vector3(final_hand_pos_right_locked.M41, final_hand_pos_right_locked.M42, final_hand_pos_right_locked.M43);
            Quaternion.RotationMatrix(ref final_hand_pos_right_locked, out _rightTouchQuat);
            var rayDirFront = sc_maths._getDirection(Vector3.ForwardRH, _rightTouchQuat);
            someRay = new Ray(centerPosRighthandposR, rayDirFront);

            Quaternion _quat_screen000;
            Matrix mater = worldMatrix_instances_screens[0][0][0];
            Quaternion.RotationMatrix(ref mater, out _quat_screen000);
            var screenNormal = sc_maths._getDirection(Vector3.ForwardRH, _quat_screen000);
            screenNormal.Normalize();

            var planer = new Plane(new Vector3(mater.M41, mater.M42, mater.M43), screenNormal);
            intersecter = someRay.Intersects(ref planer, out intersectPointRight);


            var handToScreenNormalDistance = sc_maths.sc_check_distance_node_3d(current_handposR, intersectPointRight, 2, 2, 2, 2, 2, 2, 2, 2, 2);


            if (handToScreenNormalDistance < 0.1f && intersectPointRight != Vector3.Zero)
            {

            }*/





            /*Vector3 current_handposR = new Vector3(_player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M41, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M42, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M43);

            Quaternion quater;
            Quaternion.RotationMatrix(ref WorldMatrix, out quater);

            var centerPosRight = new SharpDX.Vector3(current_handposR.X, current_handposR.Y, current_handposR.Z);    //Point3D                                                                                                                            //var rayoriginLeft = centerPosLeft;
            var rayDirForward = sc_maths._getDirection(SharpDX.Vector3.ForwardRH, quater);
            var _ray = new SharpDX.Ray(centerPosRight, rayDirForward);

            float fraction;

            JVector ray = new JVector(_ray.Direction.X, _ray.Direction.Y, _ray.Direction.Z);
            //JVector camp = Conversion.ToJitterVector(Camera.Position);
            ray = JVector.Normalize(ray) * 0.25f;

            RigidBody grabBody;
            JVector hitNormal;
            var camp = new JVector(centerPosRight.X, centerPosRight.Y, centerPosRight.Z);

            bool resulter = _jitter_world.CollisionSystem.Raycast(camp, ray, RaycastCallback, out grabBody, out hitNormal, out fraction);

            if (buttonPressedOculusTouchRight == 1)
            {
                if (resulter)
                {
                    var hitPoint = camp + fraction * ray;

                    if (_distanceConstraintRight != null)
                    {
                        _jitter_world.RemoveConstraint(_distanceConstraintRight);
                    }


                    JVector lanchor = new JVector(current_handposR.X, current_handposR.Y, current_handposR.Z) - grabBody.Position; //hitPoint
                    lanchor = JVector.Transform(lanchor, JMatrix.Transpose(grabBody.Orientation));

                    _distanceConstraintRight = new PointPointDistance(_player_rght_hnd[0][0]._arrayOfInstances[0].transform.Component.rigidbody, grabBody, camp, hitPoint);

                    _distanceConstraintRight.Softness = 0.0001f;
                    _distanceConstraintRight.BiasFactor = 0.1f;
                    _distanceConstraintRight.Distance = 0.001f;

                    _jitter_world.AddConstraint(_distanceConstraintRight);

                    _lastFraction = fraction;
                    _lastRigidGrab = grabBody;
                    _hasGrabbed = true;
                }
            }

            if (_hasGrabbed)
            {

            }

            if (buttonPressedOculusTouchRight == 2)
            {
                if (_distanceConstraintRight != null)
                {
                    _jitter_world.RemoveConstraint(_distanceConstraintRight);
                }
                grabBody = null;
                _lastRigidGrab = null;
                _hasGrabbed = false;
            }*/















            float timeSinceStart = (float)(DateTime.Now - SC_Update.startTime).TotalSeconds;
            Matrix worldmatlightrot = Matrix.Scaling(1.0f) * Matrix.RotationX(timeSinceStart * disco_sphere_rot_speed) * Matrix.RotationY(timeSinceStart * 2 * disco_sphere_rot_speed) * Matrix.RotationZ(timeSinceStart * 3 * disco_sphere_rot_speed);

            Quaternion worldmatlightquat;
            SharpDX.Quaternion.RotationMatrix(ref worldmatlightrot, out worldmatlightquat);
            Vector3 dirLight = sc_maths._getDirection(Vector3.ForwardRH, worldmatlightquat);





            lightpos = new Vector3(0, 10, 0);
            ambientColor = new Vector4(0.45f, 0.45f, 0.45f, 1.0f);
            diffuseColour = new Vector4(1, 1, 1, 1);
            lightDirection = new Vector3(0, -1, -1);
            lightpos = new Vector3(0, 10, 0);
            
            _DLightBuffer_spectrum[0].lightPosition = lightpos;
            _DLightBuffer_spectrum[0].lightDirection = dirLight;
            hmdmatrixRot_ = hmdrotMatrix;

            OriginRot = originRot;
            RotatingMatrix = rotatingMatrix;
            RotatingMatrixForPelvis = rotatingMatrixForPelvis;
            

            //SPECTRUM SINGLEOBJECT
            _world_spectrum_list[0][0].Render(SC_console_directx.D3D.device.ImmediateContext);
            SC_Update._shaderManager.RenderInstancedObjectSpectrum(SC_console_directx.D3D.device.ImmediateContext, _world_spectrum_list[0][0].IndexCount, _world_spectrum_list[0][0].InstanceCount, _world_spectrum_list[0][0]._POSITION, viewMatrix, projectionMatrix, _basicTexture.TextureResource, _DLightBuffer_spectrum, _world_spectrum_list[0][0]);
            //END OF
            

            /*float timeSinceStart = (float)(DateTime.Now - SC_Update.startTime).TotalSeconds;
            Matrix world = Matrix.Scaling(1.0f) * Matrix.RotationX(timeSinceStart * disco_sphere_rot_speed) * Matrix.RotationY(timeSinceStart * 2 * disco_sphere_rot_speed) * Matrix.RotationZ(timeSinceStart * 3 * disco_sphere_rot_speed);
            _icoSphere._TEMPPOSITION = world;// _icoSphere._TEMPPOSITION;
            _icoSphere._TEMPPOSITION.M42 = 5;
            _icoSphere.Render(SC_console_directx.D3D.device.ImmediateContext);
            SC_Update._shaderManager.RenderIcoShader(SC_console_directx.D3D.device.ImmediateContext, _icoSphere.IndexCount, _icoSphere._TEMPPOSITION, viewMatrix, projectionMatrix, _icoVertexCount, 1);
            */

















            /*// Turn off the Z buffer to begin all 2D rendering.
            SC_console_directx.D3D.TurnZBufferOff();

            // Turn on the alpha blending before rendering the text.
            SC_console_directx.D3D.TurnOnAlphaBlending();

            // Render the text user interface elements.
            //if (!Text.Render(D3D.DeviceContext, FontShader, worldMatrix, orthoD3DMatrix))
            //    return false;

            // Turn off alpha blending after rendering the text.
            SC_console_directx.D3D.TurnOffAlphaBlending();

            // Turn the Z buffer back on now that all 2D rendering has completed.
            SC_console_directx.D3D.TurnZBufferOn();*/






















            

            for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
            {
                for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                {
                    for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                    {
                        var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);

                        try
                        {
                            for (int x = 0; x < MainWindow.world_width; x++)
                            {
                                for (int y = 0; y < MainWindow.world_height; y++)
                                {
                                    for (int z = 0; z < MainWindow.world_depth; z++)
                                    {
                                        var indexer01 = x + MainWindow.world_width * (y + MainWindow.world_height * z);
                                        
                                        try
                                        {
                                            //PHYSICS VOXEL CUBES 
                                            //////////////////////about 100 ticks more per loop compared to simple physics cubes? will investigate later as when i do 
                                            //////////////////////simple cubes with the chunk it lags more even though the number of vertices are the same as the physics cube up above
                                            //////////////////////todo: culling of faces by distance from player. etc.
                                            
                                            ///Console.WriteLine(_SystemTickPerformance.ElapsedTicks);
                                        }
                                        catch (Exception ex)
                                        {
                                            MainWindow.MessageBox((IntPtr)0, ex.ToString() + "", "Oculus error", 0);
                                        }
                                    }
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.ToString());
                        }
                    }
                }
            }









            finalRotationMatrix = originRot * rotatingMatrix * rotatingMatrixForPelvis * hmdrotMatrix;
            


            //SPECTRUM AND SOUND RECORDING

            if (has_spoken_sec == 1)
            {

                //MainWindow.MessageBox((IntPtr)0, "" + "0", "sccsVD4VE_LightNWithoutVr", 0);
                //_sound_byte_array.OrderBy(x => String.Join(String.Empty, x));
                //_sound_byte_array.OrderBy(x > x).FirstOrDefault();
                //_sound_byte_array = _sound_byte_array.ToList().OrderBy(x => x).ToArray(); // new List<byte>();
                //_sound_byte_array = list.OrderBy(x => x).ToArray();
                //var test = _sound_byte_array.OrderBy(x => x, new ByteListComparer());

                int count_spec = 0;

                var widthL = (int)(_inst_spectrum_x / 2);
                var widthR = (int)((_inst_spectrum_x / 2) - 1);

                var heightL = _inst_spectrum_y;
                var heightR = _inst_spectrum_y;

                var depthL = (int)(_inst_spectrum_z / 2);
                var depthR = (int)((_inst_spectrum_z / 2) - 1);

                //var depthL = _inst_spectrum_z;// (int)(_inst_spectrum_z / 2);
                //var depthR = _inst_spectrum_z;// (int)((_inst_spectrum_z / 2) - 1);

                // wL5 and <=wR4  = -5 to 5 ... 210/2 = 105 and 105 so 105-1 = 104 for wL105 and <=wR104   

                for (int x = -widthL; x <= widthR; x++)
                {
                    for (int y = -heightL; y <= heightR; y++)
                    {
                        for (int z = -depthL; z <= depthR; z++)
                        {
                            //MainWindow.MessageBox((IntPtr)0, "" + "0", "sccsVD4VE_LightNWithoutVr", 0);

                            //the higher the closer to center middle.

                            float posX = (x);
                            float posY = (y);
                            float posZ = (z);

                            var xx = x;
                            var yy = y;
                            var zz = z;

                            if (xx < 0)
                            {
                                xx *= -1;
                                xx = widthR + xx;
                            }
                            if (yy < 0)
                            {
                                yy *= -1;
                                yy = heightR + yy;
                            }
                            if (zz < 0)
                            {
                                zz *= -1;
                                zz = depthR + zz;
                            }

                            var _index = xx + _inst_spectrum_x * (yy + _inst_spectrum_y * zz);

                            Vector3 pos = new Vector3(posX, posY, posZ);

                            if (_index < _sound_byte_array.Length)
                            {
                                //MainWindow.MessageBox((IntPtr)0, "" + _index, "sccsVD4VE_LightNWithoutVr", 0);
                            }
                            else
                            {
                                //MainWindow.MessageBox((IntPtr)0, "" + _index, "sccsVD4VE_LightNWithoutVr", 0);
                            }

                            //var dist = sc_maths.sc_check_distance_node_3d_geometry(Vector3.Zero, pos, 9, 9, 9, 9, 9, 9);

                            try
                            {
                                if (_index < _world_spectrum_list[0][0]._arrayOfInstances.Length && _index < _sound_byte_array.Length) //_inst_spectrum_x * _inst_spectrum_y * _inst_spectrum_z)
                                {
                                    if (_sound_byte_array != null)
                                    {
                                        if (_sound_byte_array.Length > 0)
                                        {
                                            var _left_touch_pos = new Vector3(WorldMatrix.M41, WorldMatrix.M42, WorldMatrix.M43);
                                            /*
                                            if (count_spec < _sound_byte_array.Length && count_spec < _world_spectrum_list[0]._arrayOfInstances.Length) // 
                                            {
                                             
                                                /*if (_has_recorded == 1)
                                                {
                                                    var planeSize = 0.01f;
                                                    var detailscale = 10;
                                                    var heightmul = 20;
                                                    var seed = 3420;
                                                    var fastNoise = new FastNoise(seed);
                                                    float noise = fastNoise.GetPerlin((((spectrum_mat.M41 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M42 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M43 * planeSize) + seed) / detailscale) * heightmul);
                                                    _has_recorded = 2;
                                                }
                                                else
                                                {
                                                    //spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M41;
                                                    //spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M42;
                                                    //spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M43;
                                                }
                                            }
                                            else
                                            {
                                                spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M41;
                                                spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value; //0.0015f // + (_sound_byte_array[count_spec] * 0.0005f)
                                                spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M43;
                                                //count_spec = 0;
                                            }*/
                                            spectrum_mat.M41 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M41;
                                            spectrum_mat.M42 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value + (_sound_byte_array[_index] * 0.005f); //0.0015f
                                            spectrum_mat.M43 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M43;

                                            worldMatrix_instances_spectrum[0][0][_index] = spectrum_mat;
                                            count_spec++;

                                            /*else
                                            {
                                                var _left_touch_pos = new Vector3(WorldMatrix.M41, WorldMatrix.M42, WorldMatrix.M43);

                                                spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M41;
                                                spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value + (_sound_byte_array[0] * 0.0015f);
                                                spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M43;

                                                if (_has_recorded == 1)
                                                {
                                                    var planeSize = 0.01f;
                                                    var detailscale = 100;
                                                    var heightmul = 20;
                                                    var seed = 3420;
                                                    var fastNoise = new FastNoise(3420);
                                                    float noise = fastNoise.GetPerlin((((spectrum_mat.M41 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M42 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M43 * planeSize) + seed) / detailscale) * heightmul);
                                                    _has_recorded = 2;
                                                }
                                                else
                                                {
                                                    //spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M41;
                                                    //spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M42;
                                                    //spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M43;
                                                }
                                                worldMatrix_instances_spectrum[0][_index] = spectrum_mat;
                                            }*/
                                        }
                                    }
                                }
                            }
                            catch (Exception ex)
                            {
                                MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sccsVD4VE_LightNWithoutVr", 0);
                            }
                            //for (int sba = 0; sba < _sound_byte_array.Length; sba++)
                            //{
                            //
                            //}
                        }
                    }
                }

                //_sound_byte_array = list.OrderBy(x => x).ThenBy().ToArray();

                /*for (int s = 0; s < worldMatrix_instances_spectrum[0].Length; s++)
                {

                }*/
                has_spoken_sec = 0;
                has_spoken_main = 0;
            }

            //Console.WriteLine("test0");

            //RECORD SOUND
            if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.R))
            {
                if (sc_start_recording == 0)
                {
                    _records_instant_counter = 0;
                    _time_of_recording_start = DateTime.Now;

                    mciSendString("open new Type waveaudio Alias recsound", null, 0, IntPtr.Zero);
                    mciSendString("record recsound", null, 0, IntPtr.Zero);

                    sc_start_recording = 1;
                    has_spoken_main = 1;
                    //has_spoken_sec = 1;
                }
            }

            if (swtchinstantsound == 1)
            {
                if (has_spoken_tier == 1 && has_spoken_main == 1)
                {
                    instant_short_path = "wave_instant_audio_" + _records_instant_counter;
                    var filename = @"C:\Users\ninekorn\Desktop\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";
                    //var filename = @"D:\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";

                    mciSendString("save recinstantsound " + filename, null, 0, IntPtr.Zero);
                    mciSendString("close recinstantsound", null, 0, IntPtr.Zero);

                    /*Process p = new Process();
                    p.StartInfo = new ProcessStartInfo()
                    {
                        FileName = short_path
                    };
                    p.Start();
                    p.Refresh();
                    p.Close();*/

                    DirectoryInfo dir = new DirectoryInfo(filename);
                    dir.Refresh();
                    if (!File.Exists(filename))
                    {
                        FileInfo filinfo = new FileInfo(filename);
                        filinfo.Refresh();
                    }

                    var nativeFileStream = new NativeFileStream(filename, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                    SoundStream soundStream = new SoundStream(nativeFileStream);
                    MemoryStream ms = new MemoryStream();

                    soundStream.CopyTo(ms);
                    _sound_byte_array_instant = ms.ToArray();

                    soundStream.Dispose();

                    ms.Dispose();
                    dir.Refresh();

                    if (File.Exists(filename))
                    {
                        File.Delete(filename);
                    }

                    _records_instant_counter++;
                    has_spoken_tier = 2;
                }


                if (has_spoken_main == 1)
                {
                    mciSendString("open new Type waveaudio Alias recinstantsound", null, 0, IntPtr.Zero);
                    mciSendString("record recinstantsound", null, 0, IntPtr.Zero);
                    //has_spoken_main = 1;
                }
            }

            if (sc_start_recording == 2)
            {
                if (sc_start_recording_counter >= 50)
                {
                    sc_save_file = 0;
                    //MessageBox((IntPtr)0, "" + sc_start_recording_counter + "", "Oculus Message", 0);
                    sc_start_recording = 0;
                    sc_start_recording_counter = 0;
                }
                sc_start_recording_counter++;
            }

            if (_frames_to_access_counter >= 0)
            {
                if (_records_counter > 0)
                {

                }
                //if (_frames_to_access == 0)
                //{
                //
                //    _frames_to_access = 1;
                //}

                _frames_to_access_counter = 0;
            }


            //SAVE SOUND
            if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.S))
            {
                if (sc_start_recording == 1)
                {
                    if (sc_save_file == 0)
                    {

                        short_path = "wave_audio_" + _records_counter;
                        var filename = @"C:\Users\ninekorn\Desktop\#RECSOUND\" + "wave_audio_" + _records_counter + ".wav";
                        mciSendString("save recsound " + filename, null, 0, IntPtr.Zero);
                        mciSendString("close recsound", null, 0, IntPtr.Zero);
                        _has_recorded = 1;
                        last_wave_filepath = filename;

                        string audiotype = "wave_audio_";

                        var stringtemp = short_path;
                        //"wave_audio_"
                        stringtemp.Substring(11, 1);
                        var info = new FileInfo(last_wave_filepath);
                        info.Refresh();



                        System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
                        customCulture.NumberFormat.NumberDecimalSeparator = ".";
                        System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

                        //short_path = "wave_audio_" + _records_counter;
                        //path = "c:\\Users\\ninekorn\\Desktop\\testXML\\" + 0 + ".xml";
                        path = @"C:\Users\ninekorn\Desktop\#RECSOUND\" + "wave_audio_" + _records_counter + ".xml";
                        last_xml_filepath = path;
                        //sc_can_start_rec_counter_before_add_index = sc_can_start_rec_counter;
                        //doc = new XmlDocument();
                        //writer = new XmlTextWriter(Console.Out);
                        writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
                        //sc_can_start_rec_counter_before_add_index = sc_can_start_rec_counter;

                        //root = doc.DocumentElement;

                        writer.WriteProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");

                        writer.Formatting = Formatting.Indented;
                        writer.Indentation = 2;

                        writer.WriteStartElement("root"); // open 0
                        writer.WriteStartElement("wave"); // open 1
                        writer.WriteStartElement("data"); // open 2



                        writer.WriteStartElement("StartTime"); //open 3
                        writer.WriteString("" + _time_of_recording_start);
                        writer.WriteEndElement(); //close 3

                        _time_of_recording_end = DateTime.Now;
                        writer.WriteStartElement("EndTime"); //open 3
                        writer.WriteString("" + _time_of_recording_end);
                        writer.WriteEndElement(); //close 3

                        writer.WriteStartElement("size"); //open 3
                        long size = new System.IO.FileInfo(last_wave_filepath).Length;
                        writer.WriteString("" + size);
                        writer.WriteEndElement(); //close 3

                        writer.WriteStartElement("length"); //open 4
                        int length = GetSoundLength(last_wave_filepath);
                        writer.WriteString("" + length);
                        writer.WriteEndElement(); //close 4


                        writer.WriteEndElement(); //close 0
                        writer.WriteEndElement(); //close 1
                        writer.WriteEndElement(); //close 2

                        writer.Close();
                        //root.WriteTo(writer);

                        //doc.Save(path);

                        sc_can_start_rec_counter++;
                        _records_counter++;
                        //sc_save_file = 0;

                        var lastAccess = info.LastAccessTime;

                        //_index = sc_can_start_rec_counter_before_add_index + "";
                        _sound_player.AddWave(_records_counter - 1 + "", last_wave_filepath);

                        /*var nativeFileStream = new NativeFileStream(last_wave_filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                        SoundStream soundStream = new SoundStream(nativeFileStream);
                        MemoryStream ms = new MemoryStream();

                        soundStream.CopyTo(ms);
                        _sound_byte_array = ms.ToArray();*/

                        var nativeFileStream = new NativeFileStream(last_wave_filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                        SoundStream soundStream = new SoundStream(nativeFileStream);
                        MemoryStream ms = new MemoryStream();

                        soundStream.CopyTo(ms);
                        _sound_byte_array = ms.ToArray();

                        soundStream.Dispose();
                        ms.Dispose();

                        _sound_player.Play((_records_counter - 1) + "");
                        sc_start_recording = 2;
                        sc_save_file = 1;
                        has_spoken_main = 1;
                        has_spoken_sec = 1;
                        //has_spoken_tier = 0;
                        //has_spoken_quart = 0;
                    }
                }
            }

            //PLAY SOUND AT INDEX
            if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.P))
            {
                if (sc_play_file == 0)
                {
                    if (File.Exists(last_wave_filepath))
                    {
                        _sound_player.Play((_records_counter - 1) + "");

                        //sc_can_start_rec_counter_before_add_index++;

                        //MessageBox((IntPtr)0, "" + stringtemp + "", "Oculus Message", 0);

                        //File.Delete(last_wave_filepath);
                        /*Process p = new Process();
                        p.StartInfo = new ProcessStartInfo()
                        {
                            FileName = last_wave_filepath
                        };
                        p.Start();
                        p.Refresh();
                        p.Close();*/

                        //int can_play_index= 0;

                        //Array.Sort(_sound_byte_array);
                        //_sound_byte_array.OrderBy(x > x).FirstOrDefault();
                        //lowestX = point3DCollection.OrderBy(x => x.X).FirstOrDefault();
                        //highestX = point3DCollection.OrderBy(x => x.X).Last();

                        //for (int s = 0; s < _sound_byte_array.Length;s++)
                        //{
                        //    if (_sound_byte_array[s] > 100)
                        //    {
                        //        MessageBox((IntPtr)0, "" + "has sound", "Oculus Message", 0);
                        //    }
                        //    //MessageBox((IntPtr)0, "" + "has recorded", "Oculus Error", 0);
                        //}
                        //_main_received_object[0]._someData[0] = _sound_byte_array;// new object[1];

                        ///MessageBox((IntPtr)0, "" + _sound_byte_array.Length, "_sc_core_systems error", 0);
                        //using (var soundStream = new SoundStream(nativeFileStream))
                        //{
                        //    using (MemoryStream ms = new MemoryStream())
                        //    {
                        //        soundStream.CopyTo(ms);
                        //        _sound_byte_array = ms.ToArray();
                        //        _main_received_messages[0]._someData[0] = _sound_byte_array;// new object[1];
                        //    }
                        //}

                        sc_play_file = 1;
                    }
                    else
                    {
                        //create blank file and start listening instead lol. kidding. nerd joke.
                    }
                }
            }

            if (sc_play_file == 1)
            {
                if (sc_play_file_counter >= 50)
                {
                    sc_play_file = 0;
                    sc_play_file_counter = 0;
                }
                sc_play_file_counter++;
            }

            return _sc_jitter_tasks;
        }


















        /*
        static string getOskPath(string dir)
        {
            string path = Path.Combine(dir, "osk.exe");
            if (File.Exists(path))
            {
                Process p = System.Diagnostics.Process.Start(path);
                if (p.IsWin64Emulator())
                {
                    path = string.Empty;
                }
                p.Kill();
                return path;
            }
            DirectoryInfo di = new DirectoryInfo(dir);
            foreach (DirectoryInfo subDir in di.GetDirectories().Reverse())
            {
                path = getOskPath(Path.Combine(dir, subDir.Name));
                if (!string.IsNullOrWhiteSpace(path))
                {
                    return path;
                }
            }
            return string.Empty;
        }*/





        public SC_message_object_jitter[][] sc_write_to_buffer(SC_message_object_jitter[][] _sc_jitter_tasks)
        {
            if (_updateFunctionBoolRight)
            {
                _updateFunctionStopwatchRight.Stop();
                _updateFunctionStopwatchRight.Reset();
                _updateFunctionStopwatchRight.Start();
                _updateFunctionBoolRight = false;
            }
            if (_updateFunctionBoolRightThumbStickGoLeft)
            {
                _updateFunctionStopwatchRightThumbstickGoLeft.Stop();
                _updateFunctionStopwatchRightThumbstickGoLeft.Reset();
                _updateFunctionStopwatchRightThumbstickGoLeft.Start();
                _updateFunctionBoolRightThumbStickGoLeft = false;
            }

            if (_updateFunctionBoolRightThumbStickGoRight)
            {
                _updateFunctionStopwatchRightThumbstickGoRight.Stop();
                _updateFunctionStopwatchRightThumbstickGoRight.Reset();
                _updateFunctionStopwatchRightThumbstickGoRight.Start();
                _updateFunctionBoolRightThumbStickGoRight = false;
            }

            if (_updateFunctionBoolLeftThumbStick)
            {
                _updateFunctionStopwatchLeftThumbstick.Stop();
                _updateFunctionStopwatchLeftThumbstick.Reset();
                _updateFunctionStopwatchLeftThumbstick.Start();
                _updateFunctionBoolLeftThumbStick = false;
            }


            


            //SPECTRUM AND SOUND RECORDING

            if (has_spoken_sec == 1)
            {

                //MainWindow.MessageBox((IntPtr)0, "" + "0", "sccoresystems", 0);
                //_sound_byte_array.OrderBy(x => String.Join(String.Empty, x));
                //_sound_byte_array.OrderBy(x > x).FirstOrDefault();
                //_sound_byte_array = _sound_byte_array.ToList().OrderBy(x => x).ToArray(); // new List<byte>();
                //_sound_byte_array = list.OrderBy(x => x).ToArray();
                //var test = _sound_byte_array.OrderBy(x => x, new ByteListComparer());

                int count_spec = 0;

                var widthL = (int)(_inst_spectrum_x / 2);
                var widthR = (int)((_inst_spectrum_x / 2) - 1);

                var heightL = _inst_spectrum_y;
                var heightR = _inst_spectrum_y;

                var depthL = (int)(_inst_spectrum_z / 2);
                var depthR = (int)((_inst_spectrum_z / 2) - 1);

                //var depthL = _inst_spectrum_z;// (int)(_inst_spectrum_z / 2);
                //var depthR = _inst_spectrum_z;// (int)((_inst_spectrum_z / 2) - 1);

                // wL5 and <=wR4  = -5 to 5 ... 210/2 = 105 and 105 so 105-1 = 104 for wL105 and <=wR104   

                for (int x = -widthL; x <= widthR; x++)
                {
                    for (int y = -heightL; y <= heightR; y++)
                    {
                        for (int z = -depthL; z <= depthR; z++)
                        {
                            //MainWindow.MessageBox((IntPtr)0, "" + "0", "sccoresystems", 0);

                            //the higher the closer to center middle.

                            float posX = (x);
                            float posY = (y);
                            float posZ = (z);

                            var xx = x;
                            var yy = y;
                            var zz = z;

                            if (xx < 0)
                            {
                                xx *= -1;
                                xx = widthR + xx;
                            }
                            if (yy < 0)
                            {
                                yy *= -1;
                                yy = heightR + yy;
                            }
                            if (zz < 0)
                            {
                                zz *= -1;
                                zz = depthR + zz;
                            }

                            var _index = xx + _inst_spectrum_x * (yy + _inst_spectrum_y * zz);

                            Vector3 pos = new Vector3(posX, posY, posZ);

                            if (_index < _sound_byte_array.Length)
                            {
                                //MainWindow.MessageBox((IntPtr)0, "" + _index, "sccoresystems", 0);
                            }
                            else
                            {
                                //MainWindow.MessageBox((IntPtr)0, "" + _index, "sccoresystems", 0);
                            }

                            //var dist = sc_maths.sc_check_distance_node_3d_geometry(Vector3.Zero, pos, 9, 9, 9, 9, 9, 9);

                            try
                            {
                                if (_index < _world_spectrum_list[0][0]._arrayOfInstances.Length && _index < _sound_byte_array.Length) //_inst_spectrum_x * _inst_spectrum_y * _inst_spectrum_z)
                                {
                                    if (_sound_byte_array != null)
                                    {
                                        if (_sound_byte_array.Length > 0)
                                        {
                                            var _left_touch_pos = new Vector3(WorldMatrix.M41, WorldMatrix.M42, WorldMatrix.M43);
                                            /*
                                            if (count_spec < _sound_byte_array.Length && count_spec < _world_spectrum_list[0]._arrayOfInstances.Length) // 
                                            {
                                             
                                                /*if (_has_recorded == 1)
                                                {
                                                    var planeSize = 0.01f;
                                                    var detailscale = 10;
                                                    var heightmul = 20;
                                                    var seed = 3420;
                                                    var fastNoise = new FastNoise(seed);
                                                    float noise = fastNoise.GetPerlin((((spectrum_mat.M41 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M42 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M43 * planeSize) + seed) / detailscale) * heightmul);
                                                    _has_recorded = 2;
                                                }
                                                else
                                                {
                                                    //spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M41;
                                                    //spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M42;
                                                    //spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M43;
                                                }
                                            }
                                            else
                                            {
                                                spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M41;
                                                spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value; //0.0015f // + (_sound_byte_array[count_spec] * 0.0005f)
                                                spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M43;
                                                //count_spec = 0;
                                            }*/
                                            spectrum_mat.M41 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M41;
                                            spectrum_mat.M42 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value + (_sound_byte_array[_index] * 0.005f); //0.0015f
                                            spectrum_mat.M43 = _world_spectrum_list[0][0]._arrayOfInstances[_index]._POSITION.M43;

                                            worldMatrix_instances_spectrum[0][0][_index] = spectrum_mat;
                                            count_spec++;

                                            /*else
                                            {
                                                var _left_touch_pos = new Vector3(WorldMatrix.M41, WorldMatrix.M42, WorldMatrix.M43);

                                                spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M41;
                                                spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M42 + spectrum_noise_value + (_sound_byte_array[0] * 0.0015f);
                                                spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[_index]._POSITION.M43;

                                                if (_has_recorded == 1)
                                                {
                                                    var planeSize = 0.01f;
                                                    var detailscale = 100;
                                                    var heightmul = 20;
                                                    var seed = 3420;
                                                    var fastNoise = new FastNoise(3420);
                                                    float noise = fastNoise.GetPerlin((((spectrum_mat.M41 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M42 * planeSize) + seed) / detailscale) * heightmul, (((spectrum_mat.M43 * planeSize) + seed) / detailscale) * heightmul);
                                                    _has_recorded = 2;
                                                }
                                                else
                                                {
                                                    //spectrum_mat.M41 = _left_touch_pos.X + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M41;
                                                    //spectrum_mat.M42 = _left_touch_pos.Y + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M42;
                                                    //spectrum_mat.M43 = _left_touch_pos.Z + _world_spectrum_list[0]._arrayOfInstances[s]._POSITION.M43;
                                                }
                                                worldMatrix_instances_spectrum[0][_index] = spectrum_mat;
                                            }*/
                                        }
                                    }
                                }
                            }
                            catch (Exception ex)
                            {
                                MainWindow.MessageBox((IntPtr)0, "" + ex.ToString(), "sccoresystems", 0);
                            }
                            //for (int sba = 0; sba < _sound_byte_array.Length; sba++)
                            //{
                            //
                            //}
                        }
                    }
                }

                //_sound_byte_array = list.OrderBy(x => x).ThenBy().ToArray();

                /*for (int s = 0; s < worldMatrix_instances_spectrum[0].Length; s++)
                {

                }*/
                has_spoken_sec = 0;
                has_spoken_main = 0;
            }

            //Console.WriteLine("test0");

            //RECORD SOUND
            if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.R))
            {
                if (sc_start_recording == 0)
                {
                    _records_instant_counter = 0;
                    _time_of_recording_start = DateTime.Now;

                    mciSendString("open new Type waveaudio Alias recsound", null, 0, IntPtr.Zero);
                    mciSendString("record recsound", null, 0, IntPtr.Zero);

                    sc_start_recording = 1;
                    has_spoken_main = 1;
                    //has_spoken_sec = 1;
                }
            }

            if (swtchinstantsound == 1)
            {
                if (has_spoken_tier == 1 && has_spoken_main == 1)
                {
                    instant_short_path = "wave_instant_audio_" + _records_instant_counter;
                    var filename = @"C:\Users\ninekorn\Desktop\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";
                    //var filename = @"D:\#RECINSTANTSOUND\" + "wave_instant_audio_" + _records_instant_counter + ".wav";

                    mciSendString("save recinstantsound " + filename, null, 0, IntPtr.Zero);
                    mciSendString("close recinstantsound", null, 0, IntPtr.Zero);

                    /*Process p = new Process();
                    p.StartInfo = new ProcessStartInfo()
                    {
                        FileName = short_path
                    };
                    p.Start();
                    p.Refresh();
                    p.Close();*/

                    DirectoryInfo dir = new DirectoryInfo(filename);
                    dir.Refresh();
                    if (!File.Exists(filename))
                    {
                        FileInfo filinfo = new FileInfo(filename);
                        filinfo.Refresh();
                    }

                    var nativeFileStream = new NativeFileStream(filename, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                    SoundStream soundStream = new SoundStream(nativeFileStream);
                    MemoryStream ms = new MemoryStream();

                    soundStream.CopyTo(ms);
                    _sound_byte_array_instant = ms.ToArray();

                    soundStream.Dispose();

                    ms.Dispose();
                    dir.Refresh();

                    if (File.Exists(filename))
                    {
                        File.Delete(filename);
                    }

                    _records_instant_counter++;
                    has_spoken_tier = 2;
                }


                if (has_spoken_main == 1)
                {
                    mciSendString("open new Type waveaudio Alias recinstantsound", null, 0, IntPtr.Zero);
                    mciSendString("record recinstantsound", null, 0, IntPtr.Zero);
                    //has_spoken_main = 1;
                }
            }

            if (sc_start_recording == 2)
            {
                if (sc_start_recording_counter >= 50)
                {
                    sc_save_file = 0;
                    //MessageBox((IntPtr)0, "" + sc_start_recording_counter + "", "Oculus Message", 0);
                    sc_start_recording = 0;
                    sc_start_recording_counter = 0;
                }
                sc_start_recording_counter++;
            }

            if (_frames_to_access_counter >= 0)
            {
                if (_records_counter > 0)
                {

                }
                //if (_frames_to_access == 0)
                //{
                //
                //    _frames_to_access = 1;
                //}

                _frames_to_access_counter = 0;
            }


            //SAVE SOUND
            if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.S))
            {
                if (sc_start_recording == 1)
                {
                    if (sc_save_file == 0)
                    {

                        short_path = "wave_audio_" + _records_counter;
                        var filename = @"C:\Users\ninekorn\Desktop\#RECSOUND\" + "wave_audio_" + _records_counter + ".wav";
                        mciSendString("save recsound " + filename, null, 0, IntPtr.Zero);
                        mciSendString("close recsound", null, 0, IntPtr.Zero);
                        _has_recorded = 1;
                        last_wave_filepath = filename;

                        string audiotype = "wave_audio_";

                        var stringtemp = short_path;
                        //"wave_audio_"
                        stringtemp.Substring(11, 1);
                        var info = new FileInfo(last_wave_filepath);
                        info.Refresh();



                        System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
                        customCulture.NumberFormat.NumberDecimalSeparator = ".";
                        System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

                        //short_path = "wave_audio_" + _records_counter;
                        //path = "c:\\Users\\ninekorn\\Desktop\\testXML\\" + 0 + ".xml";
                        path = @"C:\Users\ninekorn\Desktop\#RECSOUND\" + "wave_audio_" + _records_counter + ".xml";
                        last_xml_filepath = path;
                        //sc_can_start_rec_counter_before_add_index = sc_can_start_rec_counter;
                        //doc = new XmlDocument();
                        //writer = new XmlTextWriter(Console.Out);
                        writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
                        //sc_can_start_rec_counter_before_add_index = sc_can_start_rec_counter;

                        //root = doc.DocumentElement;

                        writer.WriteProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");

                        writer.Formatting = Formatting.Indented;
                        writer.Indentation = 2;

                        writer.WriteStartElement("root"); // open 0
                        writer.WriteStartElement("wave"); // open 1
                        writer.WriteStartElement("data"); // open 2



                        writer.WriteStartElement("StartTime"); //open 3
                        writer.WriteString("" + _time_of_recording_start);
                        writer.WriteEndElement(); //close 3

                        _time_of_recording_end = DateTime.Now;
                        writer.WriteStartElement("EndTime"); //open 3
                        writer.WriteString("" + _time_of_recording_end);
                        writer.WriteEndElement(); //close 3

                        writer.WriteStartElement("size"); //open 3
                        long size = new System.IO.FileInfo(last_wave_filepath).Length;
                        writer.WriteString("" + size);
                        writer.WriteEndElement(); //close 3

                        writer.WriteStartElement("length"); //open 4
                        int length = GetSoundLength(last_wave_filepath);
                        writer.WriteString("" + length);
                        writer.WriteEndElement(); //close 4


                        writer.WriteEndElement(); //close 0
                        writer.WriteEndElement(); //close 1
                        writer.WriteEndElement(); //close 2

                        writer.Close();
                        //root.WriteTo(writer);

                        //doc.Save(path);

                        sc_can_start_rec_counter++;
                        _records_counter++;
                        //sc_save_file = 0;

                        var lastAccess = info.LastAccessTime;

                        //_index = sc_can_start_rec_counter_before_add_index + "";
                        _sound_player.AddWave(_records_counter - 1 + "", last_wave_filepath);

                        /*var nativeFileStream = new NativeFileStream(last_wave_filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                        SoundStream soundStream = new SoundStream(nativeFileStream);
                        MemoryStream ms = new MemoryStream();

                        soundStream.CopyTo(ms);
                        _sound_byte_array = ms.ToArray();*/

                        var nativeFileStream = new NativeFileStream(last_wave_filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
                        SoundStream soundStream = new SoundStream(nativeFileStream);
                        MemoryStream ms = new MemoryStream();

                        soundStream.CopyTo(ms);
                        _sound_byte_array = ms.ToArray();

                        soundStream.Dispose();
                        ms.Dispose();

                        _sound_player.Play((_records_counter - 1) + "");
                        sc_start_recording = 2;
                        sc_save_file = 1;
                        has_spoken_main = 1;
                        has_spoken_sec = 1;
                        //has_spoken_tier = 0;
                        //has_spoken_quart = 0;
                    }
                }
            }

            //PLAY SOUND AT INDEX
            if (MainWindow._keyboard_input._KeyboardState.PressedKeys.Contains(SharpDX.DirectInput.Key.P))
            {
                if (sc_play_file == 0)
                {
                    if (File.Exists(last_wave_filepath))
                    {
                        _sound_player.Play((_records_counter - 1) + "");

                        //sc_can_start_rec_counter_before_add_index++;

                        //MessageBox((IntPtr)0, "" + stringtemp + "", "Oculus Message", 0);

                        //File.Delete(last_wave_filepath);
                        /*Process p = new Process();
                        p.StartInfo = new ProcessStartInfo()
                        {
                            FileName = last_wave_filepath
                        };
                        p.Start();
                        p.Refresh();
                        p.Close();*/

                        //int can_play_index= 0;

                        //Array.Sort(_sound_byte_array);
                        //_sound_byte_array.OrderBy(x > x).FirstOrDefault();
                        //lowestX = point3DCollection.OrderBy(x => x.X).FirstOrDefault();
                        //highestX = point3DCollection.OrderBy(x => x.X).Last();

                        //for (int s = 0; s < _sound_byte_array.Length;s++)
                        //{
                        //    if (_sound_byte_array[s] > 100)
                        //    {
                        //        MessageBox((IntPtr)0, "" + "has sound", "Oculus Message", 0);
                        //    }
                        //    //MessageBox((IntPtr)0, "" + "has recorded", "Oculus Error", 0);
                        //}
                        //_main_received_object[0]._someData[0] = _sound_byte_array;// new object[1];

                        ///MessageBox((IntPtr)0, "" + _sound_byte_array.Length, "_sc_core_systems error", 0);
                        //using (var soundStream = new SoundStream(nativeFileStream))
                        //{
                        //    using (MemoryStream ms = new MemoryStream())
                        //    {
                        //        soundStream.CopyTo(ms);
                        //        _sound_byte_array = ms.ToArray();
                        //        _main_received_messages[0]._someData[0] = _sound_byte_array;// new object[1];
                        //    }
                        //}

                        sc_play_file = 1;
                    }
                    else
                    {
                        //create blank file and start listening instead lol. kidding. nerd joke.
                    }
                }
            }

            if (sc_play_file == 1)
            {
                if (sc_play_file_counter >= 50)
                {
                    sc_play_file = 0;
                    sc_play_file_counter = 0;
                }
                sc_play_file_counter++;
            }
























            

            //SPECTRUM
            _world_spectrum_list[0][0]._WORLDMATRIXINSTANCES = worldMatrix_instances_spectrum[0][0];
            _world_spectrum_list[0][0]._POSITION = worldMatrix_base[0];

            var cuber_spectrum = _world_spectrum_list[0][0];
            var instancers_spectrum = cuber_spectrum.instances;
            var sometester_spectrum = cuber_spectrum._WORLDMATRIXINSTANCES;

            for (int i = 0; i < instancers_spectrum.Length; i++)
            {
                float xxx = sometester_spectrum[i].M41;
                float yyy = sometester_spectrum[i].M42;
                float zzz = sometester_spectrum[i].M43;

                cuber_spectrum.instances[i].position.X = xxx;
                cuber_spectrum.instances[i].position.Y = yyy;
                cuber_spectrum.instances[i].position.Z = zzz;
                cuber_spectrum.instances[i].position.W = 1;
                Quaternion.RotationMatrix(ref sometester_spectrum[i], out quat_buffers);

                var dirInstance = sc_maths._newgetdirforward(quat_buffers);
                cuber_spectrum.instancesDataForward[i].rotation.X = dirInstance.X;
                cuber_spectrum.instancesDataForward[i].rotation.Y = dirInstance.Y;
                cuber_spectrum.instancesDataForward[i].rotation.Z = dirInstance.Z;
                cuber_spectrum.instancesDataForward[i].rotation.W = 1;

                dirInstance = -sc_maths._newgetdirleft(quat_buffers);
                cuber_spectrum.instancesDataRIGHT[i].rotation.X = dirInstance.X;
                cuber_spectrum.instancesDataRIGHT[i].rotation.Y = dirInstance.Y;
                cuber_spectrum.instancesDataRIGHT[i].rotation.Z = dirInstance.Z;
                cuber_spectrum.instancesDataRIGHT[i].rotation.W = 1;

                dirInstance = dirInstance = sc_maths._newgetdirup(quat_buffers);
                cuber_spectrum.instancesDataUP[i].rotation.X = dirInstance.X;
                cuber_spectrum.instancesDataUP[i].rotation.Y = dirInstance.Y;
                cuber_spectrum.instancesDataUP[i].rotation.Z = dirInstance.Z;
                cuber_spectrum.instancesDataUP[i].rotation.W = 1;
            }
            //END OF










            
            for (int xx = 0; xx < MainWindow._physics_engine_instance_x; xx++)
            {
                for (int yy = 0; yy < MainWindow._physics_engine_instance_y; yy++)
                {
                    for (int zz = 0; zz < MainWindow._physics_engine_instance_z; zz++)
                    {
                        var indexer00 = xx + MainWindow._physics_engine_instance_x * (yy + MainWindow._physics_engine_instance_y * zz);

                        try
                        {
                            for (int x = 0; x < MainWindow.world_width; x++)
                            {
                                for (int y = 0; y < MainWindow.world_height; y++)
                                {
                                    for (int z = 0; z < MainWindow.world_depth; z++)
                                    {
                                        var indexer01 = x + MainWindow.world_width * (y + MainWindow.world_height * z);

                                        /*
                                        //PHYSICS CUBES
                                        _world_cube_list[indexer00][indexer01]._WORLDMATRIXINSTANCES = worldMatrix_instances_cubes[indexer00][indexer01]; // 
                                        _world_cube_list[indexer00][indexer01]._POSITION = worldMatrix_base[0];

                                        cuber = _world_cube_list[indexer00][indexer01];
                                        instancers = cuber.instances;
                                        sometester = cuber._WORLDMATRIXINSTANCES;

                                        for (int i = 0; i < instancers.Length; i++) 
                                        {
                                            float xxx = sometester[i].M41;
                                            float yyy = sometester[i].M42;
                                            float zzz = sometester[i].M43;

                                            cuber.instances[i].position.X = xxx;
                                            cuber.instances[i].position.Y = yyy;
                                            cuber.instances[i].position.Z = zzz;
                                            cuber.instances[i].position.W = 1;

                                            Quaternion.RotationMatrix(ref sometester[i], out quat_buffers);

                                            var dirInstance = sc_maths._newgetdirforward(quat_buffers);
                                            //var dirInstance = _newgetdirforward(_testQuater);
                                            cuber.instancesDataForward[i].rotation.X = dirInstance.X;
                                            cuber.instancesDataForward[i].rotation.Y = dirInstance.Y;
                                            cuber.instancesDataForward[i].rotation.Z = dirInstance.Z;
                                            cuber.instancesDataForward[i].rotation.W = 1;

                                            dirInstance = -sc_maths._newgetdirleft(quat_buffers);
                                            //dirInstance = -_newgetdirleft(_testQuater);
                                            cuber.instancesDataRIGHT[i].rotation.X = dirInstance.X;
                                            cuber.instancesDataRIGHT[i].rotation.Y = dirInstance.Y;
                                            cuber.instancesDataRIGHT[i].rotation.Z = dirInstance.Z;
                                            cuber.instancesDataRIGHT[i].rotation.W = 1;

                                            dirInstance = sc_maths._newgetdirup(quat_buffers);
                                            //dirInstance = dirInstance = _newgetdirup(_testQuater);
                                            cuber.instancesDataUP[i].rotation.X = dirInstance.X;
                                            cuber.instancesDataUP[i].rotation.Y = dirInstance.Y;
                                            cuber.instancesDataUP[i].rotation.Z = dirInstance.Z;
                                            cuber.instancesDataUP[i].rotation.W = 1;
                                        }
                                        //END OF*/
                                        
                                    }
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex.ToString());
                        }
                    }
                }
            }
            //SC BUFFERS
            //SC BUFFERS
            //SC BUFFERS
            

            
            return _sc_jitter_tasks;
        }

        bool _canResetCounterTouchRightButtonA = false;
        bool _canResetCounterTouchRightButtonB = false;
        int _frameCounterTouchLeft = 0;
        bool _canResetCounterTouchLeftButtonA = false;
        bool _canResetCounterTouchLeftButtonB = false;
        bool _canResetCounterTouchLeftButtonX = false;
        bool _canResetCounterTouchLeftButtonY = false;
        bool hasUsedThumbStickLeftW = false;
        bool hasUsedThumbStickLeftS = false;
        bool hasUsedThumbStickLeftA = false;
        bool hasUsedThumbStickRightD = false;
        bool hasUsedThumbStickRightQ = false;
        bool hasUsedThumbStickRightE = false;
        bool lastHasUsedHandTriggerLeft = false;
        bool hasUsedHandTriggerLeft = false;
        /*
        private void _oculus_touch_controls(double percentXRight, double percentYRight, Vector2[] thumbStickRight, double percentXLeft, double percentYLeft, Vector2[] thumbStickLeft, double realMousePosX, double realMousePosY) //
        {
            if (buttonPressedOculusTouchLeft == 1048576)
            {
                if (hasClickedHomeButtonTouchLeft == 0)
                {
                    //SC_console_directx.D3D.OVR.RecenterTrackingOrigin(SC_console_directx.D3D.sessionPtr);

                    //hmdrotMatrix

                    Quaternion currentRot;// = hmd_matrix_current;
                    Quaternion.RotationMatrix(ref hmd_matrix_current, out currentRot);

                    hmd_matrix_current = hmd_matrix_current * OriginRot * RotatingMatrix * RotatingMatrixForPelvis * hmdmatrixRot_; //viewMatrix_;


                    //Quaternion currentRotAfter;
                    //Quaternion.RotationMatrix(ref hmd_matrix_current, out currentRotAfter);
                    //Quaternion.Lerp(ref currentRot, ref currentRotAfter, 0.001f, out currentRotAfter);
                    //Matrix.RotationQuaternion(ref currentRotAfter,out hmd_matrix_current);

                    //var timeSinceStart = (float)(DateTime.Now - SC_Update.startTime).TotalSeconds;
                    //Matrix worldmatlightrot = Matrix.Scaling(1.0f) * Matrix.RotationX(timeSinceStart * disco_sphere_rot_speed) * Matrix.RotationY(timeSinceStart * 2 * disco_sphere_rot_speed) * Matrix.RotationZ(timeSinceStart * 3 * disco_sphere_rot_speed);



                    Quaternion _testQuator;
                    Quaternion.RotationMatrix(ref hmd_matrix_current, out _testQuator);

                    var xq = _testQuator.X;
                    var yq = _testQuator.Y;
                    var zq = _testQuator.Z;
                    var wq = _testQuator.W;

                    float roller = (float)(Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq));// * (180 / Math.PI));
                    float pitcher = (float)(Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq));// * (180 / Math.PI));//
                    float yawer = (float)(Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq));// * (180 / Math.PI));

                    //RotationY4Pelvis
                    //Matrix tempMat = RotatingMatrixForPelvis * hmdmatrixRot_;
                    Quaternion.RotationMatrix(ref hmd_matrix_current, out _testQuator);

                    xq = _testQuator.X;
                    yq = _testQuator.Y;
                    zq = _testQuator.Z;
                    wq = _testQuator.W;

                    float rollerPelvis = (float)(Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq));// * (180 / Math.PI));
                    float pitcherPelvis = (float)(Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq));// * (180 / Math.PI));//
                    float yawerPelvis = (float)(Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq));// * (180 / Math.PI));

                    //var pitch = (float)((Math.PI * pitcher + 45) / 180);
                    //var roll = (float)(0);
                    //var yaw = (float)(0);

                    SC_Update.RotationX4Pelvis = pitcher;
                    SC_Update.RotationY4Pelvis = 0;
                    SC_Update.RotationZ4Pelvis = 0;

                    SC_Update.rotatingMatrixForPelvis = SharpDX.Matrix.RotationYawPitchRoll(pitcher, 0, 0);
                    SC_Update.hmdmatrixRot = SharpDX.Matrix.RotationYawPitchRoll(pitcher, 0, 0);

                    hasClickedHomeButtonTouchLeft = 1;
                }
            }

            if (hasClickedHomeButtonTouchLeft == 1)
            {
                if (hasClickedHomeButtonTouchLeftCounter > 20)
                {
                    hasClickedHomeButtonTouchLeft = 0;
                    hasClickedHomeButtonTouchLeftCounter = 0;
                }
                hasClickedHomeButtonTouchLeftCounter++;
            }

            if (buttonPressedOculusTouchLeft != 0)
            {
                if (buttonPressedOculusTouchLeft == 512)
                {
                    if (sc_menu_scroller_counter >= 75)
                    {
                        if (sc_menu_scroller == 0)
                        {
                            sc_menu_scroller = 1;
                        }
                        else if (sc_menu_scroller == 1)
                        {
                            sc_menu_scroller = 2;
                        }
                        else if (sc_menu_scroller == 2)
                        {
                            sc_menu_scroller = 0;
                        }
                        sc_menu_scroller_counter = 0;
                    }
                }
            }
            sc_menu_scroller_counter++;

            if (_has_locked_screen_pos_counter >= 50)
            {
                if (buttonPressedOculusTouchLeft == 256)
                {
                    if (sc_menu_scroller == 0)
                    {
                        if (frame_counter_4_buttonY >= 75)
                        {
                            if (display_grid_type == 0)
                            {
                                display_grid_type = 1;
                            }
                            else if (display_grid_type == 1)
                            {
                                display_grid_type = 2;
                            }
                            else if (display_grid_type == 2)
                            {
                                display_grid_type = 3;
                            }
                            else if (display_grid_type == 3)
                            {
                                display_grid_type = 0;
                            }
                            frame_counter_4_buttonY = 0;
                        }
                    }
                    else if (sc_menu_scroller == 1)
                    {
                        if (_has_locked_screen_pos == 0)
                        {
                            _has_locked_screen_pos_counter = 0;
                            _has_locked_screen_pos = 1;
                        }
                        else if (_has_locked_screen_pos == 1)
                        {
                            _has_locked_screen_pos_counter = 0;
                            _has_locked_screen_pos = 0;
                        }
                    }
                }
            }
            frame_counter_4_buttonY++;
            _has_locked_screen_pos_counter++;






            if (sc_menu_scroller == 2)
            {
                /////////////LEFT OCULUS TOUCH/////////////////////////////////////////////////////////////////////////////////////
                if (percentXLeft >= 0 && percentXLeft <= 1920 && percentYLeft >= 0 && percentYLeft <= 1080)
                {
                    var absoluteMoveX = Convert.ToUInt32((percentXLeft * (65535 - 1)) / SC_console_directx.D3D.SurfaceWidth);
                    var absoluteMoveY = Convert.ToUInt32((percentYLeft * (65535 - 1)) / SC_console_directx.D3D.SurfaceHeight);

                    if (percentXLeft >= 0 && percentXLeft < SC_console_directx.D3D.SurfaceWidth)
                    {

                    }
                    else
                    {
                        percentXLeft = SC_console_directx.D3D.SurfaceWidth;
                        absoluteMoveX = Convert.ToUInt32((percentXLeft * (65535 - 1)) / SC_console_directx.D3D.SurfaceWidth);
                    }

                    if (percentYLeft >= 0 && percentYLeft < SC_console_directx.D3D.SurfaceHeight)
                    {

                    }
                    else
                    {
                        percentYLeft = SC_console_directx.D3D.SurfaceHeight;
                        absoluteMoveY = Convert.ToUInt32((percentYLeft * (65535 - 1)) / SC_console_directx.D3D.SurfaceHeight);
                    }


                    //MOUSE DOUBLE CLICK LOGIC. IF the PLAYER clicked at one location then it stores the location, and if the player re-clicks inside of 20 frames, then click at the first click location.
                    //It's very basic, and at least I should implement also a certain "radius" of distance from the first click and the second click... If the second click is too far from the first click,
                    //then disregard the first click location.
                    if (_frameCounterTouchLeft >= 50)
                    {
                        if (buttonPressedOculusTouchLeft != 0)
                        {
                            if (buttonPressedOculusTouchLeft == 256)
                            {
                                if (hasClickedBUTTONX == 0)
                                {
                                    absoluteMoveX = Convert.ToUInt32((percentXLeft * 65535 - 1) / SC_console_directx.D3D.SurfaceWidth);
                                    absoluteMoveY = Convert.ToUInt32((percentYLeft * 65535 - 1) / SC_console_directx.D3D.SurfaceHeight);

                                    //if (_out_of_bounds_left == 0)
                                    //{
                                    //  
                                    //}

                                    SetCursorPos((int)percentXLeft, (int)percentYLeft);
                                    //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, _lastMousePosXRight, _lastMousePosYRight, 0, 0);
                                    _frameCounterTouchLeft = 0;

                                    //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, absoluteMoveX, absoluteMoveY, 0, 0);
                                    //mouse_event(MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, absoluteMoveX, absoluteMoveY, 0, 0);


                                    MainWindow.mousesim.LeftButtonDown();
                                    //MainWindow.mousesim.MoveMouseToPositionOnVirtualDesktop();
                                    //MainWindow.mousesim.LeftButtonClick();

                                    //_lastMousePosXLeft = absoluteMoveX;
                                    //_lastMousePosYLeft = absoluteMoveY;
                                    _canResetCounterTouchLeftButtonX = true;
                                    hasClickedBUTTONX = 1;
                                }
                            }
                            /*else if (buttonPressedOculusTouchLeft == 512)
                            {
                                if (hasClickedBUTTONY == 0)
                                {
                                    if (_out_of_bounds_right == 0)
                                    {
                                        absoluteMoveX = Convert.ToUInt32((realMousePosX * 65535 - 1) / SC_console_directx.D3D.SurfaceWidth);
                                        absoluteMoveY = Convert.ToUInt32((realMousePosY * 65535 - 1) / SC_console_directx.D3D.SurfaceHeight);
                                    }
                                    mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0);
                                    //_lastMousePosX = absoluteMoveX;
                                    //_lastMousePosY = absoluteMoveY;
                                    //_canResetCounterTouchRight = true;
                                    hasClickedBUTTONY = 1;
                                }
                            }

                        }
                    }
                    _out_of_bounds_left = 0;
                }
                else
                {
                    _out_of_bounds_left = 1;
                }
                if (hasClickedBUTTONX == 1)
                {
                    if (_frameCounterTouchLeft >= 10)
                    {
                        hasClickedBUTTONX = 0;
                    }
                }
            }
            _frameCounterTouchLeft++;




            if (indexTriggerRight[1] > 0.0001f)
            {
                if (heightmapscale > heightmapscaleMax)
                {
                    heightmapscale = heightmapscaleMax;
                }
                else
                {
                    heightmapscale += 0.00001f;
                }
            }

            if (indexTriggerLeft[0] > 0.0001f)
            {
                if (heightmapscale < heightmapscaleMin)
                {
                    heightmapscale = heightmapscaleMin;
                }
                else
                {
                    heightmapscale -= 0.00001f;
                }
            }
            //if (Math.Abs(Math.Abs(indexTriggerRightLastAbs) - Math.Abs(indexTriggerRight[1])) > 0.0001f)
            //if (Math.Abs(Math.Abs(indexTriggerLeftLastAbs) - Math.Abs(indexTriggerLeft[0])) > 0.0001f)
            indexTriggerRightLastAbs = indexTriggerRight[1];
            indexTriggerLeftLastAbs = indexTriggerLeft[0];
















            /*
            if (buttonPressedOculusTouchLeft != 0)
            {
                //MainWindow.MessageBox((IntPtr)0, buttonPressedOculusTouchLeft + "", "sccs message", 0);

                if (buttonPressedOculusTouchLeft == 1024 && hasClickedBUTTONX == 0)
                {
                    /*Process[] p = Process.GetProcessesByName(Path.GetFileNameWithoutExtension(OnScreenKeyboardExe));

                    if (p.Length == 0)
                    {
                        // we must start osk from an MTA thread
                        if (Thread.CurrentThread.GetApartmentState() == ApartmentState.STA)
                        {
                            ThreadStart start = new ThreadStart(StartOsk);
                            Thread thread = new Thread(start);
                            thread.SetApartmentState(ApartmentState.MTA);
                            thread.Start();
                            thread.Join();
                        }
                        else
                        {
                            StartOsk();
                        }
                    }
                    else
                    {
                        // there might be a race condition if the process terminated 
                        // meanwhile -> proper exception handling should be added
                        //
                        SendMessage(p[0].MainWindowHandle, WM_SYSCOMMAND, new IntPtr(SC_RESTORE), new IntPtr(0)); //MainWindowHandle
                    }*/
            //MainWindow.MessageBox((IntPtr)0, "fuck you", "sccs message", 0);
            /*string windowsKeyboard = "osk";

            foreach (Process clsProcess in Process.GetProcesses())
            {
                if (clsProcess.ProcessName.ToLower().Contains(windowsKeyboard.ToLower()))
                {
                    break;
                }
                else
                {
                    Process proc = new Process();
                    proc.StartInfo.FileName = windowsKeyboard + ".exe";
                    proc.Start();
                    break;
                }
            }
            //hasClickedBUTTONX = 1;
        }
    }
    else if (buttonPressedOculusTouchLeft == 0 && hasClickedHomeButtonTouchLeft == 1 || buttonPressedOculusTouchLeft == 0 && hasClickedBUTTONX == 1)
    {
        if (buttonPressedOculusTouchLeft == 0 && hasClickedHomeButtonTouchLeft == 1)
        {
            hasClickedHomeButtonTouchLeft = 0;
        }
        else if (buttonPressedOculusTouchLeft == 0 && hasClickedBUTTONX == 1)
        {
            hasClickedBUTTONX = 0;
        }

    }*/



            /*if (buttonPressedOculusTouchLeft != 0)
            {
                var yo = _updateFunctionStopwatchLeftThumbstick.Elapsed.Ticks;

                if (yo >= 100)
                {
                    if (buttonPressedOculusTouchLeft == 1024)
                    {

                        _updateFunctionBoolLeftThumbStick = true;
                    }
                }
            }
        }*/

        uint lastbuttonPressedOculusTouchRight = 0;
        uint lastbuttonPressedOculusTouchLeft = 0;
        public int _indexMouseMove = 0;
        bool restartFrameCounterRight = false;

        int hasClickedHomeButtonTouchLeft = 0;
        int hasClickedHomeButtonTouchLeftCounter = 0;

        bool isHoldingBUTTONA = false;
        bool hasClickedBUTTONB = false;
        int hasClickedBUTTONX = 0;
        int hasClickedBUTTONY = 0;
        bool restartFrameCounterLeft = false;
        /*
        private void _MicrosoftWindowsMouseRight(double percentXRight, double percentYRight, Vector2[] thumbStickRight, double percentXLeft, double percentYLeft, Vector2[] thumbStickLeft, double realMousePosX, double realMousePosY) //, double realOculusRiftCursorPosX, double realOculusRiftCursorPosY
        {
            try
            {
                //MessageBox((IntPtr)0, "percentXRight: " + percentXRight + " percentYRight: " + percentYRight, "mouse move", 0);
                //Console.WriteLine("percentXRight: " + percentXRight + " percentYRight: " + percentYRight);

                if (_indexMouseMove == 0)
                {
                    //MessageBox((IntPtr)0, "test0", "mouse move", 0);
                    /////////////RIGHT OCULUS TOUCH/////////////////////////////////////////////////////////////////////////////////////
                    if (percentXRight >= 0 && percentXRight <= SC_console_directx.D3D.SurfaceWidth && percentYRight >= 0 && percentYRight <= SC_console_directx.D3D.SurfaceHeight &&
                        realMousePosX >= 0 && realMousePosX <= SC_console_directx.D3D.SurfaceWidth && realMousePosY >= 0 && realMousePosY <= SC_console_directx.D3D.SurfaceHeight)
                    {
                        //MessageBox((IntPtr)0, "test1", "mouse move", 0);

                        //var absoluteMoveX = Convert.ToUInt32((percentXRight * 65535) / SC_console_directx.D3D.SurfaceWidth);
                        //var absoluteMoveY = Convert.ToUInt32((percentYRight * 65535) / SC_console_directx.D3D.SurfaceHeight);

                        var yo = _updateFunctionStopwatchRight.Elapsed.Ticks;

                        if (_hasLockedMouse == 0)
                        {
                            if (yo >= 250)
                            {
                                var absoluteMoveX = Convert.ToUInt32((realMousePosX * (65535 - 1)) / SC_console_directx.D3D.SurfaceWidth);
                                var absoluteMoveY = Convert.ToUInt32((realMousePosY * (65535 - 1)) / SC_console_directx.D3D.SurfaceHeight);

                                if (realMousePosX >= 0 && realMousePosX < SC_console_directx.D3D.SurfaceWidth)
                                {

                                }
                                else
                                {
                                    realMousePosX = SC_console_directx.D3D.SurfaceWidth;
                                    absoluteMoveX = Convert.ToUInt32((realMousePosX * (65535 - 1)) / SC_console_directx.D3D.SurfaceWidth);
                                }

                                if (realMousePosY >= 0 && realMousePosY < SC_console_directx.D3D.SurfaceHeight)
                                {

                                }
                                else
                                {
                                    realMousePosY = SC_console_directx.D3D.SurfaceHeight;
                                    absoluteMoveY = Convert.ToUInt32((realMousePosY * (65535 - 1)) / SC_console_directx.D3D.SurfaceHeight);
                                }


                                //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, absoluteMoveX, absoluteMoveY, 0, 0);
                                if (_frameCounterTouchRight <= 20)
                                {
                                    SetCursorPos((int)realMousePosX, (int)realMousePosY);
                                    //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, _lastMousePosXRight, _lastMousePosYRight, 0, 0);
                                    _frameCounterTouchRight = 0;
                                }

                                _updateFunctionStopwatchRight.Stop();
                                _updateFunctionBoolRight = true;
                            }
                        }

                        //Console.WriteLine(percentXRight + "_" + percentYRight);
                        //MessageBox((IntPtr)0,  "test", "mouse move", 0);
                        //MOUSE DOUBLE CLICK LOGIC. IF the PLAYER clicked at one location then it stores the location, and if the player re-clicks inside of 20 frames, then click at the first click location.
                        //It's very basic, and at least I should implement also a certain "radius" of distance from the first click and the second click... If the second click is too far from the first click,
                        //then disregard the first click location.

                        if (buttonPressedOculusTouchRight != 0)
                        {
                            if (buttonPressedOculusTouchRight == 1)
                            {
                                if (hasClickedBUTTONA == 0)
                                {
                                    var absoluteMoveX = Convert.ToUInt32((realMousePosX * (65535)) / SC_console_directx.D3D.SurfaceWidth);
                                    var absoluteMoveY = Convert.ToUInt32((realMousePosY * (65535)) / SC_console_directx.D3D.SurfaceHeight);

                                    if (realMousePosX >= 0 && realMousePosX < SC_console_directx.D3D.SurfaceWidth)
                                    {

                                    }
                                    else
                                    {
                                        realMousePosX = SC_console_directx.D3D.SurfaceWidth;
                                        absoluteMoveX = Convert.ToUInt32((realMousePosX * (65535)) / SC_console_directx.D3D.SurfaceWidth);
                                    }

                                    if (realMousePosY >= 0 && realMousePosY < SC_console_directx.D3D.SurfaceHeight)
                                    {

                                    }
                                    else
                                    {
                                        realMousePosY = SC_console_directx.D3D.SurfaceHeight;
                                        absoluteMoveY = Convert.ToUInt32((realMousePosY * (65535)) / SC_console_directx.D3D.SurfaceHeight);
                                    }


                                    if (_frameCounterTouchRight <= 20 && _canResetCounterTouchRightButtonA == true)
                                    {
                                        SetCursorPos((int)realMousePosX, (int)realMousePosY);
                                        //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, _lastMousePosXRight, _lastMousePosYRight, 0, 0);
                                        _frameCounterTouchRight = 0;
                                    }

                                    //mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0); //| MOUSEEVENTF_ABSOLUTE

                                    MainWindow.mousesim.LeftButtonDown();

                                    _lastMousePosXRight = absoluteMoveX;
                                    _lastMousePosYRight = absoluteMoveY;

                                    _canResetCounterTouchRightButtonA = true;
                                    hasClickedBUTTONA = 1;
                                }
                            }
                            else if (buttonPressedOculusTouchRight == 2)
                            {
                                if (hasClickedBUTTONB == false)
                                {
                                    var absoluteMoveX = Convert.ToUInt32((realMousePosX * (65535)) / SC_console_directx.D3D.SurfaceWidth);
                                    var absoluteMoveY = Convert.ToUInt32((realMousePosY * (65535)) / SC_console_directx.D3D.SurfaceHeight);

                                    if (realMousePosX >= 0 && realMousePosX < SC_console_directx.D3D.SurfaceWidth)
                                    {

                                    }
                                    else
                                    {
                                        realMousePosX = SC_console_directx.D3D.SurfaceWidth;
                                        absoluteMoveX = Convert.ToUInt32((realMousePosX * (65535)) / SC_console_directx.D3D.SurfaceWidth);
                                    }

                                    if (realMousePosY >= 0 && realMousePosY < SC_console_directx.D3D.SurfaceHeight)
                                    {

                                    }
                                    else
                                    {
                                        realMousePosY = SC_console_directx.D3D.SurfaceHeight;
                                        absoluteMoveY = Convert.ToUInt32((realMousePosY * (65535)) / SC_console_directx.D3D.SurfaceHeight);

                                    }


                                    //mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0);
                                    MainWindow.mousesim.RightButtonDown();
                                    hasClickedBUTTONB = true;
                                }
                            }
                        }
                        _out_of_bounds_right = 0;
                    }
                    else
                    {
                        _out_of_bounds_right = 1;
                    }

                    if (hasClickedBUTTONACounter >= 25)
                    {
                        //////////OCULUS TOUCH BUTTONS PRESSED////////////////////////////////////////
                        if (hasClickedBUTTONA == 1 && buttonPressedOculusTouchRight == 0 || hasClickedBUTTONB && buttonPressedOculusTouchRight == 0)
                        {
                            if (hasClickedBUTTONA == 1 && buttonPressedOculusTouchRight == 0)
                            {
                                //mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                                MainWindow.mousesim.LeftButtonUp();
                                hasClickedBUTTONACounter = 0;
                                hasClickedBUTTONA = 0;
                            }
                            else if (hasClickedBUTTONB && buttonPressedOculusTouchRight == 0)
                            {
                                //mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0);
                                MainWindow.mousesim.RightButtonUp();
                                hasClickedBUTTONACounter = 0;
                                hasClickedBUTTONB = false;
                            }
                        }
                    }
                    //if (_canResetCounterTouchRightButtonA)
                    //{
                    //  
                    //}
                    _frameCounterTouchRight++;
                    if (_frameCounterTouchRight >= 30)
                    {
                        _frameCounterTouchRight = 0;
                        _canResetCounterTouchRightButtonA = false;
                    }

                    if (_out_of_bounds_oculus_rift == 0)
                    {
                        long test = 80;
                        /////////RIGHT THUMBSTICK///////////
                        var yo6 = _updateFunctionStopwatchRightThumbstickGoLeft.Elapsed.Milliseconds;
                        if (yo6 >= 75)
                        {
                            /*if (thumbStickRight[1].Y <= -0.1f && hasUsedThumbStickRightE == false)
                            {
                                if (test < -0.99)
                                {
                                    test = (long)-0.99;
                                }

                                mouse_event(MOUSEEVENTF_WHEEL, 0, 0, -test, 0);
                                //Console.WriteLine("test");

                                hasUsedThumbStickRightE = true;
                            }
                            else if (hasUsedThumbStickRightE)
                            {
                                hasUsedThumbStickRightE = false;
                            }
                            _updateFunctionStopwatchRightThumbstickGoLeft.Stop();
                            _updateFunctionBoolRightThumbStickGoLeft = true;
                        }
                        ///////////////////////////////////////////////////////////////////////////

                        /////////RIGHT THUMBSTICK/////////////////////////////////////////////////////
                        var yo7 = _updateFunctionStopwatchRightThumbstickGoRight.Elapsed.Milliseconds;
                        if (yo7 >= 75)
                        {
                            /*if (thumbStickRight[1].Y >= 0.1f && hasUsedThumbStickRightQ == false)
                            {
                                if (test > 0.99f)
                                {
                                    test = (long)0.99;
                                }

                                mouse_event(MOUSEEVENTF_WHEEL, 0, 0, test, 0);
                                hasUsedThumbStickRightQ = true;
                            }
                            else if (hasUsedThumbStickRightQ)
                            {
                                hasUsedThumbStickRightQ = false;
                            }
                            _updateFunctionStopwatchRightThumbstickGoRight.Stop();
                            _updateFunctionBoolRightThumbStickGoRight = true;
                        }
                    }























                    //////////OCULUS TOUCH BUTTONS PRESSED////////////////////////////////////////
                    if (hasClickedBUTTONX == 1 && buttonPressedOculusTouchLeft == 0 || hasClickedBUTTONY == 1 && buttonPressedOculusTouchLeft == 0)
                    {
                        if (hasClickedBUTTONX == 1 && buttonPressedOculusTouchLeft == 0)
                        {
                            //mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                            hasClickedBUTTONX = 0;
                        }
                        else if (hasClickedBUTTONY == 1 && buttonPressedOculusTouchLeft == 0)
                        {
                            mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0);
                            hasClickedBUTTONY = 0;
                        }
                    }

                    //if (_canResetCounterTouchLeftButtonX)
                    //{
                    //
                    //}



                    //if (buttonPressedOculusTouchLeft == 0 && hasClickedHomeButtonTouchLeft)
                    //{
                    //    hasClickedHomeButtonTouchLeft = false;
                    //}
                }
                else if (_indexMouseMove == 100)
                {
                    /////////////LEFT OCULUS TOUCH/////////////////////////////////////////////////////////////////////////////////////
                    if (percentXLeft >= 0 && percentXLeft <= 1920 && percentYLeft >= 0 && percentYLeft <= 1080)
                    {
                        var absoluteMoveX = Convert.ToUInt32((percentXLeft * 65535) / 1920);
                        var absoluteMoveY = Convert.ToUInt32((percentYLeft * 65535) / 1080);

                        var yo = _updateFunctionStopwatchLeft.Elapsed.Milliseconds;

                        if (yo >= 10)
                        {
                            mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, absoluteMoveX, absoluteMoveY, 0, 0);
                            _updateFunctionStopwatchLeft.Stop();
                            _updateFunctionBoolLeft = true;
                        }

                        //MOUSE DOUBLE CLICK LOGIC. IF the PLAYER clicked at one location then it stores the location, and if the player re-clicks inside of 20 frames, then click at the first click location.
                        //It's very basic, and at least I should implement also a certain "radius" of distance from the first click and the second click... If the second click is too far from the first click,
                        //then disregard the first click location.

                        if (buttonPressedOculusTouchLeft != 0)
                        {
                            if (buttonPressedOculusTouchLeft == 256)
                            {
                                if (hasClickedBUTTONX == 0)
                                {
                                    if (_frameCounterTouchLeft <= 20 && _canResetCounterTouchLeftButtonX == true)
                                    {
                                        mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, _lastMousePosXLeft, _lastMousePosYLeft, 0, 0);
                                        _frameCounterTouchLeft = 0;
                                    }

                                    mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);

                                    _lastMousePosXLeft = absoluteMoveX;
                                    _lastMousePosYLeft = absoluteMoveY;
                                    _canResetCounterTouchLeftButtonX = true;
                                    hasClickedBUTTONX = 1;
                                }
                            }
                            else if (buttonPressedOculusTouchLeft == 512)
                            {
                                if (hasClickedBUTTONY == 0)
                                {
                                    mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0);
                                    //_lastMousePosX = absoluteMoveX;
                                    //_lastMousePosY = absoluteMoveY;
                                    //_canResetCounterTouchRight = true;
                                    hasClickedBUTTONY = 1;
                                }
                            }
                        }
                    }

                    //////////OCULUS TOUCH BUTTONS PRESSED////////////////////////////////////////
                    if (hasClickedBUTTONX == 1 && buttonPressedOculusTouchLeft == 0 || hasClickedBUTTONY == 1 && buttonPressedOculusTouchLeft == 0)
                    {
                        if (hasClickedBUTTONX == 1 && buttonPressedOculusTouchLeft == 0)
                        {
                            mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                            hasClickedBUTTONX = 0;
                        }
                        else if (hasClickedBUTTONY == 1 && buttonPressedOculusTouchLeft == 0)
                        {
                            mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0);
                            hasClickedBUTTONY = 0;
                        }
                    }

                    if (_canResetCounterTouchLeftButtonX)
                    {
                        _frameCounterTouchLeft++;
                    }

                    if (_frameCounterTouchLeft >= 30)
                    {
                        _frameCounterTouchLeft = 0;
                        _canResetCounterTouchLeftButtonX = false;
                    }

                    /////////////RIGHT OCULUS TOUCH/////////////////////////////////////////////////////////////////////////////////////
                    if (percentXRight >= 0 && percentXRight <= SC_console_directx.D3D.SurfaceWidth && percentYRight >= 0 && percentYRight <= SC_console_directx.D3D.SurfaceHeight)
                    {
                        var absoluteMoveX = Convert.ToUInt32((percentXRight * 65535) / SC_console_directx.D3D.SurfaceWidth);
                        var absoluteMoveY = Convert.ToUInt32((percentYRight * 65535) / SC_console_directx.D3D.SurfaceHeight);

                        var yo = _updateFunctionStopwatchRight.Elapsed.Milliseconds;

                        if (yo >= 10)
                        {
                            //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, absoluteMoveX, absoluteMoveY, 0, 0);
                            _updateFunctionStopwatchRight.Stop();
                            _updateFunctionBoolRight = true;
                        }

                        //MOUSE DOUBLE CLICK LOGIC. IF the PLAYER clicked at one location then it stores the location, and if the player re-clicks inside of 20 frames, then click at the first click location.
                        //It's very basic, and at least I should implement also a certain "radius" of distance from the first click and the second click... If the second click is too far from the first click,
                        //then disregard the first click location.
                        if (buttonPressedOculusTouchRight != 0)
                        {
                            if (buttonPressedOculusTouchRight == 1)
                            {
                                if (hasClickedBUTTONA == 0)
                                {
                                    /*if (_frameCounterTouchRight <= 20 && _canResetCounterTouchRightButtonA == true)
                                    {
                                        mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, _lastMousePosXRight, _lastMousePosYRight, 0, 0);
                                        _frameCounterTouchRight = 0;
                                    }

                                    //mouse_event(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, absoluteMoveX, absoluteMoveY, 0, 0);
                                    //mouse_event(MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, absoluteMoveX, absoluteMoveY, 0, 0);

                                    SetCursorPos((int)absoluteMoveX, (int)absoluteMoveY);


                                    MainWindow.mousesim.LeftButtonDown();
                                    MainWindow.mousesim.LeftButtonUp();

                                    _lastMousePosXRight = absoluteMoveX;
                                    _lastMousePosYRight = absoluteMoveY;
                                    _canResetCounterTouchRightButtonA = true;
                                    hasClickedBUTTONA = 1;
                                }
                            }
                            else if (buttonPressedOculusTouchRight == 2)
                            {
                                if (hasClickedBUTTONB == false)
                                {
                                    //mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0);
                                    MainWindow.mousesim.RightButtonDown();
                                    hasClickedBUTTONB = true;
                                }
                            }
                        }
                    }

                    //////////OCULUS TOUCH BUTTONS PRESSED////////////////////////////////////////
                    if (hasClickedBUTTONA == 1 && buttonPressedOculusTouchRight == 0 || hasClickedBUTTONB && buttonPressedOculusTouchRight == 0)
                    {
                        if (hasClickedBUTTONA == 1 && buttonPressedOculusTouchRight == 0)
                        {
                            //mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                            hasClickedBUTTONA = 0;
                        }
                        else if (hasClickedBUTTONB && buttonPressedOculusTouchRight == 0)
                        {
                            //mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0);
                            MainWindow.mousesim.RightButtonUp();
                            hasClickedBUTTONB = false;
                        }
                    }

                    if (_canResetCounterTouchRightButtonA)
                    {
                        _frameCounterTouchRight++;
                    }

                    if (_frameCounterTouchRight >= 30)
                    {
                        _frameCounterTouchRight = 0;
                        _canResetCounterTouchRightButtonA = false;
                    }

                    
                    //////////OCULUS TOUCH BUTTONS NOT PRESSED////////////////////////////////////////
                    long test = 80;
                    /////////RIGHT THUMBSTICK///////////
                    var yo6 = _updateFunctionStopwatchRightThumbstickGoLeft.Elapsed.Milliseconds;
                    if (yo6 >= 75)
                    {
                        if (thumbStickRight[1].Y <= -0.1f && hasUsedThumbStickRightE == false)
                        {
                            //Console.WriteLine("test");
                            mouse_event(MOUSEEVENTF_WHEEL, 0, 0, -test, 0);
                            hasUsedThumbStickRightE = true;
                        }
                        else if (hasUsedThumbStickRightE)
                        {
                            hasUsedThumbStickRightE = false;
                        }
                        _updateFunctionStopwatchRightThumbstickGoLeft.Stop();
                        _updateFunctionBoolRightThumbStickGoLeft = true;
                    }
                    ///////////////////////////////////////////////////////////////////////////

                    /////////RIGHT THUMBSTICK/////////////////////////////////////////////////////
                    var yo7 = _updateFunctionStopwatchRightThumbstickGoRight.Elapsed.Milliseconds;
                    if (yo7 >= 75)
                    {
                        if (thumbStickRight[1].Y >= 0.1f && hasUsedThumbStickRightQ == false)
                        {
                            mouse_event(MOUSEEVENTF_WHEEL, 0, 0, test, 0);
                            //hasUsedThumbStickRightQ = true;
                        }
                        else if (hasUsedThumbStickRightQ)
                        {
                            hasUsedThumbStickRightQ = false;
                        }
                        _updateFunctionStopwatchRightThumbstickGoRight.Stop();
                        _updateFunctionBoolRightThumbStickGoRight = true;
                    }
                    /////////////RIGHT OCULUS TOUCH////////////////////////////////////////////
                }
            }
            catch (Exception ex)
            {
                //MessageBox((IntPtr)0, ex.ToString(), "mouse move", 0);
            }

           

            







            hasClickedBUTTONACounter++;
        }*/

        int hasClickedBUTTONA = 0;
        int hasClickedBUTTONACounter = 0;

        bool _startOnce02 = true;
        bool _updateFunctionBoolRight = true;
        bool _updateFunctionBoolLeft = true;
        bool _updateFunctionBoolLeftThumbStickGoLeft = true;
        bool _updateFunctionBoolLeftThumbStickGoRight = true;
        bool _updateFunctionBoolRightThumbStickGoLeft = true;
        bool _updateFunctionBoolRightThumbStickGoRight = true;
        bool _updateFunctionBoolLeftHandTrigger = true;
        bool _updateFunctionBoolRightHandTrigger = true;
        bool _updateFunctionBoolLeftIndexTrigger = true;
        bool _updateFunctionBoolRightIndexTrigger = true;
        bool _updateFunctionBoolTouchRightButtonA = true;
        bool _updateFunctionBoolLeftThumbStick = true;
        int _frameCounterTouchRight = 0;

        Plane planer;

        Vector3 centerPosRight;
        SharpDX.Ray someRay;
        Vector3 intersectPointRight;
        bool intersecter;

        Vector3 centerPosLeft;
        Vector3 rayDirLeft;
        SharpDX.Ray someRayLeft;
        Vector3 intersectPointLeft;
        bool intersecterLeft;
        Vector3 stabilizedIntersectionPosLeft;
        Vector3 stabilizedIntersectionPosRight;



        int currentFrameLeft = 0;
        int currentFrameRight = 0;
        double averageXRight = 0;
        double averageYRight = 0;
        double averageZRight = 0;
        double lastRightHitPointXFrameOne = 0;
        double lastRightHitPointYFrameOne = 0;
        double lastRightHitPointZFrameOne = 0;
        double positionXRight = 0;
        double positionYRight = 0;
        double positionZRight = 0;
        double averageXLeft = 0;
        double averageYLeft = 0;
        double averageZLeft = 0;
        double lastLeftHitPointXFrameOne = 0;
        double lastLeftHitPointYFrameOne = 0;
        double lastLeftHitPointZFrameOne = 0;
        double positionXLeft = 0;
        double positionYLeft = 0;
        double positionZLeft = 0;
        double differenceX = 0;
        double differenceY = 0;
        double differenceZ = 0;
        double percentXLeft;
        double percentYLeft;

        float widthLength;
        float heightLength;
        double currentPosWidth;
        double currentPosHeight;

        double percentXRight;
        double percentYRight;

        double currentX;
        double currentY;
        double currentZ;

        int _has_init_ray;
        JMatrix _last_frame_rigid_grab_rot;
        Vector3 _last_frame_rigid_grab_pos;
        Vector3 _last_frame_handPos = Vector3.Zero;
        Vector3 _last_final_hand_pos_right;

        const int _MaxArraySize0 = 10; //50
        const int _MaxArraySize1 = 9; //49

        //HERE IS THE MOUSE STABILIZER ARRAYS - THE BIGGER THE ARRAYS THE SLOWER AND MORE STABLE THE MOUSE IS ON THE SCREEN.
        Vector3[] arrayOfStabilizerPosRight = new Vector3[_MaxArraySize0];
        double[] arrayOfStabilizerPosXRight = new double[_MaxArraySize0];
        double[] arrayOfStabilizerPosDifferenceXRight = new double[_MaxArraySize1];
        double[] arrayOfStabilizerPosYRight = new double[_MaxArraySize0];
        double[] arrayOfStabilizerPosDifferenceYRight = new double[_MaxArraySize1];

        double[] arrayOfStabilizerPosZRight = new double[_MaxArraySize0];
        double[] arrayOfStabilizerPosDifferenceZRight = new double[_MaxArraySize1];



        Vector3[] arrayOfStabilizerPosLeft = new Vector3[_MaxArraySize0];
        double[] arrayOfStabilizerPosXLeft = new double[_MaxArraySize0];
        double[] arrayOfStabilizerPosDifferenceXLeft = new double[_MaxArraySize1];
        double[] arrayOfStabilizerPosYLeft = new double[_MaxArraySize0];
        double[] arrayOfStabilizerPosDifferenceYLeft = new double[_MaxArraySize1];

        double[] arrayOfStabilizerPosZLeft = new double[_MaxArraySize0];
        double[] arrayOfStabilizerPosDifferenceZLeft = new double[_MaxArraySize1];

        //
        Vector3[] _arrayOfStabilizerPosRight = new Vector3[_MaxArraySize0];
        double[] _arrayOfStabilizerPosXRight = new double[_MaxArraySize0];
        double[] _arrayOfStabilizerPosDifferenceXRight = new double[_MaxArraySize1];
        double[] _arrayOfStabilizerPosYRight = new double[_MaxArraySize0];
        double[] _arrayOfStabilizerPosDifferenceYRight = new double[_MaxArraySize1];

        Vector3[] _arrayOfStabilizerPosLeft = new Vector3[_MaxArraySize0];
        double[] _arrayOfStabilizerPosXLeft = new double[_MaxArraySize0];
        double[] _arrayOfStabilizerPosDifferenceXLeft = new double[_MaxArraySize1];
        double[] _arrayOfStabilizerPosYLeft = new double[_MaxArraySize0];
        double[] _arrayOfStabilizerPosDifferenceYLeft = new double[_MaxArraySize1];

        int _hasLockedMouse = 0;

        public const int KEY_W = 0x57;
        public const int KEY_A = 0x41;
        public const int KEY_S = 0x53;
        public const int KEY_D = 0x44;
        public const int KEY_SPACE = 0x20; //0x39
        public const int KEY_E = 0x45;
        public const int KEY_Q = 0x51;

        public const int KEYEVENTF_KEYUP = 0x0002;
        public const int KEYEVENTF_EXTENDEDKEY = 0x0001;

        const uint MOUSEEVENTF_ABSOLUTE = 0x8000;
        const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
        const uint MOUSEEVENTF_LEFTUP = 0x0004;
        const uint MOUSEEVENTF_MIDDLEDOWN = 0x0020;
        const uint MOUSEEVENTF_MIDDLEUP = 0x0040;
        const uint MOUSEEVENTF_MOVE = 0x0001;
        const uint MOUSEEVENTF_RIGHTDOWN = 0x0008;
        const uint MOUSEEVENTF_RIGHTUP = 0x0010;
        const uint MOUSEEVENTF_XDOWN = 0x0080;
        const uint MOUSEEVENTF_XUP = 0x0100;
        const uint MOUSEEVENTF_WHEEL = 0x0800;
        const uint MOUSEEVENTF_HWHEEL = 0x01000;
        [DllImport("user32.dll", CharSet = CharSet.Auto, CallingConvention = CallingConvention.StdCall, EntryPoint = "mouse_event")]
        public static extern void mouse_event(uint dwFlags, uint dx, uint dy, long dwData, uint dwExtraInfo);
        [DllImport("user32.dll", CharSet = CharSet.Auto)]
        static extern IntPtr SendMessage(IntPtr hWnd,
                                 UInt32 Msg,
                                 IntPtr wParam,
                                 IntPtr lParam);
        [DllImport("User32.dll")]
        private static extern bool SetCursorPos(int X, int Y);


        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool Wow64DisableWow64FsRedirection(ref IntPtr ptr);
        [DllImport("kernel32.dll", SetLastError = true)]
        public static extern bool Wow64RevertWow64FsRedirection(IntPtr ptr);
        private const UInt32 WM_SYSCOMMAND = 0x112;
        private const UInt32 SC_RESTORE = 0xf120;

        private const string OnScreenKeyboardExe = "osk.exe";

        private void ShowKeyboard()
        {
            var path64 = @"c:\windows\sysnative\osk.exe"; //@"C:\Windows\winsxs\amd64_microsoft-windows-osk_31bf3856ad364e35_6.1.7600.16385_none_06b1c513739fb828\osk.exe";
            var path32 = @"c:\windows\sysnative\osk.exe";// @"C:\windows\system32\osk.exe"; 
            var path = (Environment.Is64BitOperatingSystem) ? path64 : path32;
            Process.Start(path);
        }
        void StartOsk()
        {
            IntPtr ptr = new IntPtr(); ;
            bool sucessfullyDisabledWow64Redirect = false;

            // Disable x64 directory virtualization if we're on x64,
            // otherwise keyboard launch will fail.
            if (System.Environment.Is64BitOperatingSystem)
            {
                sucessfullyDisabledWow64Redirect = Wow64DisableWow64FsRedirection(ref ptr);
            }

            ProcessStartInfo psi = new ProcessStartInfo();
            psi.FileName = OnScreenKeyboardExe;
            // We must use ShellExecute to start osk from the current thread
            // with psi.UseShellExecute = false the CreateProcessWithLogon API 
            // would be used which handles process creation on a separate thread 
            // where the above call to Wow64DisableWow64FsRedirection would not 
            // have any effect.

            psi.UseShellExecute = true;
            psi.Verb = "runas";

            Process.Start(psi);

            // Re-enable directory virtualisation if it was disabled.
            if (System.Environment.Is64BitOperatingSystem)
                if (sucessfullyDisabledWow64Redirect)
                    Wow64RevertWow64FsRedirection(ptr);
        }










        SharpDX.Matrix _mouseCursorMatrix = SharpDX.Matrix.Identity;
        int _out_of_bounds_oculus_rift = 0;
        int _out_of_bounds_right = 0;
        int _out_of_bounds_left = 0;
        uint _lastMousePosXRight = 9999;
        uint _lastMousePosYRight = 9999;



        float _delta_timer_frame = 0;
        float _delta_timer_time = 0;
        DateTime time1;
        DateTime time2;
        float deltaTime;
        Stopwatch timeStopWatch00 = new Stopwatch();
        Stopwatch timeStopWatch01 = new Stopwatch();
        int _swtch = 0;
        int _swtch_counter_00 = 0;
        int _swtch_counter_01 = 0;
        int _swtch_counter_02 = 0;
        public async Task DoWork(int timeOut)
        {
            float startTime = (float)(timeStopWatch00.ElapsedMilliseconds);
        _threadLoop:

            if (_swtch == 0 || _swtch == 1)
            {
                if (_swtch == 0)
                {
                    if (_swtch_counter_00 >= 0)
                    {
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        //IMPORTANT PHYSICS TIME 
                        timeStopWatch00.Start();
                        time1 = DateTime.Now;
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        _swtch = 1;
                        _swtch_counter_00 = 0;
                    }
                }
                else if (_swtch == 1)
                {
                    if (_swtch_counter_01 >= 0)
                    {
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        timeStopWatch01.Start();
                        time2 = DateTime.Now;
                        ////////////////////
                        //UPGRADED DELTATIME
                        ////////////////////
                        _swtch = 2;
                        _swtch_counter_01 = 0;
                    }
                }
                else if (_swtch == 2)
                {

                }
            }

            /*//FRAME DELTATIME
            _delta_timer_frame = (float)Math.Abs((timeStopWatch01.Elapsed.Ticks - timeStopWatch00.Elapsed.Ticks)) * 100000000f;

            time2 = DateTime.Now;
            _delta_timer_time = (time2.Ticks - time1.Ticks) * 100000000f; //100000000f
            //time1 = time2;

            deltaTime = (float)Math.Abs(_delta_timer_time - _delta_timer_frame);
            */

            //FRAME DELTATIME
            _delta_timer_frame = (float)Math.Abs((timeStopWatch01.Elapsed.Ticks - timeStopWatch00.Elapsed.Ticks));

            time2 = DateTime.Now;
            _delta_timer_time = (time2.Ticks - time1.Ticks); //100000000f
            //time1 = time2;

            deltaTime = (float)Math.Abs(_delta_timer_frame - _delta_timer_time);

            //time1 = time2;
            await Task.Delay(1);
            Thread.Sleep(timeOut);
            _swtch_counter_00++;
            _swtch_counter_01++;
            _swtch_counter_02++;

            goto _threadLoop;
        }


    }
}





/*
Vector3 current_handposR = new Vector3(_player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M41, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M42, _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos.M43);

Matrix tempmatter = _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos;
Quaternion quater;
Quaternion.RotationMatrix(ref tempmatter, out quater);

var rayDirForward = sc_maths._getDirection(SharpDX.Vector3.ForwardRH, quater);
rayDirForward.Normalize();
var rayDirUp = sc_maths._getDirection(SharpDX.Vector3.Up, quater);
rayDirUp.Normalize();
var rayDirRight = sc_maths._getDirection(SharpDX.Vector3.Right, quater);
rayDirRight.Normalize();

Vector3 movingPointer = current_handposR + (-rayDirForward * _grab_rigid_data.dirDiffZ);
movingPointer = movingPointer + (rayDirRight * _grab_rigid_data.dirDiffX);
//movingPointer = movingPointer + (-rayDirUp * _grab_rigid_data.dirDiffY);

Matrix tempMat = _grab_rigid_data.position;// translationMatrix;
tempMat.M41 = 0;
tempMat.M42 = 0;
tempMat.M43 = 0;
tempMat.M44 = 1;

Quaternion.RotationMatrix(ref tempmatter, out quater);
JQuaternion _other_quatter = new JQuaternion(quater.X, quater.Y, quater.Z, quater.W);

Matrix anothertempmat = _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos;
anothertempmat.M41 = 0;
anothertempmat.M42 = 0;
anothertempmat.M43 = 0;
anothertempmat.M44 = 1;

var xq = _other_quatter.X;
var yq = _other_quatter.Y;
var zq = _other_quatter.Z;
var wq = _other_quatter.W;

var pitcha = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq);
var yawa = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq);
var rolla = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq);

double currentGrabrotDiffX = (pitcha - grabrotX);
double currentGrabrotDiffY = (yawa - grabrotY);
double currentGrabrotDiffZ =  (rolla - grabrotZ);

tempMatrix = SharpDX.Matrix.RotationYawPitchRoll((float)currentGrabrotDiffY, (float)currentGrabrotDiffX, (float)currentGrabrotDiffZ);

//_grab_rigid_data.position.Invert();

tempMatrix = _grab_rigid_data.position;// tempMatrix * _grab_rigid_data.position;


tempMatrix.M41 = movingPointer.X;
tempMatrix.M42 = movingPointer.Y;
tempMatrix.M43 = movingPointer.Z;

//anothertempmat.Invert();
//Matrix addMat = _grab_rigid_data.position;
//Matrix addresultMat;
//Matrix.Add(ref anothertempmat, ref addMat, out addresultMat);

Quaternion.RotationMatrix(ref tempMatrix, out quater);
body.Position = new JVector(movingPointer.X, movingPointer.Y, movingPointer.Z);
JQuaternion _other_quat = new JQuaternion(quater.X, quater.Y, quater.Z, quater.W);
var matrixIn = JMatrix.CreateFromQuaternion(_other_quat);
body.Orientation = matrixIn;*/





















/*
Matrix grabbedBodyMatrix = _grab_rigid_data.position;

var MOVINGPOINTER = new Vector3(_player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M41, _player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M42, _player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M43);
Matrix someMatRight = _rightTouchMatrix;
someMatRight.M41 = handPoseRight.Position.X + MOVINGPOINTER.X;
someMatRight.M42 = handPoseRight.Position.Y;// + MOVINGPOINTER.Y;
someMatRight.M43 = handPoseRight.Position.Z + MOVINGPOINTER.Z;
var diffNormPosX = (MOVINGPOINTER.X) - someMatRight.M41;
var diffNormPosY = (MOVINGPOINTER.Y) - someMatRight.M42;
var diffNormPosZ = (MOVINGPOINTER.Z) - someMatRight.M43;
MOVINGPOINTER = MOVINGPOINTER + -(current_rotation_of_torso_pivot_right* (diffNormPosX));
MOVINGPOINTER = MOVINGPOINTER + -(current_rotation_of_torso_pivot_up* (diffNormPosY));
MOVINGPOINTER = MOVINGPOINTER + (current_rotation_of_torso_pivot_forward* (diffNormPosZ));
Matrix finalHRMat = _rightTouchMatrix * OriginRot * RotatingMatrix * RotatingMatrixForPelvis;// ; //finalRotationMatrix
MOVINGPOINTER.X += SC_Update.OFFSETPOS.X;
MOVINGPOINTER.Y += SC_Update.OFFSETPOS.Y;
MOVINGPOINTER.Z += SC_Update.OFFSETPOS.Z;
Matrix handMatrix = _rightTouchMatrix;// _rightTouchMatrix * OriginRot * RotatingMatrix * RotatingMatrixForPelvis;
Quaternion quater;
Quaternion.RotationMatrix(ref handMatrix, out quater);
var xq = quater.X;
var yq = quater.Y;
var zq = quater.Z;
var wq = quater.W;
var pitchaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var yawaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var rollaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var pitchTouchR = (float)(Math.PI * (grabrotX - pitchaHand) / 180.0f);
var yawTouchR = (float)(Math.PI * (grabrotY - yawaHand) / 180.0f);
var rollTouchR = (float)(Math.PI * (grabrotZ - rollaHand) / 180.0f);
var rotatingMatrixForTouchR = SharpDX.Matrix.RotationYawPitchRoll(yawTouchR, pitchTouchR, rollTouchR);
var pitch = (float)(Math.PI * (SC_Update.RotationGrabbedX) / 180.0f);
var yaw = (float)(Math.PI * (SC_Update.RotationGrabbedY) / 180.0f);
var roll = (float)(Math.PI * (SC_Update.RotationGrabbedZ) / 180.0f);
var rotatingMatrixForGrabber = SharpDX.Matrix.RotationYawPitchRoll(yaw, pitch, roll);
var matrixerer = _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos * grabbedBodyMatrix; //rotatingMatrixForGrabber
matrixerer.M41 = MOVINGPOINTER.X;
matrixerer.M42 = MOVINGPOINTER.Y;
matrixerer.M43 = MOVINGPOINTER.Z;
matrixerer.M44 = 1;
body.Position = new JVector(MOVINGPOINTER.X, MOVINGPOINTER.Y, MOVINGPOINTER.Z);
Quaternion.RotationMatrix(ref matrixerer, out quater);
JQuaternion _other_quat = new JQuaternion(quater.X, quater.Y, quater.Z, quater.W);
var matrixIn = JMatrix.CreateFromQuaternion(_other_quat);
body.Orientation = matrixIn;
*/



/*
Matrix grabbedBodyMatrix = _grab_rigid_data.position;
var MOVINGPOINTER = new Vector3(_player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M41, _player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M42, _player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M43);
Matrix someMatRight = _rightTouchMatrix;
someMatRight.M41 = handPoseRight.Position.X + MOVINGPOINTER.X;
someMatRight.M42 = handPoseRight.Position.Y;// + MOVINGPOINTER.Y;
someMatRight.M43 = handPoseRight.Position.Z + MOVINGPOINTER.Z;
var diffNormPosX = (MOVINGPOINTER.X) - someMatRight.M41;
var diffNormPosY = (MOVINGPOINTER.Y) - someMatRight.M42;
var diffNormPosZ = (MOVINGPOINTER.Z) - someMatRight.M43;
MOVINGPOINTER = MOVINGPOINTER + -(current_rotation_of_torso_pivot_right * (diffNormPosX));
MOVINGPOINTER = MOVINGPOINTER + -(current_rotation_of_torso_pivot_up * (diffNormPosY));
MOVINGPOINTER = MOVINGPOINTER + (current_rotation_of_torso_pivot_forward * (diffNormPosZ));
Matrix finalHRMat = _rightTouchMatrix * OriginRot * RotatingMatrix * RotatingMatrixForPelvis;// ; //finalRotationMatrix
MOVINGPOINTER.X += SC_Update.OFFSETPOS.X;
MOVINGPOINTER.Y += SC_Update.OFFSETPOS.Y;
MOVINGPOINTER.Z += SC_Update.OFFSETPOS.Z;
Matrix handMatrix = _rightTouchMatrix;// _rightTouchMatrix * OriginRot * RotatingMatrix * RotatingMatrixForPelvis;
Quaternion quater;
Quaternion.RotationMatrix(ref handMatrix, out quater);
var xq = quater.X;
var yq = quater.Y;
var zq = quater.Z;
var wq = quater.W;
var pitchaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var yawaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var rollaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var pitchTouchR = (float)(Math.PI * (grabrotX - pitchaHand) / 180.0f);
var yawTouchR = (float)(Math.PI * (grabrotY - yawaHand) / 180.0f);
var rollTouchR = (float)(Math.PI * (grabrotZ - rollaHand) / 180.0f);
var rotatingMatrixForTouchR = SharpDX.Matrix.RotationYawPitchRoll(yawTouchR, pitchTouchR, rollTouchR);
var pitch = (float)(Math.PI * (SC_Update.RotationGrabbedX) / 180.0f);
var yaw = (float)(Math.PI * (SC_Update.RotationGrabbedY) / 180.0f);
var roll = (float)(Math.PI * (SC_Update.RotationGrabbedZ) / 180.0f);
var rotatingMatrixForGrabber = SharpDX.Matrix.RotationYawPitchRoll(yaw, pitch, roll);
var matrixerer = grabbedBodyMatrix * rotatingMatrixForGrabber* _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos; //rotatingMatrixForGrabber //_player_rght_hnd[0][0]._arrayOfInstances[0].current_pos * 
matrixerer.M41 = MOVINGPOINTER.X;
matrixerer.M42 = MOVINGPOINTER.Y;
matrixerer.M43 = MOVINGPOINTER.Z;
matrixerer.M44 = 1;
body.Position = new JVector(MOVINGPOINTER.X, MOVINGPOINTER.Y, MOVINGPOINTER.Z);
Quaternion.RotationMatrix(ref matrixerer, out quater);
JQuaternion _other_quat = new JQuaternion(quater.X, quater.Y, quater.Z, quater.W);
var matrixIn = JMatrix.CreateFromQuaternion(_other_quat);
body.Orientation = matrixIn;*/






/*
Matrix grabbedBodyMatrix = _grab_rigid_data.position;
Matrix handMatrix = _player_r_hand_grab[0][0]._arrayOfInstances[0]._TEMPPOSITION;// _rightTouchMatrix * OriginRot * RotatingMatrix * RotatingMatrixForPelvis * hmdmatrixRot_;
//_player_rght_hnd[0][0]._arrayOfInstances[0]._TEMPPOSITION; 

handMatrix.M41 = 0;
handMatrix.M42 = 0;
handMatrix.M43 = 0;
handMatrix.M44 = 1;

grabbedBodyMatrix.M41 = 0;
grabbedBodyMatrix.M42 = 0;
grabbedBodyMatrix.M43 = 0;
grabbedBodyMatrix.M44 = 1;

finalRotationMatrix.M41 = 0;
finalRotationMatrix.M42 = 0;
finalRotationMatrix.M43 = 0;
finalRotationMatrix.M44 = 1;

var MOVINGPOINTER = new Vector3(_player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M41, _player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M42, _player_torso[0][0]._arrayOfInstances[0]._ORIGINPOSITION.M43);
Matrix someMatRight = _rightTouchMatrix;// * OriginRot * RotatingMatrix * RotatingMatrixForPelvis * hmdmatrixRot_;
someMatRight.M41 = handPoseRight.Position.X + MOVINGPOINTER.X;
someMatRight.M42 = handPoseRight.Position.Y;// + MOVINGPOINTER.Y;
someMatRight.M43 = handPoseRight.Position.Z + MOVINGPOINTER.Z;
var diffNormPosX = (MOVINGPOINTER.X) - someMatRight.M41;
var diffNormPosY = (MOVINGPOINTER.Y) - someMatRight.M42;
var diffNormPosZ = (MOVINGPOINTER.Z) - someMatRight.M43;
MOVINGPOINTER = MOVINGPOINTER + -(current_rotation_of_torso_pivot_right* (diffNormPosX));
MOVINGPOINTER = MOVINGPOINTER + -(current_rotation_of_torso_pivot_up* (diffNormPosY));
MOVINGPOINTER = MOVINGPOINTER + (current_rotation_of_torso_pivot_forward* (diffNormPosZ));

MOVINGPOINTER.X += SC_Update.OFFSETPOS.X;
MOVINGPOINTER.Y += SC_Update.OFFSETPOS.Y;
MOVINGPOINTER.Z += SC_Update.OFFSETPOS.Z;

//Matrix matrixerer = _rightTouchMatrix;
//matrixerer.Invert();

Quaternion quater;
Quaternion.RotationMatrix(ref handMatrix, out quater);
var xq = quater.X;
var yq = quater.Y;
var zq = quater.Z;
var wq = quater.W;

var pitchaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var yawaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var rollaHand = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);

var pitchTouchR = (float)(Math.PI * (grabrotX - pitchaHand) / 180.0f);
var yawTouchR = (float)(Math.PI * (grabrotY - yawaHand) / 180.0f);
var rollTouchR = (float)(Math.PI * (grabrotZ - rollaHand) / 180.0f);

Matrix rotatingMatrixForTouchR = SharpDX.Matrix.RotationYawPitchRoll((float)yawTouchR, (float)pitchTouchR, (float)rollTouchR);
//Matrix rotatingMatrixForTouchR = Matrix.Scaling(1.0f) * Matrix.RotationX(pitchTouchR) * Matrix.RotationY(yawTouchR) * Matrix.RotationZ(rollTouchR);

var pitch = (float)(Math.PI * (-SC_Update.RotationGrabbedX) / 180.0f);
var yaw = (float)(Math.PI * (SC_Update.RotationGrabbedY) / 180.0f);
var roll = (float)(Math.PI * (SC_Update.RotationGrabbedZ) / 180.0f);

var rotatingMatrixForGrabber = SharpDX.Matrix.RotationYawPitchRoll(yaw, pitch, roll);




handMatrix = _player_r_hand_grab[0][0]._arrayOfInstances[0].current_pos;// * finalRotationMatrix;
//Quaternion quater;
Quaternion.RotationMatrix(ref handMatrix, out quater);

var rayDirForward = sc_maths._getDirection(SharpDX.Vector3.ForwardRH, quater);
rayDirForward.Normalize();
var rayDirUp = sc_maths._getDirection(SharpDX.Vector3.Up, quater);
rayDirUp.Normalize();
var rayDirRight = sc_maths._getDirection(SharpDX.Vector3.Right, quater);
rayDirRight.Normalize();

//handMatrix = _player_rght_hnd[0][0]._arrayOfInstances[0].current_pos;
var current_handposRR = new Vector3(MOVINGPOINTER.X,//_player_r_hand_grab[0][0]._arrayOfInstances[0].current_pos.M41,
MOVINGPOINTER.Y,//_player_r_hand_grab[0][0]._arrayOfInstances[0].current_pos.M42,
MOVINGPOINTER.Z);                //_player_r_hand_grab[0][0]._arrayOfInstances[0].current_pos.M43);

MOVINGPOINTER = current_handposRR + (rayDirForward* _grab_rigid_data.grabHitPointLength);
handMatrix = _player_r_hand_grab[0][0]._arrayOfInstances[0]._TEMPPOSITION* finalRotationMatrix;
var pitchTouchRer = (float)(Math.PI * ((float)SC_Update.RotationX4Pelvis) / 180.0f);
var yawTouchRer = (float)(Math.PI * ((float)SC_Update.RotationY4Pelvis) / 180.0f);
var rollTouchRer = (float)(Math.PI * ((float)SC_Update.RotationZ4Pelvis) / 180.0f);

var rotter = SharpDX.Matrix.RotationYawPitchRoll((float)yawTouchRer, (float)pitchTouchRer, (float)rollTouchRer);
Matrix matrixerer = handMatrix;
matrixerer.M41 = MOVINGPOINTER.X;
matrixerer.M42 = MOVINGPOINTER.Y;
matrixerer.M43 = MOVINGPOINTER.Z;
matrixerer.M44 = 1;

body.Position = new JVector(MOVINGPOINTER.X, MOVINGPOINTER.Y, MOVINGPOINTER.Z);
Quaternion.RotationMatrix(ref matrixerer, out quater);
JQuaternion _other_quat = new JQuaternion(quater.X, quater.Y, quater.Z, quater.W);
var matrixIn = JMatrix.CreateFromQuaternion(_other_quat);
body.Orientation = matrixIn;*/




//Matrix matrixerer = _rightTouchMatrix;
//matrixerer.Invert();





/*Quaternion.RotationMatrix(ref grabbedBodyMatrix, out quater);
xq = quater.X;
yq = quater.Y;
zq = quater.Z;
wq = quater.W;

var pitchaHand1 = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq) * (180 / Math.PI);
var yawaHand1 = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq)  * (180 / Math.PI);
var rollaHand1 = (float)Math.Atan2(2 * yq * wq - 2 * xq * zq, 1 - 2 * yq * yq - 2 * zq * zq)  * (180 / Math.PI);

var pitchTouchR = (float)(Math.PI * (pitchaHand1) / 180.0f); // - (grabrotX- pitchaHand)
var yawTouchR = (float)(Math.PI * (yawaHand1) / 180.0f);
var rollTouchR = (float)(Math.PI * (rollaHand1) / 180.0f); // - (grabrotZ - rollaHand)

Matrix rotatingMatrixForTouchR = SharpDX.Matrix.RotationYawPitchRoll((float)yawTouchR, (float)pitchTouchR, (float)rollTouchR);
//Matrix rotatingMatrixForTouchR = Matrix.Scaling(1.0f) * Matrix.RotationX(pitchTouchR) * Matrix.RotationY(yawTouchR) * Matrix.RotationZ(rollTouchR);
*/
//var pitch = (float)(Math.PI * (-SC_Update.RotationGrabbedX) / 180.0f);
//var yaw = (float)(Math.PI * (SC_Update.RotationGrabbedY) / 180.0f);
//var roll = (float)(Math.PI * (SC_Update.RotationGrabbedZ) / 180.0f);
//var rotatingMatrixForGrabber = SharpDX.Matrix.RotationYawPitchRoll(yaw, pitch, roll);




//handMatrix = _player_r_hand_grab[0][0]._arrayOfInstances[0].current_pos;// * finalRotationMatrix;
//Quaternion quater;






/*float timeSinceStart = (float)(DateTime.Now - SC_Update.startTime).TotalSeconds;


var pitcher = (float)(Math.PI * (pitchaHand - touchRX) / 180.0f);
var yawer = (float)(Math.PI * (yawaHand - touchRY) / 180.0f);
var roller = (float)(Math.PI * (rollaHand - touchRZ) / 180.0f);
var rotatingMatrixF = SharpDX.Matrix.RotationYawPitchRoll(yawer, pitcher, roller);

totalDiffX = pitcher;
totalDiffY = yawer;
totalDiffZ = roller;

//rotatingMatrixF *= RotatingMatrixForPelvis;

var pitch = (float)(Math.PI * (-SC_Update.RotationGrabbedX) / 180.0f);
var yaw = (float)(Math.PI * (SC_Update.RotationGrabbedY) / 180.0f);
var roll = (float)(Math.PI * (SC_Update.RotationGrabbedZ) / 180.0f);
var rotatingMatrixForGrabber = SharpDX.Matrix.RotationYawPitchRoll(yaw, pitch, roll);

///Quaternion.RotationMatrix(ref rotatingMatrixF, out quaterNion);

Quaternion.RotationMatrix(ref handMatrix, out quater);

var rayDirForwardGrab = sc_maths._getDirection(SharpDX.Vector3.ForwardRH, quater);
rayDirForwardGrab.Normalize();
var rayDirUpGrab = sc_maths._getDirection(SharpDX.Vector3.Up, quater);
rayDirUpGrab.Normalize();
var rayDirRightGrab = sc_maths._getDirection(SharpDX.Vector3.Right, quater);
rayDirRightGrab.Normalize();

Vector3 grabPos = new Vector3(grabbedBodyMatrix.M41,
grabbedBodyMatrix.M42,
grabbedBodyMatrix.M43);

Vector3 lookAt = Vector3.TransformCoordinate(Vector3.ForwardRH, rotatingMatrixHand);
Vector3 up = Vector3.TransformCoordinate(Vector3.Up, rotatingMatrixHand);

Quaternion currentRot;
Quaternion.RotationMatrix(ref grabbedBodyMatrix, out currentRot);
matrixerer = Matrix.Scaling(1.0f)* grabbedBodyMatrix * Matrix.RotationY(totalDiffY);
matrixerer.Invert();
*/
/*Quaternion quatYaw = new Quaternion(0, , 0,1);
quatYaw.Normalize();
//quatYaw *= currentRot;
//quatYaw.Normalize();
currentRot *= quatYaw;
currentRot.Normalize();


//Quaternion quatPitch = new Quaternion(pitchaHand, 0, 0, 1);
//currentRot *= quatPitch;
//currentRot.Normalize();
                                                                
Matrix.RotationQuaternion(ref currentRot, out matrixerer);*/

//Vector3 lookAt = Vector3.TransformCoordinate(rayDirForwardGrab, rotatingMatrixHand);
//Vector3 up = Vector3.TransformCoordinate(rayDirUpGrab, rotatingMatrixHand);

//Vector3 positionDisplacement = Vector3.TransformCoordinate(MOVINGPOINTER, rotatingMatrixHand);

// Finally create the view matrix from the three updated vectors.

//matrixerer = Matrix.LookAtRH(MOVINGPOINTER, MOVINGPOINTER + lookAt, up);
//matrixerer.Invert();

/*//https://stackoverflow.com/questions/29571093/sharpdx-vector3-transform-method-doesnt-seem-to-rotate-vector-correctly
Vector3 eyePos = MOVINGPOINTER;// new Vector3(0, 1, 0);
Vector3 target = MOVINGPOINTER + rayDirForward; //Vector3.Zero;



Quaternion lookAt = Quaternion.LookAtRH(eyePos, target, rayDirUp);
lookAt.Normalize();

Vector3 newForward = Vector3.Transform(rayDirForward, lookAt);
newForward.Normalize();

Vector3 newUp = Vector3.Transform(rayDirUp, lookAt);
newUp.Normalize();


//MOVINGPOINTER += newForward;

Matrix matrixerer = Matrix.LookAtRH(MOVINGPOINTER, MOVINGPOINTER + newForward, newUp);*/

//matrixerer *= _rightTouchMatrix * finalRotationMatrix;





//JUNK OF MICROSOFT NOT WORKING - the piece of shits of microsoft.
//https://stackoverflow.com/questions/2929255/unable-to-launch-onscreen-keyboard-osk-exe-from-a-32-bit-process-on-win7-x64
//https://www.dreamincode.net/forums/topic/174949-open-on-screen-keyboard-in-c%23/
/*Process[] p = Process.GetProcessesByName(Path.GetFileNameWithoutExtension(OnScreenKeyboardExe));

if (p.Length == 0)
{
    // we must start osk from an MTA thread
    if (Thread.CurrentThread.GetApartmentState() == ApartmentState.STA)
    {
        ThreadStart start = new ThreadStart(StartOsk);
        Thread thread = new Thread(start);
        thread.SetApartmentState(ApartmentState.MTA);
        thread.Start();
        thread.Join();
    }
    else
    {
        StartOsk();
    }
}
else
{
    // there might be a race condition if the process terminated 
    // meanwhile -> proper exception handling should be added
    //
    SendMessage(p[0].MainWindowHandle, WM_SYSCOMMAND, new IntPtr(SC_RESTORE), new IntPtr(0)); //MainWindowHandle
}*/


//StartOsk();

/*string windowsKeyboard = "osk";

foreach (Process clsProcess in Process.GetProcesses())
{
    if (clsProcess.ProcessName.ToLower().Contains(windowsKeyboard.ToLower()))
    {
        break;
    }
    else
    {
        Process proc = new Process();
        proc.StartInfo.FileName = windowsKeyboard + ".exe";
        proc.Start();
        break;
    }
}*/

/* ProcessStartInfo startInfo = new ProcessStartInfo();
 startInfo.CreateNoWindow = false;
 startInfo.UseShellExecute = true;
 startInfo.WorkingDirectory = @"c:\WINDOWS\system32\";
 startInfo.FileName = "osk.exe";
 startInfo.Verb = "runas";
 startInfo.WindowStyle = ProcessWindowStyle.Normal;

 try
 {
     using (Process process = Process.Start(startInfo))
     {
         process.WaitForExit();
     }
 }
 catch (Exception)
 {
     //throw;
 }*/



//System.Diagnostics.Process.Start("osk.exe");
/*string windir = Environment.GetEnvironmentVariable("windir");

Process p = new Process();
p.StartInfo.FileName = windir + @"\System32\cmd.exe";
p.StartInfo.Arguments = "/C " + windir + @"\System32\osk.exe";
p.StartInfo.CreateNoWindow = true;
p.StartInfo.UseShellExecute = false;
p.Start();
p.Dispose();*/
