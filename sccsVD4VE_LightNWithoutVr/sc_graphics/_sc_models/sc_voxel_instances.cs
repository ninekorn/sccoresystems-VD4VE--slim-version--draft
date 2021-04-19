using SharpDX;
using Jitter.Dynamics;
namespace SCCoreSystems.SC_Graphics
{
    public class sc_voxel_instances : ITransform, IComponent
    {
        public ITransform transform { get; private set; }
        IComponent ITransform.Component
        {
            get => component;
        }
        IComponent component;
        RigidBody IComponent.rigidbody { get; set; }

        SoftBody IComponent.softbody { get; set; }

        public Matrix _TEMPPIVOT { get; set; }
        public Matrix _TEMPPOSITION { get; set; }
        public Matrix _REALCENTERPOSITION { get; set; }

        public Matrix _POSITION { get; set; }
        public Matrix _ORIGINPOSITION { get; set; }
        public Matrix _LASTPOSITION { get; set; }
        public Matrix _LASTPOSITIONFORPHYSICS { get; set; }
        public Vector3 _ELBOWPOSITION { get; set; }
        public Vector3 _ELBOWCROSSVEC { get; set; }

        public Vector3 _UPPERPIVOT { get; set; }
        public Vector3 _LOWERPIVOT { get; set; }


        public Matrix _SHOULDERROT { get; set; }
        public float _ARMLENGTH { get; set; }

        public Matrix current_pos { get; set; }
        public Vector3 sc_last_frame_linearvelocity { get; set; }
        public Vector3 sc_last_frame_angularvelocity { get; set; }

        public sc_voxel_instances()
        {
            transform = this;
            component = this;
        }
    }
}