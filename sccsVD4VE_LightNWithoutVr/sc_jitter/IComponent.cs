using Jitter.Dynamics;
namespace sccsVD4VE_LightNWithoutVr
{
    public interface IComponent
    {
        RigidBody rigidbody { get; set; }
        SoftBody softbody { get; set; }
    }
}
