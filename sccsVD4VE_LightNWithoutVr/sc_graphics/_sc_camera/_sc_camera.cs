using SharpDX;

namespace sccsVD4VE_LightNWithoutVr
{
    public class _sc_camera
    {
        private float PositionX { get; set; }
        private float PositionY { get; set; }
        private float PositionZ { get; set; }
        private float RotationX { get; set; }
        private float RotationY { get; set; }
        private float RotationZ { get; set; }
        public Matrix ViewMatrix { get; set; }
        public Matrix rotationMatrix { get;  set; }

        public _sc_camera() { }

        public Vector3 GetPosition()
        {
            return new Vector3(PositionX, PositionY, PositionZ);
        }

        public void SetPosition(float x, float y, float z)
        {
            PositionX = x;
            PositionY = y;
            PositionZ = z;
        }
        public Vector3 GetRotation()
        {
            return new Vector3(RotationX, RotationY, RotationZ);
        }

        public void SetRotation(float x, float y, float z)
        {
            RotationX = x;
            RotationY = y;
            RotationZ = z;
        }

        public void Render()
        {
            Vector3 position = new Vector3(PositionX, PositionY, PositionZ);
            Vector3 lookAt = new Vector3(0, 0, 1);
            float pitch = RotationX * 0.0174532925f;
            float yaw = RotationY * 0.0174532925f; ;
            float roll = RotationZ * 0.0174532925f; ;
            Matrix rotationMatrix = Matrix.RotationYawPitchRoll(yaw, pitch, roll);
            lookAt = Vector3.TransformCoordinate(lookAt, rotationMatrix);
            Vector3 up = Vector3.TransformCoordinate(Vector3.UnitY, rotationMatrix);
            lookAt = position + lookAt;
            ViewMatrix = Matrix.LookAtLH(position, lookAt, up);
        }
    }
}