using SharpDX;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using System;
using Jitter;
using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Collections;
using Jitter;
using Jitter.Dynamics;
using Jitter.Collision;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;

namespace SCCoreSystems.SC_Graphics
{
    public class SC_jitter_cloth : ITransform, IComponent
    {
        public ITransform transform { get; private set; }
        IComponent ITransform.Component
        {
            get => component;
        }
        IComponent component;
        RigidBody IComponent.rigidbody { get; set; }

        SoftBody IComponent.softbody { get; set; }

        public Matrix[] _WORLDMATRIXINSTANCES { get; set; }
        public int[] triangles;
        public float _total_torso_height = -1;
        public float _total_torso_depth = -1;
        public float _total_torso_width = -1;

        public SCCoreSystems.SC_Graphics.SC_jitter_cloth.DInstanceData[] instancesDataRIGHT { get; set; }
        public SCCoreSystems.SC_Graphics.SC_jitter_cloth.DInstanceData[] instancesDataUP { get; set; }

        public Matrix _POSITION { get; set; }

        public float RotationY { get; set; }
        public float RotationX { get; set; }
        public float RotationZ { get; set; }
        public int IndexCount { get; set; }
        public int VertexCount { get; set; }

        public SharpDX.Quaternion Rotation { get; set; }
        public DVertex[] Vertices { get; set; }

        [StructLayout(LayoutKind.Sequential)]
        public struct DVertex
        {
            public Vector3 position;
            public Vector2 texture;
            public Vector4 color;
            public Vector3 normal;
        };

        [StructLayout(LayoutKind.Sequential)]
        public struct DMatrixBuffer
        {
            public Matrix world;
            public Matrix view;
            public Matrix projection;
            public int swtch_texture;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct DInstanceType
        {
            public Vector4 position;
        };

        [StructLayout(LayoutKind.Sequential)]
        public struct DInstanceData
        {
            public Vector4 rotation;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct DInstanceDataMatrixRotter
        {
            public Vector4 instanceRot0;
            public Vector4 instanceRot1;
            public Vector4 instanceRot2;
            public Vector4 instanceRot3;
        }

        [StructLayout(LayoutKind.Explicit)]
        public struct DLightBuffer
        {
            [FieldOffset(0)]
            public Vector4 ambientColor;
            [FieldOffset(16)]
            public Vector4 diffuseColor;
            [FieldOffset(32)]
            public Vector3 lightDirection;
            [FieldOffset(44)]
            public float padding0;
            [FieldOffset(48)]
            public Vector3 lightPosition;
            [FieldOffset(60)]
            public float padding1;
        }

        public int InstanceCount { get; private set; }
        private SharpDX.Direct3D11.Buffer VertexBuffer { get; set; }
        private SharpDX.Direct3D11.Buffer IndexBuffer { get; set; }
        public SharpDX.Direct3D11.Buffer InstanceBuffer { get; set; }
        public SharpDX.Direct3D11.Buffer InstanceRotationBuffer { get; set; }
        public SharpDX.Direct3D11.Buffer InstanceRotationBufferRIGHT { get; set; }
        public SharpDX.Direct3D11.Buffer InstanceRotationBufferUP { get; set; }

        public SharpDX.Direct3D11.Buffer InstanceRotationMatrixBuffer { get; set; }

        float _tileSize = 0;
        int _divX;
        int _divY;

        float _a;
        float _r;
        float _g;
        float _b;

        private int m_TerrainWidth, m_TerrainHeight;

        public Vector4 _color;

        public SC_jitter_cloth_shader_final _this_object_texture_shader { get; set; }

        DLightBuffer[] _DLightBuffer = new DLightBuffer[1];

        public SC_jitter_cloth_instances[] _arrayOfInstances;// { get; set; }

        public DInstanceType[] instances { get; set; }

        public DInstanceData[] instancesDataForward { get; set; }

        public int _instX;
        public int _instY;
        public int _instZ;

        public Matrix _ORIGINPOSITION { get; set; }
        public SC_jitter_cloth_instances _singleObjectOnly;


        World _the_world;

        public bool _is_static;

        Vector4[][] _array_of_colors;
        int _addtoworld;

        float _mass;

        public SC_jitter_cloth() { }
        public SCCoreSystems.sc_console.SC_console_directx.BodyTag _tag;
        public bool Initialize(SCCoreSystems.sc_console.SC_console_directx D3D, int width, int height, float tileSize, int divX, int divY, float _sizeX, float _sizeY, float _sizeZ, Vector4 color, int instX, int instY, int instZ, IntPtr windowsHandle, Matrix matroxer, int isTerrain, float offsetPosX, float offsetPosY, float offsetPosZ, World the_world, SCCoreSystems.sc_console.SC_console_directx.BodyTag tag, bool is_static, int addtoworld, float mass, float vertoffsetx, float vertoffsety, float vertoffsetz)
        {

            this._mass = mass;

            _addtoworld = addtoworld;
            _is_static = is_static;

            _tag = tag;

            _the_world = the_world;
            _ORIGINPOSITION = matroxer;
            _POSITION = matroxer;

            transform = this;
            component = this;

            this._color = color;
            this._sizeX = _sizeX;
            this._sizeY = _sizeY;
            this._sizeZ = _sizeZ;

            _tileSize = tileSize;
            m_TerrainWidth = width;
            m_TerrainHeight = height;

            this._divX = divX;
            this._divY = divY;

            this._a = color.W;
            this._r = color.X;
            this._g = color.Y;
            this._b = color.Z;


            this._instX = instX;
            this._instX = instY;
            this._instX = instZ;

            if (!InitializeBuffer(D3D, _sizeX, _sizeY, _sizeZ, tileSize, instX, instY, instZ, windowsHandle, matroxer, isTerrain, offsetPosX, offsetPosY, offsetPosZ, vertoffsetx, vertoffsety, vertoffsetz))
                return false;

            return true;
        }



        SharpDX.Direct3D11.Buffer ConstantLightBuffer;

        private float _sizeX = 0;
        private float _sizeY = 0;
        private float _sizeZ = 0;

        public void ShutDown()
        {
            ShutDownBuffers();
        }
        private bool InitializeBuffer(SCCoreSystems.sc_console.SC_console_directx D3D, float _sizeX, float _sizeY, float _sizeZ, float tileSize, int instX, int instY, int instZ, IntPtr windowsHandle, Matrix matroxer, int type_of_cube, float offsetPosX, float offsetPosY, float offsetPosZ, float vertoffsetx, float vertoffsety, float vertoffsetz)
        {
            try
            {
                int sizeWidther = (int)(m_TerrainWidth * 0.5f);
                int sizeHeighter = (int)(m_TerrainHeight * 0.5f);

                sizeWidther /= 10;
                sizeHeighter /= 10;

                var someOffsetPos = new Vector3(offsetPosX, offsetPosY, offsetPosZ);

                Vertices = new[]
                                {                                   
                    //TOP
                    new DVertex()
                    {
                        position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                        texture = new Vector2(1, 1),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                    },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ),
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },







                     //BOTTOM
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         normal = new Vector3(1, 0, 1),
                         color = _color,
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },

                    //FACE NEAR
                    new DVertex() //12
                    {
                        position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                        texture = new Vector2(1, 1),
                        color = _color,
                        normal = new Vector3(1, 0, 0),
                    },
                     new DVertex() //13
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //14
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //15
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //16
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //17
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },



                     //FACE FAR
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ),
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },






                     //FACE LEFT
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ),
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },




                     //FACE RIGHT
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                 };

                _total_torso_width = ((1 * _sizeX) + (offsetPosX * _sizeX) * 2);
                _total_torso_height = ((1 * _sizeY) + (offsetPosY * _sizeY) * 2);
                _total_torso_depth = ((1 * _sizeZ) + (offsetPosZ * _sizeZ) * 2);


                triangles = new int[]
                {
                        5,4,3,2,1,0,
                        11,10,9,8,7,6,
                        17,16,15,14,13,12,
                        23,22,21,20,19,18,
                        29,28,27,26,25,24,
                        35,34,33,32,31,30,
                 };

                List<TriangleVertexIndices> indicess = new List<TriangleVertexIndices>();
                List<JVector> verticess = new List<JVector>();

                for (int i = 0; i < Vertices.Length; i++)
                {
                    verticess.Add(new JVector(Vertices[i].position.X, Vertices[i].position.Y, Vertices[i].position.Z));
                }

                if (triangles == null)
                {
                    MainWindow.MessageBox((IntPtr)0, "null _jitter_cloth.indices ", "sc core systems message", 0);
                }

                for (int i = 0; i < triangles.Length / 3; i++)
                {
                    //var one = indices[(i * 3) + 0];
                    //var two = indices[(i * 3) + 1];
                    //var three = indices[(i * 3) + 2];
                    indicess.Add(new TriangleVertexIndices(triangles[(i * 3) + 0], triangles[(i * 3) + 1], triangles[(i * 3) + 2]));
                }



                int count = 0;
                IndexCount = triangles.Length;
                VertexCount = Vertices.Length;

                instancesDataForward = new DInstanceData[instX * instY * instZ];
                instances = new DInstanceType[instX * instY * instZ];
                _arrayOfInstances = new SC_jitter_cloth_instances[instX * instY * instZ];
                instancesDataUP = new DInstanceData[instX * instY * instZ];
                instancesDataRIGHT = new DInstanceData[instX * instY * instZ];

                count = 0;
                for (int x = 0; x < instX; x++)
                {
                    for (int y = 0; y < instY; y++)
                    {
                        for (int z = 0; z < instZ; z++)
                        {

                            Vector3 position = new Vector3((x * _sizeX) + offsetPosX, (y * _sizeY) + offsetPosY, (z * _sizeZ) + offsetPosZ);
                            Matrix _tempMatrix = matroxer;
                            position.X += matroxer.M41;
                            position.Y += matroxer.M42;
                            position.Z += matroxer.M43;

                            instances[count] = new DInstanceType()
                            {
                                position = new Vector4(position.X, position.Y, position.Z, 1)
                            };

                            instancesDataForward[count] = new DInstanceData()
                            {
                                rotation = new Vector4(0, 0, 0, 1)
                            };

                            _tempMatrix.M41 = position.X;
                            _tempMatrix.M42 = position.Y;
                            _tempMatrix.M43 = position.Z;

                            SC_jitter_cloth_instances _jitter_cloth = new SC_jitter_cloth_instances();

                            _jitter_cloth.transform.Component.softbody = new SoftBody(indicess, verticess);
                            _jitter_cloth.transform.Component.softbody.Mass = 0.01f;
                            _jitter_cloth.transform.Component.softbody.TriangleExpansion = 0.010f;
                            _jitter_cloth.transform.Component.softbody.VertexExpansion = 0.01f;
                            _jitter_cloth.transform.Component.softbody.Tag = SCCoreSystems.sc_console.SC_console_directx.BodyTag.sc_jitter_cloth;

                            _jitter_cloth.transform.Component.softbody.Pressure = 15; //0.00075f
                            _jitter_cloth._POSITION = _tempMatrix;

                            _jitter_cloth.transform.Component.softbody.Material.KineticFriction = 10;
                            _jitter_cloth.transform.Component.softbody.Material.StaticFriction = 10;
                            _jitter_cloth.transform.Component.softbody.Material.Restitution = 0.015f;

                            _jitter_cloth.transform.Component.softbody.SetSpringValues(SoftBody.SpringType.EdgeSpring, 0.1f, 0.001f);
                            _jitter_cloth.transform.Component.softbody.SetSpringValues(SoftBody.SpringType.ShearSpring, 0.1f, 0.001f);
                            _jitter_cloth.transform.Component.softbody.SetSpringValues(SoftBody.SpringType.BendSpring, 0.1f, 0.001f);
                            //_jitter_cloth.transform.Component.softbody.SelfCollision = true;
                            var _vertexBodies = _jitter_cloth.transform.Component.softbody.VertexBodies;

                            for (int i = 0; i < _vertexBodies.Count; i++)
                            {
                                var singleVertexBody = _vertexBodies[i];
                                singleVertexBody.Mass = 0.5f;
                                //singleVertexBody.Position += new JVector(posX, posY, posZ);
                            }












                            /*_cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2 * (1 + (0.01f * tileSize)), _sizeY * 2 * (1 + (0.01f * tileSize)), _sizeZ * 2 * (1 + (0.01f * tileSize))));
                            _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                            _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                            _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                            _cube.transform.Component.rigidbody.IsStatic = _is_static;
                            _cube.transform.Component.rigidbody.Tag = _tag;// SC_console_directx.BodyTag._terrain;
                            _cube.transform.Component.rigidbody.Material.Restitution = 0.015f; //0.015f
                            _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f; // 0.55f
                            _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f; //0.55f

                            //_cube.transform.Component.rigidbody.Damping = RigidBody.DampingType.Linear;

                            _cube.transform.Component.rigidbody.Mass = _mass;*/
                            _jitter_cloth.current_pos = _tempMatrix;
                            _jitter_cloth._POSITION = _tempMatrix;
                            if (_addtoworld == 1)
                            {

                            }
                            _arrayOfInstances[count] = _jitter_cloth;
                            //_the_world.AddBody(_cube.transform.Component.rigidbody);
                            //_cube._POSITION = _tempMatrix;

                            //_singleObjectOnly = _cube;


                            count++;
                        }
                    }
                }

                InstanceCount = instances.Length;
                VertexBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.VertexBuffer, Vertices);
                IndexBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.IndexBuffer, triangles);
                InstanceBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.VertexBuffer, instances);
                InstanceRotationBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.VertexBuffer, instancesDataForward);
                InstanceRotationBufferRIGHT = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.VertexBuffer, instancesDataForward);
                InstanceRotationBufferUP = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.VertexBuffer, instancesDataForward);

                InstanceRotationMatrixBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.device, BindFlags.VertexBuffer, instancesDataForward);

                BufferDescription matrixBufferDescription = new BufferDescription()
                {
                    Usage = ResourceUsage.Dynamic,
                    SizeInBytes = Utilities.SizeOf<DInstanceType>() * instances.Length,
                    BindFlags = BindFlags.VertexBuffer,
                    CpuAccessFlags = CpuAccessFlags.Write,
                    OptionFlags = ResourceOptionFlags.None,
                    StructureByteStride = 0
                };

                InstanceBuffer = new SharpDX.Direct3D11.Buffer(D3D.device, matrixBufferDescription);

                matrixBufferDescription = new BufferDescription()
                {
                    Usage = ResourceUsage.Dynamic,
                    SizeInBytes = Utilities.SizeOf<DInstanceData>() * instancesDataForward.Length,
                    BindFlags = BindFlags.VertexBuffer,
                    CpuAccessFlags = CpuAccessFlags.Write,
                    OptionFlags = ResourceOptionFlags.None,
                    StructureByteStride = 0
                };

                InstanceRotationBuffer = new SharpDX.Direct3D11.Buffer(D3D.device, matrixBufferDescription);

                matrixBufferDescription = new BufferDescription()
                {
                    Usage = ResourceUsage.Dynamic,
                    SizeInBytes = Utilities.SizeOf<DInstanceData>() * instancesDataForward.Length,
                    BindFlags = BindFlags.VertexBuffer,
                    CpuAccessFlags = CpuAccessFlags.Write,
                    OptionFlags = ResourceOptionFlags.None,
                    StructureByteStride = 0
                };

                InstanceRotationBufferRIGHT = new SharpDX.Direct3D11.Buffer(D3D.device, matrixBufferDescription);

                matrixBufferDescription = new BufferDescription()
                {
                    Usage = ResourceUsage.Dynamic,
                    SizeInBytes = Utilities.SizeOf<DInstanceData>() * instancesDataForward.Length,
                    BindFlags = BindFlags.VertexBuffer,
                    CpuAccessFlags = CpuAccessFlags.Write,
                    OptionFlags = ResourceOptionFlags.None,
                    StructureByteStride = 0
                };

                InstanceRotationBufferUP = new SharpDX.Direct3D11.Buffer(D3D.device, matrixBufferDescription);
                return true;
            }
            catch
            {
                return false;
            }
        }
        private void ShutDownBuffers()
        {
            IndexBuffer?.Dispose();
            IndexBuffer = null;
            VertexBuffer?.Dispose();
            VertexBuffer = null;
        }
        public void Render(DeviceContext deviceContext)
        {
            RenderBuffers(deviceContext);
        }
        private void RenderBuffers(DeviceContext deviceContext)
        {
            deviceContext.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(VertexBuffer, Utilities.SizeOf<DVertex>(), 0));
            deviceContext.InputAssembler.SetVertexBuffers(1, new[]
            {
                new VertexBufferBinding(InstanceBuffer, Marshal.SizeOf(typeof(DInstanceType)),0),
            });
            deviceContext.InputAssembler.SetVertexBuffers(2, new[]
            {
                new VertexBufferBinding(InstanceRotationBuffer, Marshal.SizeOf(typeof(DInstanceData)),0),
            });
            deviceContext.InputAssembler.SetVertexBuffers(3, new[]
            {
                new VertexBufferBinding(InstanceRotationBufferRIGHT, Marshal.SizeOf(typeof(DInstanceData)),0),
            });
            deviceContext.InputAssembler.SetVertexBuffers(4, new[]
            {
                new VertexBufferBinding(InstanceRotationBufferUP, Marshal.SizeOf(typeof(DInstanceData)),0),
            });
            deviceContext.InputAssembler.SetIndexBuffer(IndexBuffer, SharpDX.DXGI.Format.R32_UInt, 0);
            deviceContext.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
        }
    }
}











/*
Vertices = new[]
                {                                   
                    //TOP
                     new DVertex()
{
    position = new Vector3(-1 * _sizeX, 1 * _sizeY, -1 * _sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
{
    position = new Vector3(-1 * _sizeX, 1 * _sizeY, 1 * _sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                    },
                      new DVertex()
{
    position = new Vector3(1 * _sizeX, 1 * _sizeY, -1 * _sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                    new DVertex()
{
    position = new Vector3(1 * _sizeX, 1 * _sizeY, 1 * _sizeZ) ,
                        texture = new Vector2(1, 1),
                        color = _color,
                        normal = new Vector3(0, 1, 1),         
                    },
                     
                    //BOTTOM
                    new DVertex()
{
    position = new Vector3(-1 * _sizeX, -1 * _sizeY, -1 * _sizeZ) ,
                        texture = new Vector2(0, 0),
                        color = _color,
                        normal = new Vector3(1, 0, 1),                  
                    },
                    new DVertex()
{
    position = new Vector3(-1 * _sizeX, -1 * _sizeY, 1 * _sizeZ) ,
                        texture = new Vector2(0, 1),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                    },
                    new DVertex()
{
    position = new Vector3(1 * _sizeX, -1 * _sizeY, -1 * _sizeZ) ,
                        texture = new Vector2(1, 0),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                    },
                    new DVertex()
{
    position = new Vector3(1 * _sizeX, -1 * _sizeY, 1 * _sizeZ) ,
                        texture = new Vector2(1, 1),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                    },

                    
                    //FACE NEAR //11 10 9 8
                    new DVertex()
{
    position = new Vector3(-1 * _sizeX, -1 * _sizeY, -1 * _sizeZ) ,
                           texture = new Vector2(0, 0),
                        color = _color,
                        normal = new Vector3(1, 0, 1),   
                     },
                      new DVertex()
{
    position = new Vector3(-1 * _sizeX, 1 * _sizeY, -1 * _sizeZ) ,
                         texture = new Vector2(0, 1),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                     },
                        new DVertex()
{
    position = new Vector3(1 * _sizeX, -1 * _sizeY, -1 * _sizeZ) ,
                        texture = new Vector2(1, 0),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                     },
                    new DVertex()
{
    position = new Vector3(1 * _sizeX, 1 * _sizeY, -1 * _sizeZ) ,
                         texture = new Vector2(1, 1),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                    },

                     
                     //FACE FAR
                     new DVertex()
{
    position = new Vector3(-1 * _sizeX, -1 * _sizeY, 1 * _sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),
                     },
                     new DVertex()
{
    position = new Vector3(-1 * _sizeX, 1 * _sizeY, 1 * _sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
{
    position = new Vector3(1 * _sizeX, -1 * _sizeY, 1 * _sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
{
    position = new Vector3(1 * _sizeX, 1 * _sizeY, 1 * _sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },





                     //FACE LEFT
                      new DVertex()
{
    position = new Vector3(-1 * _sizeX, -1 * _sizeY, -1 * _sizeZ),
                          texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),
                     },
                       new DVertex()
{
    position = new Vector3(-1 * _sizeX, -1 * _sizeY, 1 * _sizeZ) ,
                          texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                      new DVertex()
{
    position = new Vector3(-1 * _sizeX, 1 * _sizeY, -1 * _sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
{
    position = new Vector3(-1 * _sizeX, 1 * _sizeY, 1 * _sizeZ),
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },






                     //FACE RIGHT
                     new DVertex()
{
    position = new Vector3(1 * _sizeX, -1 * _sizeY, -1 * _sizeZ) ,
                          texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),
                     },
                     new DVertex()
{
    position = new Vector3(1 * _sizeX, -1 * _sizeY, 1 * _sizeZ) ,
                          texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
{
    position = new Vector3(1 * _sizeX, 1 * _sizeY, -1 * _sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
{
    position = new Vector3(1 * _sizeX, 1 * _sizeY, 1 * _sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },         
                 };

                _total_torso_width = ((1 * _sizeX) + (offsetPosX* _sizeX) * 2);
                _total_torso_height = ((1 * _sizeY) + (offsetPosY* _sizeY) * 2);
                _total_torso_depth = ((1 * _sizeZ) + (offsetPosZ* _sizeZ) * 2);






                int[] triangles = new int[]
                {
                    1,2,3,2,1,0,
                    4,5,6,7,6,5,
                    9,10,11,10,9,8,
                    12,13,14,15,14,13,
                    17,18,19,18,17,16,
                    20,21,22,23,22,21

                    //0,1,2,3,2,1
                    //8,9,10,11,10,9
                    5,4,3,2,1,0,
                    11,10,9,8,7,6,
                    17,16,15,14,13,12,
                    23,22,21,20,19,18,
                    29,28,27,26,25,24,
                    35,34,33,32,31,30,   
                 };*/













/*Vertices = new[]
            {                                   
                //TOP
                new DVertex()
                {
                    position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                    texture = new Vector2(1, 1),
                    color = _color,
                    normal = new Vector3(0, 1, 1),
                },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(1, 0),
                     color = _color,
                     normal = new Vector3(0, 1, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(0, 1, 1),

                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(0, 1, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(0, 1),
                     color = _color,
                     normal = new Vector3(0, 1, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ),
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(0, 1, 1),
                 },







                 //BOTTOM
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     normal = new Vector3(1, 0, 1),
                     color = _color,
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(1, 0),
                     color = _color,
                     normal = new Vector3(1, 0, 1),

                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(1, 0, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(1, 0, 1),

                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(0, 1),
                     color = _color,
                     normal = new Vector3(1, 0, 1),

                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(1, 0, 1),

                 },

                //FACE NEAR
                new DVertex()
                {
                    position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                    texture = new Vector2(1, 1),
                    color = _color,
                    normal = new Vector3(1, 0, 0),
                },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(1, 0),
                     color = _color,
                     normal = new Vector3(1, 0, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(1, 0, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 1),
                     color = _color,
                     normal = new Vector3(1, 0, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(1, 0, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(1, 0, 0),
                 },



                 //FACE FAR
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(0, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 0),
                     color = _color,
                     normal = new Vector3(0, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(0, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ),
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(0, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(0, 1),
                     color = _color,
                     normal = new Vector3(0, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ),
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(0, 1, 0),
                 },






                 //FACE LEFT
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ),
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(0, 0, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(1, 0),
                     color = _color,
                     normal = new Vector3(0, 0, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ),
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(0, 0, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(0, 0, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(0, 1),
                     color = _color,
                     normal = new Vector3(0, 0, 1),
                 },
                 new DVertex()
                 {
                     position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(0, 0, 1),
                 },




                 //FACE RIGHT
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(1, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(1, 0),
                     color = _color,
                     normal = new Vector3(1, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(1, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(1, 1),
                     color = _color,
                     normal = new Vector3(1, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ) ,
                     texture = new Vector2(0, 1),
                     color = _color,
                     normal = new Vector3(1, 1, 0),
                 },
                 new DVertex()
                 {
                     position = new Vector3(1*_sizeX, -1*_sizeY, -1*_sizeZ) ,
                     texture = new Vector2(0, 0),
                     color = _color,
                     normal = new Vector3(1, 1, 0),
                 },
             };
*/
