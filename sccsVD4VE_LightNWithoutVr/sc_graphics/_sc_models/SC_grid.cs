using SharpDX;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using System;
using Jitter;
using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using System.Runtime.InteropServices;

using Jitter.LinearMath;
using System.Collections;
using System.Collections.Generic;

namespace SCCoreSystems.SC_Graphics
{
    public class SC_grid : ITransform, IComponent
    {
        //public Jitter.Forces.Buoyancy _buo { get; set; }


        public ITransform transform { get; private set; }
        IComponent ITransform.Component
        {
            get => component;
        }
        IComponent component;
        RigidBody IComponent.rigidbody { get; set; }

        SoftBody IComponent.softbody { get; set; }

        public Matrix[] _WORLDMATRIXINSTANCES { get; set; }
        int[] triangles;
        public float _total_torso_height = -1;
        public float _total_torso_depth = -1;
        public float _total_torso_width = -1;

        public SCCoreSystems.SC_Graphics.SC_grid.DInstanceData[] instancesDataRIGHT { get; set; }
        public SCCoreSystems.SC_Graphics.SC_grid.DInstanceData[] instancesDataUP { get; set; }

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

        public SC_grid_shader_final _this_object_texture_shader { get; set; }

        DLightBuffer[] _DLightBuffer = new DLightBuffer[1];

        public SC_grid_instances[] _arrayOfInstances;// { get; set; }

        public DInstanceType[] instances { get; set; }

        public DInstanceData[] instancesDataForward { get; set; }

        public int _instX;
        public int _instY;
        public int _instZ;

        public Matrix _ORIGINPOSITION { get; set; }
        public SC_grid_instances _singleObjectOnly;


        World _the_world;

        public bool _is_static;

        Vector4[][] _array_of_colors;
        int _addtoworld;

        float _mass;

        public SC_grid() { }
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


        /*bool _set_fluid_point(ref JVector test)
        {
            //test = new JVector(5, 5, 5);
            test = new JVector(0, 0, 0);
            return _buo.FluidBox.Contains(ref test) != JBBox.ContainmentType.Disjoint;
        }*/


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

                // Calculate the number of Vertices in the terrain mesh.
                VertexCount = (m_TerrainWidth) * (m_TerrainHeight) * (m_TerrainWidth) * (m_TerrainHeight) * 8;
                // Set the index count to the same as the vertex count.
                IndexCount = VertexCount;

                // Create the vertex array.
                Vertices = new DVertex[VertexCount];
                // Create the index array.
                triangles = new int[IndexCount];

                // Initialize the index to the vertex array.
                int index = 0;

                // Load the vertex and index arrays with the terrain data.
                for (int j = -m_TerrainWidth; j < (m_TerrainWidth); j++)
                {
                    for (int i = -m_TerrainHeight; i < (m_TerrainHeight); i++)
                    {
                        // LINE 1
                        // Upper left.
                        float positionX = (float)i;
                        float positionZ = (float)(j + 1);
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;
                        // Upper right.
                        positionX = (float)(i + 1);
                        positionZ = (float)(j + 1);
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;

                        // LINE 2
                        // Upper right.
                        positionX = (float)(i + 1);
                        positionZ = (float)(j + 1);
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;
                        // Bottom right.
                        positionX = (float)(i + 1);
                        positionZ = (float)j;
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;

                        // LINE 3
                        // Bottom right.
                        positionX = (float)(i + 1);
                        positionZ = (float)j;
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;
                        // Bottom left.
                        positionX = (float)i;
                        positionZ = (float)j;
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;

                        // LINE 4
                        // Bottom left.
                        positionX = (float)i;
                        positionZ = (float)j;
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;
                        // Upper left.
                        positionX = (float)i;
                        positionZ = (float)(j + 1);
                        Vertices[index].position = new Vector3(positionX, 0.001f, positionZ) * _tileSize;
                        Vertices[index].color = _color;
                        triangles[index] = index;
                        index++;
                    }
                }












                /*
                Jitter.Forces.Buoyancy _buo = new Jitter.Forces.Buoyancy(_the_world);

                float _size__neg_x = -5;
                float _size__pos_x = 5;

                float _size__neg_y = 0;
                float _size__pos_y = 0.45f;

                float _size__neg_z = -5;
                float _size__pos_z = 5;

                JVector _min = new JVector(_size__neg_x, _size__neg_y, _size__neg_z);
                JVector _max = new JVector(_size__pos_x, _size__pos_y, _size__pos_z);

                JBBox _box = new JBBox(_min, _max);
                //_box.Min = new JVector(_size__neg_x, _size__neg_y, _size__neg_z);



                _box.AddPoint(new JVector(_size__neg_x, _size__neg_y, _size__neg_z));
                _box.AddPoint(new JVector(_size__pos_x, _size__neg_y, _size__neg_z));
                _box.AddPoint(new JVector(_size__neg_x, _size__neg_y, _size__pos_z));
                _box.AddPoint(new JVector(_size__pos_x, _size__neg_y, _size__pos_z));

                _box.AddPoint(new JVector(_size__neg_x, _size__pos_y, _size__neg_z));
                _box.AddPoint(new JVector(_size__pos_x, _size__pos_y, _size__neg_z));
                _box.AddPoint(new JVector(_size__neg_x, _size__pos_y, _size__pos_z));
                _box.AddPoint(new JVector(_size__pos_x, _size__pos_y, _size__pos_z));

                _buo.FluidBox = _box;

                //_buo.UseOwnFluidArea
                //Action _action = new Action();
                //JVector _new_vec = new JVector(0,0,0);
                //var refreshDXEngineAction = new Action(() =>
                //{
                //    _set_fluid_point(ref _new_vec);
                //});

                Jitter.Forces.Buoyancy.DefineFluidArea test = new Jitter.Forces.Buoyancy.DefineFluidArea(ref _set_fluid_point);
                _buo.UseOwnFluidArea(test);



                //_buo.FluidBox = JBBox.LargeBox;
                _buo.Density = 2.0f;
                _buo.Damping = 0.75f;*/








                int count = 0;
                //IndexCount = triangles.Length;
                //VertexCount = Vertices.Length;

                instancesDataForward = new DInstanceData[instX * instY * instZ];
                instances = new DInstanceType[instX * instY * instZ];
                _arrayOfInstances = new SC_grid_instances[instX * instY * instZ];
                instancesDataUP = new DInstanceData[instX * instY * instZ];
                instancesDataRIGHT = new DInstanceData[instX * instY * instZ];

                count = 0;
                for (int x = 0; x < instX; x++)
                {
                    for (int y = 0; y < instY; y++)
                    {
                        for (int z = 0; z < instZ; z++)
                        {
                            Vector3 position = new Vector3(x * offsetPosX, y * offsetPosY, z * offsetPosZ);
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

                            SC_grid_instances _cube = new SC_grid_instances();
                            if (_addtoworld == 1)
                            {
                                _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2 * 1.01f, _sizeY * 2 * 1.01f, _sizeZ * 2 * 1.01f));
                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = _is_static;
                                _cube.transform.Component.rigidbody.Tag = _tag;//SC_console_directx.BodyTag.physicsInstancedCube;

                                _cube.transform.Component.rigidbody.Material.Restitution = 0.015f; //0.015f
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f; // 0.55f
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f; //0.55f
                                                                                                      // _cube.transform.Component.rigidbody.Damping = RigidBody.DampingType.Linear;
                                _cube.transform.Component.rigidbody.AllowDeactivation = false;
                                _cube.transform.Component.rigidbody.Mass = _mass;

             
                                //SC_Console_GRAPHICS._buo.Add(_cube.transform.Component.rigidbody, 3);
                                _the_world.AddBody(_cube.transform.Component.rigidbody);
                            }

                            _cube._POSITION = _tempMatrix;
                            _cube.current_pos = _tempMatrix;
                            _arrayOfInstances[count] = _cube;
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
            deviceContext.InputAssembler.PrimitiveTopology = PrimitiveTopology.LineList;
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
