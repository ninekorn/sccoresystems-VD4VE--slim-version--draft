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
    public class SC_cube : ITransform, IComponent
    {
        /// <summary>
        /// Helper method creates a triangle fan to close the ends of the cylinder.
        /// </summary>
        void CreateCap(int tessellation, float height, float radius, Vector3 normal)
        {

        }
        /// <summary>
        /// Helper method computes a point on a circle.
        /// </summary>
        Vector3 GetCircleVector(int i, int tessellation)
        {
            float angle = i * TWOPI / tessellation;

            float dx = (float)Math.Cos(angle);
            float dz = (float)Math.Sin(angle);

            return new Vector3(dx, 0, dz);
        }



        //public Jitter.Forces.Buoyancy _buo { get; set; }
        float PIovertwo = 1.57079632679f;
        float TWOPI = 6.28318530718f;


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

        public SCCoreSystems.SC_Graphics.SC_cube.DInstanceData[] instancesDataRIGHT { get; set; }
        public SCCoreSystems.SC_Graphics.SC_cube.DInstanceData[] instancesDataUP { get; set; }

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

        public SC_cube_shader_final _this_object_texture_shader { get; set; }

        DLightBuffer[] _DLightBuffer = new DLightBuffer[1];

        public SC_cube_instances[] _arrayOfInstances;// { get; set; }

        public DInstanceType[] instances { get; set; }

        public DInstanceData[] instancesDataForward { get; set; }

        public int _instX;
        public int _instY;
        public int _instZ;

        public Matrix _ORIGINPOSITION { get; set; }
        public SC_cube_instances _singleObjectOnly;


        World _the_world;

        public bool _is_static;

        Vector4[][] _array_of_colors;
        int _addtoworld;

        float _mass;

        public SC_cube() { }
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
                int tessellation = 3;
                float diameter = 5;
                float length = 5;
                float radius = 1;
                float height = 5;

                int sizeWidther = (int)(m_TerrainWidth * 0.5f);
                int sizeHeighter = (int)(m_TerrainHeight * 0.5f);

                sizeWidther /= 10;
                sizeHeighter /= 10;

                var someOffsetPos = new Vector3(offsetPosX, offsetPosY, offsetPosZ);

                if (type_of_cube == 0 || type_of_cube == 2 || type_of_cube == 3 || type_of_cube == 4 || type_of_cube == 5)
                {
                    Vertices = new[]
                    {                                   
                    //TOP
                    new DVertex()
                    {
                        position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                        texture = new Vector2(0, 0),
                        color = _color,
                        normal = new Vector3(0, 1, 1),
                    },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY,  (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY,  (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3( (-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 1),
                     },







                     //BOTTOM
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         normal = new Vector3(1, 0, 1),
                         color = _color,
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 1),

                     },

                    //FACE NEAR
                    new DVertex() //12
                    {
                        position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                        texture = new Vector2(1, 1),
                        color = _color,
                        normal = new Vector3(1, 0, 0),
                    },
                     new DVertex() //13
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(1, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //14
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //15
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //16
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(1, 1),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },
                     new DVertex() //17
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX,(-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 0, 0),
                     },



                     //FACE FAR
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY,(1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 1, 0),
                     },






                     //FACE LEFT
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ),
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY,(1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },
                     new DVertex()
                     {
                         position = new Vector3((-1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(0, 0, 1),
                     },




                     //FACE RIGHT
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (1+vertoffsetz)*_sizeZ) ,
                         texture = new Vector2(0, 0),
                         color = _color,
                         normal = new Vector3(1, 1, 0),
                     },
                     new DVertex()
                     {
                         position = new Vector3((1+vertoffsetx)*_sizeX, (-1+vertoffsety)*_sizeY, (-1+vertoffsetz)*_sizeZ) ,
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
                }
                else if (type_of_cube == 1) // JITTER SPHERE
                {
                    //tessellation = 10;

                    int verticalSegments = tessellation;
                    int horizontalSegments = tessellation * 2;
                    diameter = tileSize * 0.5f;

                    radius = diameter / 2;
                    List<DVertex> list_verts = new List<DVertex>();
                    List<Vector3> list_normals = new List<Vector3>();
                    List<int> list_triangles = new List<int>();

                    // Start with a single vertex at the bottom of the sphere.

                    DVertex vertex = new DVertex()
                    {
                        position = Vector3.Down * radius,
                        texture = new Vector2(0, 1),
                        color = _color,
                        normal = Vector3.Down,
                    };
                    list_verts.Add(vertex);



                    // Create rings of vertices at progressively higher latitudes.
                    for (int i = 0; i < verticalSegments - 1; i++)
                    {
                        float latitude = (float)(((i + 1) * Math.PI / verticalSegments) - PIovertwo);

                        float dy = (float)Math.Sin(latitude);
                        float dxz = (float)Math.Cos(latitude);

                        // Create a single ring of vertices at this latitude.
                        for (int j = 0; j < horizontalSegments; j++)
                        {
                            float longitude = j * TWOPI / horizontalSegments;

                            float dx = (float)Math.Cos(longitude) * dxz;
                            float dz = (float)Math.Sin(longitude) * dxz;

                            Vector3 normal = new Vector3(dx, dy, dz);

                            vertex = new DVertex()
                            {
                                position = normal * radius,
                                texture = new Vector2(0, 1),
                                color = _color,
                                normal = normal,
                            };
                            list_verts.Add(vertex);
                        }
                    }

                    // Finish with a single vertex at the top of the sphere.
                    vertex = new DVertex()
                    {
                        position = Vector3.Up * radius,
                        texture = new Vector2(0, 1),
                        color = _color,
                        normal = Vector3.Up,
                    };
                    list_verts.Add(vertex);


                    // Create a fan connecting the bottom vertex to the bottom latitude ring.
                    for (int i = 0; i < horizontalSegments; i++)
                    {
                        list_triangles.Add(0);
                        list_triangles.Add(1 + (i + 1) % horizontalSegments);
                        list_triangles.Add(1 + i);
                    }

                    // Fill the sphere body with triangles joining each pair of latitude rings.
                    for (int i = 0; i < verticalSegments - 2; i++)
                    {
                        for (int j = 0; j < horizontalSegments; j++)
                        {
                            int nextI = i + 1;
                            int nextJ = (j + 1) % horizontalSegments;

                            list_triangles.Add(1 + i * horizontalSegments + j);
                            list_triangles.Add(1 + i * horizontalSegments + nextJ);
                            list_triangles.Add(1 + nextI * horizontalSegments + j);

                            list_triangles.Add(1 + i * horizontalSegments + nextJ);
                            list_triangles.Add(1 + nextI * horizontalSegments + nextJ);
                            list_triangles.Add(1 + nextI * horizontalSegments + j);
                        }
                    }

                    // Create a fan connecting the top vertex to the top latitude ring.
                    for (int i = 0; i < horizontalSegments; i++)
                    {
                        list_triangles.Add(list_verts.Count - 1);
                        list_triangles.Add(list_verts.Count - 2 - (i + 1) % horizontalSegments);
                        list_triangles.Add(list_verts.Count - 2 - i);
                    }



                    Vertices = list_verts.ToArray();
                    triangles = list_triangles.ToArray();

                }
                else if (type_of_cube == 6) // CAPSULE
                {
                    int vertIndex = 0;

                    //tessellation = 10;
                    diameter = tileSize * 0.5f;
                    length = _sizeX * 40 * 0.5f;

                    int verticalSegments = tessellation;
                    int horizontalSegments = tessellation * 2;

                    radius = diameter / 2;

                    List<DVertex> list_verts = new List<DVertex>();
                    List<Vector3> list_normals = new List<Vector3>();
                    List<int> list_triangles = new List<int>();

                    DVertex vertex = new DVertex()
                    {
                        position = (Vector3.Down * tileSize) * radius + (Vector3.Down * tileSize) * 0.5f * length,
                        texture = new Vector2(0, 1),
                        color = _color,
                        normal = Vector3.Down,
                    };
                    list_verts.Add(vertex);

                    // Create rings of vertices at progressively higher latitudes.
                    for (int i = 0; i < verticalSegments - 1; i++)
                    {
                        float latitude = (float)(((i + 1) * Math.PI / verticalSegments) - PIovertwo);
                        float dy = (float)Math.Sin(latitude);
                        float dxz = (float)Math.Cos(latitude);

                        bool bla = false;

                        if (i > (verticalSegments - 2) / 2)
                        {
                            bla = true;
                        }

                        // Create a single ring of vertices at this latitude.
                        for (int j = 0; j < horizontalSegments; j++)
                        {
                            float longitude = j * TWOPI / horizontalSegments;

                            float dx = (float)Math.Cos(longitude) * dxz;
                            float dz = (float)Math.Sin(longitude) * dxz;

                            Vector3 normal = new Vector3(dx, dy, dz);
                            Vector3 position = normal * radius;

                            if (bla) position += (Vector3.Up * tileSize) * 0.5f * length;
                            else position += (Vector3.Down * tileSize) * 0.5f * length;

                            //AddVertex(position, normal);
                            vertex = new DVertex()
                            {
                                position = position,
                                texture = new Vector2(0, 1),
                                color = _color,
                                normal = normal,
                            };
                            list_verts.Add(vertex);
                        }
                    }

                    // Finish with a single vertex at the top of the sphere.
                    //AddVertex(, );
                    vertex = new DVertex()
                    {
                        position = (Vector3.Up * tileSize) * radius + (Vector3.Up * tileSize) * 0.5f * length,
                        texture = new Vector2(0, 1),
                        color = _color,
                        normal = Vector3.Up,
                    };
                    list_verts.Add(vertex);

                    // Create a fan connecting the bottom vertex to the bottom latitude ring.
                    for (int i = 0; i < horizontalSegments; i++)
                    {
                        list_triangles.Add(0);
                        list_triangles.Add(1 + (i + 1) % horizontalSegments);
                        list_triangles.Add(1 + i);
                        //AddIndex(0);
                        //AddIndex(1 + (i + 1) % horizontalSegments);
                        //AddIndex(1 + i);
                    }

                    // Fill the sphere body with triangles joining each pair of latitude rings.
                    for (int i = 0; i < verticalSegments - 2; i++)
                    {
                        for (int j = 0; j < horizontalSegments; j++)
                        {
                            int nextI = i + 1;
                            int nextJ = (j + 1) % horizontalSegments;

                            list_triangles.Add(1 + i * horizontalSegments + j);
                            list_triangles.Add(1 + i * horizontalSegments + nextJ);
                            list_triangles.Add(1 + nextI * horizontalSegments + j);

                            list_triangles.Add(1 + i * horizontalSegments + nextJ);
                            list_triangles.Add(1 + nextI * horizontalSegments + nextJ);
                            list_triangles.Add(1 + nextI * horizontalSegments + j);
                        }
                    }

                    // Create a fan connecting the top vertex to the top latitude ring.
                    for (int i = 0; i < horizontalSegments; i++)
                    {
                        list_triangles.Add((list_verts.Count) - 1);
                        list_triangles.Add((list_verts.Count) - 2 - (i + 1) % horizontalSegments);
                        list_triangles.Add((list_verts.Count) - 2 - i);

                    }

                    Vertices = list_verts.ToArray();
                    triangles = list_triangles.ToArray();

                }
                else if (type_of_cube == 7) // CONE
                {
                    List<DVertex> list_verts = new List<DVertex>();
                    List<Vector3> list_normals = new List<Vector3>();
                    List<int> list_triangles = new List<int>();

                    //tessellation = 3;
                    diameter = tileSize * 0.5f;
                    height = _sizeX * 5 * 0.5f;
                    radius = diameter * 0.5f;


                    // Create a ring of triangles around the outside of the cylinder.
                    //AddVertex(Vector3.Up * (2.0f / 3.0f) * height, Vector3.Up);
                    DVertex vertex = new DVertex()
                    {
                        position = Vector3.Up * (2.0f / 3.0f) * height,
                        texture = new Vector2(0, 0),
                        color = _color,
                        normal = Vector3.Up,
                    };
                    list_verts.Add(vertex);



                    for (int i = 0; i < tessellation; i++)
                    {
                        Vector3 normal = GetCircleVector(i, tessellation);
                        //AddVertex(normal * radius + (1.0f / 3.0f) * height * Vector3.Down, normal);
                        vertex = new DVertex()
                        {
                            position = normal * radius + (1.0f / 3.0f) * height * Vector3.Down,
                            texture = new Vector2(0, 1),
                            color = _color,
                            normal = normal,
                        };
                        list_verts.Add(vertex);

                        list_triangles.Add(0);
                        list_triangles.Add(i);
                        list_triangles.Add(i + 1);
                    }

                    list_triangles.Add(0);
                    list_triangles.Add(tessellation);
                    list_triangles.Add(1);




                    height = (1.0f / 3.0f) * height;

                    var norm = Vector3.Down;
                    //CreateCap(tessellation, (1.0f / 3.0f) * height, radius, Vector3.Down);
                    // Create cap indices.
                    for (int i = 0; i < tessellation - 2; i++)
                    {
                        if (norm.Y > 0)
                        {
                            list_triangles.Add(list_verts.Count);
                            list_triangles.Add((list_verts.Count) + (i + 1) % tessellation);
                            list_triangles.Add((list_verts.Count) + (i + 2) % tessellation);

                        }
                        else
                        {
                            list_triangles.Add(list_verts.Count);
                            list_triangles.Add((list_verts.Count) + (i + 2) % tessellation);
                            list_triangles.Add((list_verts.Count) + (i + 1) % tessellation);
                        }
                    }

                    // Create cap vertices.
                    for (int i = 0; i < tessellation; i++)
                    {
                        Vector3 position = GetCircleVector(i, tessellation) * radius + norm * height;

                        //AddVertex(position, norm);

                        vertex = new DVertex()
                        {
                            position = position,
                            texture = new Vector2(0, 0),
                            color = _color,
                            normal = norm,
                        };
                        list_verts.Add(vertex);
                    }

                    Vertices = list_verts.ToArray();
                    triangles = list_triangles.ToArray();
                }
                else if (type_of_cube == 8) //CYLINDER
                {
                    //tessellation = 10;
                    diameter = tileSize * 0.5f;
                    height = _sizeX * 5 * 0.5f;
                    radius = diameter * 0.5f;
                    //if (tessellation < 3) //throw error

                    List<DVertex> list_verts = new List<DVertex>();
                    List<Vector3> list_normals = new List<Vector3>();
                    List<int> list_triangles = new List<int>();


                    height /= 2;
                    DVertex vertex;
                    // Create a ring of triangles around the outside of the cylinder.
                    for (int i = 0; i < tessellation; i++)
                    {
                        Vector3 normal = GetCircleVector(i, tessellation);

                        vertex = new DVertex()
                        {
                            position = normal * radius + Vector3.Up * height,
                            texture = new Vector2(0, 1),
                            color = _color,
                            normal = normal,
                        };
                        list_verts.Add(vertex);

                        vertex = new DVertex()
                        {
                            position = normal * radius + Vector3.Down * height,
                            texture = new Vector2(0, 1),
                            color = _color,
                            normal = normal,
                        };
                        list_verts.Add(vertex);

                        list_triangles.Add(i * 2);
                        list_triangles.Add(i * 2 + 1);
                        list_triangles.Add((i * 2 + 2) % (tessellation * 2));

                        list_triangles.Add(i * 2 + 1);
                        list_triangles.Add((i * 2 + 3) % (tessellation * 2));
                        list_triangles.Add((i * 2 + 2) % (tessellation * 2));
                    }



                    var norm = Vector3.Down;
                    for (int i = 0; i < tessellation - 2; i++)
                    {
                        if (norm.Y > 0)
                        {
                            list_triangles.Add(list_verts.Count);
                            list_triangles.Add((list_verts.Count) + (i + 1) % tessellation);
                            list_triangles.Add((list_verts.Count) + (i + 2) % tessellation);

                        }
                        else
                        {
                            list_triangles.Add(list_verts.Count);
                            list_triangles.Add((list_verts.Count) + (i + 2) % tessellation);
                            list_triangles.Add((list_verts.Count) + (i + 1) % tessellation);
                        }
                    }

                    // Create cap vertices.
                    for (int i = 0; i < tessellation; i++)
                    {
                        Vector3 position = GetCircleVector(i, tessellation) * radius + norm * height;

                        //AddVertex(position, norm);

                        vertex = new DVertex()
                        {
                            position = position,
                            texture = new Vector2(0, 0),
                            color = _color,
                            normal = norm,
                        };
                        list_verts.Add(vertex);
                    }

                    norm = Vector3.Up;
                    for (int i = 0; i < tessellation - 2; i++)
                    {
                        if (norm.Y > 0)
                        {
                            list_triangles.Add(list_verts.Count);
                            list_triangles.Add((list_verts.Count) + (i + 1) % tessellation);
                            list_triangles.Add((list_verts.Count) + (i + 2) % tessellation);

                        }
                        else
                        {
                            list_triangles.Add(list_verts.Count);
                            list_triangles.Add((list_verts.Count) + (i + 2) % tessellation);
                            list_triangles.Add((list_verts.Count) + (i + 1) % tessellation);
                        }
                    }

                    // Create cap vertices.
                    for (int i = 0; i < tessellation; i++)
                    {
                        Vector3 position = GetCircleVector(i, tessellation) * radius + norm * height;

                        //AddVertex(position, norm);

                        vertex = new DVertex()
                        {
                            position = position,
                            texture = new Vector2(0, 0),
                            color = _color,
                            normal = norm,
                        };
                        list_verts.Add(vertex);
                    }

                    Vertices = list_verts.ToArray();
                    triangles = list_triangles.ToArray();
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
                IndexCount = triangles.Length;
                VertexCount = Vertices.Length;

                instancesDataForward = new DInstanceData[instX * instY * instZ];
                instances = new DInstanceType[instX * instY * instZ];
                _arrayOfInstances = new SC_cube_instances[instX * instY * instZ];
                instancesDataUP = new DInstanceData[instX * instY * instZ];
                instancesDataRIGHT = new DInstanceData[instX * instY * instZ];


                //Random r = new Random();
                //int rInt = r.Next(0, 100);

                count = 0;
                for (int x = 0; x < instX; x++)
                {
                    for (int y = 0; y < instY; y++)
                    {
                        for (int z = 0; z < instZ; z++)
                        {
                            if (type_of_cube == 0)
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

                                SC_cube_instances _cube = new SC_cube_instances();

                                _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2 * (1 + (0.01f)), _sizeY * 2 * (1 + (0.01f)), _sizeZ * 2 * (1 + (0.01f))));
                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = _is_static;
                                _cube.transform.Component.rigidbody.Tag = _tag;// SC_console_directx.BodyTag._terrain;
                                _cube.transform.Component.rigidbody.Material.Restitution = 0.015f; //0.015f
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f; // 0.55f
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f; //0.55f

                                //_cube.transform.Component.rigidbody.Damping = RigidBody.DampingType.Linear;
                                _cube.transform.Component.rigidbody.AllowDeactivation = false;
                                _cube.transform.Component.rigidbody.Mass = _mass;
                                _cube.current_pos = _tempMatrix;
                                _cube._POSITION = _tempMatrix;
                                if (_addtoworld == 1)
                                {

                                }
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_buo.Add(_cube.transform.Component.rigidbody, 3);
                                //_the_world.AddBody(_cube.transform.Component.rigidbody);
                                //_cube._POSITION = _tempMatrix;

                                //_singleObjectOnly = _cube;
                            }
                            else if (type_of_cube == 1) // JITTER SPHERE
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

                                SC_cube_instances _cube = new SC_cube_instances();
                                if (_addtoworld == 1)
                                {


                                }
                                //_cube._POSITION = _tempMatrix;

                                _cube.transform.Component.rigidbody = new RigidBody(new SphereShape(radius));



                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = _is_static;
                                _cube.transform.Component.rigidbody.Tag = _tag;//SC_console_directx.BodyTag._terrain_tiles; 

                                _cube.transform.Component.rigidbody.Material.Restitution = 0.015f; //0.015f
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f; // 0.55f
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f; //0.55f

                                //_cube.transform.Component.rigidbody.Damping = RigidBody.DampingType.Linear;

                                _cube.transform.Component.rigidbody.Mass = _mass;
                                _cube.transform.Component.rigidbody.AllowDeactivation = false;
                                _cube.current_pos = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                _the_world.AddBody(_cube.transform.Component.rigidbody);
                                //_buo.Add(_cube.transform.Component.rigidbody, 3);
                                //_singleObjectOnly = _cube;
                            }
                            else if (type_of_cube == 2)
                            {
                                Vector3 position = new Vector3((x * _sizeX * 2.5f) + offsetPosX, (y * _sizeY * 2.5f) + offsetPosY, (z * _sizeZ * 2.5f) + offsetPosZ);
                                //Vector3 position = new Vector3(x + offsetPosX, y + offsetPosY, z + offsetPosZ);
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

                                SC_cube_instances _cube = new SC_cube_instances();
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

                                    _cube.current_pos = _tempMatrix;
                                    //SC_Console_GRAPHICS._buo.Add(_cube.transform.Component.rigidbody, 3);
                                    _the_world.AddBody(_cube.transform.Component.rigidbody);
                                }
                                //_cube._POSITION = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_singleObjectOnly = _cube;
                            }
                            else if (type_of_cube == 3) // PHYSICS CUBES
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

                                SC_cube_instances _cube = new SC_cube_instances();
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

                                    //_cube.transform.Component.rigidbody.Damping = RigidBody.DampingType.Linear;

                                    //Console.WriteLine();
                                    _cube.transform.Component.rigidbody.AllowDeactivation = false;
                                    _cube.transform.Component.rigidbody.Mass = _mass;

                                    _cube.current_pos = _tempMatrix;

                                    _the_world.AddBody(_cube.transform.Component.rigidbody);
                                    //_buo.Add(_cube.transform.Component.rigidbody, 3);
                                }                                //_cube._POSITION = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_singleObjectOnly = _cube;
                            }

                            else if (type_of_cube == 4)
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

                                SC_cube_instances _cube = new SC_cube_instances();
                                if (_addtoworld == 1)
                                {
                                    _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2, _sizeY * 2, _sizeZ * 2));
                                    _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                    _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                    _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                    _cube.transform.Component.rigidbody.IsStatic = _is_static;
                                    _cube.transform.Component.rigidbody.Tag = _tag;//SC_console_directx.BodyTag.physicsInstancedCube;

                                    _cube.transform.Component.rigidbody.Material.Restitution = 0.015f; //0.015f
                                    _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f; // 0.55f
                                    _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f; //0.55f

                                    //_cube.transform.Component.rigidbody.Damping = RigidBody.DampingType.Linear;

                                    //Console.WriteLine();

                                    _cube.transform.Component.rigidbody.Mass = _mass;

                                    _cube.current_pos = _tempMatrix;
                                    _cube.transform.Component.rigidbody.AllowDeactivation = false;
                                    _the_world.AddBody(_cube.transform.Component.rigidbody);
                                    //_buo.Add(_cube.transform.Component.rigidbody, 3);
                                }                                //_cube._POSITION = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_singleObjectOnly = _cube;
                            }
                            else if (type_of_cube == 6) // capsule
                            {
                                Vector3 position = new Vector3((x * _sizeX * 5) + offsetPosX, (y * _sizeY * 5) + offsetPosY, (z * _sizeZ * 5) + offsetPosZ);
                                //Vector3 position = new Vector3(x + offsetPosX, y + offsetPosY, z + offsetPosZ);
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

                                SC_cube_instances _cube = new SC_cube_instances();
                                if (_addtoworld == 1)
                                {
                                    //_cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2 * 1.01f, _sizeY * 2 * 1.01f, _sizeZ * 2 * 1.01f));

                                    _cube.transform.Component.rigidbody = new RigidBody(new CapsuleShape(length * 0.1f, radius));

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

                                    _cube.current_pos = _tempMatrix;
                                    //SC_Console_GRAPHICS._buo.Add(_cube.transform.Component.rigidbody, 3);
                                    _the_world.AddBody(_cube.transform.Component.rigidbody);
                                }
                                //_cube._POSITION = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_singleObjectOnly = _cube;
                            }

                            else if (type_of_cube == 7) //cone
                            {
                                Vector3 position = new Vector3((x * _sizeX * 5) + offsetPosX, (y * _sizeY * 5) + offsetPosY, (z * _sizeZ * 5) + offsetPosZ);
                                //Vector3 position = new Vector3(x + offsetPosX, y + offsetPosY, z + offsetPosZ);
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

                                SC_cube_instances _cube = new SC_cube_instances();
                                if (_addtoworld == 1)
                                {
                                    //_cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2 * 1.01f, _sizeY * 2 * 1.01f, _sizeZ * 2 * 1.01f));
                                    //_cube.transform.Component.rigidbody = new RigidBody(new CapsuleShape(_sizeX * 50 * 0.5f, tileSize * 0.5f * 0.5f));
                                    _cube.transform.Component.rigidbody = new RigidBody(new ConeShape(height * 3, radius * 3));
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

                                    _cube.current_pos = _tempMatrix;
                                    //SC_Console_GRAPHICS._buo.Add(_cube.transform.Component.rigidbody, 3);
                                    _the_world.AddBody(_cube.transform.Component.rigidbody);
                                }
                                //_cube._POSITION = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_singleObjectOnly = _cube;
                            }
                            else if (type_of_cube == 8)
                            {
                                Vector3 position = new Vector3((x * _sizeX * 5) + offsetPosX, (y * _sizeY * 5) + offsetPosY, (z * _sizeZ * 5) + offsetPosZ);
                                //Vector3 position = new Vector3(x + offsetPosX, y + offsetPosY, z + offsetPosZ);
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

                                SC_cube_instances _cube = new SC_cube_instances();
                                if (_addtoworld == 1)
                                {
                                    //_cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2 * 1.01f, _sizeY * 2 * 1.01f, _sizeZ * 2 * 1.01f));
                                    //_cube.transform.Component.rigidbody = new RigidBody(new CapsuleShape(_sizeX * 50 * 0.5f, tileSize * 0.5f * 0.5f));
                                    _cube.transform.Component.rigidbody = new RigidBody(new CylinderShape(height * 2, radius));

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

                                    _cube.current_pos = _tempMatrix;
                                    //SC_Console_GRAPHICS._buo.Add(_cube.transform.Component.rigidbody, 3);
                                    _the_world.AddBody(_cube.transform.Component.rigidbody);
                                }
                                //_cube._POSITION = _tempMatrix;
                                _arrayOfInstances[count] = _cube;
                                _arrayOfInstances[count]._POSITION = _tempMatrix;
                                //_singleObjectOnly = _cube;
                            }

                            else if (type_of_cube == 9)
                            {

                                float sc_dist = 2.25f;

                                Vector3 position = new Vector3(((x + offsetPosX) * _sizeX * sc_dist), ((y + offsetPosY) * _sizeY * sc_dist), ((z + offsetPosZ) * _sizeZ * sc_dist));
                                //Vector3 position = new Vector3(((x) * _sizeX), ((y )), ((z ) * _sizeZ));
                                //Vector3 position = new Vector3(x * _sizeX, y * _sizeY, z * _sizeZ);
                                //Vector3 position = new Vector3(x * offsetPosX, y * offsetPosY, z * offsetPosZ);
                                //Vector3 position = new Vector3((x * _sizeX) + offsetPosX, (y * _sizeY) + offsetPosY, (z * _sizeZ) + offsetPosZ);

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

                                SC_cube_instances _cube = new SC_cube_instances();


                                _cube._POSITION = _tempMatrix;
                                _cube._ORIGINPOSITION = _tempMatrix;
                                //_cube._ELBOWCROSSVEC = Vector3.Zero;
                                _cube._LASTPOSITION = Matrix.Identity;
                                //_cube._UPPERARMPIVOT = Vector3.Zero;

                                //_cube._ARMLENGTH = _sizeX * ((ChunkWidth_L + ChunkWidth_R));
                                //_cube._SHOULDERROT = Matrix.Identity;
                                //_cube._ELBOWPOSITION = Vector3.Zero;


                                _arrayOfInstances[count] = _cube;
                            }








                            /*else if (isTerrain == 2)
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

                                instancesData[count] = new DInstanceData()
                                {
                                    rotation = new Vector4(0, 0, 0, 1)
                                };
                                _tempMatrix.M41 = position.X;
                                _tempMatrix.M42 = position.Y;
                                _tempMatrix.M43 = position.Z;

                                SC_cube_instances _cube = new SC_cube_instances();
                                _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2, _sizeY * 2, _sizeZ * 2));
                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = true;
                                _cube.transform.Component.rigidbody.Tag = _sc_console_directx.BodyTag.cloth_cube;


                                _cube.transform.Component.rigidbody.Material.Restitution = 0.001f;
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.65f;
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.65f;

                                _cube.transform.Component.rigidbody.Mass = _sizeX;

                                _the_world.AddBody(_cube.transform.Component.rigidbody);
                                //_cube._POSITION = _tempMatrix;

                                _singleObjectOnly = _cube;
                            }
                            else if (isTerrain == 3)
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

                                instancesData[count] = new DInstanceData()
                                {
                                    rotation = new Vector4(0, 0, 0, 1)
                                };
                                _tempMatrix.M41 = position.X;
                                _tempMatrix.M42 = position.Y;
                                _tempMatrix.M43 = position.Z;

                                SC_cube_instances _cube = new SC_cube_instances();
                                _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2, _sizeY * 2, _sizeZ * 2));
                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = true;
                                _cube.transform.Component.rigidbody.Tag = _sc_console_directx.BodyTag.screen_corners;


                                _cube.transform.Component.rigidbody.Material.Restitution = 0.05f;
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f;
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f;

                                _cube.transform.Component.rigidbody.Mass = _sizeX * 5;

                                _the_world.AddBody(_cube.transform.Component.rigidbody);
                                //_cube._POSITION = _tempMatrix;

                                _singleObjectOnly = _cube;
                            }
                            else if (isTerrain == 4)
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

                                instancesData[count] = new DInstanceData()
                                {
                                    rotation = new Vector4(0, 0, 0, 1)
                                };
                                _tempMatrix.M41 = position.X;
                                _tempMatrix.M42 = position.Y;
                                _tempMatrix.M43 = position.Z;

                                SC_cube_instances _cube = new SC_cube_instances();
                                _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2, _sizeY * 2, _sizeZ * 2));
                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = true;
                                _cube.transform.Component.rigidbody.Tag = _sc_console_directx.BodyTag.screen_pointer_touch;


                                _cube.transform.Component.rigidbody.Material.Restitution = 0.05f;
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f;
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f;

                                _cube.transform.Component.rigidbody.Mass = _sizeX * 5;

                                _the_world.AddBody(_cube.transform.Component.rigidbody);
                                //_cube._POSITION = _tempMatrix;

                                _singleObjectOnly = _cube;
                            }
                            else if (isTerrain == 5)
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

                                instancesData[count] = new DInstanceData()
                                {
                                    rotation = new Vector4(0, 0, 0, 1)
                                };
                                _tempMatrix.M41 = position.X;
                                _tempMatrix.M42 = position.Y;
                                _tempMatrix.M43 = position.Z;

                                SC_cube_instances _cube = new SC_cube_instances();
                                _cube.transform.Component.rigidbody = new RigidBody(new BoxShape(_sizeX * 2, _sizeY * 2, _sizeZ * 2));
                                _cube.transform.Component.rigidbody.Position = new Jitter.LinearMath.JVector(_tempMatrix.M41, _tempMatrix.M42, _tempMatrix.M43);
                                _cube.transform.Component.rigidbody.Orientation = Conversion.ToJitterMatrix(_tempMatrix);
                                _cube.transform.Component.rigidbody.LinearVelocity = new Jitter.LinearMath.JVector(0, 0, 0);
                                _cube.transform.Component.rigidbody.IsStatic = true;
                                _cube.transform.Component.rigidbody.Tag = _sc_console_directx.BodyTag.screen_pointer_HMD;


                                _cube.transform.Component.rigidbody.Material.Restitution = 0.05f;
                                _cube.transform.Component.rigidbody.Material.StaticFriction = 0.55f;
                                _cube.transform.Component.rigidbody.Material.KineticFriction = 0.55f;

                                _cube.transform.Component.rigidbody.Mass = _sizeX * 5;

                                _the_world.AddBody(_cube.transform.Component.rigidbody);

                                _singleObjectOnly = _cube;
                            }*/

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
