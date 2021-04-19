using SharpDX;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;

//using SC_WPF_RENDER.SC_Graphics.SC_Textures.SC_VR_Touch_Textures;

using System.Linq;
using System;


using System.Runtime.InteropServices;

namespace SCCoreSystems
{
    public class SC_VR_Cube //: ITransform, IComponent
    {

        public Matrix[] _LASTPOSITIONS;
        public int[] _DRAWINST;

        /*public ITransform transform { get; private set; }
        IComponent ITransform.Component
        {
            get => component;
        }
        IComponent component;


        IRigidbody IComponent.rigidbody { get; set; }
        //SoftBody IComponent.softbody { get; set; }
        */

        public bool isStatic { get; set; }

        public SharpDX.Direct3D11.Buffer InstanceBuffer { get; set; }


        private SharpDX.Direct3D11.Buffer VertexBuffer { get; set; }
        private SharpDX.Direct3D11.Buffer IndexBuffer { get; set; }
        private int VertexCount { get; set; }
        public int IndexCount { get; set; }

        public SC_VR_Touch_Shader.DVertex[] Vertices { get; set; }
        public int[] indices;

        public SharpDX.Matrix worldMatrix { get; set; }
        public SharpDX.Vector3 Position { get; set; }

        public SharpDX.Vector3 Velocity { get; set; }

        public SharpDX.Quaternion Rotation { get; set; }
        public SharpDX.Vector3 Forward { get; set; }

        private float _sizeX = 0;
        private float _sizeY = 0;
        private float _sizeZ = 0;


        public SharpDX.Matrix _MatrixPos { get; set; }

        // Constructor
        public SC_VR_Cube() { }

        public Vector4 _color;

        [StructLayout(LayoutKind.Sequential)]
        public struct DInstanceType
        {
            public Vector3 position;
        };
        float InstanceCount = 0;



        



        public boundingBox _boundingBox { get; set; }// = new SharpDX.BoundingBox();
        public SharpDX.Vector3[] _vertArray { get; set; }


        SharpDX.Vector3 lowestX;
        SharpDX.Vector3 highestX;
        SharpDX.Vector3 lowestY;
        SharpDX.Vector3 highestY;
        SharpDX.Vector3 lowestZ;
        SharpDX.Vector3 highestZ;




        public struct boundingBox
        {
            public Vector3 center;
            public Vector3 min;
            public Vector3 max;

            public boundingBox(Vector3 cent, Vector3 min, Vector3 max)
            {
                this.center = cent;
                this.min = min;
                this.max = max;
            }
        }

        public bool boundingBoxIntersects(boundingBox a, boundingBox b,Vector3 posA,Vector3 posB)
        {

            /*if (a.max.X < b.min.X || a.min.X > b.max.X)
            {
                return false;
            }
            if (a.max.Y < b.min.Y || a.min.Y > b.max.Y)
            {
                return false;
            }
            if(a.max.Z < b.min.Z || a.min.Z > b.max.Z)
            {
                return false;
            }*/








            // Exit with no intersection if found separated along an axis
            if (a.max.X + posA.X < b.min.X + posB.X || a.min.X + posA.X > b.max.X + posB.X)
            {
                return false;
            }
            if (a.max.Y + posA.Y < b.min.Y + posB.Y || a.min.Y + posA.Y > b.max.Y + posB.Y)
            {
                return false;
            }
            if (a.max.Z + posA.Z < b.min.Z + posB.Z || a.min.Z + posA.Z > b.max.Z + posB.Z)
            {
                return false;
            }

            // No separating axis found, therefor there is at least one overlapping axis
            return true;
        }

        public Vector3 GetBoundingBoxCenter()
        {
            return _boundingBox.center + this.Position;
            //return _boundingBox + this.transform.Component.rigidbody.position;
        }

        // Methods.
        public bool Initialize(SharpDX.Direct3D11.Device device,float x, float y, float z, Vector4  color,int ChunkWidth_L, int ChunkWidth_R, int ChunkHeight_L, int ChunkHeight_R, int ChunkDepth_L, int ChunkDepth_R)
        {
            //transform = this;
            //component = this;
            //transform.Component.rigidbody.boundingBox = new boundingBox();
            //_boundingBox = transform.Component.rigidbody.boundingBox;



            this._color = color;
            this._sizeX = x;
            this._sizeY = y;
            this._sizeZ = z;


            // Initialize the vertex and index buffer that hold the geometry for the triangle.
            return InitializeBuffer(device,  ChunkWidth_L,  ChunkWidth_R,  ChunkHeight_L,  ChunkHeight_R,  ChunkDepth_L,  ChunkDepth_R);
        }
        public void ShutDown()
        {
            // Release the vertex and index buffers.
            ShutDownBuffers();
        }
        public void Render(DeviceContext deviceContext)
        {
            // Put the vertex and index buffers on the graphics pipeline to prepare for drawings.
            RenderBuffers(deviceContext);
        }


      
        private bool InitializeBuffer(SharpDX.Direct3D11.Device device, int ChunkWidth_L, int ChunkWidth_R, int ChunkHeight_L, int ChunkHeight_R, int ChunkDepth_L, int ChunkDepth_R)
        {
            try
            {
                 //Set number of vertices in the vertex array.
                 VertexCount = 8;
                 // Set number of vertices in the index array.
                 IndexCount = 36;


                 // Create the vertex array and load it with data.
                 Vertices = new[]
                 {
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, 1*_sizeZ),
                         color = _color,

                     },
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, 1*_sizeZ),
                         color = _color,
                     },
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(1*_sizeX, -1*_sizeY, 1*_sizeZ),
                         color = _color,
                     },
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, 1*_sizeZ),
                         color = _color,
                     },


                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(-1*_sizeX, -1*_sizeY, -1*_sizeZ),
                         color = _color,
                     },
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(-1*_sizeX, 1*_sizeY, -1*_sizeZ),
                         color = _color,
                     },
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(1*_sizeX,-1*_sizeY, -1*_sizeZ),
                         color = _color,
                     },
                     new SC_VR_Touch_Shader.DVertex()
                     {
                         position = new Vector3(1*_sizeX, 1*_sizeY, -1*_sizeZ),
                         color = _color,
                     },                
                 };

                // Create Indicies to load into the IndexBuffer.
                indices = new int[]
                {
                     0, // Bottom left.
                     1, // Top middle.
                     2,  // Bottom right.
                     3,
                     2,
                     1,

                     1,
                     5,
                     3,
                     7,
                     3,
                     5,

                     2,
                     3,
                     6,
                     7,
                     6,
                     3,

                     6,
                     7,
                     4,
                     5,
                     4,
                     7,

                     4,
                     5,
                     0,
                     1,
                     0,
                     5,

                     4,
                     0,
                     6,
                     2,
                     6,
                     0
                 };



                /*lowestX = _vertArray.OrderBy(xx => xx.X).FirstOrDefault();
                highestX = _vertArray.OrderBy(xx => xx.X).Last();

                lowestY = _vertArray.OrderBy(yy => yy.X).FirstOrDefault();
                highestY = _vertArray.OrderBy(yy => yy.X).Last();

                lowestZ = _vertArray.OrderBy(zz => zz.X).FirstOrDefault();
                highestZ = _vertArray.OrderBy(zz => zz.X).Last();
                */


                //var lowestXTopY = _vertArray.OrderBy(xx => xx.X).ThenBy(xx=>xx.Y).FirstOrDefault();
                //var highestXTopY = _vertArray.OrderBy(xx => xx.X).ThenBy(xx => xx.Y).Last();
                /*

                lowestY = _vertArray.OrderBy(yy => yy.X).FirstOrDefault();
                highestY = _vertArray.OrderBy(yy => yy.X).Last();

                lowestZ = _vertArray.OrderBy(zz => zz.X).FirstOrDefault();
                highestZ = _vertArray.OrderBy(zz => zz.X).Last();
                */

                //double minX = Math.Min(Math.Min(lowestX.X, highestX.X), Math.Min(lb.X, rb.X));

           
                _vertArray = new SharpDX.Vector3[Vertices.Length];

                for (int i = 0; i < Vertices.Length; i++)
                {
                    _vertArray[i] = Vertices[i].position;
                }
                var min = _vertArray.OrderBy(xx => xx.X).ThenBy(xx=>xx.Y).ThenBy(xx => xx.Z).FirstOrDefault();
                var max = _vertArray.OrderBy(xx => xx.X).ThenBy(xx => xx.Y).ThenBy(xx => xx.Z).Last();

                float sumX = 0;
                float sumY = 0;
                float sumZ = 0;

                for (int i = 0; i < Vertices.Length; i++)
                {
                    sumX += Vertices[i].position.X;
                    sumY += Vertices[i].position.Y;
                    sumZ += Vertices[i].position.Z;
                }

                sumX /= Vertices.Length;
                sumY /= Vertices.Length;
                sumZ /= Vertices.Length;

                var center = new Vector3(sumX, sumY, sumZ);
                _boundingBox = new boundingBox(center,min, max);




                //_boundingBox.center = center;
                //_boundingBox.min = min;
                //_boundingBox.max = max;

                //_boundingBox = new SharpDX.BoundingBox(min,max);







                //double minX = Math.Min(Math.Min(lt.X, rt.X), Math.Min(lb.X, rb.X));
                //double minY = Math.Min(Math.Min(lt.Y, rt.Y), Math.Min(lb.Y, rb.Y));

                //double maxX = Math.Max(Math.Max(lt.X, rt.X), Math.Max(lb.X, rb.X));
                //double maxY = Math.Max(Math.Max(lt.Y, rt.Y), Math.Max(lb.Y, rb.Y));



                int width = 4;
                int heigth = 4;
                int depth = 4;

                InstanceCount = width* heigth * depth;
                DInstanceType[] instances = new DInstanceType[width* heigth * depth];
                _LASTPOSITIONS = new Matrix[width * heigth * depth];

                int counter = 0;


                for (int x = -ChunkWidth_L; x <= ChunkWidth_R; x++)
                {
                    for (int y = -ChunkHeight_L; y <= ChunkHeight_R; y++)
                    {
                        for (int z = -ChunkDepth_L; z <= ChunkDepth_R; z++)
                        {
                            float posX = (x);
                            float posY = (y);
                            float posZ = (z);

                            var xxi = x;
                            var yyi = y;
                            var zzi = z;

                            if (xxi < 0)
                            {
                                xxi *= -1;
                                xxi = (ChunkWidth_R) + xxi;
                            }
                            if (yyi < 0)
                            {
                                yyi *= -1;
                                yyi = (ChunkHeight_R) + yyi;
                            }
                            if (zzi < 0)
                            {
                                zzi *= -1;
                                zzi = (ChunkDepth_R) + zzi;
                            }

                            int _index = xxi + (ChunkWidth_L + ChunkWidth_R + 1) * (yyi + (ChunkHeight_L + ChunkHeight_R + 1) * zzi);


                            instances[counter] = new DInstanceType()
                            {
                                position = new Vector3(x, y, z),
                            };
                            _LASTPOSITIONS[counter] = Matrix.Identity;
                            _LASTPOSITIONS[counter].M41 = instances[counter].position.X;
                            _LASTPOSITIONS[counter].M42 = instances[counter].position.Y;
                            _LASTPOSITIONS[counter].M43 = instances[counter].position.Z;


                            counter++;
                        }
                    }
                }

                // Create the vertex buffer.
                VertexBuffer = SharpDX.Direct3D11.Buffer.Create(device, BindFlags.VertexBuffer, Vertices);

                // Create the index buffer.
                IndexBuffer = SharpDX.Direct3D11.Buffer.Create(device, BindFlags.IndexBuffer, indices);

                InstanceBuffer = SharpDX.Direct3D11.Buffer.Create(device, BindFlags.VertexBuffer, instances);
                //InstanceBuffer = SharpDX.Direct3D11.Buffer.Create(device, BindFlags.VertexBuffer, instances, Utilities.SizeOf<DInstanceType>() * instances.Length, ResourceUsage.Dynamic, CpuAccessFlags.Write, ResourceOptionFlags.None, 0);

                // Delete arrays now that they are in their respective vertex and index buffers.
                //Vertices = null;
                //indices = null;

                return true;
            }
            catch
            {
                return false;
            }
        }
        private void ShutDownBuffers()
        {
            // Release the index buffer.
            IndexBuffer?.Dispose();
            IndexBuffer = null;
            // Release the vertex buffer.
            VertexBuffer?.Dispose();
            VertexBuffer = null;

            InstanceBuffer?.Dispose();
            InstanceBuffer = null;
        }
        private void RenderBuffers(DeviceContext deviceContext)
        {
            deviceContext.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(VertexBuffer, Utilities.SizeOf<SC_VR_Touch_Shader.DVertex>(), 0), new VertexBufferBinding(InstanceBuffer, Utilities.SizeOf<DInstanceType>(), 0));
            deviceContext.InputAssembler.SetIndexBuffer(IndexBuffer, SharpDX.DXGI.Format.R32_UInt, 0);
            deviceContext.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
        }
    }
}








































//VertexCount = 36;
// Set number of vertices in the index array.
//IndexCount = 36;



/*Vertices = new[]
{
     new SC_VR_Touch_Shader.DVertex()
     {
         position = new Vector3( 1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =  new Vector3(1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3(-1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3(-1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position = new Vector3( 1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =  new Vector3(-1, -1, -1)*_sizeX,
         color = _color,
     },




     new SC_VR_Touch_Shader.DVertex()
     {
         position =new Vector3(-1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =          new Vector3( 1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3( 1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3( 1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =  new Vector3(-1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =  new Vector3(-1, -1,  1)*_sizeX,
         color = _color,
     },


     new SC_VR_Touch_Shader.DVertex()
     {
         position =new Vector3(-1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =         new Vector3(-1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =      new Vector3(-1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =    new Vector3(-1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =            new Vector3(-1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3(-1,  1,  1)*_sizeX,
         color = _color,
     },


     new SC_VR_Touch_Shader.DVertex()
     {
         position =new Vector3( 1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =    new Vector3( 1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =        new Vector3( 1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =      new Vector3( 1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3( 1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =  new Vector3( 1, -1, -1)*_sizeX,
         color = _color,
     },




     //BOTTOM
     new SC_VR_Touch_Shader.DVertex()
     {
         position = new Vector3(-1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =    new Vector3( 1, -1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =     new Vector3( 1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3( 1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =             new Vector3(-1, -1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =       new Vector3(-1, -1, -1)*_sizeX,
         color = _color,
     },


     //TOP
      new SC_VR_Touch_Shader.DVertex()
     {
         position =     new Vector3( 1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =    new Vector3( 1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =       new Vector3(-1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =     new Vector3(-1,  1, -1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =        new Vector3(-1,  1,  1)*_sizeX,
         color = _color,
     },
     new SC_VR_Touch_Shader.DVertex()
     {
         position =   new Vector3( 1,  1,  1)*_sizeX,
         color = _color,
     },


};*/
