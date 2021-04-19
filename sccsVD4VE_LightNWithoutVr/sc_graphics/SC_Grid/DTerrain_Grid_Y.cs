using SharpDX;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System.Runtime.InteropServices;

using System;

namespace SCCoreSystems
{
    public class DTerrain_Grid_Y                   // 179 lines
    {
        // Structs
        [StructLayout(LayoutKind.Sequential)]
        internal struct DVertexType
        {
            internal Vector3 position;
            internal Vector4 color;
        }

        // Variables
        private int m_TerrainWidth, m_TerrainHeight;

        // Properties
        private SharpDX.Direct3D11.Buffer VertexBuffer { get; set; }
        private SharpDX.Direct3D11.Buffer IndexBuffer { get; set; }
        private int VertexCount { get; set; }
        public int IndexCount { get; private set; }

        // Constructor
        public DTerrain_Grid_Y() { }


        float _tileSize = 0;
        int _divX;
        int _divY;


        float _a; 
        float _r; 
        float _g; 
        float _b;


        // Methods.
        public bool Initialize(SharpDX.Direct3D11.Device device, int width, int height, float tileSize, int divX, int divY,float a, float r, float g, float b)
        {
            _tileSize = tileSize;
            // Manually set the width and height of the terrain.
            m_TerrainWidth = width; 
            m_TerrainHeight = height;

            this._divX = divX;
            this._divY = divY;

            this._a = a;
            this._r = r;
            this._g = g;
            this._b = b;

            // Initialize the vertex and index buffer that hold the geometry for the terrain.
            if (!InitializeBuffers(device))
                return false;

            return true;
        }
        private bool InitializeBuffers(SharpDX.Direct3D11.Device device)
        {
            try
            {

                int sizeWidther = (int)(m_TerrainWidth * 0.5f);
                int sizeHeighter = (int)(m_TerrainHeight * 0.5f);


                //sizeWidther /= 10;
                //sizeHeighter /= 10;


                // Calculate the number of vertices in the terrain mesh.
                //VertexCount = (_divX - 1) * (_divY - 1)* (_divX - 1) * (_divY - 1) * 8;
                VertexCount = (_divX) * (_divY) * (_divX) * (_divY) * 8;


                //VertexCount = (_divX) * (_divY) * (_divX) * (_divY) * 8;
                // Set the index count to the same as the vertex count.
                IndexCount = VertexCount;

                // Create the vertex array.
                DVertexType[] vertices = new DVertexType[VertexCount];         
                // Create the index array.
                int[] indices = new int[IndexCount];

                // Initialize the index to the vertex array.
                int index = 0;

                // Load the vertex and index arrays with the terrain data.

                for (int x = 0; x < (_divX); x++)
                {
                    for (int y = 0; y < (_divY); y++)
                    {
                        // LINE 1
                        // Upper left.
                        float positionX = (float)x;
                        float positionZ = (float)(y + 1);

                        //position = new Vector3(-sizeWidther*totalSize, -sizeHeighter*totalSize, 0),

                        vertices[index].position = new Vector3(positionX* sizeWidther, positionZ* sizeHeighter, 0.0f)* _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Upper right.
                        positionX = (float)(x + 1);
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 2
                        // Upper right.
                        positionX = (float)(x + 1);
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Bottom right.
                        positionX = (float)(x + 1);
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 3
                        // Bottom right.
                        positionX = (float)(x + 1);
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Bottom left.
                        positionX = (float)x;
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 4
                        // Bottom left.
                        positionX = (float)x;
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Upper left.
                        positionX = (float)x;
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(positionX * sizeWidther, positionZ * sizeHeighter, 0.0f) * _tileSize;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;	
                    }
                }

                // Create the vertex buffer.
                VertexBuffer = SharpDX.Direct3D11.Buffer.Create(device, BindFlags.VertexBuffer, vertices);

                // Create the index buffer.
                IndexBuffer = SharpDX.Direct3D11.Buffer.Create(device, BindFlags.IndexBuffer, indices);

                // Release the arrays now that the buffers have been created and loaded.
                vertices = null;
                indices = null;

                return true;
            }
            catch
            {
                return false;
            }
        }
       
        public void ShutDown()
        {
            // Release the vertex and index buffers.
            ShutdownBuffers();
        }
        private void ShutdownBuffers()
        {
            // Return the index buffer.
            IndexBuffer?.Dispose();
            IndexBuffer = null;
            // Release the vertex buffer.
            VertexBuffer?.Dispose();
            VertexBuffer = null;
        }
        public void Render(DeviceContext deviceContext)
        {
            // Put the vertex and index buffers on the graphics pipeline to prepare them for drawing.
            RenderBuffers(deviceContext);
        }
        private void RenderBuffers(DeviceContext deviceContext)
        {
            // Set the vertex buffer to active in the input assembler so it can be rendered.
            deviceContext.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(VertexBuffer, Utilities.SizeOf<DVertexType>(), 0));
            // Set the index buffer to active in the input assembler so it can be rendered.
            deviceContext.InputAssembler.SetIndexBuffer(IndexBuffer, Format.R32_UInt, 0);
            // Set the type of the primitive that should be rendered from this vertex buffer, in this case triangles.
            deviceContext.InputAssembler.PrimitiveTopology = PrimitiveTopology.LineList;
        }
    }
}