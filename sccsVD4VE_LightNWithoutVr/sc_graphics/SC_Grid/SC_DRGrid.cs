using SharpDX;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System.Runtime.InteropServices;

namespace SC_WPF_RENDER.SC_Graphics.SC_Grid
{
    public class SC_DRGrid                   // 179 lines
    {
        // Structs
        [StructLayout(LayoutKind.Sequential)]
        internal struct DVertexType
        {
            internal Vector3 position;
            internal Vector4 color;
        }

        // Variables
        private int _vertexcountwidth, _vertexcountheight;

        // Properties
        private SharpDX.Direct3D11.Buffer VertexBuffer { get; set; }
        private SharpDX.Direct3D11.Buffer IndexBuffer { get; set; }
        private int VertexCount { get; set; }
        public int IndexCount { get; private set; }

        // Constructor
        public SC_DRGrid() { }


        float _tileSize = 0;

        // Methods.
        public bool Initialize(SharpDX.Direct3D11.Device device, int vertexcountwidth, int vertexcountheight, float tileSize)
        {
            _tileSize = tileSize;
            // Manually set the width and height of the terrain.
            _vertexcountwidth = vertexcountwidth;
            _vertexcountheight = vertexcountheight;

            // Initialize the vertex and index buffer that hold the geometry for the terrain.
            if (!InitializeBuffers(device))
                return false;

            return true;
        }
        private bool InitializeBuffers(SharpDX.Direct3D11.Device device)
        {
            try
            {
                Vector4 color = new Vector4(0.45f, 0.45f, 0.45f, 1.0f);

                // Calculate the number of vertices in the terrain mesh.
                VertexCount = (_vertexcountwidth) * (_vertexcountheight) * (_vertexcountwidth) * (_vertexcountheight) * 8;
                // Set the index count to the same as the vertex count.
                IndexCount = VertexCount;

                // Create the vertex array.
                DVertexType[] vertices = new DVertexType[VertexCount];         
                // Create the index array.
                int[] indices = new int[IndexCount];

                // Initialize the index to the vertex array.
                int index = 0;
               
                // Load the vertex and index arrays with the terrain data.
                for (int j = -_vertexcountheight; j < (_vertexcountheight); j++)
                {
                    for (int i = -_vertexcountwidth; i < (_vertexcountwidth); i++)
                    {
                        // LINE 1
                        // Upper left.
                        float positionX = (float)i;
                        float positionZ = (float)(j + 1);
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;
                        // Upper right.
                        positionX = (float)(i + 1);
                        positionZ = (float)(j + 1);
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;

                        // LINE 2
                        // Upper right.
                        positionX = (float)(i + 1);
                        positionZ = (float)(j + 1);
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;
                        // Bottom right.
                        positionX = (float)(i + 1);
                        positionZ = (float)j;
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;

                        // LINE 3
                        // Bottom right.
                        positionX = (float)(i + 1);
                        positionZ = (float)j;
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;
                        // Bottom left.
                        positionX = (float)i;
                        positionZ = (float)j;
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;

                        // LINE 4
                        // Bottom left.
                        positionX = (float)i;
                        positionZ = (float)j;
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
                        indices[index] = index;
                        index++;
                        // Upper left.
                        positionX = (float)i;
                        positionZ = (float)(j + 1);
                        vertices[index].position = new Vector3(positionX, 0.001f, positionZ)* _tileSize;
                        vertices[index].color = color;
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