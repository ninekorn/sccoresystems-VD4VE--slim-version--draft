using SharpDX;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System.Runtime.InteropServices;

using System;


namespace SCCoreSystems
{
    public class DTerrain_Screen_Metric                   // 179 lines
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
        public DTerrain_Screen_Metric() { }


        float _size_screen = 0;
        int _divX;
        int _divY;


        float _a; 
        float _r; 
        float _g; 
        float _b;


        // Methods.
        public bool Initialize(SCCoreSystems.sc_console.SC_console_directx D3D, int width, int height, float size_screen, int divX, int divY,float a, float r, float g, float b, float depthScreen, float offsetDepth, float maxwidthMul, float maxheightMul)
        {
            _size_screen = size_screen;
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
            if (!InitializeBuffers(D3D, depthScreen, offsetDepth, maxwidthMul, maxheightMul))
                return false;

            return true;
        }
        private bool InitializeBuffers(SCCoreSystems.sc_console.SC_console_directx D3D,float depthScreen, float offsetDepth, float maxwidthMul, float maxheightMul)
        {
            try
            {
                /*float oriWidth = (float)(D3D.SurfaceWidth * 0.5f);
                float oriHeight = (float)(D3D.SurfaceWidth * 0.5f);

                oriWidth /= 10;
                oriHeight /= 10;

                oriWidth *= 0.0006f;
                oriHeight *= 0.0006f;*/


                int sizeWidther = (int)(m_TerrainWidth * 0.5f);
                int sizeHeighter = (int)(m_TerrainHeight * 0.5f);


                sizeWidther /= 10;
                sizeHeighter /= 10;


                // Calculate the number of vertices in the terrain mesh.
                //VertexCount  = (_divX - 1) * (_divY - 1)* (_divX - 1) * (_divY - 1) * 8;
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
                int remains = (int)Math.Round((m_TerrainWidth - m_TerrainHeight) * 0.5f);

                float offsetZ = (depthScreen * 2) + (1 * offsetDepth);


                for (int x = (int)(-_divX * maxwidthMul); x < (int)(_divX * maxwidthMul); x++)
                {
                    //float xx = x - oriWidth;
                    for (int y = (int)(-_divY * maxheightMul); y < (int)(_divY * maxheightMul); y++)
                    {

                        // LINE 1
                        // Upper left.
                        float positionX = (float)x;
                        float positionZ = (float)(y + 1);

                        //position = new Vector3(-sizeWidther*totalSize, -sizeHeighter*totalSize, 0),

                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Upper right.
                        positionX = (float)(x + 1);
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 2
                        // Upper right.
                        positionX = (float)(x + 1);
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Bottom right.
                        positionX = (float)(x + 1);
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 3
                        // Bottom right.
                        positionX = (float)(x + 1);
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Bottom left.
                        positionX = (float)x;
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 4
                        // Bottom left.
                        positionX = (float)x;
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Upper left.
                        positionX = (float)x;
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(positionX * sizeWidther, (positionZ * sizeHeighter) + remains, offsetZ) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;





























                        /*
                        //float yy = y - oriHeight;
                        // LINE 1
                        // Upper left.
                        float positionX = (float)x;
                        float positionZ = (float)(y + 1);


                        float xi = positionX * sizeWidther;
                        float yi = positionZ * sizeHeighter;






                        //position = new Vector3(-sizeWidther*totalSize, -sizeHeighter*totalSize, 0),

                        vertices[index].position = new Vector3(xi,yi, 0.0f)* _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Upper right.
                        positionX = (float)(x + 1);
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 2
                        // Upper right.
                        positionX = (float)(x + 1);
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Bottom right.
                        positionX = (float)(x + 1);
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 3
                        // Bottom right.
                        positionX = (float)(x + 1);
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Bottom left.
                        positionX = (float)x;
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;

                        // LINE 4
                        // Bottom left.
                        positionX = (float)x;
                        positionZ = (float)y;
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;
                        // Upper left.
                        positionX = (float)x;
                        positionZ = (float)(y + 1);
                        vertices[index].position = new Vector3(xi, yi, 0.0f) * _size_screen;
                        vertices[index].color = new Vector4(_r, _g, _b, _a);
                        indices[index] = index;
                        index++;	*/
                    }
                }

                // Create the vertex buffer.
                VertexBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.Device, BindFlags.VertexBuffer, vertices);

                // Create the index buffer.
                IndexBuffer = SharpDX.Direct3D11.Buffer.Create(D3D.Device, BindFlags.IndexBuffer, indices);

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