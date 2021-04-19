using SharpDX;
using SharpDX.Direct3D11;
using System;

//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Textures.SC_VR_Desktop_Screen_Textures;
//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Textures.SC_VR_Touch_Textures;
//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Grid;

using System.Runtime.InteropServices;

//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Grid;
//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Textures;
//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Textures.SC_VR_Touch_Textures;
//using sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_Models;



namespace sccsVD4VE_LightNWithoutVr.SC_Graphics.SC_ShaderManager
{
    public class SC_ShaderManager                 // 77 lines
    {

        public sc_spectrum_shader_final _spectrum_texture_shader { get; set; }

        sc_spectrum.DLightBuffer[] _DLightBuffer_spectrum = new sc_spectrum.DLightBuffer[1];
        



        Vector4 ambientColor = new Vector4(0.15f, 0.15f, 0.15f, 1.0f);
        Vector4 diffuseColour = new Vector4(1, 1, 1, 1);
        Vector3 lightDirection = new Vector3(1, 0, 0);
        Vector3 lightPosition = new Vector3(0, 0, 0);

        BufferDescription lightBufferDesc = new BufferDescription()
        {
            Usage = ResourceUsage.Dynamic,
            SizeInBytes = Utilities.SizeOf<sc_spectrum.DLightBuffer>(),
            BindFlags = BindFlags.ConstantBuffer,
            CpuAccessFlags = CpuAccessFlags.Write,
            OptionFlags = ResourceOptionFlags.None,
            StructureByteStride = 0
        };


        public bool Initialize(Device device, IntPtr windowsHandle) //, float x, float y, float z, Vector4 color,Matrix worldMatrix
        {

      


            //////////////////////
            //SC PHYSICS SPECTRUM
            //////////////////////
            _DLightBuffer_spectrum[0] = new sc_spectrum.DLightBuffer()
            {
                ambientColor = ambientColor,
                diffuseColor = diffuseColour,
                lightDirection = lightDirection,
                padding0 = 0,
                lightPosition = lightPosition,
                padding1 = 0
            };

            SharpDX.Direct3D11.Buffer ConstantLightBuffar01 = new SharpDX.Direct3D11.Buffer(device, lightBufferDesc);
            _spectrum_texture_shader = new sc_spectrum_shader_final();
            _spectrum_texture_shader.Initialize(device, windowsHandle, ConstantLightBuffar01, _DLightBuffer_spectrum);
            //////////////////////
            //SC PHYSICS SPECTRUM
            //////////////////////


            
            return true;
        }


        public bool RenderInstancedObjectSpectrum(DeviceContext deviceContext, int VertexCount, int InstanceCount, Matrix worldMatrix, Matrix viewMatrix, Matrix projectionMatrix, ShaderResourceView texture, sc_spectrum.DLightBuffer[] _DLightBuffer_, sc_spectrum _cuber)
        {
            _spectrum_texture_shader.Render(deviceContext, VertexCount, InstanceCount, worldMatrix, viewMatrix, projectionMatrix, texture, _DLightBuffer_, _cuber);
            return true;
        }

    }
}