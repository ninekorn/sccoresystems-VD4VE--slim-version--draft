using System;
using SharpDX;
using SharpDX.D3DCompiler;
using SharpDX.Direct3D11;
using System.Runtime.InteropServices;

namespace SCCoreSystems
{
    public class _sc_texture_shader
    {
        [DllImport("User32.dll", CharSet = CharSet.Unicode)]
        public static extern int MessageBox(IntPtr h, string m, string c, int type);

        [StructLayout(LayoutKind.Sequential)]
        internal struct DMatrixBuffer
        {
            public Matrix world;
            public Matrix view;
            public Matrix projection;
        }

        public VertexShader VertexShader { get; set; }
        public PixelShader PixelShader { get; set; }
        public InputLayout Layout { get; set; }
        public SharpDX.Direct3D11.Buffer ConstantMatrixBuffer { get; set; }
        public SamplerState SamplerState { get; set; }
        public ShaderResourceView shaderResourceView2D { get; set; }
        public _sc_texture_shader() { }

        public bool Initialize(Device device, IntPtr windowsHandler)
        {
            return InitializeShader(device, windowsHandler);
        }
        ShaderBytecode vertexShaderByteCode;
        ShaderBytecode pixelShaderByteCode;
        private bool InitializeShader(Device device, IntPtr windowsHandler)
        {
            try
            {
                try
                {
                    /*string vsFileName = "../../../sc_instance_shader/" + "InstancedTexture.vs";
                    string psFileName = "../../../sc_instance_shader/" + "InstancedTexture.ps";

                    vertexShaderByteCode = ShaderBytecode.CompileFromFile(vsFileName, "TextureVertexShader", "vs_4_0", ShaderFlags.None, EffectFlags.None);
                    pixelShaderByteCode = ShaderBytecode.CompileFromFile(psFileName, "TexturePixelShader", "ps_4_0", ShaderFlags.None, EffectFlags.None);
                    */


                    /*
                    if (Program.is_wpf == 1)
                    {
                        string vsFileName = SCCoreSystems.Properties.Resources.InstancedTexture1;// "../../../_sc_instance_shader/" + "texture.vs";
                        string psFileName = SCCoreSystems.Properties.Resources.InstancedTexture;// "../../../_sc_instance_shader/" + "texture.ps";

                         vertexShaderByteCode = ShaderBytecode.Compile(vsFileName, "TextureVertexShader", "vs_4_0", ShaderFlags.None, SharpDX.D3DCompiler.EffectFlags.None);
                         pixelShaderByteCode = ShaderBytecode.Compile(psFileName, "TexturePixelShader", "ps_4_0", ShaderFlags.None, SharpDX.D3DCompiler.EffectFlags.None);
                    }
                    else
                    {

                    }*/


                    string vsFileName = SCCoreSystems.Properties.Resources.InstancedTexture1;// "../../../_sc_instance_shader/" + "texture.vs";
                    string psFileName = SCCoreSystems.Properties.Resources.InstancedTexture;// "../../../_sc_instance_shader/" + "texture.ps";

                    vertexShaderByteCode = ShaderBytecode.Compile(vsFileName, "TextureVertexShader", "vs_4_0", ShaderFlags.None, SharpDX.D3DCompiler.EffectFlags.None);
                    pixelShaderByteCode = ShaderBytecode.Compile(psFileName, "TexturePixelShader", "ps_4_0", ShaderFlags.None, SharpDX.D3DCompiler.EffectFlags.None);












                    VertexShader = new VertexShader(device, vertexShaderByteCode);
                    PixelShader = new PixelShader(device, pixelShaderByteCode);

                    InputElement[] inputElements = new InputElement[]
                    {
                    new InputElement()
                    {
                        SemanticName = "POSITION",
                        SemanticIndex = 0,
                        Format = SharpDX.DXGI.Format.R32G32B32_Float,
                        Slot = 0,
                        AlignedByteOffset = 0,
                        Classification = InputClassification.PerVertexData,
                        InstanceDataStepRate = 0
                    },
                    new InputElement()
                    {
                        SemanticName = "TEXCOORD",
                        SemanticIndex = 0,
                        Format = SharpDX.DXGI.Format.R32G32_Float,
                        Slot = 0,
                        AlignedByteOffset = InputElement.AppendAligned,
                        Classification = InputClassification.PerVertexData,
                        InstanceDataStepRate = 0
                    },
                    new InputElement()
                    {
                        SemanticName = "TEXCOORD",
                        SemanticIndex = 1,
                        Format = SharpDX.DXGI.Format.R32G32B32_Float,
                        Slot = 1,
                        AlignedByteOffset = 0,
                        Classification = InputClassification.PerInstanceData,
                        InstanceDataStepRate = 1
                    }
                    };

                    Layout = new InputLayout(device, ShaderSignature.GetInputSignature(vertexShaderByteCode), inputElements);

                    vertexShaderByteCode.Dispose();
                    pixelShaderByteCode.Dispose();

                    BufferDescription matrixBufferDescription = new BufferDescription()
                    {
                        Usage = ResourceUsage.Dynamic,
                        SizeInBytes = Utilities.SizeOf<DMatrixBuffer>(),
                        BindFlags = BindFlags.ConstantBuffer,
                        CpuAccessFlags = CpuAccessFlags.Write,
                        OptionFlags = ResourceOptionFlags.None,
                        StructureByteStride = 0
                    };

                    ConstantMatrixBuffer = new SharpDX.Direct3D11.Buffer(device, matrixBufferDescription);

                    SamplerStateDescription samplerDesc = new SamplerStateDescription()
                    {
                        Filter = Filter.MinMagMipLinear,
                        AddressU = TextureAddressMode.Wrap,
                        AddressV = TextureAddressMode.Wrap,
                        AddressW = TextureAddressMode.Wrap,
                        MipLodBias = 0,
                        MaximumAnisotropy = 1,
                        ComparisonFunction = Comparison.Always,
                        BorderColor = new Color4(0, 0, 0, 0),
                        MinimumLod = 0,
                        MaximumLod = float.MaxValue
                    };

                    SamplerState = new SamplerState(device, samplerDesc);

                }
                catch (Exception ex)
                {
                    MessageBox((IntPtr)0, ex.ToString(), "Oculus Error", 0);
                }
                return true;
            }
            catch (Exception ex)
            {
                MainWindow.MessageBox((IntPtr)0, "Fail 01" + ex.Message, "Oculus error", 0);
                return false;
            }
        }
        public void ShutDown()
        {
            ShuddownShader();
        }
        private void ShuddownShader()
        {
            SamplerState?.Dispose();
            SamplerState = null;
            ConstantMatrixBuffer?.Dispose();
            ConstantMatrixBuffer = null;
            Layout?.Dispose();
            Layout = null;
            PixelShader?.Dispose();
            PixelShader = null;
            VertexShader?.Dispose();
            VertexShader = null;
        }
        public bool Render(DeviceContext deviceContext, int vertexCount, int instanceCount, Matrix worldMatrix, Matrix viewMatrix, Matrix projectionMatrix, ShaderResourceView texture)
        {
            if (!SetShaderParameters(deviceContext, worldMatrix, viewMatrix, projectionMatrix, texture))
                return false;

            RenderShader(deviceContext, vertexCount, instanceCount);
            return true;
        }
        private bool SetShaderParameters(DeviceContext deviceContext, Matrix worldMatrix, Matrix viewMatrix, Matrix projectionMatrix, ShaderResourceView texture)
        {
            try
            {
                worldMatrix.Transpose();
                viewMatrix.Transpose();
                projectionMatrix.Transpose();
                DataStream mappedResource;
                deviceContext.MapSubresource(ConstantMatrixBuffer, MapMode.WriteDiscard, MapFlags.None, out mappedResource);
                DMatrixBuffer matrixBuffer = new DMatrixBuffer()
                {
                    world = worldMatrix,
                    view = viewMatrix,
                    projection = projectionMatrix
                };
                mappedResource.Write(matrixBuffer);
                deviceContext.UnmapSubresource(ConstantMatrixBuffer, 0);
                int bufferPositionNumber = 0;
                deviceContext.VertexShader.SetConstantBuffer(bufferPositionNumber, ConstantMatrixBuffer);
                deviceContext.PixelShader.SetShaderResource(0, texture);

                return true;
            }
            catch
            {
                return false;
            }
        }
        private void RenderShader(DeviceContext deviceContext, int vertexCount, int instanceCount)
        {
            deviceContext.InputAssembler.InputLayout = Layout;
            deviceContext.VertexShader.Set(VertexShader);
            deviceContext.GeometryShader.Set(null);
            deviceContext.PixelShader.Set(PixelShader);
            deviceContext.PixelShader.SetSampler(0, SamplerState);
            deviceContext.DrawIndexedInstanced(vertexCount, instanceCount, 0, 0, 0);
        }
    }
}