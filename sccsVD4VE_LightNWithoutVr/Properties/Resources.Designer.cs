﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace sccsVD4VE_LightNWithoutVr.Properties {
    using System;
    
    
    /// <summary>
    ///   A strongly-typed resource class, for looking up localized strings, etc.
    /// </summary>
    // This class was auto-generated by the StronglyTypedResourceBuilder
    // class via a tool like ResGen or Visual Studio.
    // To add or remove a member, edit your .ResX file then rerun ResGen
    // with the /str option, or rebuild your VS project.
    [global::System.CodeDom.Compiler.GeneratedCodeAttribute("System.Resources.Tools.StronglyTypedResourceBuilder", "15.0.0.0")]
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
    [global::System.Runtime.CompilerServices.CompilerGeneratedAttribute()]
    internal class Resources {
        
        private static global::System.Resources.ResourceManager resourceMan;
        
        private static global::System.Globalization.CultureInfo resourceCulture;
        
        [global::System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Resources() {
        }
        
        /// <summary>
        ///   Returns the cached ResourceManager instance used by this class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Resources.ResourceManager ResourceManager {
            get {
                if (object.ReferenceEquals(resourceMan, null)) {
                    global::System.Resources.ResourceManager temp = new global::System.Resources.ResourceManager("sccsVD4VE_LightNWithoutVr.Properties.Resources", typeof(Resources).Assembly);
                    resourceMan = temp;
                }
                return resourceMan;
            }
        }
        
        /// <summary>
        ///   Overrides the current thread's CurrentUICulture property for all
        ///   resource lookups using this strongly typed resource class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Globalization.CultureInfo Culture {
            get {
                return resourceCulture;
            }
            set {
                resourceCulture = value;
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer :register(b0)
        ///{
        ///	float4x4 worldMatrix;
        ///	float4x4 viewMatrix;
        ///	float4x4 projectionMatrix;
        ///	//float4x4 worldViewProjection;
        ///}
        ///
        ///Texture2D diffuseMap;
        ///SamplerState textureSampler;
        ///
        ///struct VS_INPUT
        ///{
        ///    float4 Pos : POSITION;
        ///	float4 Col : COLOR;
        ///	float2 tex: TEXCOORD;
        ///	//float3 normal : NORMAL;
        ///	//float4 instancePosition : POSITION1;
        ///	//float4 instanceRadRot : POSITION2;
        ///	//float4 instanceRadRotRIGHT : POSITION3;
        ///	//float4 instanceRadRotUP : POSITION4;
        ///};
        ///
        ///struct GS_ [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string HLSL {
            get {
                return ResourceManager.GetString("HLSL", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Texture2D shaderTexture;
        ///SamplerState SampleType;
        ///
        ///SamplerState textureSampler
        ///{
        ///    Filter = MIN_MAG_MIP_LINEAR;
        ///    AddressU = Wrap;
        ///    AddressV = Wrap;
        ///};
        ///
        ///cbuffer LightBuffer
        ///{
        ///	float4 ambientColor;
        ///	float4 diffuseColor;
        ///	float3 lightDirection;
        ///	float padding0;
        ///	float3 lightPosition;
        ///	float padding1;
        ///};
        ///
        ///struct PixelInputType
        ///{ 
        ///	float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture {
            get {
                return ResourceManager.GetString("texture", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to cbuffer MatrixBuffer : register(b0)
        ///{
        ///	matrix worldMatrix;
        ///	matrix viewMatrix;
        ///	matrix projectionMatrix;
        ///};
        ///
        ///struct VertexInputType
        ///{
        ///    float4 position : POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	float4 color : COLOR;
        ///	float3 normal : NORMAL;
        ///	float4 instancePosition : POSITION1;
        ///	float4 instanceRadRot : POSITION2;
        ///	float4 instanceRadRotRIGHT : POSITION3;
        ///	float4 instanceRadRotUP : POSITION4;
        ///};
        ///
        ///struct PixelInputType
        ///{
        ///    float4 position : SV_POSITION;
        ///    float2 tex : TEXCOORD0;
        ///	fl [rest of string was truncated]&quot;;.
        /// </summary>
        internal static string texture1 {
            get {
                return ResourceManager.GetString("texture1", resourceCulture);
            }
        }
    }
}
