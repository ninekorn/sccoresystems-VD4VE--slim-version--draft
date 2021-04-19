using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using SharpDX;
using SharpDX.Direct3D11;

using System.Drawing;


using SharpDX.D3DCompiler;

using System.Runtime.InteropServices;
//using System.Windows.Forms;



namespace sccsVD4VE_LightNWithoutVr
{
    public struct SC_SharpDX_ScreenFrame
    {
        public int width;
        public int height;
        public byte[] bitmapByteArray;
        public byte[] bitmapEmptyByteArray;
        //public Bitmap mouseCursor;
        //public byte[] cursorByteArray;
        public System.Drawing.Point cursorPointPos;

        public int _desktopWidth;
        public int _desktopHeight;
        public ShaderResourceView _ShaderResource;
        public ShaderResourceView[] _ShaderResourceArray;
    }
}
