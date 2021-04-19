using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SharpDX;

//FROM CATLIKE CODING TUTORIAL BASICALLY COPY PASTED I BELIEVE. I've gotta check his license. if it's not that, it's from unifywiki or unitywiki or something like that.

namespace SCCoreSystems
{
    public class RoundedCube 
    {

        //public float xSize, ySize, zSize;
        public int roundness;

        private Vector3[] vertices;
        private Vector3[] normals;
        //private Color32[] cubeUV;
        private int[] triangles;


        int _width = 0;
        int _height = 0;
        int _depth = 0;

        private float _sizeX = 0;
        private float _sizeY = 0;
        private float _sizeZ = 0;

        public void Generate(out Vector3[] verts, out int[] trigs,int width,int height,int depth,int rounded,float sizeX, float sizeY, float sizeZ)
        {

            this._width = width;
            this._height = height;
            this._depth = depth;


            this._sizeX = sizeX;
            this._sizeY = sizeY;
            this._sizeZ = sizeZ;

            this.roundness = rounded;

            //GetComponent<MeshFilter>().mesh = mesh = new Mesh();
            //mesh.name = "Procedural Cube";
            CreateVertices();
            CreateTriangles();


            verts = vertices;
            trigs = triangles;

        }

        private void CreateVertices()
        {
            int cornerVertices = 8;
            int edgeVertices = (_width + _height + _depth - 3) * 4;
            int faceVertices = (
                (_width - 1) * (_height - 1) +
                (_width - 1) * (_depth - 1) +
                (_height - 1) * (_depth - 1)) * 2;
            vertices = new Vector3[cornerVertices + edgeVertices + faceVertices];
            normals = new Vector3[vertices.Length];
            //cubeUV = new Color32[vertices.Length];

            int v = 0;
            for (int y = 0; y <= _height; y++)
            {
                for (int x = 0; x <= _width; x++)
                {
                    SetVertex(v++, x* _sizeX, y* _sizeY, 0* _sizeZ);
                }
                for (int z = 1; z <= _depth; z++)
                {
                    SetVertex(v++, _width * _sizeX, y * _sizeY, z * _sizeZ);
                }
                for (int x = _width - 1; x >= 0; x--)
                {
                    SetVertex(v++, x * _sizeX, y * _sizeY, _depth * _sizeZ);
                }
                for (int z = _depth - 1; z > 0; z--)
                {
                    SetVertex(v++, 0 * _sizeX, y * _sizeY, z * _sizeZ);
                }
            }
            for (int z = 1; z < _depth; z++)
            {
                for (int x = 1; x < _width; x++)
                {
                    SetVertex(v++, x * _sizeX, _height * _sizeY, z * _sizeZ);
                }
            }
            for (int z = 1; z < _depth; z++)
            {
                for (int x = 1; x < _width; x++)
                {
                    SetVertex(v++, x * _sizeX, 0 * _sizeY, z * _sizeZ);
                }
            }

            //mesh.vertices = vertices;
            //mesh.normals = normals;
            //mesh.colors32 = cubeUV;
        }

        private void SetVertex(int i, float x, float y, float z)
        {
            Vector3 inner = vertices[i] = new Vector3(x, y, z);

            if (x < roundness)
            {
                inner.X = roundness;
            }
            else if (x > _width - roundness)
            {
                inner.X = _width - roundness;
            }
            if (y < roundness)
            {
                inner.Y = roundness;
            }
            else if (y > _height - roundness)
            {
                inner.Y = _height - roundness;
            }
            if (z < roundness)
            {
                inner.Z = roundness;
            }
            else if (z > _depth - roundness)
            {
                inner.Z = _depth - roundness;
            }

            //normals[i] = (vertices[i] - inner).normalized;
            //vertices[i] = inner + normals[i] * roundness;
            //cubeUV[i] = new Color32((byte)x, (byte)y, (byte)z, 0);
        }

        private void CreateTriangles()
        {
            int[] trianglesZ = new int[(_width * _height) * 12];
            int[] trianglesX = new int[(_height * _depth) * 12];
            int[] trianglesY = new int[(_width * _depth) * 12];
            int ring = (_width + _depth) * 2;
            int tZ = 0, tX = 0, tY = 0, v = 0;

            for (int y = 0; y < _height; y++, v++)
            {
                for (int q = 0; q < _width; q++, v++)
                {
                    tZ = SetQuad(trianglesZ, tZ, v, v + 1, v + ring, v + ring + 1);
                }
                for (int q = 0; q < _depth; q++, v++)
                {
                    tX = SetQuad(trianglesX, tX, v, v + 1, v + ring, v + ring + 1);
                }
                for (int q = 0; q < _width; q++, v++)
                {
                    tZ = SetQuad(trianglesZ, tZ, v, v + 1, v + ring, v + ring + 1);
                }
                for (int q = 0; q < _depth - 1; q++, v++)
                {
                    tX = SetQuad(trianglesX, tX, v, v + 1, v + ring, v + ring + 1);
                }
                tX = SetQuad(trianglesX, tX, v, v - ring + 1, v + ring, v + 1);
            }

            tY = CreateTopFace(trianglesY, tY, ring);
            tY = CreateBottomFace(trianglesY, tY, ring);

            //mesh.subMeshCount = 3;
            //mesh.SetTriangles(trianglesZ, 0);
            //mesh.SetTriangles(trianglesX, 1);
            //mesh.SetTriangles(trianglesY, 2);


            triangles = new int[trianglesZ.Length + trianglesX.Length + trianglesY.Length];


            for (int i = 0;i < trianglesZ.Length;i++)
            {
                triangles[i] = trianglesZ[i];
            }
            for (int i = 0; i < trianglesX.Length; i++)
            {
                triangles[i+ trianglesZ.Length] = trianglesX[i];
            }
            for (int i = 0; i < trianglesY.Length; i++)
            {
                triangles[i + trianglesZ.Length+ trianglesX.Length] = trianglesY[i];
            }
        }

        private int CreateTopFace(int[] triangles, int t, int ring)
        {
            int v = ring * _height;
            for (int x = 0; x < _width - 1; x++, v++)
            {
                t = SetQuad(triangles, t, v, v + 1, v + ring - 1, v + ring);
            }
            t = SetQuad(triangles, t, v, v + 1, v + ring - 1, v + 2);

            int vMin = ring * (_height + 1) - 1;
            int vMid = vMin + 1;
            int vMax = v + 2;

            for (int z = 1; z < _depth - 1; z++, vMin--, vMid++, vMax++)
            {
                t = SetQuad(triangles, t, vMin, vMid, vMin - 1, vMid + _width - 1);
                for (int x = 1; x < _width - 1; x++, vMid++)
                {
                    t = SetQuad(
                        triangles, t,
                        vMid, vMid + 1, vMid + _width - 1, vMid + _width);
                }
                t = SetQuad(triangles, t, vMid, vMax, vMid + _width - 1, vMax + 1);
            }

            int vTop = vMin - 2;
            t = SetQuad(triangles, t, vMin, vMid, vTop + 1, vTop);
            for (int x = 1; x < _width - 1; x++, vTop--, vMid++)
            {
                t = SetQuad(triangles, t, vMid, vMid + 1, vTop, vTop - 1);
            }
            t = SetQuad(triangles, t, vMid, vTop - 2, vTop, vTop - 1);

            return t;
        }

        private int CreateBottomFace(int[] triangles, int t, int ring)
        {
            int v = 1;
            int vMid = vertices.Length - (_width - 1) * (_depth - 1);
            t = SetQuad(triangles, t, ring - 1, vMid, 0, 1);
            for (int x = 1; x < _width - 1; x++, v++, vMid++)
            {
                t = SetQuad(triangles, t, vMid, vMid + 1, v, v + 1);
            }
            t = SetQuad(triangles, t, vMid, v + 2, v, v + 1);

            int vMin = ring - 2;
            vMid -= _width - 2;
            int vMax = v + 2;

            for (int z = 1; z < _depth - 1; z++, vMin--, vMid++, vMax++)
            {
                t = SetQuad(triangles, t, vMin, vMid + _width - 1, vMin + 1, vMid);
                for (int x = 1; x < _width - 1; x++, vMid++)
                {
                    t = SetQuad(
                        triangles, t,
                        vMid + _width - 1, vMid + _width, vMid, vMid + 1);
                }
                t = SetQuad(triangles, t, vMid + _width - 1, vMax + 1, vMid, vMax);
            }

            int vTop = vMin - 1;
            t = SetQuad(triangles, t, vTop + 1, vTop, vTop + 2, vMid);
            for (int x = 1; x < _width - 1; x++, vTop--, vMid++)
            {
                t = SetQuad(triangles, t, vTop, vTop - 1, vMid, vMid + 1);
            }
            t = SetQuad(triangles, t, vTop, vTop - 1, vMid, vTop - 2);

            return t;
        }

        private static int
        SetQuad(int[] triangles, int i, int v00, int v10, int v01, int v11)
        {
            triangles[i] = v00;
            triangles[i + 1] = triangles[i + 4] = v01;
            triangles[i + 2] = triangles[i + 3] = v10;
            triangles[i + 5] = v11;
            return i + 6;
        }
    }
}
