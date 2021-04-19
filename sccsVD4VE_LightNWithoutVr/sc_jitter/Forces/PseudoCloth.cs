using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;
//using Jitter.Dynamics.Joints;
using Jitter;
using Jitter.Collision;

using Jitter.Forces;


using sccsVD4VE_LightNWithoutVr.SC_Graphics;
using sccsVD4VE_LightNWithoutVr;

using SharpDX;

using System.Collections;

namespace Jitter.Forces
{
    public class PseudoCloth
    {

        public Matrix _POSITION { get; set; }
        public Matrix[] _WORLDMATRIXINSTANCES { get; set; }
        public List<Constraint> constraints = new List<Constraint>();

        public class PseudoClothBody : RigidBody
        {
            //SC_VR_Cube tester = new SC_VR_Cube();

            public PseudoClothBody() : base(new BoxShape(0.5f, 0.2f, 0.5f)) //new SphereShape(sphereRadius) float sphereRadius
            {
                //tester = test;
            }
        }

        int sizeX, sizeY;
        float scale;

        World world;

        PseudoClothBody[] bodies;
        //SC_VR_Cube[] bodies;

        public SC_cube _cube;

        public PseudoCloth(World world, sccsVD4VE_LightNWithoutVr.sc_console.SC_console_directx D3D, IntPtr windowsHandle, int sizeX, int sizeY, float scale, Matrix matter) //World world, 
        {
            this.world = world;
            bodies = new PseudoClothBody[sizeX * sizeY];
            //bodies = new SC_VR_Cube[sizeX * sizeY];


            float r = 0.05f;
            float g = 0.05f;
            float b = 0.05f;
            float a = 1;

            int instX = 1;
            int instY = 1;
            int instZ = 1;

            float offsetPosX = 0;
            float offsetPosY = 0;
            float offsetPosZ = 0;

            Matrix _tempMatroxer = matter;
            //_tempMatroxer.M42 = 0;
            float cubesizex = 0.5f;
            float cubesizey = 0.1f;
            float cubesizez = 0.5f;


            float _size_screen = 0.05f;
            _cube = new SC_cube();
            //bool _hasinit0 = _cube.Initialize(D3D, D3D.SurfaceWidth, D3D.SurfaceHeight, _size_screen, 1, 1, 0.05f, 0.01f, 0.05f, new Vector4(r, g, b, a), instX, instY, instZ, windowsHandle, _tempMatroxer, 3, offsetPosX, offsetPosY, offsetPosZ); //, "terrainGrassDirt.bmp" //0.00035f
            var _hasinit0 = _cube.Initialize(D3D, D3D.SurfaceWidth, D3D.SurfaceHeight, _size_screen, 1, 1, cubesizex * _size_screen, cubesizey * _size_screen, cubesizez * _size_screen, new Vector4(r, g, b, a), sizeX, instY, sizeX, windowsHandle, _tempMatroxer, 4, offsetPosX, offsetPosY, offsetPosZ, world, sccsVD4VE_LightNWithoutVr.sc_console.SC_console_directx.BodyTag.sc_jitter_cloth, true, 1, 10, 0, 0, 0); //, "terrainGrassDirt.bmp" //0.00035f




            _size_screen = 0.0006f;
            var counter = 0;

            for (int x = 0; x < sizeX; x++)
            {
                for (int y = 0; y < sizeX; y++)
                {
                    _tempMatroxer = matter;
                    //Console.WriteLine("fuck off");
                    // new PseudoClothBody(0.1f);

                    //_cube.Initialize(SC_Console_DIRECTX._device, 0.1f, 0.1f, 0.1f,1,1,1, matter, 1, 1, 1);

                    //test.Initialize(DirectXComponent._device, 0.25f, 0.1f, 0.25f, new Vector4(0.1f, 0.1f, 0.1f, 1));

                    //SharpDX.Direct3D11.Device device, float _sizeX, float _sizeY, float _sizeZ

                    //test.transform.Component.rigidbody = new RigidBody(new BoxShape(0.5f, 0.2f, 0.5f));
                    //test.transform.Component.rigidbody.Position = new JVector(i * scale, 0, e * scale); //+ JVector.Up * 10.0f
                    //test.transform.Component.rigidbody.Material.KineticFriction = 50;
                    //test.transform.Component.rigidbody.Material.StaticFriction = 50;
                    //test.transform.Component.rigidbody.Mass = 0.1f; //1
                    //test.transform.Component.rigidbody.Tag = DirectXComponent.BodyTag.someTest;

                    List<JVector> vertices = new List<JVector>();


                    for (int j = 0;j < _cube.Vertices.Length;j++ )
                    {
                        vertices.Add(new JVector(_cube.Vertices[j].position.X, _cube.Vertices[j].position.Y, _cube.Vertices[j].position.Z));
                    }


                    //test.transform.Component.rigidbody.Shape = new ConvexHullShape(vertices);
                    var pos = new JVector((x * scale * 0.5f), 0, (y * scale * 0.5f)) + new JVector(_tempMatroxer.M41, _tempMatroxer.M42, _tempMatroxer.M43);

                    bodies[x + y * sizeY] = new PseudoClothBody();
                    bodies[x + y * sizeY].Position = pos;// new JVector(i * scale, 0, e * scale) + new JVector(_tempMatroxer.M41, _tempMatroxer.M42, _tempMatroxer.M43); //  + JVector.Up * 5 + JVector.Up * 10.0f //i * scale, 0, e * scale                  
                    bodies[x + y * sizeY].Orientation = JMatrix.Identity;// (0,0,0,1);
                    bodies[x + y * sizeY].Material.StaticFriction = 0.45f;
                    bodies[x + y * sizeY].Material.KineticFriction = 0.45f;
                    bodies[x + y * sizeY].Mass = 5f; //1
                    bodies[x + y * sizeY].Tag = sccsVD4VE_LightNWithoutVr.sc_console.SC_console_directx.BodyTag.pseudoCloth;
                    bodies[x + y * sizeY].Shape = new ConvexHullShape(vertices);
                    //bodies[x + y * sizeY].Shape = new BoxShape();

                    _tempMatroxer.M41 = new JVector(x * scale * cubesizex, 0, y * scale * cubesizey).X;// + (sizeX * 0.25f);
                    _tempMatroxer.M42 = new JVector(x * scale * cubesizex, 0, y * scale * cubesizey).Y;// + (sizeX * 0.25f);
                    _tempMatroxer.M43 = new JVector(x * scale * cubesizex, 0, y * scale * cubesizey).Z;// + (sizeX * 0.25f);

                    _cube._arrayOfInstances[counter]._POSITION = _tempMatroxer;

                    //bodies[i + e * sizeY].IsStatic = true;

                    if (x == 0 && y ==0)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (x == sizeX - 1 && y == 0)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (x == 0 && y == sizeX - 1)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (x == sizeX - 1 && y == sizeX - 1)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (x == 0)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (y == 0)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (x == (int)sizeX-1)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (y == (int)sizeY-1)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }


                    /*else if (x == (int)sizeX * 0.5f && y == 0)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }
                    else if (x == 0 && y == (int)sizeY * 0.5f)
                    {
                        bodies[x + y * sizeY].IsStatic = true;
                    }*/

                    //TO READADD OTHERWISE IT WONT WORK. BUT IT WORKS WITH THIS VERSION SOMEWHOW
                    //SC_Console_GRAPHICS._arrayOfClothCubes.Add(_cube);
                    //TO READADD OTHERWISE IT WONT WORK. BUT IT WORKS WITH THIS VERSION SOMEWHOW







                    //DirectXComponent.World.AddBody(bodies[i + e * sizeY].transform.Component.rigidbody);*/

                    world.AddBody(bodies[x + y * sizeY]);
                    //DirectXComponent.World.AddBody(test.transform.Component.rigidbody);
                    counter++;
                }
            }

            world.CollisionSystem.PassedBroadphase += new Collision.PassedBroadphaseHandler(CollisionSystem_PassedBroadphase);
            world.Events.PostStep += new World.WorldStep(world_PostStep);

  

            for (int i = 0; i < sizeX; i++)
            {
                for (int e = 0; e < sizeY; e++)
                {
                    //i *= 0.1f;

                    if (i + 1 < sizeX)
                    {
                        AddDistance(e * sizeY + i, (i + 1) + e * sizeY);
                        // (i,e) and (i+1,e)
                    }

                    if (e + 1 < sizeY)
                    {
                        AddDistance(e * sizeY + i, ((e + 1) * sizeY) + i);
                        // (e,i) and (e+1,i)
                    }

                    if ((i + 1 < sizeX) && (e + 1 < sizeY))
                    {
                        AddDistance(e * sizeY + i, ((e + 1) * sizeY) + (i + 1));
                    }


                    if ((i > 0) && (e + 1 < sizeY))
                    {
                        AddDistance(e * sizeY + i, ((e + 1) * sizeY) + (i - 1));
                    }

                    /*if (i == (int)sizeX - 1)
                    {

                    }
                    if (e == (int)sizeY - 1)
                    {

                    }*/
                }
            }

            this.sizeX = sizeX;
            this.sizeY = sizeY;
            this.scale = scale;
        }

        void world_PostStep(float timeStep)
        {
            CheckConstraints();
        }

        public RigidBody GetCorner(int e, int i)
        {
            return bodies[e * sizeY + i];
        }

        private void AddDistance(int p1, int p2)
        {
            //Console.WriteLine("AddDistance");
            PointPointDistance dc = new PointPointDistance(bodies[p1], bodies[p2], bodies[p1].Position, bodies[p2].Position);
            //Console.WriteLine(bodies[p1].Position + " _ " + bodies[p2].Position);

            //DistanceConstraint dc = new DistanceConstraint(bodies[p1], bodies[p2], bodies[p1].position, bodies[p2].position);
            dc.Softness = 0.0001f; //2
            dc.BiasFactor = 0.75f;
            //dc.Distance *= 0.01f;
            dc.Behavior = PointPointDistance.DistanceBehavior.LimitDistance;
            world.AddConstraint(dc);
            this.constraints.Add(dc);

            /*PointPointDistance dcer = new PointPointDistance(DirectXComponent._arrayOfClothCubes[p1].transform.Component.rigidbody, DirectXComponent._arrayOfClothCubes[p2].transform.Component.rigidbody, DirectXComponent._arrayOfClothCubes[p1].transform.Component.rigidbody.Position, DirectXComponent._arrayOfClothCubes[p2].transform.Component.rigidbody.Position);
            dcer.Softness = 0.0000001f; //2
            dcer.BiasFactor = 0.000000001f;
            dcer.Distance = 0.01f;

            world.AddConstraint(dcer);
            this.constraints.Add(dcer);*/
        }

        public void CheckConstraints()
        {
            foreach (Constraint c in constraints)
            {
                /*if ((c as PointPointDistance).AppliedImpulse > 0.0001)
                {
                    world.RemoveConstraint(c);
                }*/
            }
        }

        private bool CollisionSystem_PassedBroadphase(IBroadphaseEntity body1, IBroadphaseEntity body2) //RigidBody body1, RigidBody body2
        {
            //Console.WriteLine("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            // prevent PseudoClothBody,PseudoClothBody collisions
            return !(body1 is PseudoClothBody && body2 is PseudoClothBody); //PseudoClothBody
        }
    }
}
