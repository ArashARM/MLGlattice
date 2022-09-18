using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Grasshopper;
using Grasshopper.Kernel.Data;
using System.Linq;
using System.IO;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Threading;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;


namespace MLGlattice
{
    class GLFunctions
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="edgesize"></Edge size for unit cell>
        /// <param name="VoxNumb"></number of Voxels on each edge>
        /// <returns/ a unit cell decomposed to VoxNumb*VoxNum*VoxNum voxels>
        public static List<Brep> VoxelizeUnitcell(double edgesize,double VoxNumb)
        {
            List<Brep> Voxels = new List<Brep>();

            double Voxedge = edgesize/VoxNumb;
            double VoxTOTALNum = Math.Pow(VoxNumb, 3);
            Interval dm = new Interval(0, Voxedge);
           

            List<Point3d> CenterPoints = new List<Point3d>();
            List<Point3d> CenterPoints1 = new List<Point3d>();
            List<Point3d> CenterPoints2 = new List<Point3d>();


            for (int i = 0; i < VoxNumb; i++)
            {

                Point3d Center0 = new Point3d((Voxedge / 2)+(Voxedge*i), Voxedge / 2, Voxedge / 2);
                CenterPoints.Add(Center0);
            }

            for (int i = 1; i < VoxNumb; i++)
            {
                for (int j = 0; j < CenterPoints.Count; j++)
                {
                    Point3d Cen = new Point3d(CenterPoints[j].X, (CenterPoints[j].Y)+(Voxedge * i), CenterPoints[j].Z);
                    CenterPoints1.Add(Cen);
                }
            }

            CenterPoints.AddRange(CenterPoints1);

            for (int i = 1; i < VoxNumb; i++)
            {
                for (int j = 0; j < CenterPoints.Count; j++)
                {
                    Point3d Cen = new Point3d(CenterPoints[j].X, (CenterPoints[j].Y) , CenterPoints[j].Z + (Voxedge * i));
                    CenterPoints2.Add(Cen);
                }
            }

            CenterPoints.AddRange(CenterPoints2);

            for (int i = 0; i < CenterPoints.Count; i++)
            {
                Box boxVox = new Box(Plane.WorldXY, dm, dm, dm);
                Brep Voxel = boxVox.ToBrep();

                Voxel.Translate(CenterPoints[i] - CenterPoints[0]);
                Voxels.Add(Voxel);
            }

            CenterPoints.Distinct().ToList();

            return Voxels;
        }

        public static List<Brep> MapGLatticeToVoxels(List<NurbsCurve> GlatticeModel, double StrutRadii,List<Brep> Voxels)
        {
            List<Brep> SolidVoxels = new List<Brep>();

            for (int i = 0; i < GlatticeModel.Count; i++)
            {
                for (int j = 0; j < Voxels.Count; j++)
                {

                    var testCube = Voxels[j];
                    Brep[] VVAD = new Brep[1];
                    VVAD[0] = testCube;
                    var A = GlatticeModel[i].ClosestPoints(VVAD, out Point3d CurvePT, out Point3d Pointonbox,out int Gem, StrutRadii);

                    if (A == true)
                        SolidVoxels.Add(testCube);

                }
                
            }

            return SolidVoxels;
        }

        public static List<NurbsCurve> UnitcellMaker(List<NurbsCurve> GlatticeModel)
        {
            List<NurbsCurve> CoreCurves = GlatticeModel;

            List<NurbsCurve> CoreCurves1 = DeepCopyCRV(CoreCurves);
            Plane XY = new Plane(new Point3d(0, 0, 1), Vector3d.ZAxis);
            Plane XZ = new Plane(new Point3d(0, 1, 0), Vector3d.YAxis);
            Plane YZ = new Plane(new Point3d(1, 0, 0), Vector3d.XAxis);
            Transform MYZ = Transform.Mirror(YZ);
            Transform MXZ = Transform.Mirror(XZ);
            Transform MXY = Transform.Mirror(XY);
            for (int i = 0; i < CoreCurves1.Count; i++)
            {
                CoreCurves1[i].Transform(MYZ);
            }

            CoreCurves.AddRange(CoreCurves1);

            List<NurbsCurve> CoreCurves2 = DeepCopyCRV(CoreCurves);

            for (int i = 0; i < CoreCurves2.Count; i++)
            {
                CoreCurves2[i].Transform(MXZ);
            }

            CoreCurves.AddRange(CoreCurves2);

            List<NurbsCurve> CoreCurves3 = DeepCopyCRV(CoreCurves);

            for (int i = 0; i < CoreCurves3.Count; i++)
            {
                CoreCurves3[i].Transform(MXY);
                
            }

            CoreCurves.AddRange(CoreCurves3);

            for (int i = 0; i < CoreCurves.Count; i++)
            {
                CoreCurves[i].Scale(0.5);
            }

            return CoreCurves;
        }

        public static List<NurbsCurve> DeepCopyCRV(List<NurbsCurve> CRVs)
        {
            List<NurbsCurve> NewCRv = new List<NurbsCurve>();
            for (int i = 0; i < CRVs.Count; i++)
            {
                NurbsCurve A = new NurbsCurve(CRVs[i]);
                NewCRv.Add(A);
            }

            return NewCRv;
        }


    }
}
