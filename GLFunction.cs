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

        public static List<Brep> MapGLatticeToVoxels(List<NurbsCurve> GlatticeModel, double StrutRadii,List<Brep> Voxels,out List<double> VoxelIndex)
        {
            List<Brep> SolidVoxels = new List<Brep>();

            for (int i = 0; i < GlatticeModel.Count; i++)
            {
                for (int j = 0; j < Voxels.Count; j++)
                {

                    var testCube = Voxels[j];
                    Brep[] VVAD = new Brep[1];
                    VVAD[0] = testCube;
                    var A = GlatticeModel[i].ClosestPoints(VVAD, out Point3d CurvePT, out Point3d Pointonbox,out int Gem, 0.0000001);

                    if (A == true)
                        SolidVoxels.Add(testCube);

                }
                
            }

            SolidVoxels = SolidVoxels.Distinct().ToList();

            VoxelIndex = GLFunctions.VoxelIndexing(Voxels,SolidVoxels);
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
        public static List<NurbsCurve> UnitcellMakerNoscale(List<NurbsCurve> GlatticeModel)
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

        public static List<NurbsCurve> LastCRVS(List<NurbsCurve> CRVs)
        {
            int dex = 0;
            while (true)
            {
                bool GoOn = false;
                for (int i = 0; i < CRVs.Count; i++)
                {
                    for (int j = 0; j < CRVs.Count; j++)
                    {
                        if (i == j)
                            continue;
                        if (CRVs[i].PointAtStart.DistanceTo(CRVs[j].PointAtStart) < 0.0001 && CRVs[i].PointAtEnd.DistanceTo(CRVs[j].PointAtEnd) < 0.0001)
                        {
                            CRVs.RemoveAt(i);
                            GoOn = true;
                            break;
                        }
                        if (CRVs[i].PointAtStart.DistanceTo(CRVs[j].PointAtEnd) < 0.0001 && CRVs[i].PointAtEnd.DistanceTo(CRVs[j].PointAtStart) < 0.0001)
                        {
                            CRVs.RemoveAt(i);
                            GoOn = true;
                            break;
                        }

                        dex = i;
                    }
                    if (GoOn)
                        break;
                }
                if (GoOn)
                    continue;
                if (!GoOn && dex == CRVs.Count - 1)
                    break;
            }
            return CRVs;
        }

        public static List<NurbsCurve> GraphCurve(List<NurbsCurve> CRVS, out List<Point3d> GrPoints)
        {
            List<Point3d> GraphNodePt = new List<Point3d>();

            List<NurbsCurve> GraphCurves = new List<NurbsCurve>();


            for (int i = 0; i < CRVS.Count; i++)
            {
                List<double> SplitSect = new List<double>();


                for (int j = 0; j < CRVS.Count; j++)
                {
                    if (i != j)
                    {

                        var pp0 = CRVS[j].PointAtStart;
                        var pp1 = CRVS[j].PointAtEnd;
                        CRVS[i].ClosestPoint(pp0, out double t);
                        CRVS[i].ClosestPoint(pp1, out double t1);
                        double Dis = CRVS[i].PointAt(t).DistanceTo(pp0);
                        double Dis1 = CRVS[i].PointAt(t1).DistanceTo(pp1);

                        if (t < 0.0001 && t > 0)
                            t = 0;
                        if (t > 0.999 && t < 1)
                            t = 1;

                        if (t1 < 0.0001 && t1 > 0)
                            t1 = 0;
                        if (t1 > 0.999 && t1 < 1)
                            t1 = 1;

                        if (Dis < 0.0001 & t > 0 && t < 1)
                        {
                            SplitSect.Add(Math.Round(t, 3));
                        }

                        if (Dis1 < 0.0001 & t1 > 0 && t1 < 1)
                        {
                            SplitSect.Add(Math.Round(t1, 3));
                        }
                    }

                    CRVS[i].ClosestPoints(CRVS[j], out Point3d Pi1, out Point3d Pi2);
                    if (Pi1.DistanceTo(Pi2) < 0.0001)
                    {
                        CRVS[i].ClosestPoint(Pi1, out double t2);
                        SplitSect.Add(Math.Round(t2, 3));
                    }

                }

                if (SplitSect.Count != 0)
                {

                    SplitSect.Add(0);
                    SplitSect.Add(1);
                    SplitSect = SplitSect.Distinct().ToList();
                    SplitSect.Sort();

                    for (int k = 0; k < SplitSect.Count - 1; k++)
                    {

                        var C1 = CRVS[i].Trim(SplitSect[k], SplitSect[k + 1]);
                        NurbsCurve C2 = C1.ToNurbsCurve();
                        C2.Domain = new Interval(0, 1);
                        GraphCurves.Add(C2);
                    }

                }
                else
                {
                    GraphCurves.Add(CRVS[i]);
                }

            }

            for (int i = 0; i < GraphCurves.Count; i++)
            {
                var P0 = new Point3d(Math.Round(GraphCurves[i].PointAtStart.X, 4), Math.Round(GraphCurves[i].PointAtStart.Y, 4), Math.Round(GraphCurves[i].PointAtStart.Z, 4));
                var P1 = new Point3d(Math.Round(GraphCurves[i].PointAtEnd.X, 4), Math.Round(GraphCurves[i].PointAtEnd.Y, 4), Math.Round(GraphCurves[i].PointAtEnd.Z, 4));

                //Point3d P0 = GraphCurves[i].PointAtStart;
                //Point3d P1 = GraphCurves[i].PointAtEnd;

                GraphNodePt.Add(P0);
                GraphNodePt.Add(P1);

            }

            GraphNodePt = GraphNodePt.Distinct().ToList();
            GrPoints = GraphNodePt;

            return GraphCurves;
        }

        public static Vector3d CrossProduct(Vector3d v1, Vector3d v2)
        {
            double x, y, z;
            x = v1.Y * v2.Z - v2.Y * v1.Z;
            y = (v1.X * v2.Z - v2.X * v1.Z) * -1;
            z = v1.X * v2.Y - v2.X * v1.Y;

            var rtnvector = new Vector3d(x, y, z);
            rtnvector.Unitize(); //optional
            return rtnvector;
        }

        public static Tuple<double, double,double> UnitCellFEM(List<NurbsCurve> InputCVS, Vector3d DisVec, double partdia, double Spharedia)
        {

            List<NurbsCurve> CRVS = DeepCopyCRV(InputCVS);

            CRVS = GLFunctions.UnitcellMaker(CRVS);
            string Modname = "UnitCellFEM";
           
            List<NurbsCurve> m_CCurvs = new List<NurbsCurve>();
            List<Point3d> m_Gpts = new List<Point3d>();
            for (int i = 0; i < CRVS.Count; i++)
            {
                Point3d P00 = CRVS[i].PointAtStart;
                Point3d P11 = CRVS[i].PointAtEnd;

                if (P11.Z < P00.Z)
                {
                    CRVS[i].Reverse();
                    CRVS[i].Domain = new Interval(0, 1);
                }
                    
            }

            for (int i = 0; i < CRVS.Count; i++)
            {
                for (int j = 0; j < CRVS.Count; j++)
                {
                    if (i == j)
                        continue;

                    
                    if (CRVS[i].PointAtEnd.DistanceTo(CRVS[j].PointAtStart)<0.00001 && CRVS[i].TangentAtEnd == CRVS[j].TangentAtStart)
                    {
                        Line C1 = new Line(CRVS[i].PointAtStart, CRVS[j].PointAtEnd);
                        var CC = C1.ToNurbsCurve();
                        CC.Domain = new Interval(0, 1);

                        CRVS.RemoveAt(i);
                        if(i>j)
                            CRVS.RemoveAt(j);
                        if(i<j)
                            CRVS.RemoveAt(j-1);
                        CRVS.Add(CC);
                        i = 0;
                        break;
                    }

                }
            }

            m_CCurvs = GraphCurve(CRVS, out m_Gpts);
            m_CCurvs = LastCRVS(m_CCurvs);


            

 
            string Pycode = "C://Users//Arash//Desktop//MLGlattice//bin//" + Modname + "Code.py";
            System.IO.File.Delete(Pycode);
       
            //Libreries
            System.IO.File.AppendAllText(Pycode, "#PhyCIde:\n");
            System.IO.File.AppendAllText(Pycode, "from part import *\n");
            System.IO.File.AppendAllText(Pycode, "from material import *\n");
            System.IO.File.AppendAllText(Pycode, "from section import *\n");
            System.IO.File.AppendAllText(Pycode, "from assembly import *\n");
            System.IO.File.AppendAllText(Pycode, "from step import *\n");
            System.IO.File.AppendAllText(Pycode, "from interaction import *\n");
            System.IO.File.AppendAllText(Pycode, "from load import *\n");
            System.IO.File.AppendAllText(Pycode, "from mesh import *\n");
            System.IO.File.AppendAllText(Pycode, "from optimization import *\n");
            System.IO.File.AppendAllText(Pycode, "from job import *\n");
            System.IO.File.AppendAllText(Pycode, "from sketch import *\n");
            System.IO.File.AppendAllText(Pycode, "from visualization import *\n");
            System.IO.File.AppendAllText(Pycode, "from connectorBehavior import *\n");
            System.IO.File.AppendAllText(Pycode, "from odbAccess import *\n");
            System.IO.File.AppendAllText(Pycode, "import math\n");
            System.IO.File.AppendAllText(Pycode, "import odbAccess\n");

            System.IO.File.AppendAllText(Pycode, "sortie =open('RD.txt', 'w')\n");

            //Material Properties
            System.IO.File.AppendAllText(Pycode, "# Startmaterialproperties\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Material(name = 'InconelMaterial')\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].materials['InconelMaterial'].Density(table = ((8.24e-09, ),))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].materials['InconelMaterial'].Elastic(table = ((207500.0, 0.304), ))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].HomogeneousSolidSection(material = 'InconelMaterial', name ='inconelSECTION', thickness = None)\n");
            System.IO.File.AppendAllText(Pycode, "#Endmaterialproperties\n");

            //defining outer box to cut
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), point2 = (2, 2))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type= DEFORMABLE_BODY)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Part-1'].BaseSolidExtrude(depth=2, sketch= mdb.models['Model-1'].sketches['__profile__'])\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].sketches['__profile__']\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0),point2 = (1.0, 1.0))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-2', type= DEFORMABLE_BODY)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Part-2'].BaseSolidExtrude(depth=1.0, sketch= mdb.models['Model-1'].sketches['__profile__'])\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].sketches['__profile__']\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-1-1',part = mdb.models['Model-1'].parts['Part-1'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.translate(instanceList=('Part-1-1', ),vector = (-0.5,-0.5,-0.5))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-2-1', part = mdb.models['Model-1'].parts['Part-2'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.InstanceFromBooleanCut(cuttingInstances=(mdb.models['Model-1'].rootAssembly.instances['Part-2-1'], ), instanceToBeCut = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], name = 'OutBox', originalInstances = DELETE)\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].rootAssembly.features['OutBox-1']\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].parts['Part-1']\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].parts['Part-2']\n");


            System.IO.File.AppendAllText(Pycode, "try:\n");

            List<Point3d> PPs = new List<Point3d>();
            for (int i = 0; i < m_CCurvs.Count; i++)
            {
                var C1 = m_CCurvs[i];
                Vector3d C1Vec = new Vector3d();
                Point3d TrasPoints = new Point3d();
                double Length = C1.GetLength();
                if (C1.PointAtEnd.Z < C1.PointAtStart.Z)
                {
                    C1Vec = C1.PointAtStart - C1.PointAtEnd;
                    TrasPoints = C1.PointAtEnd;
                }

                else
                {
                    C1Vec = C1.PointAtEnd - C1.PointAtStart;
                    TrasPoints = C1.PointAtStart;
                }


                Line L1 = new Line(Point3d.Origin, C1Vec);
                Vector3d RotationAxis = CrossProduct(C1Vec, Vector3d.ZAxis);
                double Xangke = Vector3d.VectorAngle(C1Vec, Vector3d.ZAxis) * 180 / Math.PI;
                if (Xangke > 90)
                    Xangke = 180 - Xangke;
                //double Yangke = Vector3d.VectorAngle(C1Vec, Vector3d.YAxis) * 180 / Math.PI;
                //if (Yangke > 90)
                //    Yangke = 180 - Yangke;

                //Xangke = 90 - Xangke;
                //Yangke = 90 - Yangke;

                var Pstart = m_CCurvs[i].PointAtStart;
                var Pend = m_CCurvs[i].PointAtEnd;
                bool StartExsited = false;
                bool EndExsited = false;
                for (int j = 0; j < PPs.Count; j++)
                {
                    if (Pstart.DistanceTo(PPs[j]) < 0.0001)
                    {
                        StartExsited = true;
                        break;
                    }

                }

                for (int j = 0; j < PPs.Count; j++)
                {
                    if (Pend.DistanceTo(PPs[j]) < 0.0001)
                    {
                        EndExsited = true;
                        break;
                    }

                }

                

                if (!StartExsited)
                {
                    PPs.Add(Pstart);

                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].ConstrainedSketch(name = '__profile__', sheetSize = 200.0)\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0,- 0.5), point2 = (0.0, 0.5))\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity= mdb.models['Model-1'].sketches['__profile__'].geometry[2])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(center = (0.0, 0.0), direction = CLOCKWISE, point1 = (0.0, " + Spharedia.ToString("0.000") + "), point2 = (0.0, -" + Spharedia.ToString("0.000") + "))\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].Line(point1 = (0.0, " + Spharedia.ToString("0.000") + "), point2 = ( 0.0, -" + Spharedia.ToString("0.000") + "))\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState = False, entity = mdb.models['Model-1'].sketches['__profile__'].geometry[4])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint( addUndoState = False, entity1 = mdb.models['Model-1'].sketches['__profile__'].geometry[3], entity2 =mdb.models['Model-1'].sketches['__profile__'].geometry[4])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].Part(dimensionality=THREE_D, name='SphereStart" + (i).ToString("0") + "', type=DEFORMABLE_BODY)\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['SphereStart" + (i).ToString("0") + "'].BaseSolidRevolve(angle=360.0,  flipRevolveDirection = OFF, sketch = mdb.models['Model-1'].sketches['__profile__'])\n");
                    System.IO.File.AppendAllText(Pycode, " del mdb.models['Model-1'].sketches['__profile__']\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.Instance(dependent = ON, name = 'SphereStart" + (i).ToString("0") + "',part = mdb.models['Model-1'].parts['SphereStart" + (i).ToString("0") + "'])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.translate(instanceList = ('SphereStart" + (i).ToString("0") + "', ), vector = (" + Pstart.X.ToString("0.000") + ", " + Pstart.Y.ToString("0.000") + ", " + Pstart.Z.ToString("0.000") + "))\n");

                }


                System.IO.File.AppendAllText(Pycode, "# Pipe" + i.ToString("0.0000") + " \n");

                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].ConstrainedSketch(name = '__profile__', sheetSize = 200.0)\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].CircleByCenterPerimeter(center = ( 0.0, 0.0), point1 = (0.0," + partdia.ToString("0.0000") + "))\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].Part(dimensionality = THREE_D, name = 'Pipe-GEB" + (i + 1).ToString("0") + "', type = DEFORMABLE_BODY)\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['Pipe-GEB" + (i + 1).ToString("0") + "'].BaseSolidExtrude(depth = " + Length.ToString("0.0000") + ", sketch = mdb.models['Model-1'].sketches['__profile__'])\n");
                System.IO.File.AppendAllText(Pycode, " del mdb.models['Model-1'].sketches['__profile__']\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['Pipe-GEB" + (i + 1).ToString("0") + "'].Set(cells = mdb.models['Model-1'].parts['Pipe-GEB" + (i + 1).ToString("0") + "'].cells.getSequenceFromMask(('[#1 ]',), ), name = 'Set-1')\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['Pipe-GEB" + (i + 1).ToString("0") + "'].SectionAssignment(offset = 0.0, offsetField = '', offsetType = MIDDLE_SURFACE, region = mdb.models['Model-1'].parts['Pipe-GEB" + (i + 1).ToString("0") + "'].sets['Set-1'], sectionName ='inconelSECTION', thicknessAssignment = FROM_SECTION)\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.Instance(dependent = ON, name = 'Pipe-GEB-" + (i + 1).ToString("0") + "', part = mdb.models['Model-1'].parts['Pipe-GEB" + (i + 1).ToString("0") + "'])\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.rotate(angle = -" + Xangke.ToString("0.0000") + ", axisDirection = (" + RotationAxis.X.ToString("0.0000") + ", " + RotationAxis.Y.ToString("0.0000") + "," + RotationAxis.Z.ToString("0.0000") + "), axisPoint = (0.0, 0.0, 0.0), instanceList = ('Pipe-GEB-" + (i + 1).ToString("0") + "', ))\n");
                //System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.rotate(angle = -" + Yangke.ToString("0.0000") + ", axisDirection = (0.0, 10.0, 0.0), axisPoint = (0.0, 0.0, 0.0), instanceList = ('Pipe-GEB-" + (i + 1).ToString("0") + "', ))\n");
                System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.translate(instanceList = ('Pipe-GEB-" + (i + 1).ToString("0") + "', ), vector = (" + TrasPoints.X.ToString("0.0000") + "," + TrasPoints.Y.ToString("0.0000") + ", " + TrasPoints.Z.ToString("0.0000") + "))\n");

                if (!EndExsited)
                {
                    PPs.Add(Pend);

                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].ConstrainedSketch(name = '__profile__', sheetSize = 200.0)\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0,- 0.5), point2 = (0.0, 0.5))\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity= mdb.models['Model-1'].sketches['__profile__'].geometry[2])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(center = (0.0, 0.0), direction = CLOCKWISE, point1 = (0.0, " + Spharedia.ToString("0.000") + "), point2 = (0.0, -" + Spharedia.ToString("0.000") + "))\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].Line(point1 = (0.0, " + Spharedia.ToString("0.000") + "), point2 = ( 0.0, -" + Spharedia.ToString("0.000") + "))\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState = False, entity = mdb.models['Model-1'].sketches['__profile__'].geometry[4])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint( addUndoState = False, entity1 = mdb.models['Model-1'].sketches['__profile__'].geometry[3], entity2 =mdb.models['Model-1'].sketches['__profile__'].geometry[4])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].Part(dimensionality=THREE_D, name='SphereEnd" + (i).ToString("0") + "', type=DEFORMABLE_BODY)\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['SphereEnd" + (i).ToString("0") + "'].BaseSolidRevolve(angle=360.0,  flipRevolveDirection = OFF, sketch = mdb.models['Model-1'].sketches['__profile__'])\n");
                    System.IO.File.AppendAllText(Pycode, " del mdb.models['Model-1'].sketches['__profile__']\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.Instance(dependent = ON, name = 'SphereEnd" + (i).ToString("0") + "',part = mdb.models['Model-1'].parts['SphereEnd" + (i).ToString("0") + "'])\n");
                    System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.translate(instanceList = ('SphereEnd" + (i).ToString("0") + "', ), vector = (" + Pend.X.ToString("0.000") + ", " + Pend.Y.ToString("0.000") + ", " + Pend.Z.ToString("0.000") + "))\n");

                }
            }



            //uniting struts and sphere to one model
            
            System.IO.File.AppendAllText(Pycode, " a = mdb.models['Model-1'].rootAssembly\n");
            System.IO.File.AppendAllText(Pycode, " SingleInstances_List = mdb.models['Model-1'].rootAssembly.instances.keys()\n");
            System.IO.File.AppendAllText(Pycode, " a.InstanceFromBooleanMerge(name='SingleUnit', instances=([a.instances[SingleInstances_List[i]] for i in range(len(SingleInstances_List))] ), domain = GEOMETRY, originalInstances = DELETE)\n");

            //Cutting Model lattice edges with outer box

            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='OutBox-1', part = mdb.models['Model-1'].parts['OutBox'])\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.InstanceFromBooleanCut(cuttingInstances=(mdb.models['Model-1'].rootAssembly.instances['OutBox-1'], ), instanceToBeCut = mdb.models['Model-1'].rootAssembly.instances['SingleUnit-1'], name ='CuttedUnitCell', originalInstances = DELETE)\n");
            System.IO.File.AppendAllText(Pycode, " del mdb.models['Model-1'].rootAssembly.features['CuttedUnitCell-1']\n");

            //if cannt booalena
            System.IO.File.AppendAllText(Pycode, "except:\n");
            System.IO.File.AppendAllText(Pycode, " sortie.write('%f,%f,%f' %(0,0,0))\n");
            System.IO.File.AppendAllText(Pycode, " sortie.close()\n");
            System.IO.File.AppendAllText(Pycode, " sortie = open('Done.txt', 'w')\n");
            System.IO.File.AppendAllText(Pycode, " sortie.close()\n");


            //defining walls 
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), point2 = (1.001, 1.001))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type= DEFORMABLE_BODY)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Part-1'].BaseSolidExtrude(depth=1.001, sketch= mdb.models['Model-1'].sketches['__profile__'])\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].sketches['__profile__']\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0),point2 = (1.0, 1.0))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-2', type= DEFORMABLE_BODY)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Part-2'].BaseSolidExtrude(depth=1.0, sketch= mdb.models['Model-1'].sketches['__profile__'])\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].sketches['__profile__']\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-1-1',part = mdb.models['Model-1'].parts['Part-1'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.translate(instanceList=('Part-1-1', ),vector = (-0.0005,-0.0005,-0.0005))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-2-1', part = mdb.models['Model-1'].parts['Part-2'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.InstanceFromBooleanCut(cuttingInstances=(mdb.models['Model-1'].rootAssembly.instances['Part-2-1'], ), instanceToBeCut = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], name = 'OutBoxNew', originalInstances = DELETE)\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].rootAssembly.features['OutBoxNew-1']\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].parts['Part-1']\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].parts['Part-2']\n");

            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='CuttedUnitCell',part = mdb.models['Model-1'].parts['CuttedUnitCell'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='OutBoxNew',part = mdb.models['Model-1'].parts['OutBoxNew'])\n");

            //Merging outerbox with lattice structure and define material for it

            System.IO.File.AppendAllText(Pycode, "try:\n");
            System.IO.File.AppendAllText(Pycode, " a = mdb.models['Model-1'].rootAssembly\n");
            System.IO.File.AppendAllText(Pycode, " SingleInstances_List = mdb.models['Model-1'].rootAssembly.instances.keys()\n");
            System.IO.File.AppendAllText(Pycode, " a.InstanceFromBooleanMerge(name='LsStruc', instances=([a.instances[SingleInstances_List[i]] for i in range(len(SingleInstances_List))] ), domain = GEOMETRY, originalInstances = SUPPRESS)\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['LsStruc'].Set(cells=mdb.models['Model-1'].parts['LsStruc'].cells.getSequenceFromMask(('[#1 ]', ), ), name = 'Set-1')\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['LsStruc'].SectionAssignment(offset=0.0, offsetField = '', offsetType = MIDDLE_SURFACE, region =mdb.models['Model-1'].parts['LsStruc'].sets['Set-1'], sectionName ='inconelSECTION', thicknessAssignment = FROM_SECTION)\n");

            //if cannt mesh

            System.IO.File.AppendAllText(Pycode, "except:\n");
            System.IO.File.AppendAllText(Pycode, " sortie.write('%f,%f,%f' %(0,0,0))\n");
            System.IO.File.AppendAllText(Pycode, " sortie.close()\n");
            System.IO.File.AppendAllText(Pycode, " sortie = open('Done.txt', 'w')\n");
            System.IO.File.AppendAllText(Pycode, " sortie.close()\n");

            ////RigidPlates
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(-0.3, -0.3),point2 = (1.3, 1.3))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Plate', type= DISCRETE_RIGID_SURFACE)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Plate'].BaseSolidExtrude(depth=0.1, sketch=mdb.models['Model-1'].sketches['__profile__'])\n");
            System.IO.File.AppendAllText(Pycode, "del mdb.models['Model-1'].sketches['__profile__']\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Plate'].RemoveCells(cellList=mdb.models['Model-1'].parts['Plate'].cells.getSequenceFromMask(mask = ('[#1 ]', ), ))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].parts['Plate'].ReferencePoint(point=(0.5, 0.5, 0.1))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Plate-1', part=mdb.models['Model-1'].parts['Plate'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.translate(instanceList=('Plate-1', ), vector = (0.0, 0.0, -0.1005))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Plate-2', part=mdb.models['Model-1'].parts['Plate'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.translate(instanceList=('Plate-2', ), vector = (0.0, 0.0, 1.0005))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.regenerate()\n");

            //Step and outPut Definition
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Set(name='REFPoint', referencePoints=( mdb.models['Model-1'].rootAssembly.instances['Plate-2'].referencePoints[3],))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].HistoryOutputRequest(createStepName='Step-1', name='H-Output-2', rebar = EXCLUDE, region = mdb.models['Model-1'].rootAssembly.sets['REFPoint'], sectionPoints = DEFAULT, variables = ('RF1', 'RF2', 'RF3'))\n");

            // Interaction and regid ody definition
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Surface(name='CP-1-Plate-1', side1Faces= mdb.models['Model-1'].rootAssembly.instances['Plate-1'].faces.getSequenceFromMask(('[#10 ]', ), ))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Surface(name='CP-1-LsStruc-1', side1Faces =mdb.models['Model-1'].rootAssembly.instances['LsStruc-1'].faces.getSequenceFromMask(('[#20 ]', ), ))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Surface(name='CP-2-Plate-2', side1Faces= mdb.models['Model-1'].rootAssembly.instances['Plate-2'].faces.getSequenceFromMask(('[#20 ]', ), ))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Surface(name='CP-2-LsStruc-1', side1Faces =mdb.models['Model-1'].rootAssembly.instances['LsStruc-1'].faces.getSequenceFromMask(('[#10 ]', ), ))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Tie(adjust=ON, constraintEnforcement=SURFACE_TO_SURFACE, master = mdb.models['Model-1'].rootAssembly.surfaces['CP-1-Plate-1'], name ='CP-1-Plate-1-LsStruc-1', positionToleranceMethod = COMPUTED, slave =mdb.models['Model-1'].rootAssembly.surfaces['CP-1-LsStruc-1'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].Tie(adjust=ON, constraintEnforcement=SURFACE_TO_SURFACE, master = mdb.models['Model-1'].rootAssembly.surfaces['CP-2-Plate-2'], name ='CP-2-Plate-2-LsStruc-1', positionToleranceMethod = COMPUTED, slave =mdb.models['Model-1'].rootAssembly.surfaces['CP-2-LsStruc-1'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Set(faces=mdb.models['Model-1'].rootAssembly.instances['Plate-2'].faces.getSequenceFromMask(('[#3f ]', ), ), name = 'b_Set-2')\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].RigidBody(bodyRegion= mdb.models['Model-1'].rootAssembly.sets['b_Set-2'], name = 'Constraint-3',refPointRegion = Region(referencePoints = (mdb.models['Model-1'].rootAssembly.instances['Plate-2'].referencePoints[3],)))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Set(faces=mdb.models['Model-1'].rootAssembly.instances['Plate-1'].faces.getSequenceFromMask(('[#3f ]', ), ), name = 'b_Set-4')\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].RigidBody(bodyRegion=mdb.models['Model-1'].rootAssembly.sets['b_Set-4'], name = 'Constraint-4',refPointRegion = Region(referencePoints = (mdb.models['Model-1'].rootAssembly.instances['Plate-1'].referencePoints[3],)))\n");

            //Loading and boundary Condition definition
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Set(name='Set-6', referencePoints=(mdb.models['Model-1'].rootAssembly.instances['Plate-1'].referencePoints[3],))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].EncastreBC(createStepName='Step-1', localCsys=None, name='BC-1', region = mdb.models['Model-1'].rootAssembly.sets['Set-6'])\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].rootAssembly.Set(name='Set-7', referencePoints=( mdb.models['Model-1'].rootAssembly.instances['Plate-2'].referencePoints[3],))\n");
            System.IO.File.AppendAllText(Pycode, "mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Step-1', distributionType = UNIFORM, fieldName = '', fixed= OFF, localCsys = None, name ='BC-2', region = mdb.models['Model-1'].rootAssembly.sets['Set-7'], u1 = " + DisVec.X.ToString() + ", u2 = " + DisVec.Y.ToString() + ", u3 = " + DisVec.Z.ToString() + ", ur1 = 0.0, ur2 = 0.0, ur3 = 0.0)\n");

            //meshing
            System.IO.File.AppendAllText(Pycode, "try:\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['LsStruc'].seedPart(deviationFactor=0.1, minSizeFactor = 0.1, size = 0.02)\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['LsStruc'].setMeshControls(elemShape=TET, regions =mdb.models['Model-1'].parts['LsStruc'].cells.getSequenceFromMask(('[#1 ]', ), ), technique = FREE)\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['LsStruc'].setElementType(elemTypes=(ElemType(elemCode = C3D20R, elemLibrary = STANDARD), ElemType(elemCode = C3D15,elemLibrary = STANDARD), ElemType(elemCode = C3D10, elemLibrary = STANDARD)), regions = (mdb.models['Model-1'].parts['LsStruc'].cells.getSequenceFromMask(('[#1 ]', ), ), ))\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['LsStruc'].generateMesh()\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['Plate'].seedPart(deviationFactor=0.1, minSizeFactor = 0.1, size = 0.5)\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].parts['Plate'].generateMesh()\n");
            System.IO.File.AppendAllText(Pycode, " mdb.models['Model-1'].rootAssembly.regenerate()\n");

            //if cannt mesh
            System.IO.File.AppendAllText(Pycode, "except:\n");
            System.IO.File.AppendAllText(Pycode, " sortie.write('%f,%f,%f' %(0,0,0))\n");
            System.IO.File.AppendAllText(Pycode, " sortie.close()\n");
            System.IO.File.AppendAllText(Pycode, " sortie = open('Done.txt', 'w')\n");
            System.IO.File.AppendAllText(Pycode, " sortie.close()\n");



            ////Job definition 
            System.IO.File.AppendAllText(Pycode, "mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, explicitPrecision = SINGLE, getMemoryFromAnalysis = True, historyPrint = OFF,memory = 90, memoryUnits = PERCENTAGE, model = 'Model-1', modelPrint = OFF,multiprocessingMode = DEFAULT, name = 'TestJob', nodalOutputPrecision =SINGLE, numCpus = 1, numGPUs = 0, queue = None, resultsFormat = ODB, scratch = '',type = ANALYSIS, userSubroutine = '', waitHours = 0, waitMinutes = 0)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.jobs['TestJob'].submit(consistencyChecking = OFF)\n");
            System.IO.File.AppendAllText(Pycode, "mdb.jobs['TestJob'].waitForCompletion()\n");

            //Reading RF values
            System.IO.File.AppendAllText(Pycode, "A = mdb.jobs['TestJob'].status\n");
            System.IO.File.AppendAllText(Pycode, "if ( A == COMPLETED ):\n");
            System.IO.File.AppendAllText(Pycode, "   odbname = 'TestJob'\n");
            System.IO.File.AppendAllText(Pycode, "   path='./' \n");
            System.IO.File.AppendAllText(Pycode, "   myodbpath = path + odbname + '.odb'\n");
            System.IO.File.AppendAllText(Pycode, "   odb = openOdb(myodbpath)\n");
            System.IO.File.AppendAllText(Pycode, "   region = odb.steps['Step-1'].historyRegions['Node PLATE-2.33']\n");
            System.IO.File.AppendAllText(Pycode, "   RFx = region.historyOutputs['RF1'].data[1][1]\n");
            System.IO.File.AppendAllText(Pycode, "   RFy = region.historyOutputs['RF2'].data[1][1]\n");
            System.IO.File.AppendAllText(Pycode, "   RFz = region.historyOutputs['RF3'].data[1][1]\n");
            System.IO.File.AppendAllText(Pycode, "   totallRf = sqrt(RFx * RFx + RFy * RFy + RFz * RFz)\n");
            System.IO.File.AppendAllText(Pycode, "   mask=mdb.models['Model-1'].parts['CuttedUnitCell'].cells.getMask()\n");
            System.IO.File.AppendAllText(Pycode, "   cellobj_sequence= mdb.models['Model-1'].parts['CuttedUnitCell'].cells.getSequenceFromMask(mask=mask)\n");
            System.IO.File.AppendAllText(Pycode, "   StVol=mdb.models['Model-1'].parts['CuttedUnitCell'].getVolume(cells=cellobj_sequence)\n");
            System.IO.File.AppendAllText(Pycode, "   FVratio=totallRf/StVol\n");

            System.IO.File.AppendAllText(Pycode, "   partName = odb.rootAssembly.instances['LSSTRUC-1']\n");
            System.IO.File.AppendAllText(Pycode, "   fieldOutput = odb.steps['Step-1'].frames[-1].fieldOutputs['S'].getSubset(region=partName, position=ELEMENT_NODAL)\n");
            System.IO.File.AppendAllText(Pycode, "   fieldValues = fieldOutput.values\n");
            System.IO.File.AppendAllText(Pycode, "   Smis=[]\n");
            System.IO.File.AppendAllText(Pycode, "   for q in range (0,len(fieldValues)):\n");
            System.IO.File.AppendAllText(Pycode, "        s11 = fieldValues[q].mises\n");
            System.IO.File.AppendAllText(Pycode, "        Smis.append(s11)\n");
            System.IO.File.AppendAllText(Pycode, "   MaxStress = max(Smis)\n");




            System.IO.File.AppendAllText(Pycode, "   odb.close()\n");
            System.IO.File.AppendAllText(Pycode, "else:\n");
            System.IO.File.AppendAllText(Pycode, "   totallRf = 0.00\n");
            System.IO.File.AppendAllText(Pycode, "   StVol = 0.00\n");
            System.IO.File.AppendAllText(Pycode, "   FVratio = 0.00\n");
            System.IO.File.AppendAllText(Pycode, "   MaxStress = 0.00\n");

            System.IO.File.AppendAllText(Pycode, "sortie.write('%f,%f,%f' %(totallRf,FVratio,MaxStress))\n");
            System.IO.File.AppendAllText(Pycode, "sortie.close()\n");
            System.IO.File.AppendAllText(Pycode, "sortie = open('Done.txt', 'w')\n");
            System.IO.File.AppendAllText(Pycode, "sortie.close()\n");

            System.IO.File.AppendAllText(Pycode, "sys.exit()\n");
            
            //string command1 = "/C abaqus cae noGUI=" + Modname + "Code.py";
            string command = "/C abaqus cae script=" + Modname + "Code.py";
            Process.Start("cmd.exe", command);

            string path = "C://Users//Arash//Desktop//MLGlattice//bin//Done.txt";
       
            while (true)
            {
                Thread.Sleep(3000);

                if (File.Exists(path))
                {
                    Thread.Sleep(2000);
                    //Process.GetCurrentProcess().Kill();
                    break;
                }

            }

            Tuple<double, double,double> m_RFs = ImportFEMResults("C://Users//Arash//Desktop//MLGlattice//bin//RD.txt");

            System.IO.File.Delete("C://Users//Arash//Desktop//MLGlattice//bin//RD.txt");
            System.IO.File.Delete("C://Users//Arash//Desktop//MLGlattice//bin//Done.txt");

           

            return m_RFs;
        }

        public static Tuple<double, double,double> ImportFEMResults(string FileName)
        {
            string Forces = System.IO.File.ReadAllText(FileName);

            string[] coords;
            string[] lines = Forces.Split(new string[] { "\n" }, StringSplitOptions.None);


            coords = lines[0].Split(new string[] { "," }, StringSplitOptions.None);
            double RF = Convert.ToDouble(coords[0]);
            double RFV = Convert.ToDouble(coords[1]);
            double MaxSTr = Convert.ToDouble(coords[2]);

            Tuple<double, double,double>  RFs = new Tuple<double, double,double>(RF,RFV, MaxSTr);

            return RFs;
        }

        public static List<double> VoxelIndexing(List<Brep> m_TotVoxs,List<Brep> m_SolidVoxs)
        {
            List<double> VoxInd = new List<double>();
            m_SolidVoxs = m_SolidVoxs.Distinct().ToList();
            for (int i = 0; i < m_TotVoxs.Count; i++)
            {
                bool Isssolid = false;
                Brep TestVox = m_TotVoxs[i];

                for (int j = 0; j < m_SolidVoxs.Count; j++)
                {
                    if (TestVox.Equals(m_SolidVoxs[j]))
                    {
                        Isssolid = true;
                        VoxInd.Add(1);
                        break;
                    }    
                }
                if (!Isssolid)
                {
                    VoxInd.Add(0);
                }    

            }


            return VoxInd;
        }

    }
}
