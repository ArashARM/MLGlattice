using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading.Tasks;
using Genlattice;
using System.Diagnostics;
using System.Threading;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using MLGlattice;



// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace Genlattice
{
    public class GenlatticeComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public GenlatticeComponent()
          : base("Genlattice", "Nickname",
              "Description",
              "MLG-Lattice", "MLDATA")
        {
        }

        List<NurbsCurve> m_CoreCurves;
        List<Curve> m_CNcurves;
        List<Curve> m_Mrcurves;
        List<List<Brep>> m_StrutsGraph;
        List<Brep> m_struts;
        List<Brep> m_NEWstruts;
        List<Brep> m_PrunedST;
        List<Brep[]> m_SubLat;
        List<Brep[]> m_TrunkSet;
        List<Brep[]> m_Glattice;
        //double m_Tau1;
        double m_Tau2;
        double m_Tau3;
        double m_Tau4;
        double m_Tau5;
        
        double m_alpha;
        double m_theta;
        double m_STradi;
        double m_vol;
        double STorCD;
        List<string> m_OutText;
        Interval dm;
        double tolarance;
        double angletol;
        Random random;
        private static DateTime m_PreviousTime;
        string path;
        Vector3d m_loadVecor;
        List<Brep> m_Voxels;
        
        List<Brep> m_solidVoxels;
        double m_edgeSize;
        double m_VoxNumonEdge;



        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Choice", "Choise", "1 for ST struts , 2 for CD struts", GH_ParamAccess.item);
            pManager.AddNumberParameter("m_STradi", "m_STradi", "Particle diameter", GH_ParamAccess.item);
            //pManager.AddNumberParameter("Tau4", "Tau4", "Min Volume required", GH_ParamAccess.item);
            pManager.AddNumberParameter("VoxNum", "VoxNum", "number of voxels on each edge", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddNumberParameter("ComTime", "ComTime", "ComTime", GH_ParamAccess.item);
            pManager.AddCurveParameter("listedCurves", "listedCurves", "listedCurves", GH_ParamAccess.list);
            //pManager.AddBrepParameter("Voxels", "Voxels", "Voxels", GH_ParamAccess.list);
            //pManager.AddNumberParameter("VoxIndex", "VoxIndex", "VoxIndex", GH_ParamAccess.item);
            //pManager.AddNumberParameter("Fmax", "Fmax", "Fmax", GH_ParamAccess.item);
            //pManager.AddNumberParameter("F/Vmax", "F/Vmax", "F/Vmax", GH_ParamAccess.item);
            //pManager.AddNumberParameter("MaxSt", "MaxSt", "MaxSt", GH_ParamAccess.item);


        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Random FR = new Random();
            m_loadVecor = new Vector3d();
            List<List<double>> GeneratedModelsIndex = new List<List<double>>();
            int Tracedmodels = 0;

            DA.GetData(0, ref STorCD);
            DA.GetData(1, ref m_STradi);
            //DA.GetData(2, ref m_Tau4);
            DA.GetData(2, ref m_VoxNumonEdge);

            Init();
            m_PreviousTime = DateTime.Now;
            Genlattice();


            path = "D://LatticeStructure//Traindata.csv";

            if (File.Exists(path))
            {
                GeneratedModelsIndex = GLFunctions.ReadTracedInfo(path);
                Tracedmodels = GeneratedModelsIndex.Count;
            }

            else
            {
                System.IO.File.AppendAllText(path, " Model#" + ",");
                for (int i = 0; i < Math.Pow(m_VoxNumonEdge, 3); i++)
                {
                    System.IO.File.AppendAllText(path, "X" + i.ToString() + ",");

                }

                System.IO.File.AppendAllText(path, "Y" + 0.ToString() + "(RF/V),");
                System.IO.File.AppendAllText(path, "Y" + 1.ToString() + "(MaxST),");
                System.IO.File.AppendAllText(path, "Y" + 2.ToString() + "(StrainEN)\n");
            }


            m_Voxels = GLFunctions.VoxelizeUnitcell(m_edgeSize, m_VoxNumonEdge);


            for (int i0 = Tracedmodels; i0 < 100000; i0++)
            {

                List<double> VoxIndex = new List<double>();

                m_Tau4 = FR.NextDouble() * (0.0012) + 0.0025;


                if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.lck"))
                {
                    System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.lck");
                }


                m_CoreCurves = new List<NurbsCurve>();
                bool ValidModel = true;

                if (i0 == 0)
                {
                St:
                    m_CoreCurves = new List<NurbsCurve>();
                    Genlattice();
                    var solidVox0 = GLFunctions.MapGLatticeToVoxels(m_CoreCurves, m_Voxels, out VoxIndex);

                    for (int ii = 0; ii < m_CoreCurves.Count; ii++)
                    {
                        if (m_CoreCurves[ii].GetLength() < 0.01)
                        {
                            i0--;
                            ValidModel = false;
                            goto St;
                        }
                    }
                }
                else
                {
                    List<List<NurbsCurve>> NewModels = new List<List<NurbsCurve>>();
                    List<List<double>> NewModelsIndexs = new List<List<double>>();
                    int cntr = 0;
                    for (int j = 0; j < 100; j++)
                    {
                        m_CoreCurves = new List<NurbsCurve>();
                        Genlattice();
                        ValidModel = true;
                        for (int ii = 0; ii < m_CoreCurves.Count; ii++)
                        {
                            if (m_CoreCurves[ii].GetLength() < 0.01)
                            {
                                j--;
                                cntr++;
                                ValidModel = false;
                                break;
                            }
                        }

                        if (!ValidModel)
                            continue;

                        var solidVox0 = GLFunctions.MapGLatticeToVoxels(m_CoreCurves, m_Voxels, out VoxIndex);
                        NewModelsIndexs.Add(VoxIndex);
                        var C1 = GLFunctions.DeepCopyCRV(m_CoreCurves);
                        NewModels.Add(C1);
                    }

                    m_CoreCurves = GLFunctions.SelectModelMinimizedE(NewModels, NewModelsIndexs, GeneratedModelsIndex, out VoxIndex);


                }


                Tuple<double, double, double,double> FEMresults = GLFunctions.UnitCellFEM(m_CoreCurves, new Vector3d(0, 0, -0.0001), m_STradi, m_STradi + 0.008);

                if (FEMresults.Item1 == 0 && FEMresults.Item2 == 0 && FEMresults.Item3 == 0)
                {
                    i0--;
                    continue;
                }
                for (int j = 0; j < VoxIndex.Count; j++)
                {
                    if (j == 0)
                        System.IO.File.AppendAllText(path, " Model" + i0.ToString() + ",");

                    System.IO.File.AppendAllText(path, VoxIndex[j].ToString() + ",");
                }

                System.IO.File.AppendAllText(path, FEMresults.Item2.ToString() + ",");
                System.IO.File.AppendAllText(path, FEMresults.Item3.ToString() + ",");
                System.IO.File.AppendAllText(path, FEMresults.Item4.ToString() + "\n");



                GeneratedModelsIndex.Add(VoxIndex);
                GLFunctions.SaveST(m_CoreCurves, i0.ToString());
                DeleteFEMfiles();
            }


            double TimeCom = (DateTime.Now - m_PreviousTime).Minutes;

            DA.SetDataList(0, m_CoreCurves);
        }

        void Genlattice()
        {
            if (STorCD == 1)
            {
                bool Genlat = false;
                while (Genlat == false)
                {

                    Point3d P0 = RandFirstPt();
                    m_TrunkSet = ParTrace(P0, Vector3d.Zero, m_STradi, true, out m_CoreCurves);
                    if (m_TrunkSet == null)
                        continue;

                    m_SubLat = ParProfi(m_TrunkSet, m_CoreCurves, m_STradi, out m_CoreCurves, out m_vol, out bool solved);

                    if (solved == false)
                        continue;

                    if (m_TrunkSet != null && m_SubLat != null)
                    {
                        Genlat = true;
                    }

                }

            }


            if (STorCD == 2)
            {
                bool Genlat = false;
                while (Genlat == false)
                {
                    

                    Point3d P0 = RandFirstPt();
                    m_TrunkSet = ParTraceCD(P0, Vector3d.Zero, m_STradi, out m_CoreCurves, out double V1);
                    m_SubLat = ParProfiCD(m_TrunkSet, m_CoreCurves, m_STradi, out m_CoreCurves, out m_vol);

                    if (m_TrunkSet != null && m_SubLat != null)
                    {
                        Genlat = true;
                    }
                }

            }

            
        }


        void Init()
        {
            Random rd = new Random();
            m_CoreCurves = new List<NurbsCurve>();
            m_CNcurves = new List<Curve>();
            m_Mrcurves = new List<Curve>();
            m_TrunkSet = new List<Brep[]>();
            m_SubLat = new List<Brep[]>();
            m_Glattice = new List<Brep[]>();
            m_OutText = new List<string>();
            
            m_struts = new List<Brep>();
            m_NEWstruts = new List<Brep>();
            m_StrutsGraph = new List<List<Brep>>();
            
            m_PrunedST = new List<Brep>();
            //m_STradi *= 2;
            path = "D://LatticeStructure//Traindata.csv";

            m_edgeSize = 1;
            m_Tau2 = 45;
            m_Tau3 = 95;
            m_Tau4 = rd.NextDouble()*(0.0012)+0.0025;
            m_Tau5 = 1;
            m_alpha = (90 - m_Tau2) * (Math.PI / 180);
            m_theta = (180 - m_Tau3) * (Math.PI / 180);

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//RD.txt"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//.txt");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.lck"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.lck");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//Done.txt"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//Done.txt");
            }

            //DeleteFEMfiles();

            dm = new Interval(0, 1);
            tolarance = DocumentTolerance();
            angletol = DocumentAngleTolerance();
            random = new Random();

        }


        void DeleteFEMfiles()
        {
            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//abaqus.rpy"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//abaqus.rpy");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.lok"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.lok");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.com"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.com");
            }
            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.dat"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.dat");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.inp"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.inp");
            }
            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.ipm"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.ipm");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.txt"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.txt");
            }
            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.msg"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.msg");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.odb"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.odb");
            }
            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.prt"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.prt");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.sim"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.sim");
            }

            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.sta"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//TestJob.sta");
            }
            if (File.Exists("C://Users//Arash//Desktop//MLGLattice//bin//UnitCellFEMCode.py"))
            {
                System.IO.File.Delete("C://Users//Arash//Desktop//MLGLattice//bin//UnitCellFEMCode.py");
            }
        }
        /// <summary>
        /// Randomizing a direction vector satisfying overhang and passage constraint
        /// </summary> cases : 1. PreDir=0 , Intersection , NoN-intersection
        /// <param name="PrevDir"></param>
        /// <returns></returns>
        /// 
        
        Vector3d RandStDir(Vector3d PrevDir)
        {

            Vector3d V;
            double n1, n2, n3, n4;
            Point3d[] Pt = new Point3d[2];
            Point3d Ptn;
            Point3d OveCen = new Point3d(0, 0, 1);
            var dm = new Interval(0, 1);

            m_alpha = (90 - m_Tau2) * (Math.PI / 180);
            m_theta = (180 - m_Tau3) * (Math.PI / 180);


            double Vanglex = Vector3d.VectorAngle(PrevDir, Vector3d.XAxis) * (180 / (Math.PI));
            if (Vanglex > 90 && Vanglex < 180)
            {
                Vanglex = 180 - Vanglex;
            }
            else if (Vanglex > 180)
            {
                Vanglex = Vanglex - 180;
            }
            double Vanglez = Vector3d.VectorAngle(PrevDir, Vector3d.ZAxis) * (180 / (Math.PI));
            double Ro, Ro0, Rp;

            Plane OVEPlane = new Plane(OveCen, OveCen - Point3d.Origin);
            Ro = Math.Tan(m_alpha);
            Plane PASPLane = new Plane(Point3d.Origin, PrevDir);
            Rp = Math.Tan(m_theta);



            double OVEconeDir = 1;
            if (Vanglez > 90)
            {
                OVEconeDir = -1;
            }

            Cone PASCone = new Cone(PASPLane, 1, Rp);
            Cone OVECone = new Cone(Plane.WorldXY, OVEconeDir, Ro);

            if (Vanglez > 90)
            {
                OveCen.Z *= -1;
            }

            List<double> LoadVecANG = new List<double>();
            List<Vector3d> LoadV = new List<Vector3d>();

            while (true)
            {
                if (PrevDir == Vector3d.Zero)

                {
                    if (m_Tau2 < (m_Tau3 / 2.0))
                    {

                        Ro0 = Math.Tan((90 - (m_Tau3 / 2.0)) * (Math.PI / 180));
                        OVECone.Radius = Ro0;

                        Pt = RandPtonCone(OVECone);

                    }

                    else
                    {
                        Pt = RandPtonCone(OVECone);

                    }

                }



                else if (Vanglex + (180 - m_Tau3) < 180 - m_Tau2)
                {
                    var IntCurves = InterCones(PASCone, OVECone);

                    if (IntCurves == null)
                        return Vector3d.YAxis;
                    n1 = random.NextDouble();
                    Pt[0] = IntCurves[0].PointAt(n1);
                    n2 = random.NextDouble();
                    Pt[1] = IntCurves[1].PointAt(n2);

                }

                else
                {

                    Pt = RandPtonCone(OVECone);


                }



                Line L1 = new Line(Pt[0], Pt[1]);

                n3 = random.NextDouble();

                Ptn = L1.PointAt(n3);

                V = Ptn - Point3d.Origin;

                V.Unitize();


                //n4 = random.NextDouble();
                V = Rhino.Geometry.Vector3d.Multiply(2, V);

                if (m_loadVecor == Vector3d.Zero)
                {
                    return V;
                }

                if (LoadVecANG.Count > 20)
                {
                    var Ind = LoadVecANG.IndexOf(LoadVecANG.Min());
                    return LoadV[Ind];
                }

                var Testangle = Vector3d.VectorAngle(V, m_loadVecor);
                if (Testangle > Math.PI / 2.00)
                    Testangle = Math.PI - Testangle;

                if (Testangle > (0.0174533 * 5))
                {
                    LoadVecANG.Add(Testangle);
                    LoadV.Add(V);
                    continue;
                }
                else
                    return V;
            }


        }

        /// <summary>
        /// Randomize two points on a circle;
        /// </summary>
        /// <param name="Plane"></param>
        /// <param name="Radii"></param>
        /// <returns></returns>

        Point3d[] RandPtonCone(Cone cone)
        {

            Point3d[] Pt = new Point3d[2];
            double n1, n2;

            Plane CirPlane = new Plane(cone.BasePoint, cone.Axis);
            Circle BaseCircle = new Circle(CirPlane, cone.Radius);

            n1 = random.NextDouble() * (2 * Math.PI);
            Pt[0] = BaseCircle.PointAt(n1);
            n2 = random.NextDouble() * (2 * Math.PI);
            Pt[1] = BaseCircle.PointAt(n2);

            return Pt;

        }
        /// <summary>
        /// Rand a vector in a cone!!! -Tested-
        /// </summary>
        /// <param name="cone"></param>
        /// <returns></returns>
        Vector3d RandVecCone(Cone cone)
        {

            Point3d[] Pt = new Point3d[3];
            Vector3d V = new Vector3d();
            double n1, n2, n3, n4;

            Plane CirPlane = new Plane(cone.BasePoint, cone.Axis);
            Circle BaseCircle = new Circle(CirPlane, cone.Radius);

            n1 = random.NextDouble() * (2 * Math.PI);
            Pt[0] = BaseCircle.PointAt(n1);
            n2 = random.NextDouble() * (2 * Math.PI);
            Pt[1] = BaseCircle.PointAt(n2);

            Line L1 = new Line(Pt[0], Pt[1]);
            n3 = random.NextDouble();
            Pt[2] = L1.PointAt(n3);

            V = Pt[2] - cone.ApexPoint;

            V.Unitize();

            n4 = random.NextDouble();

            V *= 2;

            return V;

        }
        /// <summary>
        /// Explorinhg intersection curves between two cones
        /// </summary> Order of cones are important!!!!
        /// <param name="pcone : Passage Cone"></param>
        /// <param name="ocone: Overhang Cone"></param>
        /// <returns></returns>
        List<NurbsCurve> InterCones(Cone pcone, Cone ocone)
        {
            List<NurbsCurve> Arcs = new List<NurbsCurve>();
            bool Cap_bottom = false;

            Vector3d PconeAxis = (pcone.Axis) * (pcone.Height);
            Vector3d OconeAxis = (ocone.Axis) * (ocone.Height);


            Brep PCone = pcone.ToBrep(Cap_bottom);
            Brep OCone = ocone.ToBrep(Cap_bottom);





            Rhino.Geometry.Intersect.Intersection.BrepBrep(PCone, OCone, tolarance, out Curve[] intcurve, out Point3d[] intpoints);

            if (intcurve.Length == 0)
                return null;

            Point3d Pt1 = intcurve[0].PointAtStart;
            Point3d Pt2 = intcurve[0].PointAtEnd;
            Point3d PtcP = new Point3d(PconeAxis.X, PconeAxis.Y, PconeAxis.Z);
            Point3d PtcO = new Point3d(OconeAxis.X, OconeAxis.Y, OconeAxis.Z);

            Vector3d V1P = Pt1 - PtcP;
            Vector3d V1O = Pt1 - PtcO;

            Plane PasConePlane = new Plane(PtcP, PconeAxis);
            Plane OveConePlane = new Plane(PtcO, OconeAxis);

            Circle CircleP = new Circle(PasConePlane, V1P.Length);
            CircleP.ClosestParameter(Pt1, out double Pt1t);
            Vector3d TanPt1 = CircleP.TangentAt(Pt1t);

            Circle CircleO = new Circle(OveConePlane, V1O.Length);
            CircleO.ClosestParameter(Pt1, out double Pt2t);
            Vector3d TanPt2 = CircleO.TangentAt(Pt2t);


            if (Rhino.Geometry.Vector3d.VectorAngle(PconeAxis, Vector3d.XAxis) > Math.PI / 2.0)
            {
                TanPt1.Reverse();
            }
            else
            {
                TanPt1.Reverse();
            }


            Arc arc1 = new Arc(Pt1, TanPt1, Pt2);
            Arc arc2 = new Arc(Pt1, TanPt2, Pt2);


            var Arc1 = arc1.ToNurbsCurve();
            var Arc2 = arc2.ToNurbsCurve();

            var dm = new Interval(0, 1);
            Arc1.Domain = dm;
            Arc2.Domain = dm;



            Arcs.Add(Arc1);
            Arcs.Add(Arc2);

            return Arcs;

        }
        /// <summary>
        /// Tracing in continous movement
        /// </summary>
        /// <param name="PartDia"></param>
        /// <param name="FirstTrace"></param>
        /// <param name="CoreCurves"></param>
        /// <returns></returns>
        List<Brep[]> ParTrace(Point3d P0, Vector3d V0, double m_STradi, bool FirstTrace, out List<NurbsCurve> CoreCurves)
        {

            int FCont = 0;


            H:

            FCont++;

            if (FCont > 5)
            {
                CoreCurves = new List<NurbsCurve>();    
                return null;
            }

            List<Brep[]> S = new List<Brep[]>();
            //List<Point3d> P = new List<Point3d>();
            Point3d[] P = new Point3d[100];
            //List<Vector3d> v = new List<Vector3d>();
            Vector3d[] v = new Vector3d[100];
            //List<Line> C = new List<Line>();
            Line[] C = new Line[100];
            List<NurbsCurve> Curves = new List<NurbsCurve>();


            // Randomizing P0 points 

            Box D = new Rhino.Geometry.Box(Plane.WorldXY, dm, dm, dm);
            Brep A = D.ToBrep();
            bool Stand = false;
            int i = 0;
            P[i] = P0;

            int counter = 0;
            while (Stand == false)
            {
                counter++;
                if (counter == 100)
                {
                    goto H;
                }
                if (i == 0)
                {
                    v[i] = RandStDir(V0);
                    if (v[i]== Vector3d.YAxis)
                    {
                        CoreCurves = new List<NurbsCurve>();
                        return null;
                    }    
                        
                }
                else
                {
                    if (P[i].DistanceTo(A.ClosestPoint(P[i])) < 0.0001)
                    {

                        v[i] = RandStDir(Vector3d.Zero);
                        if (v[i] == Vector3d.YAxis)
                        {
                            CoreCurves = new List<NurbsCurve>();
                            return null;
                        }

                        if (v[i - 1].Z < 0)
                        {
                            v[i].Z = v[i].Z * -1;
                        }
                        if (Math.Abs(P[i].X - 1.00) < 0.0001)
                        {

                            v[i].X = Math.Abs(v[i].X) * -1;
                        }
                        if (Math.Abs(P[i].X) < 0.0001)
                        {

                            v[i].X = Math.Abs(v[i].X);
                        }
                        if (Math.Abs(P[i].Y - 1.00) < 0.0001)
                        {

                            v[i].Y = Math.Abs(v[i].X) * -1;
                        }
                        if (Math.Abs(P[i].Y) < 0.0001)
                        {

                            v[i].Y = Math.Abs(v[i].X);
                        }
                    }

                    else
                    {
                        v[i] = RandStDir(v[i - 1]);
                        if (v[i] == Vector3d.YAxis)
                        {
                            CoreCurves = new List<NurbsCurve>();
                            return null;
                        }
                    }

                }
                if (P[i].Z + v[i].Z > 0.95 && P[i].Z + v[i].Z < 1.05)
                {
                    Vector3d.Multiply(1.5, v[i]);
                }



                
                C[i] = new Line(P[i], v[i],v[i].Length);
                C[i] = TrimStrut(C[i]);

                var LL = C[i].ToNurbsCurve();
                
                Point3d PPtest = LL.PointAtEnd;

                if (PPtest.Z > 0.9)
                {
                    PPtest.Z = 1;
                }

                if (PPtest.Z < 0.1)
                {
                    PPtest.Z = 0;
                }

                if (PPtest.X > 0.9)
                {
                    PPtest.X = 1;
                }

                if (PPtest.X < 0.1)
                {
                    PPtest.X = 0;
                }
                if (PPtest.Y > 0.9)
                {
                    PPtest.Y = 1;
                }

                if (PPtest.Y < 0.1)
                {
                    PPtest.Y = 0;
                }

                C[i] = new Line(P[i], PPtest);
                P[i + 1] = C[i].PointAt(1.0);

                if (P[i + 1].X > 1 || P[i + 1].X < 0 || P[i + 1].Y > 1 || P[i + 1].Y < 0)
                {
                    continue;
                }

                if (Math.Abs(Math.Round(P[i + 1].X - 1, 2)) == 0 || Math.Abs(Math.Round(P[i + 1].X, 2)) == 0 || Math.Abs(Math.Round(P[i + 1].Y - 1, 2)) == 0 || Math.Abs(Math.Round(P[i + 1].Y, 2)) == 0)
                {
                    double AngP = Vector3d.VectorAngle(Vector3d.ZAxis, P[i + 1] - P[i]);
                    if (AngP > Math.PI / 2)
                    {
                        AngP = Math.PI - AngP;
                    }

                    if (AngP * 3 < m_theta)
                    {
                        continue;
                    }

                }

                if (Math.Abs(Math.Round(P[i + 1].Z - 1, 2)) == 0 || Math.Abs(Math.Round(P[i + 1].Z, 2)) == 0)
                {
                    double AngP = Vector3d.VectorAngle(Vector3d.XAxis, P[i + 1] - P[i]);
                    if (AngP > Math.PI / 2)
                    {
                        AngP = Math.PI - AngP;
                    }

                    if (AngP * 3 < m_theta)
                    {
                        continue;
                    }

                }

                var AA = C[i].ToNurbsCurve();
                AA.Domain = dm;
                Curves.Add(AA);

                S.Add(Brep.CreatePipe(AA, m_STradi, true, PipeCapMode.Round, true, tolarance, angletol));

               

                if (P[i + 1].Z > 0.999 || P[i + 1].Z < 0.00000001)
                {
                    Stand = true;
                }

                i++;


            }

            CoreCurves = Curves;
            return S;


            //int i = 1;
            //P.Add(P0);

            //v.Add(RandStDir(V0));
            //Test = P[0] + v[0];



            //int counter = 0;

            //while (Test.X > 1 || Test.X < 0 || Test.Y > 1 || Test.Y < 0)
            //{
            //    v[0] = RandStDir(V0);
            //    Test = P[0] + v[0];
            //}


            //while (Stand)
            //{
            //    if (counter > 100)
            //    {
            //        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Cannot find requiered values");
            //        break;
            //    }

            //    Test1 = RandStDir(v[i - 1]);
            //    Test11 = P[i - 1] + Test1;
            //    counter++;

            //    if (Test11.X > 1 || Test11.X < 0 || Test11.Y > 1 || Test11.Y < 0)
            //    {
            //        continue;
            //    }

            //    P.Add(P[i - 1] + v[i - 1]);
            //    v.Add(Test1);

            //    C.Add(new Line(P[i - 1], v[i - 1], v[i - 1].Length));


            //    C[i - 1] = TrimStrut(C[i - 1]);

            //    P[i] = C[i - 1].PointAt(1.0);

            //    var AA = C[i - 1].ToNurbsCurve();

            //    AA.Domain = dm;

            //    Curves.Add(AA);


            //    S.Add(Brep.CreatePipe(AA, m_STradi, true, PipeCapMode.Round, true, tolarance, angletol));


            //    if (P[i].Z == 1 || P[i].Z < 0.00000001)
            //    {
            //        Stand = false;
            //    }

            //    i++;


            //}


            //CoreCurves = Curves;
            //return S;

        }

        Point3d RandFirstPt()
        {
            Point3d Pt = new Point3d();
            double n1, n2;

            n1 = random.NextDouble();
            n2 = random.NextDouble();

            Pt = new Point3d(n1, n2, 0);

            return Pt;
        }
        /// <summary>
        /// Trim struts out o boundary
        /// </summary>
        /// <param name="L"></param>
        /// <returns></returns>
        Line TrimStrut(Line L)
        {
            Line C = new Line();
            List<Plane> D = new List<Plane>();

            double[] IntParam = new double[6];

            /*Plane D1 = Plane.WorldXY;
            D1.Translate(Vector3d.ZAxis);
            Plane D2 = Plane.WorldZX;
            D2.Translate(Vector3d.YAxis);
            Plane D3 = Plane.WorldYZ;
            D3.Translate(Vector3d.XAxis);*/
            bool A;
            Point3d X1 = new Point3d(1, 0, 0);
            Point3d X2 = new Point3d(0, 1, 0);
            Point3d X3 = new Point3d(0, 0, 1);
            Plane D1 = new Plane(X1, Vector3d.XAxis);
            Plane D2 = new Plane(X2, Vector3d.YAxis);
            Plane D3 = new Plane(X3, Vector3d.ZAxis);

            D.Add(Plane.WorldXY);
            D.Add(Plane.WorldZX);
            D.Add(Plane.WorldYZ);
            D.Add(D1);
            D.Add(D2);
            D.Add(D3);
            
            int j = 0;
            for (int i = 0; i < 6; i++)
            {
                A = Rhino.Geometry.Intersect.Intersection.LinePlane(L, D[i], out IntParam[i]);

                if (IntParam[i] > 0.00001 && IntParam[i] < 1)
                {
                    L = new Line(L.PointAt(0.0), L.PointAt(IntParam[i]));
                }
                j++;
            }

            C = L;

            return C;
        }


        List<Brep[]> ParProfi(List<Brep[]> TrunkST, List<NurbsCurve> CoreCurves, double dia, out List<NurbsCurve> CoreCurvesUpdated, out double OcVolume, out bool Solved)
        {
            List<Brep[]> SubLat = new List<Brep[]>();
            List<Brep[]> TotalS = new List<Brep[]>();
            List<Brep> TraceStruts = new List<Brep>();
            double Vol1;
            double nd1, nd2;
            int ni1, ni2;
            bool ReqVolume = false;

            SubLat = TraceTWBounds(CoreCurves, m_STradi, out Solved);
            if (Solved == false)
            {
                OcVolume = 0;
                CoreCurvesUpdated = CoreCurves;
                return SubLat;
            }

            TotalS.AddRange(TrunkST);
            TotalS.AddRange(SubLat);

            for (int k = 0; k < TotalS.Count; k++)
                for (int i = 0; i < TotalS[k].Length; i++)
                {
                    TraceStruts.Add(TotalS[k][i]);
                }

            Vol1 = GetBrepListVol1(TraceStruts);

            CoreCurvesUpdated = CoreCurves;


            if (Vol1 > m_Tau5)
            {
                OcVolume = 0;
                CoreCurvesUpdated = null;
                return null;
            }

            if (Vol1 <= m_Tau5 && Vol1 >= m_Tau4)
            {

                OcVolume = Vol1;
                CoreCurvesUpdated = CoreCurves;
                return SubLat;
            }

            else
            {
                ReqVolume = false;
                int Counter = 0;
                while (ReqVolume == false)
                {
                    Counter++;
                    if (Counter > 100)
                    {
                        OcVolume = 0;
                        CoreCurvesUpdated = null;
                        return null;
                    }

                    Point3d Pt0 = new Point3d();
                    Point3d Pt1 = new Point3d();
                    Vector3d V0 = new Vector3d();
                    Point3d[] PtF = new Point3d[2];
                    ni1 = random.Next(0, CoreCurves.Count);
                    nd1 = random.NextDouble();

                    Pt0 = CoreCurves[ni1].PointAt(nd1);
                    Plane Bp1 = new Plane(Pt0, Vector3d.ZAxis);
                    Cone Co1 = new Cone(Bp1, 1, Math.Tan(m_alpha));
                    Cone Co2 = new Cone(Bp1, -1, Math.Tan(m_alpha));

                    ComeH:
                    if (Pt0.Z > 0.95)
                        PtF = RandPtonCone(Co2);
                    else if (Pt0.Z < 0.01)
                        PtF = RandPtonCone(Co1);
                    else
                    {
                        ni2 = random.Next(0, 2);

                        if (ni2 == 0)
                        {
                            PtF = RandPtonCone(Co1);
                        }
                        else
                        {
                            PtF = RandPtonCone(Co2);
                        }
                    } 

                    Line L1 = new Line(PtF[0], PtF[1]);

                    nd2 = random.NextDouble();
                    Pt1 = L1.PointAt(nd2);

                    V0 = Pt1 - Pt0;


                    var NewS = ParTrace(Pt0, V0, dia, true, out List<NurbsCurve> NewCoreCurves);

                    if (NewS == null)
                    {
                        Solved = false;
                        OcVolume = 0;
                        CoreCurvesUpdated = CoreCurves;
                        return SubLat;
                    }

                    Vector3d v1 = CoreCurves[ni1].TangentAtEnd;
                    Vector3d v2 = NewCoreCurves[0].TangentAtEnd;
                    double ANGG = Vector3d.VectorAngle(v1, v2) * (180 / Math.PI);

                    if (ANGG > 90)
                        ANGG = 180 - ANGG;

                    if (ANGG < 10)
                        goto ComeH;

                    bool SelfIntersect = false;

                    if (CoreCurves.Count > 100)
                    {
                        continue;
                    }
                    for (int i = 0; i < CoreCurves.Count; i++)
                    {
                        for (int j = 0; j < NewCoreCurves.Count; j++)
                        {
                            if (i == ni1)
                                continue;
                            var a1 = NewCoreCurves[j].ClosestPoints(CoreCurves[i], out Point3d Pti1, out Point3d Pti2);
                            Point3d PP0 = NewCoreCurves[j].PointAtLength(dia);

                            if ( Pti1.DistanceTo(Pti2) < 2.1*dia)
                            {
                                SelfIntersect = true;
                                break;
                            }
                        }

                        if (SelfIntersect)
                            break;
                    }

                    if (SelfIntersect)
                        continue;

                    List<Brep> NEEWs = new List<Brep>();
                    for (int i = 0; i < NewS.Count; i++)
                    {
                        for (int k1 = 0; k1 < NewS[i].Length; k1++)
                        {
                            NEEWs.Add(NewS[i][k1]);
                        }

                    }

                    double Vol2 = GetBrepListVol1(NEEWs);

                    if (Vol2 + Vol1 > m_Tau5)
                        continue;


                    CoreCurves.AddRange(NewCoreCurves);
                    SubLat.AddRange(NewS);
                    Vol1 = Vol1 + Vol2;

                    if (Vol1 > m_Tau4)
                        ReqVolume = true;

                }

                CoreCurvesUpdated = CoreCurves;
                OcVolume = Vol1;
                return SubLat;

            }






        }

        List<Brep[]> ParProfiCD(List<Brep[]> TrunkST, List<NurbsCurve> CoreCurves, double dia, out List<NurbsCurve> CoreCurvesUpdated, out double OcVolume)
        {
            List<Brep[]> SubLat = new List<Brep[]>();
            double Vol1;
            double nd1;
            int ni1;
            bool ReqVolume = false;

            SubLat = TraceTWBoundsCD(TrunkST, CoreCurves, m_STradi, out List<NurbsCurve> CoreCurves1, out double VOLL);
            Vol1 = GetBrepListVol(CoreCurves1);

            if (VOLL > m_Tau5)
            {
                OcVolume = 0;
                CoreCurvesUpdated = null;
                return null;
            }

            if (VOLL < m_Tau5 && Vol1 > m_Tau4)
            {
                OcVolume = GetBrepListVol(CoreCurves);
                CoreCurvesUpdated = CoreCurves;
                return SubLat;
            }

            else
            {
                ReqVolume = false;
                int Counter = 0;
                while (ReqVolume == false)
                {
                    Counter++;
                    if (Counter > 100)
                    {
                        OcVolume = 0;
                        CoreCurvesUpdated = null;
                        return null;
                    }

                    Point3d Pt0 = new Point3d();

                    ni1 = random.Next(0, CoreCurves.Count);
                    nd1 = random.NextDouble();

                    Pt0 = CoreCurves[ni1].PointAt(nd1);
                    if (Pt0.Z < 0.1 || Pt0.Z > 0.9)
                        continue;


                    var NewS = ParTraceCD(Pt0, Vector3d.YAxis, dia, out List<NurbsCurve> NewCoreCurves, out double VV);
                    if (NewS == null)
                        continue;

                    bool SelfIntersect = false;

                    if (CoreCurves.Count > 100)
                    {
                        continue;
                    }


                    for (int i = 0; i < CoreCurves.Count; i++)
                    {
                        for (int j = 0; j < NewCoreCurves.Count; j++)
                        {
                            var a1 = NewCoreCurves[j].ClosestPoints(CoreCurves[i], out Point3d Pti1, out Point3d Pti2);
                            Point3d PP0 = NewCoreCurves[j].PointAtLength(dia);

                            if (PP0.DistanceTo(NewCoreCurves[j].PointAtStart) < Pti1.DistanceTo(NewCoreCurves[j].PointAtStart) && Pti1.DistanceTo(Pti2) < dia + dia / 2.0)
                            {
                                SelfIntersect = true;
                                break;
                            }
                        }

                        if (SelfIntersect)
                            break;
                    }

                    if (SelfIntersect)
                        continue;

                    double Vol2 = GetBrepListVol(NewCoreCurves);

                    if (Vol2 + Vol1 > m_Tau5)
                        continue;


                    CoreCurves.AddRange(NewCoreCurves);
                    SubLat.AddRange(NewS);
                    Vol1 += Vol2;

                    if (Vol1 > m_Tau4)
                        ReqVolume = true;

                }

                OcVolume = Vol1;
                CoreCurvesUpdated = CoreCurves;
                return SubLat;

            }

        }

        List<Brep[]> TraceTWBounds(List<NurbsCurve> CoreCurves, double dia, out bool Solved)
        {
            List<Brep[]> S = new List<Brep[]>();
            double n;
            int ni;
            Point3d PtProf = new Point3d();
            Vector3d V0 = new Vector3d();
            int counter = 0;
            
            double Vol = 0.0;
            Solved = true;


            Surface[] BoundPL = new Surface[4];

            Point3d X1 = new Point3d(1, 0, 0);
            Point3d X2 = new Point3d(0, 1, 0);
            Point3d X3 = new Point3d(-0.05, 0, 0);
            Point3d X4 = new Point3d(0, -0.05, 0);
            Plane PL1 = new Plane(X1, Vector3d.XAxis);
            Plane PL2 = new Plane(X2, Vector3d.YAxis);
            Plane PL3 = new Plane(X3, Vector3d.XAxis);
            Plane PL4 = new Plane(X4, Vector3d.YAxis);

            Plane[] PL = new Plane[4];

            PL[0] = Plane.WorldZX;
            PL[1] = Plane.WorldYZ;
            PL[2] = PL1;
            PL[3] = PL2;

            for (int i = PL.Length - 1; i > 0; i--)
            {
                int randomIndex = random.Next(0, i + 1);

                Plane temp = PL[i];
                PL[i] = PL[randomIndex];
                PL[randomIndex] = temp;
            }



            BoundPL[0] = new PlaneSurface(PL[0], dm, dm);
            BoundPL[1] = new PlaneSurface(PL[1], dm, dm);
            BoundPL[2] = new PlaneSurface(PL[2], dm, dm);
            BoundPL[3] = new PlaneSurface(PL[3], dm, dm);


            for (int i = 0; i < 4; i++)
            {
                bool isbreaked = false;
                bool untouched = true;
                bool Selfintersect = false;
                Brep[] Sb0 = new Brep[1];
                List<Brep[]> TSb = new List<Brep[]>();
               

                for (int j = 0; j < CoreCurves.Count; j++)
                {
                    var IntInfo = Rhino.Geometry.Intersect.Intersection.CurveSurface(CoreCurves[j], BoundPL[i], tolarance, tolarance);

                    if (IntInfo.Count != 0)
                    {
                        untouched = false;
                        isbreaked = true;
                        break;
                    }

                }

                if (isbreaked)
                    continue;

                int X = 0;
                while (untouched)
                {
                    X = 0;
                    counter++;
                    if (counter > 500)
                    {
                        //AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, counter.ToString() + "  ModelL/Models failed  !!!!!!");
                        Solved = false;
                        break;

                    }

                    ni = random.Next(0, CoreCurves.Count);
                    n = random.NextDouble();
                    PtProf = CoreCurves[ni].PointAt(n);
                    V0 = GetDir(PtProf, PL[i]);

                    if (V0 == Vector3d.Zero)
                    {
                        continue;
                    }

                    double ANGGG = Vector3d.VectorAngle(V0, CoreCurves[ni].TangentAtEnd) * (180 / Math.PI);
                    if(ANGGG>90)
                        ANGGG =180-ANGGG;
                    if (ANGGG < 10)
                        continue;

                    untouched = false;
                    

                    
                    if (untouched == false)
                    {
                        var L1 = new Line(PtProf, PtProf + V0);
                        NurbsCurve C0 = L1.ToNurbsCurve();


                        Point3d PPtest = C0.PointAtEnd;

                        if (PPtest.Z > 0.9)
                        {
                            PPtest.Z = 1;
                        }

                        if (PPtest.Z < 0.1)
                        {
                            PPtest.Z = 0;
                        }

                        L1 = new Line(PtProf, PPtest);
                        C0 = L1.ToNurbsCurve();
                        C0.Domain = dm;

                        if (Math.Abs(Math.Round(C0.PointAtEnd.X - 1, 2)) == 0 || Math.Abs(Math.Round(C0.PointAtEnd.X, 2)) == 0 || Math.Abs(Math.Round(C0.PointAtEnd.Y - 1, 2)) == 0 || Math.Abs(Math.Round(C0.PointAtEnd.Y, 2)) == 0)
                        {
                            double AngP = Vector3d.VectorAngle(Vector3d.ZAxis, C0.PointAtEnd - C0.PointAtStart);
                            if (AngP > Math.PI / 2)
                            {
                                AngP = Math.PI - AngP;
                            }

                            if (AngP * 2 < 1)
                            {
                                continue;
                            }

                        }

                        for (int k = 0; k < CoreCurves.Count; k++)
                        {
                            if (k == ni)
                                continue;
                            var a = C0.ClosestPoints(CoreCurves[k], out Point3d Pti1, out Point3d Pti2);
                            

                            if ( Pti1.DistanceTo(Pti2) < dia*2.1)
                            {
                                Selfintersect = true;
                                break;

                            }
                        }

                        if (Selfintersect)
                        {
                            untouched = true;
                            X = 0;
                            continue;

                        }



                        Sb0 = Brep.CreatePipe(C0, dia, true, PipeCapMode.Round, true, tolarance, angletol);

                        CoreCurves.Add(C0);
                        S.Add(Sb0);
                        X++;

                        List<Brep[]> Tsb1 = new List<Brep[]>();
                        List<NurbsCurve> NewCoreCurves = new List<NurbsCurve>();

                        if (Math.Round(C0.PointAtEnd.Z,1)> 0 && C0.PointAtEnd.Z < 1)
                        {
                            //Point3d PP0 = C0.PointAtEnd;
                            //Point3d PP1 = new Point3d();
                            //double ANGG = Vector3d.VectorAngle(V0, Vector3d.ZAxis);
                            //if (ANGG < Math.PI / 2)
                            //    PP1 = new Point3d(C0.PointAtEnd.X, C0.PointAtEnd.Y, 1);
                            //else
                            //{
                            //    PP1 = new Point3d(C0.PointAtEnd.X, C0.PointAtEnd.Y, 0);
                            //}
                            //Line LL = new Line(PP0, PP1);
                            //var cc = LL.ToNurbsCurve();
                            //cc.Domain = dm;
                            //var TSb1 = Brep.CreatePipe(cc, m_STradi, true, PipeCapMode.Round, true, tolarance, angletol);

                            //CoreCurves.Add(cc);
                            //S.Add(TSb1);


                            //******************************StartOFDEFAULT
                            bool NotIntSecc = true;
                            int ContM = 0;
                            while (NotIntSecc)
                            {
                                ContM++;
                                Tsb1 = new List<Brep[]>();
                                NewCoreCurves = new List<NurbsCurve>();
                                if (Math.Abs(ContM) > 50)
                                {
                                    Solved = false;
                                    Tsb1 = null;
                                    break;
                                }

                                Tsb1 = ParTrace(C0.PointAtEnd, V0, dia, true, out NewCoreCurves);

                                if(Tsb1 == null)
                                {
                                    Solved = false;
                                    break;
                                }

                                Selfintersect = false;

                                for (int k1 = 0; k1 < CoreCurves.Count; k1++)
                                {
                                    for (int k2 = 0; k2 < NewCoreCurves.Count; k2++)
                                    {
                                        if (k1 == CoreCurves.Count - 1)
                                            continue;
                                        var a1 = NewCoreCurves[k2].ClosestPoints(CoreCurves[k1], out Point3d Pti1, out Point3d Pti2);
                                        Point3d PP0 = NewCoreCurves[k2].PointAtLength(dia);

                                        if ( Pti1.DistanceTo(Pti2) < dia*2.1)
                                        {
                                            Selfintersect = true;
                                            break;
                                        }
                                    }

                                    if (Selfintersect)
                                        break;

                                }

                                if (Selfintersect)
                                {
                                    continue;
                                }

                                NotIntSecc = false;

                            }


                            CoreCurves.AddRange(NewCoreCurves);

                            if (Tsb1 != null)
                            {
                                S.AddRange(Tsb1);
                            }

                           
                            ///******************************EndOf default

                        }


                    }


                }

                if (X == 0)
                {
                    Solved = false;
                    return null;
                }
            }



            return S;

        }

        List<Brep[]> TraceTWBoundsCD(List<Brep[]> TrunkSts, List<NurbsCurve> CoreCurves, double dia, out List<NurbsCurve> CoreCurvesUpdated, out double VOLupdated)
        {
            List<Brep[]> S = new List<Brep[]>();
            double n;
            int ni;
            Point3d PtProf = new Point3d();
            Vector3d V0 = new Vector3d();
            Point3d P0 = new Point3d();
            int counter = 0;
            int X = 0;
            double Vol = 0.0;


            Surface[] BoundPL = new Surface[4];

            Point3d X1 = new Point3d(1, 0, 0);
            Point3d X2 = new Point3d(0, 1, 0);
            Plane PL1 = new Plane(X1, Vector3d.XAxis);
            Plane PL2 = new Plane(X2, Vector3d.YAxis);

            Plane[] PL = new Plane[4];

            PL[0] = Plane.WorldZX;
            PL[1] = Plane.WorldYZ;
            PL[2] = PL1;
            PL[3] = PL2;

            for (int i = PL.Length - 1; i > 0; i--)
            {
                int randomIndex = random.Next(0, i + 1);

                Plane temp = PL[i];
                PL[i] = PL[randomIndex];
                PL[randomIndex] = temp;
            }



            BoundPL[0] = new PlaneSurface(PL[0], dm, dm);
            BoundPL[1] = new PlaneSurface(PL[1], dm, dm);
            BoundPL[2] = new PlaneSurface(PL[2], dm, dm);
            BoundPL[3] = new PlaneSurface(PL[3], dm, dm);

            bool isbreaked = false;

            for (int i = 0; i < 4; i++)
            {
                isbreaked = false;
                bool untouched = true;
                bool Selfintersect = false;
                Brep[] Sb0 = new Brep[1];
                List<Brep[]> TSb = new List<Brep[]>();
                List<NurbsCurve> NewCoreCurves = new List<NurbsCurve>();

                for (int j = 0; j < CoreCurves.Count; j++)
                {
                    var IntInfo = Rhino.Geometry.Intersect.Intersection.CurveSurface(CoreCurves[j], BoundPL[i], tolarance, tolarance);

                    if (IntInfo.Count != 0)
                    {
                        untouched = false;
                        isbreaked = true;
                        break;
                    }

                }

                if (isbreaked)
                {
                    X++;
                    continue;
                }


                while (untouched)
                {
                    counter++;
                    if (counter > 500)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Cannot find Branches!!!!!!");
                        break;

                    }

                    bool Curved = false;
                    Point3d p1 = new Point3d(-1, -1, -1);

                    int counter1 = 0;
                    while (Curved == false)
                    {
                        counter1++;
                        if (counter1 > 50)
                            break;

                        ni = random.Next(0, CoreCurves.Count);
                        n = random.NextDouble();
                        PtProf = CoreCurves[ni].PointAt(n);
                        P0 = GetDirCD(PtProf, PL[i], ((90 - m_Tau2) - 10) * (Math.PI / 180));
                        if (P0 != p1)
                            Curved = true;
                    }


                    while (Curved == false)
                    {
                        if (P0 == p1)
                        {
                            ni = random.Next(0, CoreCurves.Count);
                            n = random.NextDouble();
                            PtProf = CoreCurves[ni].PointAt(n);
                            P0 = GetDirCD(PtProf, PL[i], m_alpha);
                            if (P0 != p1)
                                Curved = true;

                        }
                    }


                    Vector3d T0 = new Vector3d();
                    Vector3d T1 = new Vector3d();

                    var Ang1 = Vector3d.VectorAngle(P0 - PtProf, Vector3d.ZAxis) * (180 / Math.PI);

                    Cone OVEcone = new Cone(Plane.WorldXY, 1, Math.Tan(m_alpha));

                    OVEcone.Height = (P0 - PtProf).Length;
                    OVEcone.Radius = OVEcone.Height * Math.Tan(m_alpha);

                    if (Ang1 > 90)
                    {
                        Ang1 = 180 - Ang1;
                        OVEcone.Height = -1 * OVEcone.Height;
                    }

                    NurbsCurve C = null;

                    bool Valid = false;
                    int counter2 = 0;

                    while (Valid == false)
                    {
                        counter2++;
                        if (counter2 > 50)
                        {
                            break;
                        }


                        T0 = RandVecCone(OVEcone);
                        T1 = RandVecCone(OVEcone);
                        C = HerimteCurve(PtProf, P0, T0, T1);
                        Valid = ValidatCD(C);
                        //Valid = true;

                        if (Valid == false)
                            continue;


                        for (int k = 0; k < CoreCurves.Count; k++)
                        {
                            var a = C.ClosestPoints(CoreCurves[k], out Point3d Pti1, out Point3d Pti2);
                            Point3d PP0 = C.PointAtLength(dia);

                            if (PP0.DistanceTo(C.PointAtStart) < Pti1.DistanceTo(C.PointAtStart) && Pti1.DistanceTo(Pti2) < dia*2.1)
                            {
                                Selfintersect = true;
                                break;

                            }
                        }

                        if (Selfintersect)
                        {
                            continue;
                        }
                    }

                    Sb0 = Brep.CreatePipe(C, dia, true, PipeCapMode.Round, true, tolarance, angletol);

                    if (T1.Z > 0)
                    {
                        T1 = Vector3d.XAxis;
                    }
                    else
                    {
                        T1 = -1 * Vector3d.XAxis;
                    }


                    if (P0.Z > 0.95 || P0.Z < 0.05)
                    {
                        CoreCurves.Add(C);
                        S.Add(Sb0);
                        untouched = false;
                    }
                    else
                    {
                        int counter3 = 0;
                        bool IINN = true;
                        while (IINN)
                        {
                            counter3++;
                            if (counter3 > 12)
                                break;

                            TSb = ParTraceCD(P0, T1, dia, out NewCoreCurves, out double Vol1);
                            if (TSb == null)
                            {
                                continue;
                            }

                            for (int k1 = 0; k1 < CoreCurves.Count; k1++)
                            {
                                for (int k2 = 0; k2 < NewCoreCurves.Count; k2++)
                                {
                                    var a1 = NewCoreCurves[k2].ClosestPoints(CoreCurves[k1], out Point3d Pti1, out Point3d Pti2);
                                    Point3d PP0 = NewCoreCurves[k2].PointAtLength(dia);

                                    if (PP0.DistanceTo(NewCoreCurves[k2].PointAtStart) < Pti1.DistanceTo(NewCoreCurves[k2].PointAtStart) && Pti1.DistanceTo(Pti2) < dia + dia / 2)
                                    {
                                        Selfintersect = true;
                                        break;
                                    }
                                }

                                if (Selfintersect)
                                    break;

                            }

                            if (Selfintersect)
                            {
                                continue;
                            }

                            CoreCurves.Add(C);
                            CoreCurves.AddRange(NewCoreCurves);

                            S.Add(Sb0);
                            S.AddRange(TSb);

                            IINN = false;
                            untouched = false;
                        }


                    }




                }

            }


            if (X == 4)
            {
                VOLupdated = GetBrepListVol(CoreCurves);
                CoreCurvesUpdated = CoreCurves;
                return TrunkSts;
            }

            VOLupdated = GetBrepListVol(CoreCurves);
            CoreCurvesUpdated = CoreCurves;

            return S;

        }
        Vector3d VecRanConePlane(Cone Cone1, Plane Plane1, out List<NurbsCurve> Curves, out bool intersected)
        {
            Vector3d V = new Vector3d(0, 0, 0);
            Point3d Pt1, Pt2, Ptn1, Ptn2, Ptn;
            Double n1, n2, n3;


            bool Cap_bottom = false;
            List<NurbsCurve> C = new List<NurbsCurve>();

            Surface SPlane = new PlaneSurface(Plane1, dm, dm);
            Brep PL = SPlane.ToBrep();

            Brep BCone = Cone1.ToBrep(Cap_bottom);
            var A = Rhino.Geometry.Intersect.Intersection.BrepSurface(BCone, SPlane, tolarance, out Curve[] IntCurves, out Point3d[] intpoint);

            if (intpoint == null)
            {
                intersected = false;
                Curves = null;

                return V;
            }
            else if (IntCurves[0].PointAtStart == IntCurves[0].PointAtEnd)
            {
                intersected = true;
                Curves = null;

                Ptn = Plane1.ClosestPoint(Cone1.BasePoint);
                V = Ptn - Cone1.ApexPoint;
                return V;

            }

            else
            {
                intersected = true;

                Pt1 = IntCurves[0].PointAtStart;
                Pt2 = IntCurves[0].PointAtEnd;
                Plane BasePlane = new Plane(Cone1.BasePoint, Cone1.Axis);
                var AAA = BasePlane.Normal;
                Circle Circle = new Circle(BasePlane, Cone1.Radius);
                Circle.ClosestParameter(Pt1, out double Pt1t);
                Vector3d TanPt1 = Circle.TangentAt(Pt1t);

                Vector3d PP1 = Plane1.ClosestPoint(Cone1.ApexPoint) - Cone1.ApexPoint;
                var ConeAxis = (Cone1.BasePoint - Cone1.ApexPoint);

                if (ConeAxis.Z > 0 && (PP1.X < 0 || PP1.Y < 0 || PP1.Z < 0))
                {
                    TanPt1.Reverse();
                }

                else if (ConeAxis.Z < 0 && (PP1.X > 0 || PP1.Y > 0 || PP1.Z > 0))
                {
                    TanPt1.Reverse();
                }

                Arc arc1 = new Arc(Pt1, TanPt1, Pt2);
                var Arc1 = arc1.ToNurbsCurve();
                Arc1.Domain = dm;

                Line L = new Line(Pt1, Pt2);
                var Li = L.ToNurbsCurve();
                Li.Domain = dm;

                C.Add(Arc1);
                C.Add(Li);
                Curves = C;

                n1 = random.NextDouble();
                Ptn1 = C[0].PointAt(n1);
                n2 = random.NextDouble();
                Ptn2 = C[1].PointAt(n2);

                Line L1 = new Line(Ptn1, Ptn2);

                n3 = random.NextDouble();

                Ptn = L1.PointAt(n3);

                V = Ptn - Cone1.ApexPoint;



                Line VecLinne = new Line(Cone1.ApexPoint, Cone1.ApexPoint + V);

                Rhino.Geometry.Intersect.Intersection.LinePlane(VecLinne, Plane1, out double t);

                V = VecLinne.PointAt(t) - Cone1.ApexPoint;

                return V;

            }

        }


        Vector3d GetDir(Point3d Location, Plane PL)
        {
            Vector3d V = new Vector3d();
            Point3d[] Pt = new Point3d[2];
            Point3d Pnext = new Point3d();
            Point3d Ptn = new Point3d();

            m_alpha = (90 - m_Tau2) * (Math.PI / 180);
            m_theta = (180 - m_Tau3) * (Math.PI / 180);

            Surface SPlane = new PlaneSurface(PL, dm, dm);

            Plane BaseP = new Plane(Location, Vector3d.ZAxis);
            var OVECONE1 = new Cone(BaseP, 1, Math.Tan(Math.PI / 5));
            var OVECONE2 = new Cone(BaseP, -1, Math.Tan(Math.PI / 5));
            bool untouched = true;
            int counter = 0;
            Line L1 = new Line();



            while (untouched)
            {
                Pt = RandPtonCone(OVECONE1);
                L1 = new Line(Pt[0], Pt[1]);
                double n3 = random.NextDouble();
                Ptn = L1.PointAt(n3);
                V = Ptn - OVECONE1.ApexPoint;
                L1 = new Line(OVECONE1.ApexPoint, OVECONE1.ApexPoint + V);

                var A = Rhino.Geometry.Intersect.Intersection.CurveSurface(L1.ToNurbsCurve(), SPlane, tolarance, tolarance);

                if (A.Count == 0)
                {
                    counter++;

                    if (counter > 50)
                    {
                        break;
                    }
                    continue;
                }

                untouched = false;

            }

            while (untouched)
            {
                Pt = RandPtonCone(OVECONE2);
                L1 = new Line(Pt[0], Pt[1]);
                double n3 = random.NextDouble();
                Ptn = L1.PointAt(n3);
                V = Ptn - OVECONE2.ApexPoint;
                L1 = new Line(OVECONE2.ApexPoint, OVECONE2.ApexPoint + V);

                var A = Rhino.Geometry.Intersect.Intersection.CurveSurface(L1.ToNurbsCurve(), SPlane, tolarance, tolarance);

                if (A.Count == 0)
                {
                    counter++;

                    if (counter > 100)
                    {
                        break;
                    }
                    continue;


                }

                untouched = false;
            }

            if (untouched == true)
            {
                V = Vector3d.Zero;
                return V;
            }



            Line L2 = new Line(Location, Location + V);
            Rhino.Geometry.Intersect.Intersection.LinePlane(L2, PL, out double t);

            V = L2.PointAt(t) - Location;


            return V;
        }

        /// <summary>
        /// Gives a point on a plane considering location of given point and overhang cone -TESTED-
        /// </summary>
        /// <param name="Location"></param>
        /// <param name="PL"></param>
        /// <returns></returns>
        Point3d GetDirCD(Point3d Location, Plane PL, double ang)
        {
            Vector3d V = new Vector3d();
            Point3d P = new Point3d();
            Point3d[] Pt = new Point3d[2];
            Point3d Pnext = new Point3d();
            Point3d Ptn = new Point3d();

            m_alpha = (90 - m_Tau2) * (Math.PI / 180);
            m_theta = (180 - m_Tau3) * (Math.PI / 180);

            Surface SPlane = new PlaneSurface(PL, dm, dm);

            Plane BaseP = new Plane(Location, Vector3d.ZAxis);
            var OVECONE1 = new Cone(BaseP, 1, Math.Tan(ang));
            var OVECONE2 = new Cone(BaseP, -1, Math.Tan(ang));
            bool untouched = true;
            int counter = 0;
            Line L1 = new Line();



            while (untouched)
            {
                Pt = RandPtonCone(OVECONE1);
                L1 = new Line(Pt[0], Pt[1]);
                double n3 = random.NextDouble();
                Ptn = L1.PointAt(n3);
                V = Ptn - OVECONE1.ApexPoint;
                L1 = new Line(OVECONE1.ApexPoint, OVECONE1.ApexPoint + V);

                var A = Rhino.Geometry.Intersect.Intersection.CurveSurface(L1.ToNurbsCurve(), SPlane, tolarance, tolarance);

                if (A.Count == 0)
                {
                    counter++;

                    if (counter > 50)
                    {
                        break;
                    }
                    continue;
                }

                untouched = false;

            }

            while (untouched)
            {
                Pt = RandPtonCone(OVECONE2);
                L1 = new Line(Pt[0], Pt[1]);
                double n3 = random.NextDouble();
                Ptn = L1.PointAt(n3);
                V = Ptn - OVECONE2.ApexPoint;
                L1 = new Line(OVECONE2.ApexPoint, OVECONE2.ApexPoint + V);

                var A = Rhino.Geometry.Intersect.Intersection.CurveSurface(L1.ToNurbsCurve(), SPlane, tolarance, tolarance);

                if (A.Count == 0)
                {
                    counter++;

                    if (counter > 100)
                    {
                        break;
                    }
                    continue;


                }

                untouched = false;
            }

            if (untouched == true)
            {
                P = new Point3d(-1, -1, -1);
                return P;
            }



            Line L2 = new Line(Location, Location + V);
            Rhino.Geometry.Intersect.Intersection.LinePlane(L2, PL, out double t);

            P = L2.PointAt(t);


            return P;
        }


        double GetBrepListVol1(List<Brep> SS)
        {
            double Vol = 0;


            for (int i = 0; i < SS.Count; i++)
            {
                double Vs = SS[i].GetVolume();
                Vol = Vol + Vs;
            }

            return Vol;

        }

        double GetBrepListVol(List<NurbsCurve> Corcurves)
        {
            double Vol = 0;


            for (int i = 0; i < Corcurves.Count; i++)
            {
                double Vs = Corcurves[i].GetLength() * (Math.Pow(m_STradi, 2)) * Math.PI;
                Vol = Vol + Vs;
            }

            return Vol;

        }

        NurbsCurve HerimteCurve(Point3d Pt0, Point3d Pt1, Vector3d T0, Vector3d T1)
        {
            NurbsCurve C = null;
            BezierCurve bC = null;
            Point3d[] CP = new Point3d[4];

            CP[0] = Pt0;
            CP[1] = Pt0 + (T0 / 3);
            CP[2] = Pt1 - (T1 / 3);
            CP[3] = Pt1;

            bC = new BezierCurve(CP);
            C = bC.ToNurbsCurve();
            C.Domain = dm;



            return C;

        }
        /// <summary>
        /// Trace automatically toward upper and lower : if Vend = zero : first --  Vend = Y free -- Vend = X side up --- Vend = -x side down ****ALL TESTED***
        /// </summary>
        /// <param name="Pt0"></param>
        /// <param name="Vend"></param>
        /// <param name="dia"></param>
        /// <param name="CoreCurves"></param>
        /// <param name="Vol"></param>
        /// <returns></returns>
        List<Brep[]> ParTraceCD(Point3d Pt0, Vector3d Vend, double dia, out List<NurbsCurve> CoreCurves, out double Vol)
        {
            List<Brep[]> TrunkStrutsCD = new List<Brep[]>();
            List<NurbsCurve> C = new List<NurbsCurve>();
            List<Vector3d> Tv = new List<Vector3d>();
            List<Point3d> Pt = new List<Point3d>();

            //int X = 0;
            //int n1;

            int i = 0;
            bool Notstand = true;

            //Plane P1 = new Plane(Pt0, Vector3d.ZAxis);
            //Cone OVEcone1 = new Cone(P1, 1, Math.Tan(m_alpha));
            //Cone OVEcone2 = new Cone(P1, -1, Math.Tan(m_alpha));
            Pt.Add(Pt0);
            Tv.Add(Vend);

            while (Notstand)
            {

                NurbsCurve S = ValidHermiteC(Pt[i], Tv[i], out Point3d Pnext, out Vector3d Tvnext);

                if (S == null)
                {
                    CoreCurves = null;
                    Vol = 0;
                    return null;
                }


                C.Add(S);

                TrunkStrutsCD.Add(Brep.CreatePipe(C[i], dia, true, PipeCapMode.Round, true, tolarance, angletol));
                Pt.Add(Pnext);
                Tv.Add(Tvnext);

                //if (Pnext.X < 0.05 || Pnext.X > 0.95 || Pnext.Y < 0.05 || Pnext.Y > 0.95)
                //{

                //    if (Tvnext.Z >= 0)
                //    {
                //        Tv.Add(Vector3d.XAxis);
                //    }
                //    if (Tvnext.Z < 0)
                //    {
                //        Tv.Add(-1*Vector3d.XAxis);
                //    }

                //}
                //else
                //{
                //    Tv.Add(Tvnext);
                //}

                if (Vend == Vector3d.Zero || Tvnext.Z > 0 || Vend == Vector3d.XAxis)
                {
                    if (Pnext.Z > 0.95)
                    {
                        Notstand = false;
                    }
                }
                if (Vend == -1 * Vector3d.XAxis || Tvnext.Z < 0)
                {
                    if (Pnext.Z < 0.05)
                    {
                        Notstand = false;
                    }
                }


                //if (Pnext.Z > 0.95 || Pnext.Z < 0.05)
                //{
                //    Notstand = false;
                //}

                i++;

            }

            Vol = GetBrepListVol(C);
            CoreCurves = C;
            return TrunkStrutsCD;
        }
        /// <summary>
        /// Return a Valid (satisfying overhang) Hermite curved within subcell <VALID>
        /// </summary>
        /// <param name="P0"></param> Point to start
        /// <param name="V0"></param> start tangent (Vector.zero if its first tracung)
        /// <param name="P1"></param>
        /// <param name="T1"></param>
        /// <returns></returns>
        NurbsCurve ValidHermiteC(Point3d P0, Vector3d V0, out Point3d P1, out Vector3d T1)
        {
            NurbsCurve C1 = null;
            NurbsCurve C = null;
            Point3d Pt1 = new Point3d();
            Vector3d Vdir = new Vector3d();
            Vector3d[] T = new Vector3d[2];
            int n1 = 0;



            m_alpha = (90 - m_Tau2) * (Math.PI / 180);
            m_theta = (180 - m_Tau3) * (Math.PI / 180);

            Cone OVEcone = new Cone(Plane.WorldXY, 1, Math.Tan(m_alpha));

            double VangleZ = Vector3d.VectorAngle(V0, Vector3d.ZAxis) * (180 / Math.PI);

            if (V0 == Vector3d.YAxis)
            {
                if (P0.Z > 0.95)
                {
                    n1 = 1;
                }
                if (P0.Z < 0.05)
                {
                    n1 = 0;
                }
                else
                {
                    n1 = random.Next(0, 2);
                }
            }

            if (VangleZ > 90 || V0 == -1 * Vector3d.XAxis || n1 == 1)
            {
                OVEcone.Height = -1;
            }

            bool Notfound = true;

            int counter = 0;


            while (Notfound)
            {

                counter++;
                if (counter > 200)
                {
                    //AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "NO WAY!!!");
                    P1 = Point3d.Origin;
                    T1 = Vector3d.Zero;

                    return null;
                }

                Vdir = RandVecCone(OVEcone);
                if (Vdir.Length < 3 * m_STradi)
                {
                    continue;
                }
                if (P0.X > -1 && P0.X < 0.0001)
                {
                    if (Vdir.X < 0)
                    {
                        continue;
                    }
                }
                if (P0.X > 0.99 && P0.X < 1.1)
                {
                    if (Vdir.X > 0)
                    {
                        continue;
                    }
                }
                if (P0.Y > -1 && P0.Y < 0.0001)
                {
                    if (Vdir.Y < 0)
                    {
                        continue;
                    }
                }
                if (P0.Y > 0.99 && P0.Y < 1.1)
                {
                    if (Vdir.Y > 0)
                    {
                        continue;
                    }
                }
                if (P0.Z > -1 && P0.Z < 0.0001)
                {
                    if (Vdir.Z < 0)
                    {
                        continue;
                    }
                }
                if (P0.Z > 0.99 && P0.Z < 1.1)
                {
                    if (Vdir.Z > 0)
                    {
                        continue;
                    }
                }

                Pt1 = P0 + Vdir;

                if (Pt1.Z < 0.05 && Pt1.Z > -0.05)
                {
                    Pt1.Z = 0;
                }
                if (Pt1.Z > 0.95 && Pt1.Z < 1.05)
                {
                    Pt1.Z = 1;
                }
                if (Pt1.X < 0.000001 && Pt1.X > -0.00001)
                {
                    Pt1.Z = 0;
                }
                if (Pt1.X > 0.98 && Pt1.X < 1.001)
                {
                    Pt1.X = 1;
                }
                if (Pt1.Y < 0.000001 && Pt1.Y > -0.00001)
                {
                    Pt1.Y = 0;
                }
                if (Pt1.Y > 0.98 && Pt1.Y < 1.001)
                {
                    Pt1.Y = 1;
                }


                OVEcone.Height = Vdir.Length;
                OVEcone.Radius = OVEcone.Height * Math.Tan(m_alpha);

                if (VangleZ > 90 || V0 == -1 * Vector3d.XAxis || n1 == 1)
                {
                    OVEcone.Height = -1 * Vdir.Length;
                }

                if (V0 == Vector3d.Zero)

                {
                    if (m_Tau2 < (m_Tau3 / 2.0))

                    {
                        OVEcone.Radius = Math.Tan((90 - (m_Tau3 / 2.0)) * (Math.PI / 180));
                        bool Accepted = true;
                        while (Accepted)
                        {
                            T[0] = RandVecCone(OVEcone);


                            if (P0.X > -0.00001 && P0.X < 0.0001)
                            {
                                if (T[0].X < 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.X > 0.99 && P0.X < 1.1)
                            {
                                if (T[0].X > 0)
                                {
                                    continue;
                                }
                            }

                            if (P0.Y > -0.00001 && P0.Y < 0.0001)
                            {
                                if (T[0].Y < 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.Y > 0.99 && P0.Y < 1.1)
                            {
                                if (T[0].Y > 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.Z > -0.00001 && P0.Z < 0.0001)
                            {
                                if (T[0].Z < 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.Z > 0.99 && P0.Z < 1.1)
                            {
                                if (T[0].Z > 0)
                                {
                                    continue;
                                }
                            }
                            Accepted = false;
                        }



                        OVEcone.Radius = Math.Tan(m_alpha);
                    }

                    else
                    {

                        bool Accepted = true;
                        while (Accepted)
                        {
                            T[0] = RandVecCone(OVEcone);


                            if (P0.X > -0.00001 && P0.X < 0.0001)
                            {
                                if (T[0].X < 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.X > 0.99 && P0.X < 1.1)
                            {
                                if (T[0].X > 0)
                                {
                                    continue;
                                }
                            }

                            if (P0.Y > -0.00001 && P0.Y < 0.0001)
                            {
                                if (T[0].Y < 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.Y > 0.99 && P0.Y < 1.1)
                            {
                                if (T[0].Y > 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.Z > -0.00001 && P0.Z < 0.0001)
                            {
                                if (T[0].Z < 0)
                                {
                                    continue;
                                }
                            }
                            if (P0.Z > 0.99 && P0.Z < 1.1)
                            {
                                if (T[0].Z > 0)
                                {
                                    continue;
                                }
                            }
                            Accepted = false;
                        }

                    }

                }

                else if (V0 == Vector3d.XAxis || V0 == -1 * Vector3d.XAxis || V0 == Vector3d.YAxis)

                {
                    bool Accepted = true;
                    while (Accepted)
                    {
                        T[0] = RandVecCone(OVEcone);


                        if (P0.X > -0.00001 && P0.X < 0.0001)
                        {
                            if (T[0].X < 0)
                            {
                                continue;
                            }
                        }
                        if (P0.X > 0.99 && P0.X < 1.1)
                        {
                            if (T[0].X > 0)
                            {
                                continue;
                            }
                        }

                        if (P0.Y > -0.00001 && P0.Y < 0.0001)
                        {
                            if (T[0].Y < 0)
                            {
                                continue;
                            }
                        }
                        if (P0.Y > 0.99 && P0.Y < 1.1)
                        {
                            if (T[0].Y > 0)
                            {
                                continue;
                            }
                        }
                        if (P0.Z > -0.00001 && P0.Z < 0.0001)
                        {
                            if (T[0].Z < 0)
                            {
                                continue;
                            }
                        }
                        if (P0.Z > 0.99 && P0.Z < 1.1)
                        {
                            if (T[0].Z > 0)
                            {
                                continue;
                            }
                        }
                        Accepted = false;
                    }


                }

                else
                {
                    T[0] = V0;
                    T[0] = T[0] * (Vdir.Length / T[0].Length);
                }


                T[1] = RandVecCone(OVEcone);

                C1 = HerimteCurve(P0, Pt1, T[0], T[1]);
                C = TrimCurve(C1);

                if (C != C1)
                {
                    Pt1 = C.PointAtEnd;
                    if (Pt1.X > 0.99 || Pt1.X < 0.05 || Pt1.Y > 0.99 || Pt1.Y < 0.05)
                    {
                        if (T[1].Z > 0)
                        {

                            T[1] = Vector3d.XAxis;
                        }
                        if (T[1].Z < 0)
                        {
                            T[1] = -1 * Vector3d.XAxis;
                        }
                    }

                }


                int X = 0;
                var EXz = C.ExtremeParameters(Vector3d.ZAxis);
                if (EXz != null)
                {
                    for (int i = 0; i < EXz.Length; i++)
                    {
                        if (EXz[i] > 0 && EXz[i] < 1)
                        {
                            X++;
                            break;

                        }
                    }

                    if (X != 0)
                    {
                        continue;
                    }

                }

                for (double j = 0; j < 1.00; j += 0.02)
                {
                    Vector3d InfTAN = C.TangentAt(j);
                    double ANG = Vector3d.VectorAngle(InfTAN, OVEcone.BasePoint - OVEcone.ApexPoint);
                    if (ANG > m_alpha)
                    {
                        X++;
                        break;
                    }
                }

                if (X != 0)
                    continue;

                Notfound = false;


            }

            P1 = Pt1;
            T1 = T[1];
            return C;

        }

      

        bool ValidatCD(NurbsCurve C)
        {
            var EXz = C.ExtremeParameters(Vector3d.ZAxis);
            if (EXz != null)
            {
                for (int i = 0; i < EXz.Length; i++)
                {
                    if (EXz[i] > 0 && EXz[i] < 1)
                    {
                        return false;

                    }
                }

            }

            for (double j = 0; j < 1.00; j += 0.02)
            {
                Vector3d InfTAN = C.TangentAt(j);
                double ANG = Vector3d.VectorAngle(InfTAN, Vector3d.ZAxis) * (180 / Math.PI);

                if (ANG > 90)
                {
                    ANG = 180 - ANG;
                }

                if (ANG > 90 - m_Tau2)
                {
                    return false;
                }
            }

            return true;


        }

        NurbsCurve TrimCurve(NurbsCurve C0)
        {
            NurbsCurve C = null;
            Point3d X1 = new Point3d(1, 0, 0);
            Point3d X2 = new Point3d(0, 1, 0);
            Point3d X3 = new Point3d(0, 0, 1);
            Plane PL1 = new Plane(X1, Vector3d.XAxis);
            Plane PL2 = new Plane(X2, Vector3d.YAxis);
            Plane PL3 = new Plane(X3, Vector3d.ZAxis);

            Plane[] PL = new Plane[6];

            PL[0] = Plane.WorldZX;
            PL[1] = Plane.WorldYZ;
            PL[2] = PL1;
            PL[3] = PL2;
            PL[4] = Plane.WorldXY;
            PL[5] = PL3;

            for (int i = PL.Length - 1; i > 0; i--)
            {
                int randomIndex = random.Next(0, i + 1);

                Plane temp = PL[i];
                PL[i] = PL[randomIndex];
                PL[randomIndex] = temp;
            }

            Surface[] BoundPL = new Surface[6];


            BoundPL[0] = new PlaneSurface(PL[0], dm, dm);
            BoundPL[1] = new PlaneSurface(PL[1], dm, dm);
            BoundPL[2] = new PlaneSurface(PL[2], dm, dm);
            BoundPL[3] = new PlaneSurface(PL[3], dm, dm);
            BoundPL[4] = new PlaneSurface(PL[4], dm, dm);
            BoundPL[5] = new PlaneSurface(PL[5], dm, dm);

            for (int i = 0; i < 6; i++)
            {

                var IntInfo = Rhino.Geometry.Intersect.Intersection.CurveSurface(C0, BoundPL[i], tolarance, tolarance);

                if (IntInfo.Count == 0 || IntInfo[0].ParameterA == 0 || IntInfo[0].ParameterA == 1)
                {
                    C = C0;
                }

                else
                {
                    double t = IntInfo[0].ParameterA;
                    var F = C0.Trim(0.0, t);
                    C = F.ToNurbsCurve();
                    C.Domain = dm;
                    C0 = C;
                }
            }

            return C;

        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("7d391ae9-af05-45d1-bde6-d2b44df96885"); }
        }
    }

}
