using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Threading;


namespace MLGlattice
{
    public class MLGLatANALIZE : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MLGLatANALIZE class.
        /// </summary>
        public MLGLatANALIZE()
          : base("MLGLatANALIZE", "Nickname",
              "Description",
              "X", "MLGLA")
        {
        }

        List<NurbsCurve> m_CCurvs;
        List<Point3d> m_Gpts;

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("CoreCurve", "CoreCurve", "CoreCurve", GH_ParamAccess.list);
            //pManager.AddTextParameter("ModelName", "modelNAme", "ModelName", GH_ParamAccess.item);
            //pManager.AddNumberParameter("LoadIndi", "LoadIndi", "LoadIndi", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("merCurve", "merCurve", "merCurve", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string Modname = "AA";
            double loadindi = 0;
            Vector3d DisVec = new Vector3d();
            m_CCurvs = new List<NurbsCurve>();
            List<NurbsCurve> InputCVS = new List<NurbsCurve>();
            m_Gpts = new List<Point3d>();
            double partdia = 0.015;
            double Spharedia = 0.03;
            List<Curve> AA = new List<Curve>();
            //double Tw = 0;


            //var Vects = GenLoadDir(45, 2, 4);


            DA.GetDataList(0, AA);
            //DA.GetData(1, ref Modname);
            //DA.GetData(2, ref loadindi);

            //DisVec = Vects[0];

            for (int i = 0; i < AA.Count; i++)
            {
                NurbsCurve C = AA[i].ToNurbsCurve();
                C.Domain = new Interval(0, 1);

                InputCVS.Add(C);
            }


            DA.SetDataList(0, InputCVS);

            var A = GLFunctions.UnitCellFEM(InputCVS, new Vector3d(0, 0, -0.0001), partdia, partdia + 0.008);


        }

        private List<Vector3d> GenLoadDir(double MaxXangle, int XSeg, int Zseg)
        {
            Vector3d v0 = new Vector3d(0, 0, 1);
            List<Vector3d> VectList = new List<Vector3d>();

            double endAngleX = MaxXangle;
            int DivX = XSeg;
            double rotANGX = endAngleX / DivX;
            rotANGX = (rotANGX * Math.PI / 180);

            double endAngleZ = 90;
            int DivZ = Zseg;
            double rotANGZ = endAngleZ / DivZ;
            rotANGZ = (rotANGZ * Math.PI / 180);

            VectList.Add(v0);
            for (int i = 1; i < DivX + 1; i++)
            {
                var V1 = new Vector3d(v0.X, v0.Y, v0.Z);

                double roang1 = rotANGX * i;
                V1.Rotate(roang1, Vector3d.YAxis);

                for (int j = 0; j < DivZ; j++)
                {
                    Vector3d Vt = new Vector3d(V1.X, V1.Y, V1.Z);

                    double roang = rotANGZ * j;

                    Vt.Rotate(roang, Vector3d.ZAxis);

                    VectList.Add(Vt);

                }
            }

            for (int i = 0; i < VectList.Count; i++)
            {
                VectList[i] = VectList[i] * -0.0001;
            }

            return VectList;
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("72A507C0-4380-47E3-AA5E-3C020BC6AEF7"); }
        }
    }
}