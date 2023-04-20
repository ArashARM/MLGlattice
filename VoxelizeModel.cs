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
    public class VoxelizeModel : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the VoxelizeModel class.
        /// </summary>
        public VoxelizeModel()
          : base("VoxelizeModel", "VoxelizeModel",
              "Description",
              "VoxelizeModel", "VoxelizeModel")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("CoreCurve", "CoreCurve", "CoreCurve", GH_ParamAccess.list);
            pManager.AddNumberParameter("VoxNum", "VoxNum", "number of voxels on each edge", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("VoxIndexes", "VoxIndexes", "VoxIndexes", GH_ParamAccess.list);
            pManager.AddBrepParameter("SolidVox", "SolidVox", "SolidVox", GH_ParamAccess.list);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {


            List<NurbsCurve> InputCVS = new List<NurbsCurve>();
            List<Curve> AA = new List<Curve>();
            List<double> VoxIndex = new List<double>();
            string pathNormal = "D://LST//MlartCom//NormalLoad.csv";
            //File.Delete(pathNormal);
            string pathInclined = "D://LST//MlartCom//InclindedLoad.csv";
            //File.Delete(pathInclined);
            double m_edgeSize = 1;
            double VoxOnedge = 0;
            double partdia = 0.015;

            DA.GetDataList(0, AA);
            DA.GetData(1, ref VoxOnedge);

            for (int i = 0; i < AA.Count; i++)
            {
                NurbsCurve C = AA[i].ToNurbsCurve();
                C.Domain = new Interval(0, 1);

                InputCVS.Add(C);
            }


            var m_Voxels = GLFunctions.VoxelizeUnitcell(m_edgeSize, VoxOnedge, out List<Point3d> ALLPTSS);
            var solidVox0 = GLFunctions.MapGLatticeToVoxels(InputCVS, m_Voxels, out VoxIndex);

            for (int i = 0; i < Math.Pow(VoxOnedge, 3); i++)
            {
                System.IO.File.AppendAllText(pathNormal, VoxIndex[i].ToString() + ",");

                System.IO.File.AppendAllText(pathInclined, VoxIndex[i].ToString() + ",");

            }

            var NormalLoad = GLFunctions.UnitCellFEM(InputCVS, new Vector3d(0, 0, -0.0001), partdia, partdia + 0.008);
            System.IO.File.AppendAllText(pathNormal, NormalLoad.Item2.ToString() + ",");
            System.IO.File.AppendAllText(pathNormal, NormalLoad.Item3.ToString() + "\n");

            Thread.Sleep(5000);

            var InclinedLoad = GLFunctions.UnitCellFEM(InputCVS, new Vector3d(-0.000058, -0.000058, -0.000058), partdia, partdia + 0.008);
            System.IO.File.AppendAllText(pathInclined, InclinedLoad.Item2.ToString() + ",");
            System.IO.File.AppendAllText(pathInclined, InclinedLoad.Item3.ToString() + "\n");
            DA.SetDataList(0, VoxIndex);
            DA.SetDataList(1, solidVox0);
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
            get { return new Guid("FDD718FF-E73F-4D37-8474-865EF31255B8"); }
        }
    }
}