using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MLGlattice
{
    public class TESTCPS : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TESTCPS class.
        /// </summary>
        public TESTCPS()
          : base("TESTCPS", "Nickname",
              "Description",
              "MNX", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("VoxNum","VoxNum", "VoxNum",GH_ParamAccess.item);
            pManager.AddNumberParameter("EdgS", "EdgS", "EdgS", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("CEnpts","CenPts","CenPts",GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double edgeSize = 0;
            double VoxNumberOnEdge = 0;

            DA.GetData(0, ref edgeSize);
            DA.GetData(1, ref VoxNumberOnEdge);

            var m_Voxels = GLFunctions.VoxelizeUnitcell(edgeSize, VoxNumberOnEdge,out List<Point3d> Alpts);

            DA.GetDataList(0, Alpts);
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
            get { return new Guid("9B03E79B-665D-4145-ABF0-94B369C3B669"); }
        }
    }
}