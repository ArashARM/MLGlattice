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
using MLGlattice;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace MLGlattice
{
    public class MLGlatticeComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MLGlatticeComponent()
          : base("MLGlattice", "Nickname",
              "Description",
              "MLG-Lattice", "Subcategory")
        {
        }

        List<NurbsCurve> m_GLattice;
        List<Brep> m_Voxels;
        double m_StRadii;
        List<Brep> m_solidVoxels;

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("INPUTCRVS", "INPUTCRVS", "INPUTCRVS", GH_ParamAccess.list);
            pManager.AddNumberParameter("Radii", "Radii", "Radii", GH_ParamAccess.item);
            pManager.AddNumberParameter("UnitEdgeSiz", "UnitEdgeSiz", "UnitEdgeSiz", GH_ParamAccess.item);
            pManager.AddNumberParameter("VoxNumberOnEdge", "VoxNumberOnEdge", "VoxNumberOnEdge", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddPointParameter("Cenpoints", "Cenpoints", "Cenpopints", GH_ParamAccess.list);
            pManager.AddBrepParameter("Voxels", "Voxels", "Voxels", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double edgeSize = 0;
            double VoxNumberOnEdge = 0;

        
            List<NurbsCurve> InputCVS = new List<NurbsCurve>();
            List<Curve> AA = new List<Curve>();


            DA.GetDataList(0, AA);
            DA.GetData(1, ref m_StRadii);
            DA.GetData(2, ref edgeSize);  
            DA.GetData(3, ref VoxNumberOnEdge);

            

            for (int i = 0; i < AA.Count; i++)
            {
                NurbsCurve C = AA[i].ToNurbsCurve();
                C.Domain = new Interval(0, 1);

                InputCVS.Add(C);
            }


            Init();

            m_Voxels = GLFunctions.VoxelizeUnitcell(edgeSize, VoxNumberOnEdge, out List<Point3d> ALLPTSS);
            //m_solidVoxels = GLFunctions.MapGLatticeToVoxels(InputCVS, m_StRadii, m_Voxels);


            DA.SetDataList(0, m_solidVoxels);
        }

        void Init()
        {
            m_GLattice = new List<NurbsCurve>();
            m_Voxels = new List<Brep>();
            m_solidVoxels = new List<Brep>();

        }
        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
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
            get { return new Guid("86209ae2-d6ab-487f-a40b-58cf69e0988a"); }
        }
    }
}
