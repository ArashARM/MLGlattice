using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MLGlattice
{
    public class ResoChange : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the ResoChange class.
        /// </summary>
        public ResoChange()
          : base("ResoChange", "Nickname",
              "Description",
              "MLG-Lattice", "Res")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("edgeSize", "edgesize", "edgesize", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double Es = 0;

            DA.GetData(0, ref Es);

            List<List<NurbsCurve>> GLaticcess = new List<List<NurbsCurve>>();

            for (int i = 35; i < 1016; i++)
            {
                List<NurbsCurve> Glatt = new List<NurbsCurve>();
                Glatt = GLFunctions.LoadFiles("D://MLModels//GLattice" + i.ToString() + ".txt");
                GLaticcess.Add(Glatt);
            }

            for (int i = 0; i < GLaticcess.Count; i++)
            {

            }
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
            get { return new Guid("A61AD423-610C-44BD-92BE-844024650C3F"); }
        }
    }
}