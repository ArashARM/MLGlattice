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



namespace MLGlattice
{
    public class MlGLatInclinedLoad : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MlGLatInclinedLoad class.
        /// </summary>
        public MlGLatInclinedLoad()
          : base("MlGLatInclinedLoad", "Nickname",
              "Description",
              "MLG-Lattice", "Inclined Loading")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("GLatticeDir", "GLatticeDir", "GLatticeDir", GH_ParamAccess.item);
            pManager.AddVectorParameter("DisVec", "DisVec", "DisVec", GH_ParamAccess.item);
            pManager.AddNumberParameter("StartIND", "StartIND", "StartIND", GH_ParamAccess.item);
            pManager.AddNumberParameter("EndIND", "EndIND", "EndIND", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string path = "D://LatticeStructure//";
            Vector3d DisVec = new Vector3d();
            double StartIND = 0;
            double EndIND = 0;

            DA.GetData(0, ref path);
            DA.GetData(1, ref DisVec);
            DA.GetData(2, ref StartIND);
            DA.GetData(3, ref EndIND);


            double m_STradi = 0.015;
            string pathOld = path + "Traindata.csv";
            string pathNEW = path + "TraindataINCLIMED.csv";

            if (File.Exists(pathNEW))
            {
                var GeneratedModelsIndex = GLFunctions.ReadTracedInfo22(pathNEW);
                StartIND = StartIND+GeneratedModelsIndex.Count;
            }

            else
            {
                System.IO.File.AppendAllText(pathNEW, " Model#" + ",");
                for (int i = 0; i < Math.Pow(10, 3); i++)
                {
                    System.IO.File.AppendAllText(pathNEW, "X" + i.ToString() + ",");

                }

                System.IO.File.AppendAllText(pathNEW, "Y" + 0.ToString() + ",");
                System.IO.File.AppendAllText(pathNEW, "Y" + 1.ToString() + "\n");
            }

            var TracedData = GLFunctions.ReadTracedInfo22(pathOld);

            for (int i = (int)StartIND; i < (int)EndIND; i++)
            {
               

                var GL = GLFunctions.LoadFiles(path + "GLattice" + i.ToString("0") + ".txt");

                Tuple<double, double, double> FEMresults = GLFunctions.UnitCellFEM(GL, DisVec, m_STradi, m_STradi + 0.008);



                System.IO.File.AppendAllText(pathNEW, " Model" + i.ToString() + ",");

                for (int j = 0; j < TracedData[i].Count; j++)
                {
                    System.IO.File.AppendAllText(pathNEW, TracedData[i][j].ToString() + ",");
                }


                System.IO.File.AppendAllText(pathNEW, FEMresults.Item2.ToString() + ",");
                System.IO.File.AppendAllText(pathNEW, FEMresults.Item3.ToString() + "\n");
            }

        }
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>

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
            get { return new Guid("34D046CF-5BFE-4B24-8EA9-BB24B26E5874"); }
        }
    }
}