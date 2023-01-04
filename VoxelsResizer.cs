using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MLGlattice
{
    public class VoxelsResizer : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the VoxelsResizer class.
        /// </summary>
        public VoxelsResizer()
          : base("VoxelsResizer", "Nickname",
              "Description",
              "MLG-Lattice", "ResizeVox")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("FileLoc", "FileLoc", "FileLoc", GH_ParamAccess.item);
            pManager.AddNumberParameter("VoxelSize", "VoxelSize", "VoxelSize", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("SolidVox0", "SolidVox0", "SolidVox0", GH_ParamAccess.item);
            pManager.AddBrepParameter("SolidVox1", "SolidVox1", "SolidVox1", GH_ParamAccess.item);
            pManager.AddBrepParameter("SolidVox2", "SolidVox2", "SolidVox2", GH_ParamAccess.item);
            pManager.AddBrepParameter("SolidVox3", "SolidVox3", "SolidVox3", GH_ParamAccess.item);
            pManager.AddBrepParameter("EMPVOX0", "EMPVOX0", "EMPVOX0", GH_ParamAccess.item);
            pManager.AddBrepParameter("EMPVOX1", "EMPVOX1", "EMPVOX1", GH_ParamAccess.item);
            pManager.AddBrepParameter("EMPVOX2", "EMPVOX2", "EMPVOX2", GH_ParamAccess.item);
            pManager.AddBrepParameter("EMPVOX3", "EMPVOX3", "EMPVOX3", GH_ParamAccess.item);
            //pManager.AddBrepParameter("SolidVox", "SolidVox", "SolidVox", GH_ParamAccess.item);
            //pManager.AddBrepParameter("SolidVox", "SolidVox", "SolidVox", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string path = "D://LatticeStructure//";
            string pathNew = "";
            string pathOld = "";
            double Voxsize = 0;
            List<List<double>> TracedData = new List<List<double>>();
            List<List<double>> NewIndexes = new List<List<double>>();
            List<List<Brep>> SolidBR = new List<List<Brep>>();
            List<List<Brep>> VacantBR = new List<List<Brep>>();

            DA.GetData(0, ref path);
            DA.GetData(1, ref Voxsize);

            
            pathNew = path + "Traindata" + Voxsize.ToString("0")+".csv";
            pathOld = path + "Traindata.csv";


            var Voxels = GLFunctions.VoxelizeUnitcell(1, Voxsize);
            TracedData = GLFunctions.ReadTracedInfo1(pathOld);

            System.IO.File.AppendAllText(pathNew, " Model#" + ",");
            for (int i = 0; i < Math.Pow(Voxsize, 3); i++)
            {
                System.IO.File.AppendAllText(pathNew, "X" + i.ToString() + ",");

            }

            System.IO.File.AppendAllText(pathNew, "Y" + 0.ToString() + "(RF/V),");
            System.IO.File.AppendAllText(pathNew, "Y" + 1.ToString() + "(MaxST),");
            System.IO.File.AppendAllText(pathNew, "Y" + 2.ToString() + "(StrainEN)\n");


            for (int i = 0; i < TracedData.Count; i++)
            {
                System.IO.File.AppendAllText(pathNew, " Model" + i.ToString() + ",");

                var GL = GLFunctions.LoadFiles(path + "GLattice" + i.ToString("0") + ".txt");
                List<double> VoxelIND = new List<double>();
                List<Brep> SOL = new List<Brep>();
                List<Brep> VAC = new List<Brep>();
                var Svoxz = GLFunctions.MapGLatticeToVoxels(GL, Voxels, out VoxelIND);
                NewIndexes.Add(VoxelIND);

                for (int k1 = 0; k1 < VoxelIND.Count; k1++)
                {
                    if (VoxelIND[k1] == 1)
                    {
                        SOL.Add(Voxels[k1]);
                        continue;
                    }
                    VAC.Add(Voxels[k1]);
                }


                SolidBR.Add(SOL);
                VacantBR.Add(VAC);
                for (int j = 0; j < VoxelIND.Count; j++)
                {
                    System.IO.File.AppendAllText(pathNew, VoxelIND[j].ToString() + ",");
                }

                System.IO.File.AppendAllText(pathNew, TracedData[i][0].ToString() + ",");
                System.IO.File.AppendAllText(pathNew, TracedData[i][1].ToString() + ",");
                System.IO.File.AppendAllText(pathNew, TracedData[i][2].ToString() + "\n");
            }

            for (int i = 0; i < SolidBR.Count; i++)
            {
                DA.SetDataList(i, SolidBR[i]);
            }
            for (int i = 0; i < SolidBR.Count; i++)
            {
                DA.SetDataList(SolidBR.Count+i, VacantBR[i]);
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
            get { return new Guid("8A2B0CF7-9992-42FD-B6B7-DC4558BCDF92"); }
        }
    }
}