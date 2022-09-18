using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace MLGlattice
{
    public class MLGlatticeInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "MLGlattice";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("93c1a863-5836-40f5-8fd3-3ab29deba7f2");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
