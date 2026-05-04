using SolidWorks.Interop.sldworks;
using System.Numerics;

namespace ManifoldSolver.UI.ViewModels
{
    public class StartPageViewModel
    {
        //public IFace2;

        public Vector3[]? StartNormals { get; set; }
        public Vector3[]? EndNormals { get; set; }


        public double TargetLength { get; set; } = 100;
        public double PipeDiameter { get; set; } = 0;
        public double MinStraight { get; set; } = 0;
        public double LengthTolerance { get; set; } = 0;
        public double Clearance { get; set; } = 0;
        public double MaxAngle { get; set; } = 180;
        public int MaxBacktrack { get; set; } = 0;
        public int MaxBends { get; set; } = 0;
    }
}
