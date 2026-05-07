using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using SolidWorks.Interop.sldworks;

namespace ManifoldSolver.Core.ViewModels
{
    public class AnalyserViewModel
    {
        public Vector3[]? StartPoints { get; set; }
        public Vector3[]? EndPoints { get; set; }

        public Vector3[]? StartNormals { get; set; }
        public Vector3[]? EndNormals { get; set; }

        public IComponent2? Component { get; set; }

        public IComponent2[]? ConditionComponents { get; set; }

        public double TargetLength { get; set; } = 100;
        public double PipeDiameter { get; set; } = 0;
        public double MinStraight { get; set; } = 0;
        public double LengthTolerance { get; set; } = 0;
        public double Clearance { get; set; } = 0;
        public double MaxAngle { get; set; } = 180;
        public int MaxBacktrack { get; set; } = 0;
        public int MaxBends { get; set; } = 0;

        public (bool, string?) Verify()
        {
            if (StartPoints is null) return (false, "No Start Points Selected");
            if (EndPoints is null) return (false, "No End Points Selected");
            if (StartNormals is null) return (false, "No Start Normals Selected");
            if (EndNormals is null) return (false, "No End Normals Selected");

            if (StartPoints.Length != EndPoints.Length) return (false, "All starts must have maatching end points");
            if (StartNormals.Length != StartPoints.Length && StartNormals.Length != 1) return (false, $"There must be either {StartPoints.Length} or 1 normal selected.");
            if (EndNormals.Length != EndPoints.Length && EndNormals.Length != 1) return (false, $"There must be either {EndPoints.Length} or 1 normal selected.");

            return (true, null);
        }
    }
}
