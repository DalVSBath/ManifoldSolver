using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Numerics;
using System.Runtime.CompilerServices;
using SolidWorks.Interop.sldworks;

namespace ManifoldSolver.Core.ViewModels
{
    public class AnalyserViewModel : INotifyPropertyChanged
    {
        public Vector3[]? StartPoints { get; set; }
        public Vector3[]? EndPoints { get; set; }
        public Vector3[]? StartNormals { get; set; }
        public Vector3[]? EndNormals { get; set; }
        public IComponent2? Component { get; set; }
        public IComponent2[]? ConditionComponents { get; set; }

        public double TargetLength { get; set; } = 490;

        private double _pipeDiameter = 41.3;
        public double PipeDiameter
        {
            get => _pipeDiameter;
            set { _pipeDiameter = value; OnChanged(); }
        }

        public double WallThickness { get; set; } = 1.5;
        public double MinStraight { get; set; } = 4;
        public double LengthTolerance { get; set; } = 10;
        public double Clearance { get; set; } = 0;
        public double MaxAngle { get; set; } = 180;
        public int MaxBacktrack { get; set; } = 5;
        public int MaxBends { get; set; } = 5;

        public ObservableCollection<BendRadiusEntry> BendRadii { get; } = new ObservableCollection<BendRadiusEntry>
        {
            new BendRadiusEntry { Value = 1.5,  IsMultiplier = true },
            new BendRadiusEntry { Value = 2.0,  IsMultiplier = true },
            new BendRadiusEntry { Value = 2.25, IsMultiplier = true },
            new BendRadiusEntry { Value = 2.5,  IsMultiplier = true },
            new BendRadiusEntry { Value = 2.75, IsMultiplier = true },
            new BendRadiusEntry { Value = 3.0,  IsMultiplier = true },
        };

        public (bool, string?) Verify()
        {
            if (StartPoints is null) return (false, "No Start Points Selected");
            if (EndPoints is null) return (false, "No End Points Selected");
            if (StartNormals is null) return (false, "No Start Normals Selected");
            if (EndNormals is null) return (false, "No End Normals Selected");

            if (StartPoints.Length != EndPoints.Length) return (false, "All starts must have matching end points");
            if (StartNormals.Length != StartPoints.Length && StartNormals.Length != 1) return (false, $"There must be either {StartPoints.Length} or 1 normal selected.");
            if (EndNormals.Length != EndPoints.Length && EndNormals.Length != 1) return (false, $"There must be either {EndPoints.Length} or 1 normal selected.");

            return (true, null);
        }

        public event PropertyChangedEventHandler? PropertyChanged;

        private void OnChanged([CallerMemberName] string? prop = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
    }
}
