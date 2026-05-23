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

        private double _targetLength = 490;
        public double TargetLength
        {
            get => _targetLength;
            set { _targetLength = value; OnChanged(); }
        }

        private double _pipeDiameter = 41.3;
        public double PipeDiameter
        {
            get => _pipeDiameter;
            set { _pipeDiameter = value; OnChanged(); }
        }

        private double _wallThickness = 1.5;
        public double WallThickness
        {
            get => _wallThickness;
            set { _wallThickness = value; OnChanged(); }
        }

        private double _minStraight = 4;
        public double MinStraight
        {
            get => _minStraight;
            set { _minStraight = value; OnChanged(); }
        }

        private double _lengthTolerance = 10;
        public double LengthTolerance
        {
            get => _lengthTolerance;
            set { _lengthTolerance = value; OnChanged(); }
        }

        private double _clearance = 0;
        public double Clearance
        {
            get => _clearance;
            set { _clearance = value; OnChanged(); }
        }

        private double _maxAngle = 180;
        public double MaxAngle
        {
            get => _maxAngle;
            set { _maxAngle = value; OnChanged(); }
        }

        private int _maxBacktrack = 5;
        public int MaxBacktrack
        {
            get => _maxBacktrack;
            set { _maxBacktrack = value; OnChanged(); }
        }

        private int _maxBends = 5;
        public int MaxBends
        {
            get => _maxBends;
            set { _maxBends = value; OnChanged(); }
        }

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
