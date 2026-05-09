using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace ManifoldSolver.Core.ViewModels
{
    public class BendRadiusEntry : INotifyPropertyChanged
    {
        private double _value = 1.5;
        private bool _isMultiplier = true;
        private double _pipeDiameter;

        public double Value
        {
            get => _value;
            set { _value = value; OnChanged(); OnChanged(nameof(ResolvedPreview)); }
        }

        public bool IsMultiplier
        {
            get => _isMultiplier;
            set { _isMultiplier = value; OnChanged(); OnChanged(nameof(ModeLabel)); OnChanged(nameof(ResolvedPreview)); }
        }

        public double PipeDiameter
        {
            get => _pipeDiameter;
            set { _pipeDiameter = value; OnChanged(nameof(ResolvedPreview)); }
        }

        public string ModeLabel => _isMultiplier ? "×D" : "mm";

        public string ResolvedPreview => _isMultiplier
            ? $"= {_value * _pipeDiameter:F1} mm"
            : $"= {_value:F1} mm";

        public float Resolve(double pipeDiameter) =>
            _isMultiplier ? (float)(_value * pipeDiameter) : (float)_value;

        public event PropertyChangedEventHandler? PropertyChanged;

        private void OnChanged([CallerMemberName] string? prop = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
    }
}
