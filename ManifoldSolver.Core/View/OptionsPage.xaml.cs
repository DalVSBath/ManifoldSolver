using ManifoldSolver.Core.ViewModels;
using System.ComponentModel;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;

namespace ManifoldSolver.Core.View
{
    public partial class OptionsPage : UserControl
    {
        private AnalyserViewModel? _vm;

        public OptionsPage()
        {
            InitializeComponent();
        }

        public AnalyserViewModel? ViewModel
        {
            get => _vm;
            set
            {
                if (_vm != null)
                    _vm.PropertyChanged -= OnVmPropertyChanged;
                _vm = value;
                DataContext = value;
                if (_vm != null)
                {
                    _vm.PropertyChanged += OnVmPropertyChanged;
                    SyncDiameterToBendRadii();
                }
            }
        }

        private void OnVmPropertyChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(AnalyserViewModel.PipeDiameter))
                SyncDiameterToBendRadii();
        }

        private void SyncDiameterToBendRadii()
        {
            if (_vm == null) return;
            foreach (var entry in _vm.BendRadii)
                entry.PipeDiameter = _vm.PipeDiameter;
        }

        private void AddRadius_Click(object sender, RoutedEventArgs e)
        {
            _vm?.BendRadii.Add(new BendRadiusEntry
            {
                Value = 1.5,
                IsMultiplier = true,
                PipeDiameter = _vm.PipeDiameter
            });
        }

        private void DeleteRadius_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is BendRadiusEntry entry)
                _vm?.BendRadii.Remove(entry);
        }

        private static readonly Regex _decimalRegex = new Regex(@"^[0-9.\-]$");
        private static readonly Regex _intRegex = new Regex(@"^[0-9\-]$");

        private void NumericBox_PreviewTextInput(object sender, TextCompositionEventArgs e)
            => e.Handled = !_decimalRegex.IsMatch(e.Text);

        private void IntBox_PreviewTextInput(object sender, TextCompositionEventArgs e)
            => e.Handled = !_intRegex.IsMatch(e.Text);

        private void RadiusValueBox_PreviewTextInput(object sender, TextCompositionEventArgs e)
            => e.Handled = !_decimalRegex.IsMatch(e.Text);
    }
}
