using ManifoldSolver.Core.ViewModels;
using System.Windows;

namespace ManifoldSolver.Core
{
    public partial class MainWindow : Window
    {
        public MainWindow(AnalyserViewModel vm)
        {
            InitializeComponent();
            OptionsControl.ViewModel = vm;
            RunnerControl.RunCompleted += () => Close();
        }

        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (RunnerControl.IsRunning)
            {
                e.Cancel = true;
                RunnerControl.RequestCancelAndClose();
            }
        }
    }
}
