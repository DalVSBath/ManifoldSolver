using System.Windows;

namespace ManifoldSolver.Core
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            RunnerControl.RunCompleted += () => Close();
        }

        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (RunnerControl.IsRunning)
            {
                e.Cancel = true;
                RunnerControl.RequestCancel();
            }
        }
    }
}
