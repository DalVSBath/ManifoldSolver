using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace ManifoldSolver.Core.View
{
    public partial class RunnerPage : System.Windows.Controls.UserControl
    {
        private CancellationTokenSource _cts = new CancellationTokenSource();
        private ManualResetEventSlim _pauseGate = new ManualResetEventSlim(true); // set = running
        private bool _isPaused = false;
        private bool _isRunning = false;
        private bool _closeAfterCancel = false;

        public RunnerPage()
        {
            InitializeComponent();
        }

        internal void UpdateProgress(double percent, string status)
        {
            Dispatcher.Invoke(() =>
            {
                ProgressBar.Value = percent;
                if (status != null) StatusText.Text = status;
            });
        }

        internal void AppendLog(string line)
        {
            Dispatcher.Invoke(() =>
            {
                LogBox.AppendText($"[{DateTime.Now:HH:mm:ss}] {line}{Environment.NewLine}");
                LogBox.ScrollToEnd();
            });
        }

        private void PauseButton_Click(object sender, RoutedEventArgs e)
        {
            _isPaused = !_isPaused;
            if (_isPaused)
            {
                _pauseGate.Reset();   // worker will block at the gate
                PauseButton.Content = "Resume";
                AppendLog("--- Paused ---");
            }
            else
            {
                _pauseGate.Set();     // worker resumes
                PauseButton.Content = "Pause";
                AppendLog("--- Resumed ---");
            }
        }


        private void CancelButton_Click(object sender, RoutedEventArgs e) { }
        private void StartButton_Click(object sender, RoutedEventArgs e) { }
    }
}