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
        private bool _completed = false;

        public RunnerPage()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Runs the given work on a background thread, reporting progress to this dialog.
        /// </summary>
        public Task RunAsync(Func<IProgressReporter, CancellationToken, Task> work)
        {
            var reporter = new ProgressReporter(this, _pauseGate);
            return Task.Run(async () =>
            {
                try
                {
                    await work(reporter, _cts.Token);
                    Dispatcher.Invoke(() => OnCompleted(success: true));
                }
                catch (OperationCanceledException)
                {
                    Dispatcher.Invoke(() => OnCompleted(success: false, cancelled: true));
                }
                catch (Exception ex)
                {
                    Dispatcher.Invoke(() => OnCompleted(success: false, error: ex));
                }
            });
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

        public event Action RunCompleted;
        public bool IsRunning => !_completed;

        public void RequestCancel()
        {
            _cts.Cancel();
            _pauseGate.Set();
            CancelButton.IsEnabled = false;
            CancelButton.Content = "Cancelling...";
        }

        private void CancelButton_Click(object sender, RoutedEventArgs e)
            => RequestCancel();

        private void OnCompleted(bool success, bool cancelled = false, Exception error = null)
        {
            _completed = true;
            PauseButton.IsEnabled = false;
            CancelButton.Content = "Close";
            CancelButton.IsEnabled = true;
            CancelButton.Click -= CancelButton_Click;
            CancelButton.Click += (s, e) => RunCompleted?.Invoke();

            if (cancelled) StatusText.Text = "Cancelled.";
            else if (error != null) StatusText.Text = $"Error: {error.Message}";
            else StatusText.Text = "Complete.";
        }
    }

    public interface IProgressReporter
    {
        void Report(double percent, string status = null);
        void Log(string line);
        void ThrowIfCancelled();
        void WaitIfPaused();
    }

    internal class ProgressReporter : IProgressReporter
    {
        private readonly RunnerPage _dialog;
        private readonly ManualResetEventSlim _pauseGate;

        public ProgressReporter(RunnerPage dialog, ManualResetEventSlim pauseGate)
        {
            _dialog = dialog;
            _pauseGate = pauseGate;
        }

        public void Report(double percent, string status = null)
            => _dialog.UpdateProgress(percent, status);
        public void Log(string line) => _dialog.AppendLog(line);
        public void ThrowIfCancelled() { /* worker uses CancellationToken directly */ }
        public void WaitIfPaused() => _pauseGate.Wait();
    }
}