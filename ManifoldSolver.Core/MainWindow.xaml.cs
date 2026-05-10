using ManifoldSolver.Core.View;
using ManifoldSolver.Core.ViewModels;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace ManifoldSolver.Core
{
    public partial class MainWindow : Window
    {
        private CancellationTokenSource _cts = new CancellationTokenSource();
        private ManualResetEventSlim _pauseGate = new ManualResetEventSlim(true); // set = running
        private bool _isPaused = false;
        private bool _isRunning = false;
        private bool _closeAfterCancel = false;


        public event Action RunCompleted;
        public bool IsRunning => _isRunning;


        public MainWindow(AnalyserViewModel vm)
        {
            InitializeComponent();
            OptionsControl.ViewModel = vm;
            RunCompleted += () => RunComplete();
        }

        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (_isRunning)
            {
                e.Cancel = true;
                RequestCancelAndClose();
            }
        }

        public Task RunAsync(Func<IProgressReporter, CancellationToken, Task> work)
        {
            _isRunning = true;
            RunnerControl.StartButton.IsEnabled = false;
            RunnerControl.PauseButton.IsEnabled = true;
            RunnerControl.CancelButton.IsEnabled = true;

            RunnerControl.CancelButton.Click += CancelButton_Click;

            var reporter = new ProgressReporter(this.RunnerControl, _pauseGate);
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


        public void RequestCancel() => RequestCancel(closeAfter: false);
        public void RequestCancelAndClose() => RequestCancel(closeAfter: true);
        private void RequestCancel(bool closeAfter)
        {
            _closeAfterCancel = closeAfter;
            _cts.Cancel();
            _pauseGate.Set();
            RunnerControl.CancelButton.IsEnabled = false;
            RunnerControl.CancelButton.Content = "Cancelling...";
        }
        private void CancelButton_Click(object sender, RoutedEventArgs e)
            => RequestCancel();



        private void OnCompleted(bool success, bool cancelled = false, Exception error = null)
        {
            _isRunning = false;
            RunnerControl.StartButton.IsEnabled = true;
            RunnerControl.PauseButton.IsEnabled = false;
            RunnerControl.CancelButton.IsEnabled = false;

            if (_closeAfterCancel)
            {
                Close();
                return;
            }

            RunnerControl.CancelButton.Content = "Close";
            RunnerControl.CancelButton.IsEnabled = true;
            RunnerControl.CancelButton.Click -= CancelButton_Click;
            RunnerControl.CancelButton.Click += (s, e) => RunCompleted?.Invoke();

            if (cancelled) RunnerControl.StatusText.Text = "Cancelled.";
            else if (error != null) RunnerControl.StatusText.Text = $"Error: {error.Message}";
            else RunnerControl.StatusText.Text = "Complete.";
        }



        // Floater Function
        private void RunComplete() { }

        // Unsused copy code
        /*
        internal void UpdateProgress(double percent, string status)
        {
            Dispatcher.Invoke(() =>
            {
                RunnerControl.ProgressBar.Value = percent;
                if (status != null) RunnerControl.StatusText.Text = status;
            });
        }

        internal void AppendLog(string line)
        {
            Dispatcher.Invoke(() =>
            {
                RunnerControl.LogBox.AppendText($"[{DateTime.Now:HH:mm:ss}] {line}{Environment.NewLine}");
                RunnerControl.LogBox.ScrollToEnd();
            });
        }
        */
    }

    public interface IProgressReporter
    {
        void Report(double percent, string status = "");
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

        public void Report(double percent, string status = "")
            => _dialog.UpdateProgress(percent, status);
        public void Log(string line) => _dialog.AppendLog(line);
        public void ThrowIfCancelled() { /* worker uses CancellationToken directly */ }
        public void WaitIfPaused() => _pauseGate.Wait();
    }
}
