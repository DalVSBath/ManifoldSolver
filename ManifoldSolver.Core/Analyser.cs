using ManifoldSolver.Core.View;
using SolidWorks.Interop.sldworks;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Interop;

namespace ManifoldSolver.Core
{
    public class Analyser
    {
        public async Task RunAnalysis(ISldWorks swApp)
        {
            // Lock SW user control so they can't edit while we run
            swApp.UserControl = false;

            var dialog = new RunnerPage();
            WindowOwnerHelper.OwnToSolidWorks(dialog, swApp);
            dialog.Show();

            try
            {
                await dialog.RunAsync(async (progress, ct) =>
                {
                    progress.Log("Starting analysis...");

                    for (int i = 0; i < 100; i++)
                    {
                        ct.ThrowIfCancellationRequested();
                        progress.WaitIfPaused();

                        // Pure C# work — fine on background thread
                        await Task.Delay(50, ct);

                        progress.Report(i + 1, $"Processing step {i + 1} of 100");
                        if (i % 10 == 0) progress.Log($"Reached step {i + 1}");
                    }

                    progress.Log("Done.");
                });
            }
            finally
            {
                swApp.UserControl = true;
            }
        }

        public static class WindowOwnerHelper
        {
            public static void OwnToSolidWorks(Window window, ISldWorks swApp)
            {
                // Get SW's main window handle
                IntPtr swHwnd = new IntPtr(swApp.IFrameObject().GetHWnd());

                // Set it as the owner of our WPF window
                var helper = new WindowInteropHelper(window);
                helper.Owner = swHwnd;
            }
        }
    }
}
