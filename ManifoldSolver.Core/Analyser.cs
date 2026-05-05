using ManifoldSolver.Core.ViewModels;
using SolidWorks.Interop.sldworks;
using System;
using GeometrySolver.Solver;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Interop;
using static GeometrySolver.Solver.ManifoldGeoSolver;
using System.Collections.Generic;

namespace ManifoldSolver.Core
{
    public class Analyser
    {
        public async Task RunAnalysis(ISldWorks swApp, AnalyserViewModel vm)
        {

            var mainWindow = new MainWindow();
            WindowOwnerHelper.OwnToSolidWorks(mainWindow, swApp);

            var manifold = new ManifoldGeoSolver
            {
                BendRadii = new float[] {(float)(1.5 * vm.PipeDiameter), 
                    (float)(2 * vm.PipeDiameter), (float)(2.25 * vm.PipeDiameter), (float)(2.5 * vm.PipeDiameter), 
                    (float)(2.75 * vm.PipeDiameter), (float)(3 * vm.PipeDiameter) },
                TargetLength = (float)vm.TargetLength,
                Diameter = (float)vm.PipeDiameter,
                MaxBends = vm.MaxBends,
                Verbose = false,
                EqualizeLength = false,
                MinStraightLength = (float)vm.MinStraight,
                MinClearance = 0f,
                ClearanceExcludeEndMm = 0f,
                LengthToleranceFraction = 0.03f,  // ±3% = 582–618 mm
                MaxBacktrackCandidates = vm.MaxBacktrack,       // 5 candidates per pipe for backtracking
            };

            //mainWindow.DataControl.

            List<PipeDef> Pipe = new List<PipeDef>();

            for (int i = 0; i < vm.StartPoints.Length; i++)
            {
                var sNormal = vm.StartNormals.Length == 1 ? vm.StartNormals[1] : vm.StartNormals[i];
                var eNormal = vm.EndNormals.Length == 1 ? vm.EndNormals[1] : vm.EndNormals[i];

                var def = new PipeDef(vm.StartPoints[i], sNormal, vm.EndPoints[i], eNormal);

                mainWindow.DataControl.AddRow(def);
                Pipe.Add(def);
            }



            mainWindow.Show();

            try
            {
                // Lock SW user control so they can't edit while we run
                swApp.UserControl = false;
                await mainWindow.RunnerControl.RunAsync(async (progress, ct) =>
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
