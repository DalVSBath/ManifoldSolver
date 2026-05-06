using GeometrySolver.Solver;
using ManifoldSolver.Core.ViewModels;
using SolidWorks.Interop.sldworks;
using System;
using System.Collections.Generic;
using System.Globalization;

#if DEBUG
using System.IO;
#endif
using System.Linq;
using System.Numerics;
using System.Security.Cryptography;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Interop;
using System.Windows.Shapes;

namespace ManifoldSolver.Core
{
    public class Analyser
    {
        private ISldWorks _swApp;
        private MainWindow mainWindow;
        private ManifoldGeoSolver? manifold;
        List<PipeDef> Pipes;


        public readonly record struct PipeDef(string Name,
            Vector3 Start, Vector3 StartDir, Vector3 End, Vector3 EndDir);

        public Analyser(ISldWorks swApp)
        {
            mainWindow = new MainWindow();

            _swApp = swApp;
        }

#if DEBUG
        static float F(string s) => float.Parse(s.Trim(), CultureInfo.InvariantCulture);

        private List<PipeDef> GetCSVPipes ()
        {
            List<PipeDef> rows = new List<PipeDef>();
            foreach (var line in File.ReadLines("C:\\Users\\Dan\\Documents\\SW-Addins\\ManifoldSolver\\TestPipes-V1.csv").Skip(1))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var c = line.Split(',');
                if (c.Length < 13) continue;

                var def = new PipeDef(
                    c[0].Trim(),
                    new Vector3(F(c[1]), F(c[2]), F(c[3])),
                    Vector3.Normalize(new Vector3(F(c[4]), F(c[5]), F(c[6]))),
                    new Vector3(F(c[7]), F(c[8]), F(c[9])),
                    Vector3.Normalize(new Vector3(F(c[10]), F(c[11]), F(c[12]))));

                rows.Add(def);
                mainWindow.DataControl.AddRow(def);
            }

            return rows;
        }
#endif

        public async Task RunAnalysis(AnalyserViewModel vm)
        {
            WindowOwnerHelper.OwnToSolidWorks(mainWindow, _swApp);

            manifold = new ManifoldGeoSolver
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

            Pipes = new List<PipeDef>();

            for (int i = 0; i < vm.StartPoints.Length; i++)
            {
                var sNormal = Vector3.Normalize(vm.StartNormals.Length == 1 ? vm.StartNormals[0] : vm.StartNormals[i]);
                var eNormal = Vector3.Normalize(vm.EndNormals.Length == 1 ? vm.EndNormals[0] : vm.EndNormals[i]);

                var def = new PipeDef($"Pipe {i}", vm.StartPoints[i], sNormal, vm.EndPoints[i], eNormal);

                mainWindow.DataControl.AddRow(def);
                Pipes.Add(def);
            }

#if DEBUG
            bool CSV = true;

            if(CSV)
            {
                mainWindow.DataControl.Clear();
                Pipes = GetCSVPipes();
                foreach (var p in Pipes) mainWindow.DataControl.AddRow(p);
            }
#endif

            mainWindow.RunnerControl.StartButton.Click += (object o, RoutedEventArgs r) => RunFullSolver();

            mainWindow.DataControl.PreAnalyse.Click += PreAnalyse_Click;

            mainWindow.Show();

            
        }

        private void PreAnalyse_Click(object sender, RoutedEventArgs e)
        {
            manifold.ClearPipes();
            foreach (var p in Pipes)
            {
                manifold.AddPipe(p.Start, p.StartDir, p.End, p.EndDir);
            }

            mainWindow.RunnerControl.RunAsync(async (progress, ct) =>
            {
                progress.Log("Starting pre-detemination...");

                manifold.OnUpdateLog += (object o, string msg) => progress.Log(msg);

                var results = manifold.GetNaturals();

                if (results == null)
                    return;

                progress.Log("Done. Reordering pipes by percieved difficulty.");

                Dictionary<float, PipeDef> PipeOrders = new Dictionary<float, PipeDef>();

                for (int i = 0; i < Pipes.Count; i++)
                {
                    if (results[i] == null)
                    {
                        progress.Log("Potential Unsolvability - Care");
                        PipeOrders.Add(2000, Pipes[i]);
                    } else
                    {
                        PipeOrders.Add(Math.Abs(results[i].TotalLength - manifold.TargetLength), Pipes[i]);
                    }
                }

                Pipes = PipeOrders.OrderByDescending(k => k.Key).Select(p => p.Value).ToList();
                mainWindow.RunnerControl.Dispatcher.Invoke(() => { mainWindow.DataControl.Clear();  foreach (var p in Pipes) mainWindow.DataControl.AddRow(p); });


            });
        }



        private async Task RunFullSolver()
        {
            try
            {
                // Lock SW user control so they can't edit while we run
                _swApp.UserControl = false;

                manifold.ClearPipes();
                foreach (var p in Pipes)
                {
                    manifold.AddPipe(p.Start, p.StartDir, p.End, p.EndDir);
                }

                await mainWindow.RunnerControl.RunAsync(async (progress, ct) =>
                {
                    progress.Log("Starting analysis...");

                    manifold.OnUpdateLog += (object o, string msg) => progress.Log(msg);

                    var results = manifold.Solve();

                    progress.Log("Done.");
                });
            }
            finally
            {
                _swApp.UserControl = true;
            }
        }

        private void Manifold_OnUpdateLog(object sender, string e)
        {
            throw new NotImplementedException();
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


namespace System.Runtime.CompilerServices
{
    internal static class IsExternalInit { }
}