using GeometrySolver.Conditions;
using GeometrySolver.Solver;
using ManifoldSolver.Core.ViewModels;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
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
using System.Windows.Forms;
using System.Windows.Interop;
using System.Windows.Shapes;

namespace ManifoldSolver.Core
{
    public class Analyser
    {
        private ISldWorks _swApp;
        private IComponent2? _component;
        private MainWindow mainWindow;
        private ManifoldGeoSolver? manifold;
        private AnalyserViewModel? _vm;
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
            foreach (var line in File.ReadLines("C:\\Users\\dalhome3\\source\\repos\\GeometrySolver\\TestPipes-V1.csv").Skip(1))
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

        public async Task RunAnalysis(AnalyserViewModel vm, IComponent2? component = null)
        {
            _vm = vm;
            if (vm.Component != null)
                component = vm.Component;
            _component = component;
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
            bool CSV = false;

            if(CSV)
            {
                mainWindow.DataControl.Clear();
                Pipes = GetCSVPipes();
            }
#endif

            mainWindow.RunnerControl.StartButton.Click += (object o, RoutedEventArgs r) => RunFullSolver();
            mainWindow.DataControl.PreAnalyse.Click += PreAnalyse_Click;
            mainWindow.DataControl.BuildObstacles.Click += DoBuildObs;

            mainWindow.Show();

            
        }

        private void DoBuildObs(object o, RoutedEventArgs r)
        {
            BuildObstacleConditions();
        }

        private void BuildObstacleConditions()
        {
            if (_vm?.ConditionComponents == null || _vm.ConditionComponents.Length == 0)
            {
                mainWindow.RunnerControl.AppendLog("[Obstacles] No condition components set.");
                return;
            }

            mainWindow.RunnerControl.RunAsync(async (progress, ct) =>
            {
                progress.Log($"Building obstacle conditions for {_vm.ConditionComponents.Length} component(s)...");
                manifold.ClearSharedConditions();

                foreach (var comp in _vm.ConditionComponents)
                {
                    var condition = BuildOptimalObstacleCondition(comp);
                    manifold.AddSharedCondition(condition);
                    progress.Log($"  [{condition.Type}] added for {comp.Name}");
                }

                progress.Log("Obstacle conditions ready.");
            });
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
                        PipeOrders.Add(2000 + i, Pipes[i]);
                    } else
                    {
                        PipeOrders.Add(Math.Abs(results[i].TotalLength - manifold.TargetLength), Pipes[i]);
                    }
                }

                Pipes = PipeOrders.OrderByDescending(k => k.Key).Select(p => p.Value).ToList();
                mainWindow.RunnerControl.Dispatcher.Invoke(() => { mainWindow.DataControl.Clear();  foreach (var p in Pipes) mainWindow.DataControl.AddRow(p); });


            });
        }



        bool solved = false;
        List<SolverResult?> results;

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


                    if (!solved)
                    {
                        manifold.OnUpdateLog += (object o, string msg) => progress.Log(msg);
                        results = manifold.Solve();
                        solved = true;
                    }

                    if (_component != null)
                        SketchBuilder.DrawPipeSketches(_swApp, _component, Pipes, results, progress.Log);

                    progress.Log("Done.");
                });
            }
            finally
            {
                _swApp.UserControl = true;
            }
        }

        private ISolverCondition BuildOptimalObstacleCondition(IComponent2 comp)
        {
            var mathUtil = (MathUtility)_swApp.GetMathUtility();

            // Build component-to-world transform by walking up parent chain
            MathTransform compToWorld = comp.Transform2;
            var parent = comp.GetParent() as Component2;
            while (parent != null)
            {
                compToWorld = (MathTransform)parent.Transform2.Multiply(compToWorld);
                parent = parent.GetParent() as Component2;
            }

            var vertices = GetWorldVerticesMm(mathUtil, comp, compToWorld);

            if (vertices.Count == 0)
            {
                // No tessellation available — fall back to tight AABB from GetBox
                var rawBox = (double[])comp.GetBox(false, false);
                return new ObstacleCondition(new IgnoreRect(
                    new Vector3((float)(rawBox[0] * 1000), (float)(rawBox[1] * 1000), (float)(rawBox[2] * 1000)),
                    new Vector3((float)(rawBox[3] * 1000), (float)(rawBox[4] * 1000), (float)(rawBox[5] * 1000))));
            }

            // AABB from actual surface vertices
            var aabbMin = vertices[0]; var aabbMax = vertices[0];
            foreach (var v in vertices) { aabbMin = Vector3.Min(aabbMin, v); aabbMax = Vector3.Max(aabbMax, v); }
            float aabbVol = (aabbMax.X - aabbMin.X) * (aabbMax.Y - aabbMin.Y) * (aabbMax.Z - aabbMin.Z);
            IIgnoreArea aabb = new IgnoreRect(aabbMin, aabbMax);

            // Sphere: center at AABB center, radius = max distance to any surface vertex
            var aabbCenter = (aabbMin + aabbMax) * 0.5f;
            float sphereR = 0f;
            foreach (var v in vertices) sphereR = Math.Max(sphereR, Vector3.Distance(aabbCenter, v));
            float sphereVol = (float)((4f / 3f) * Math.PI * sphereR * sphereR * sphereR);
            IIgnoreArea sphere = new IgnoreSphere(aabbCenter, sphereR);

            // Collect cylinder axis candidates: X/Y/Z plus any axis found on an actual
            // cylindrical face. For a tilted cylinder part the face axis is exact;
            // X/Y/Z cover non-cylindrical or axis-aligned parts.
            var cylAxes = new List<Vector3> { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ };

            var faceBody = comp.GetBody() as IBody2;
            if (faceBody != null)
            {
                var faces = faceBody.GetFaces() as object[];
                if (faces != null)
                {
                    foreach (var fObj in faces)
                    {
                        if (fObj is not IFace2 face) continue;
                        var surf = face.GetSurface() as ISurface;
                        if (surf == null || !surf.IsCylinder()) continue;
                        var cp = surf.CylinderParams as double[];
                        if (cp == null || cp.Length < 7) continue;

                        // cp = [px, py, pz, ax, ay, az, radius] in metres (local space)
                        var localAxis = new Vector3((float)cp[3], (float)cp[4], (float)cp[5]);
                        var worldAxis = TransformDirection(mathUtil, compToWorld, localAxis);

                        // Deduplicate: skip axes nearly parallel to one already in the list
                        bool duplicate = false;
                        foreach (var existing in cylAxes)
                            if (Math.Abs(Vector3.Dot(worldAxis, existing)) > 0.9999f)
                                { duplicate = true; break; }
                        if (!duplicate) cylAxes.Add(worldAxis);
                    }
                }
            }

            IIgnoreArea bestCyl = null; float bestCylVol = float.MaxValue;
            foreach (var axis in cylAxes)
            {
                var (cyl, vol) = TightCylinder(vertices, axis);
                if (vol < bestCylVol) { bestCyl = cyl; bestCylVol = vol; }
            }

            IIgnoreArea winner;
            if (aabbVol <= sphereVol && aabbVol <= bestCylVol) winner = aabb;
            else if (sphereVol <= bestCylVol)                  winner = sphere;
            else                                               winner = bestCyl;

            return new ObstacleCondition(winner);
        }

        private static Vector3 TransformDirection(MathUtility mathUtil, MathTransform xform, Vector3 localDir)
        {
            // Rotate a direction (not a point) through the transform by translating a unit
            // vector from the origin and subtracting the transformed origin.
            double[] origin = { 0, 0, 0 };
            double[] tip = { localDir.X, localDir.Y, localDir.Z };
            var ptOrigin = (MathPoint)((MathPoint)mathUtil.CreatePoint(origin)).MultiplyTransform(xform);
            var ptTip    = (MathPoint)((MathPoint)mathUtil.CreatePoint(tip)).MultiplyTransform(xform);
            var o = (double[])ptOrigin.ArrayData;
            var t = (double[])ptTip.ArrayData;
            return Vector3.Normalize(new Vector3((float)(t[0] - o[0]), (float)(t[1] - o[1]), (float)(t[2] - o[2])));
        }

        private static List<Vector3> GetWorldVerticesMm(
            MathUtility mathUtil, IComponent2 comp, MathTransform compToWorld)
        {
            var result = new List<Vector3>();

            var body = comp.GetBody() as IBody2;
            if (body == null) return result;

            var tess = body.GetTessellation(null) as ITessellation;
            if (tess == null) return result;

            tess.NeedFaceFacetMap = true;
            tess.NeedVertexParams = true;
            tess.ImprovedQuality  = true;
            tess.MatchType        = (int)swTesselationMatchType_e.swTesselationMatchFacetTopology; // swTesselationMatchFacetTopology

            if (!tess.Tessellate()) return result;

            // Traverse face → facets → fins → vertices (SolidWorks tessellation model).
            // Deduplicate by vertex ID so shared-edge vertices are only transformed once.
            var visited = new HashSet<int>();
            var face = body.GetFirstFace() as IFace2;
            while (face != null)
            {
                var facetIds = tess.GetFaceFacets(face) as int[];
                if (facetIds != null)
                {
                    foreach (int facetId in facetIds)
                    {
                        var finIds = tess.GetFacetFins(facetId) as int[];
                        if (finIds == null) continue;
                        foreach (int finId in finIds)
                        {
                            var vertexIds = tess.GetFinVertices(finId) as int[];
                            if (vertexIds == null) continue;
                            foreach (int vid in vertexIds)
                            {
                                if (!visited.Add(vid)) continue;
                                var coords = tess.GetVertexPoint(vid) as double[];
                                if (coords == null || coords.Length < 3) continue;

                                // coords are in metres, local component space
                                var pt      = (MathPoint)mathUtil.CreatePoint(new double[] { coords[0], coords[1], coords[2] });
                                var worldPt = (MathPoint)pt.MultiplyTransform(compToWorld);
                                var d       = (double[])worldPt.ArrayData;
                                result.Add(new Vector3((float)(d[0] * 1000), (float)(d[1] * 1000), (float)(d[2] * 1000)));
                            }
                        }
                    }
                }
                face = face.GetNextFace() as IFace2;
            }

            return result;
        }

        private static (IIgnoreArea cylinder, float volume) TightCylinder(List<Vector3> vertices, Vector3 axis)
        {
            // Find height range and centroid of perpendicular projections.
            // Using the perpendicular centroid as the axis-line position minimises the
            // average radius; for symmetric parts (cylinders, spheres) it equals the true axis.
            float minProj = float.MaxValue, maxProj = float.MinValue;
            var perpCentroid = Vector3.Zero;
            foreach (var v in vertices)
            {
                float proj = Vector3.Dot(v, axis);
                if (proj < minProj) minProj = proj;
                if (proj > maxProj) maxProj = proj;
                perpCentroid += v - proj * axis;
            }
            perpCentroid /= vertices.Count;

            float height = maxProj - minProj;
            var cylCenter = perpCentroid + axis * ((minProj + maxProj) * 0.5f);

            // Radius = max perpendicular distance from the axis line through cylCenter
            float radius = 0f;
            foreach (var v in vertices)
            {
                var delta = v - cylCenter;
                float axialComp = Vector3.Dot(delta, axis);
                float perpDistSq = delta.LengthSquared() - axialComp * axialComp;
                radius = (float)Math.Max(radius, Math.Sqrt(Math.Max(0f, perpDistSq)));
            }

            float volume = (float)( Math.PI * radius * radius * height);
            return (new IgnoreCylinder(cylCenter, axis, radius, height), volume);
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