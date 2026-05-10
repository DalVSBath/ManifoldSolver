using GeometrySolver.Conditions;
using GeometrySolver.Solver;
using ManifoldSolver.Core.ViewModels;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.InteropServices;

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
        private readonly List<Body2> _previewBodies = new List<Body2>();


        public readonly record struct PipeDef(string Name,
            Vector3 Start, Vector3 StartDir, Vector3 End, Vector3 EndDir);

        public Analyser(ISldWorks swApp)
        {
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

            mainWindow = new MainWindow(vm);
            WindowOwnerHelper.OwnToSolidWorks(mainWindow, _swApp);

            manifold = new ManifoldGeoSolver
            {
                BendRadii = vm.BendRadii.Select(r => r.Resolve(vm.PipeDiameter)).ToArray(),
                TargetLength = (float)vm.TargetLength,
                Diameter = (float)vm.PipeDiameter - 3f,
                MaxBends = vm.MaxBends,
                Verbose = true,
                EqualizeLength = false,
                MinStraightLength = (float)vm.MinStraight,
                MinClearance = (float)vm.Clearance,
                ClearanceExcludeEndMm = 0f,
                LengthToleranceFraction = vm.TargetLength > 0 ? (float)(vm.LengthTolerance / vm.TargetLength) : 0f,
                MaxBacktrackCandidates = vm.MaxBacktrack,
                MaxBendAngleDeg = (float)vm.MaxAngle,
            };

            //mainWindow.DataControl.

            Pipes = new List<PipeDef>();

            for (int i = 0; i < vm.StartPoints.Length; i++)
            {
                var sNormal = Vector3.Normalize(vm.StartNormals.Length == 1 ? vm.StartNormals[0] : vm.StartNormals[i]);
                var eNormal = Vector3.Normalize(vm.EndNormals.Length == 1 ? vm.EndNormals[0] : vm.EndNormals[i]);

                var def = new PipeDef($"Pipe {i}", vm.StartPoints[i], sNormal, vm.EndPoints[i], eNormal * -1);

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
            mainWindow.Closed += (s, e) => ClearPreviewBodies();

            mainWindow.Show();

            
        }

        private void ClearPreviewBodies()
        {
            foreach (var body in _previewBodies)
                Marshal.ReleaseComObject(body);
            _previewBodies.Clear();
            (_swApp.ActiveDoc as ModelDoc2)?.GraphicsRedraw2();
        }

        private void DoBuildObs(object o, RoutedEventArgs r)
        {
            mainWindow.DataControl.BuildObstacles.IsEnabled = false;
            BuildObstacleConditions();
            mainWindow.DataControl.BuildObstacles.IsEnabled = true;
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
                ClearPreviewBodies();

                progress.Log($"Building obstacle conditions for {_vm.ConditionComponents.Length} component(s)...");
                manifold.ClearSharedConditions();

                int totalComps = _vm.ConditionComponents.Length;
                for (int ci = 0; ci < totalComps; ci++)
                {
                    var comp = _vm.ConditionComponents[ci];
                    double compBase  = ci * 100.0 / totalComps;
                    double compSlice = 100.0 / totalComps;
                    var condition = BuildOptimalObstacleCondition(comp, d => progress.Report(d), compBase, compSlice);
                    manifold.AddSharedCondition(condition);
                    progress.Log($"  [{condition.Type}] added for {comp.Name}");

                    var preview = ((ObstacleCondition)condition).Area switch
                    {
                        IgnoreRect r     => ObstaclePreview.PreviewIgnoreRect(r, _swApp, _component),
                        IgnoreCylinder c => ObstaclePreview.PreviewIgnoreCylinder(c, _swApp, _component),
                        IgnoreSphere s   => ObstaclePreview.PreviewIgnoreSphere(s, _swApp, _component),
                        _                => null
                    };
                    if (preview != null)
                        _previewBodies.Add(preview);
                }

                ((ModelDoc2)_swApp.ActiveDoc).GraphicsRedraw2();
                progress.Log("Obstacle conditions ready.");

                const float WarnDist = 10f;
                foreach (var cond in manifold.SharedConditions.OfType<ObstacleCondition>())
                {
                    foreach (var pipe in Pipes)
                    {
                        float ds = cond.Area.DistanceTo(pipe.Start);
                        float de = cond.Area.DistanceTo(pipe.End);
                        if (ds < WarnDist)
                            progress.Log($"  [WARN] Obstacle [{cond.Type}] is {ds:F1} mm from {pipe.Name} start");
                        if (de < WarnDist)
                            progress.Log($"  [WARN] Obstacle [{cond.Type}] is {de:F1} mm from {pipe.Name} end");
                    }
                }
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
                // Re-read VM settings so changes made in OptionsPage take effect
                manifold.BendRadii                = _vm!.BendRadii.Select(r => r.Resolve(_vm.PipeDiameter)).ToArray();
                manifold.TargetLength             = (float)_vm.TargetLength;
                manifold.Diameter                 = (float)_vm.PipeDiameter - 2f;
                manifold.MaxBends                 = _vm.MaxBends;
                manifold.MinStraightLength        = (float)_vm.MinStraight;
                manifold.MinClearance             = (float)_vm.Clearance;
                manifold.LengthToleranceFraction  = _vm.TargetLength > 0 ? (float)(_vm.LengthTolerance / _vm.TargetLength) : 0f;
                manifold.MaxBacktrackCandidates   = _vm.MaxBacktrack;
                manifold.MaxBendAngleDeg          = (float)_vm.MaxAngle;

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

        private ISolverCondition BuildOptimalObstacleCondition(IComponent2 comp, Action<double> onProgress = null, double progressBase = 0, double progressSlice = 100)
        {
            MathTransform compToWorld = comp.Transform2;
            var xform = ToMatrix4x4(compToWorld);

            var vertices = GetWorldVerticesMm(comp, xform, onProgress, progressBase, progressSlice);

            if (vertices.Count == 0)
            {
                // No tessellation available — fall back to tight AABB from GetBox
                var rawBox = (double[])comp.GetBox(true, false);
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

                        var localAxis = new Vector3((float)cp[3], (float)cp[4], (float)cp[5]);
                        var worldAxis = Vector3.Normalize(Vector3.TransformNormal(localAxis, xform));

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

        private static Matrix4x4 ToMatrix4x4(MathTransform xform)
        {
            // SolidWorks ArrayData layout: [0-2]=X-axis, [3-5]=Y-axis, [6-8]=Z-axis (column vectors),
            // [9-11]=translation (metres), [12]=scale.
            // Build a row-vector Matrix4x4 compatible with Vector3.Transform / TransformNormal,
            // with translation converted to mm so all geometry stays in mm.
            var d = (double[])xform.ArrayData;
            float s = (float)d[12];
            return new Matrix4x4(
                (float)(s * d[0]), (float)(s * d[1]), (float)(s * d[2]), 0f,
                (float)(s * d[3]), (float)(s * d[4]), (float)(s * d[5]), 0f,
                (float)(s * d[6]), (float)(s * d[7]), (float)(s * d[8]), 0f,
                (float)(d[9]  * 1000), (float)(d[10] * 1000), (float)(d[11] * 1000), 1f);
        }

        private static List<Vector3> GetWorldVerticesMm(
            IComponent2 comp, Matrix4x4 xform,
            Action<double> onProgress = null, double progressBase = 0, double progressSlice = 100)
        {
            var result = new List<Vector3>();

            var body = comp.GetBody() as IBody2;
            if (body == null) return result;

            var tess = body.GetTessellation(null) as ITessellation;
            if (tess == null) return result;

            tess.NeedFaceFacetMap = true;
            tess.NeedVertexParams = true;
            tess.ImprovedQuality  = true;
            tess.MatchType        = (int)swTesselationMatchType_e.swTesselationMatchFacetTopology;

            if (!tess.Tessellate()) return result;

            int totalFaces = body.GetFaceCount();
            int faceIdx = 0;

            // Traverse face → facets → fins → vertices (SolidWorks tessellation model).
            // Deduplicate by vertex ID so shared-edge vertices are only transformed once.
            // Coords from GetVertexPoint are in metres; scale to mm before applying xform
            // (xform translation is already in mm, so inputs and outputs are consistent).
            var visited = new HashSet<int>();
            var face = body.GetFirstFace() as IFace2;
            while (face != null)
            {
                var facetIds = tess.GetFaceFacets(face) as int[];
                if (facetIds != null)
                {
                    double faceBase  = progressBase + faceIdx * progressSlice / totalFaces;
                    double faceSlice = progressSlice / totalFaces;
                    int numFacets    = facetIds.Length;

                    for (int fi = 0; fi < numFacets; fi++)
                    {
                        var finIds = tess.GetFacetFins(facetIds[fi]) as int[];
                        if (finIds != null)
                        {
                            foreach (int finId in finIds)
                            {
                                var vertexIds = tess.GetFinVertices(finId) as int[];
                                if (vertexIds == null) continue;
                                foreach (int vid in vertexIds)
                                {
                                    if (!visited.Add(vid)) continue;
                                    var coords = tess.GetVertexPoint(vid) as double[];
                                    if (coords == null || coords.Length < 3) continue;
                                    var local = new Vector3(
                                        (float)(coords[0] * 1000),
                                        (float)(coords[1] * 1000),
                                        (float)(coords[2] * 1000));
                                    result.Add(Vector3.Transform(local, xform));
                                }
                            }
                        }
                        onProgress?.Invoke(faceBase + (fi + 1.0) / numFacets * faceSlice);
                    }
                }

                faceIdx++;
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
            return (new IgnoreCylinder(cylCenter, axis, radius, height - 5.0f), volume);
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