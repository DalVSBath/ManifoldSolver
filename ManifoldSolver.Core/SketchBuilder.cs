using CADBooster.SolidDna;
using GeometrySolver.Solver;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Windows.Shapes;
using static System.Windows.Forms.LinkLabel;

namespace ManifoldSolver.Core
{
    public static class SketchBuilder
    {
        /// <summary>
        /// Creates one named 3D sketch per solved pipe inside the component's part document.
        /// Straight sections become sketch lines; bends become sketch arcs.
        /// </summary>
        public static void DrawPipeSketches(
            ISldWorks swApp,
            IComponent2 component,
            IReadOnlyList<Analyser.PipeDef> pipes,
            IReadOnlyList<SolverResult?> results,
            float pipeDiameter,
            float wallThickness,
            Action<string>? log = null)
        {
            var compDoc = (IModelDoc2)component.GetModelDoc2();
            if (compDoc == null) return;

            var mathUtil = (MathUtility)swApp.GetMathUtility(); 
            
            MathTransform accumulated = component.Transform2;
            var parent = component.GetParent() as Component2;
            while (parent != null)
            {
                accumulated = (MathTransform)parent.Transform2.Multiply(accumulated);
                parent = parent.GetParent() as Component2;
            }

            var worldToLocal = (MathTransform)accumulated.IInverse();

            IModelDoc2 ParentDoc = (IModelDoc2)swApp.ActiveDoc;

            if (swApp.ActiveDoc is IAssemblyDoc asmDoc)
            {
                component.Select4(false, null, false);
                int editErr = 0;
                asmDoc.EditPart2(true, false, ref editErr);
            }

            // After EditPart2 the active doc is the part; in a standalone part it already is.
            // Mirror the test handler: always drive sketches via swApp.ActiveDoc.
            compDoc = (IModelDoc2)swApp.ActiveDoc;
            ISketch?[] pipeSketches = new ISketch[pipes.Count];

            var sketchMgr = compDoc.SketchManager;
            sketchMgr.AddToDB = true;
            for (int i = 0; i < results.Count && i < pipes.Count; i++)
            {
                var result = results[i];
                if (result == null) continue;

                pipeSketches[i] = DrawPipeSketch(compDoc, mathUtil, worldToLocal, pipes[i], result, log);
                //ISketch? pipeProfile = null; // DrawPipeProfile(swApp, ParentDoc, compDoc, mathUtil, worldToLocal, pipes[i], pipeDiameter, wallThickness, log, component.Name2);
                
            }
            sketchMgr.AddToDB = false;

            compDoc.ForceRebuild3(false);

            //swApp.AllowFailedFeatureCreation(true);
            for (int i = 0; i < pipeSketches.Length; i++)
            {
                if (pipeSketches[i] != null)
                {
                    var pipeFeature = PerformSweep(ParentDoc, compDoc, pipeSketches[i], pipes[i], pipeDiameter, wallThickness, log);
                }
            }
            //swApp.AllowFailedFeatureCreation(false);

            if (swApp.ActiveDoc is IAssemblyDoc asmDoc2)
            {
                asmDoc2.EditAssembly();
            }
        }

        private static ISketch? DrawPipeSketch(
            IModelDoc2 compDoc,
            MathUtility mathUtil,
            MathTransform worldToLocal,
            Analyser.PipeDef pipe,
            SolverResult result,
            Action<string>? log = null)
        {
            var sketchMgr = compDoc.SketchManager;

            sketchMgr.Insert3DSketch(true);

            var pipeSketch = DrawSegments(sketchMgr, compDoc, mathUtil, worldToLocal, pipe.Start, pipe.StartDir, result.Segments, pipe.End, log);

            sketchMgr.Insert3DSketch(false);

            var feature = (IFeature?)pipeSketch;
            if (feature != null)
                feature.Name = pipe.Name;

            return pipeSketch;
        }

        private static ISketch? DrawSegments(
            ISketchManager sketchMgr,
            IModelDoc2 compDoc,
            MathUtility mathUtil,
            MathTransform worldToLocal,
            Vector3 startPos,
            Vector3 startDir,
            IReadOnlyList<BendSegment> segments,
            Vector3 expectedEnd,
            Action<string>? log = null)
        {
            Vector3 pos = startPos;
            Vector3 dir = Vector3.Normalize(startDir);
            // Track the current position in local coords to avoid re-converting the same
            // world-space point twice — independent MathTransform multiplications on the
            // same Vector3 introduce µm-scale drift that SolidWorks treats as separate
            // vertices, producing two open loops instead of one continuous sketch.
            double[] curLocal = ToLocalMeters(mathUtil, worldToLocal, startPos);
            double[] startLocal = curLocal;
            int segIdx = 0;

            ISketch? sketch = null;
            ISketchSegment? prevSeg = null;
            ISketchSegment? firstSeg = null;

            foreach (var seg in segments)
            {
                segIdx++;

                // Straight section
                if (seg.StraightLength > 1e-4f)
                {
                    Vector3 lineEnd = pos + dir * seg.StraightLength;
                    log?.Invoke($"[SketchBuilder] Seg {segIdx} straight end: ({lineEnd.X:F3}, {lineEnd.Y:F3}, {lineEnd.Z:F3})");
                    double[] lineEndLocal = ToLocalMeters(mathUtil, worldToLocal, lineEnd);
                    var line = sketchMgr.CreateLine(curLocal[0], curLocal[1], curLocal[2],
                                         lineEndLocal[0], lineEndLocal[1], lineEndLocal[2]) as ISketchSegment;

                    if (line != null)
                    {
                        sketch = line.GetSketch();
                        firstSeg ??= line;
                        prevSeg = line;
                    }

                    pos = lineEnd;
                    curLocal = lineEndLocal;
                }

                // Bend arc
                if (seg.Angle > 1e-6f)
                {
                    BuildFrame(dir, out Vector3 b0, out Vector3 b1);
                    Vector3 B = b0 * (float)Math.Cos(seg.Rotation) + b1 * (float)Math.Sin(seg.Rotation);

                    float R = seg.CLR;
                    double fullAngle = seg.Angle;
                    Vector3 arcDir = dir;

                    // Arc geometry: end and midpoint computed from the closed-form arc parametrisation.
                    // CreateArc (centre+start+end) does not work in 3D sketches; Create3PointArc does.
                    Vector3 arcEnd = pos
                        + R * (float)Math.Sin(fullAngle) * arcDir
                        + R * (float)(1.0 - Math.Cos(fullAngle)) * B;
                    Vector3 arcMid = pos
                        + R * (float)Math.Sin(fullAngle / 2) * arcDir
                        + R * (float)(1.0 - Math.Cos(fullAngle / 2)) * B;

                    log?.Invoke($"[SketchBuilder] Seg {segIdx} arc end:      ({arcEnd.X:F3}, {arcEnd.Y:F3}, {arcEnd.Z:F3})  rot={seg.Rotation:F4} rad");

                    double[] arcEndLocal = ToLocalMeters(mathUtil, worldToLocal, arcEnd);
                    double[] arcMidLocal = ToLocalMeters(mathUtil, worldToLocal, arcMid);

                    // curLocal is reused as arc start — same doubles as the line endpoint above.
                    var arc = sketchMgr.Create3PointArc(
                        curLocal[0],    curLocal[1],    curLocal[2],
                        arcEndLocal[0], arcEndLocal[1], arcEndLocal[2],
                        arcMidLocal[0], arcMidLocal[1], arcMidLocal[2]) as ISketchSegment;
                    if (arc == null) throw new InvalidOperationException(
                        $"Create3PointArc returned null for pipe segment (angle={seg.Angle:F3} rad, CLR={seg.CLR:F4} m).");
                    sketch = arc.GetSketch();
                    firstSeg ??= arc;
                    prevSeg = arc;

                    pos = arcEnd;
                    // Create3PointArc fits a circle through the three points and recomputes
                    // the endpoint from the fitted centre+radius, which can differ slightly
                    // from arcEndLocal. Read the actual stored coordinate back so the next
                    // segment starts at exactly the same vertex.
                    var (arcPtA, arcPtB) = GetEndpoints(arc);
                    var fittedEnd = ClosestToLocal(arcPtA, arcPtB, arcEndLocal);
                    curLocal = fittedEnd != null
                        ? new[] { fittedEnd.X, fittedEnd.Y, fittedEnd.Z }
                        : arcEndLocal;
                    dir = Vector3.Normalize(
                        (float)Math.Cos(fullAngle) * arcDir + (float)Math.Sin(fullAngle) * B);
                }
            }

            // Anchor the first and last points so the sketch cannot drift on rebuild.
            // Use coordinate comparison to find the correct endpoint regardless of arc orientation.
            if (firstSeg != null)
            {
                var (fa, fb) = GetEndpoints(firstSeg);
                var firstPt = ClosestToLocal(fa, fb, startLocal);
                if (firstPt != null)
                {
                    compDoc.ClearSelection2(true);
                    firstPt.Select4(false, null);
                    compDoc.SketchAddConstraints("sgFIXED");
                    compDoc.ClearSelection2(true);
                }
            }
            if (prevSeg != null)
            {
                var (la, lb) = GetEndpoints(prevSeg);
                var lastPt = ClosestToLocal(la, lb, curLocal);
                if (lastPt != null)
                {
                    compDoc.ClearSelection2(true);
                    lastPt.Select4(false, null);
                    compDoc.SketchAddConstraints("sgFIXED");
                    compDoc.ClearSelection2(true);
                }
            }

            float endError = Vector3.Distance(pos, expectedEnd);
            log?.Invoke($"[SketchBuilder] Final pos:      ({pos.X:F3}, {pos.Y:F3}, {pos.Z:F3})");
            log?.Invoke($"[SketchBuilder] Expected end:   ({expectedEnd.X:F3}, {expectedEnd.Y:F3}, {expectedEnd.Z:F3})");
            log?.Invoke($"[SketchBuilder] Endpoint error: {endError:F3} mm");

            return sketch;
        }

        private static (ISketchPoint? a, ISketchPoint? b) GetEndpoints(ISketchSegment seg) =>
            seg is ISketchLine l ? ((ISketchPoint?)l.GetStartPoint2(), (ISketchPoint?)l.GetEndPoint2()) :
            seg is ISketchArc  a ? ((ISketchPoint?)a.GetStartPoint2(), (ISketchPoint?)a.GetEndPoint2()) :
            (null, null);

        private static ISketchPoint? ClosestToLocal(ISketchPoint? a, ISketchPoint? b, double[] local)
        {
            if (a == null) return b;
            if (b == null) return a;
            double da = (a.X-local[0])*(a.X-local[0]) + (a.Y-local[1])*(a.Y-local[1]) + (a.Z-local[2])*(a.Z-local[2]);
            double db = (b.X-local[0])*(b.X-local[0]) + (b.Y-local[1])*(b.Y-local[1]) + (b.Z-local[2])*(b.Z-local[2]);
            return da <= db ? a : b;
        }

        private static double[] ToLocalMeters(MathUtility mathUtil, MathTransform worldToLocal, Vector3 worldMm)
        {
            double[] data = { worldMm.X / 1000.0, worldMm.Y / 1000.0, worldMm.Z / 1000.0 };
            var pt = (MathPoint)mathUtil.CreatePoint(data);
            var local = (MathPoint)pt.MultiplyTransform(worldToLocal);
            var raw = (double[])local.ArrayData;
            return new[] { raw[0], raw[1], raw[2] };
        }

        private static void BuildFrame(Vector3 dir, out Vector3 b0, out Vector3 b1)
        {
            // Must match PathSimulator.BuildFrame exactly — seg.Rotation was encoded with this frame.
            Vector3 arbitrary = (Math.Abs(dir.X) < 0.9f && Math.Abs(dir.Z) < 0.9f)
                ? Vector3.UnitX : Vector3.UnitY;
            b0 = Vector3.Normalize(Vector3.Cross(dir, arbitrary));
            b1 = Vector3.Cross(dir, b0);
        }

        private static ISketch? DrawPipeProfile(
            ISldWorks swApp,
            IModelDoc2 parentDoc,
            IModelDoc2 compDoc,
            MathUtility mathUtil,
            MathTransform worldToLocal,
            Analyser.PipeDef pipe,
            float pipeDiameter,
            float wallThickness,
            Action<string>? log = null, string? name = null)
        {
            var assySelMgr = (ISelectionMgr)parentDoc.SelectionManager;
            var selectManager = (ISelectionMgr)compDoc.SelectionManager;
            var sketchMgr = compDoc.SketchManager;
            double[] startLocal = ToLocalMeters(mathUtil, worldToLocal, pipe.Start);

            // ── A: Guide sketch — 1 mm line in the pipe's start direction.
            // Used as the "perpendicular to curve" reference for InsertRefPlane.
            Vector3 tipWorld = pipe.Start + Vector3.Normalize(pipe.StartDir) * 1f;
            double[] tipLocal = ToLocalMeters(mathUtil, worldToLocal, tipWorld);

            //sketchMgr.AddToDB = true;
            sketchMgr.Insert3DSketch(true);
            var guideSeg = sketchMgr.CreateLine(startLocal[0], startLocal[1], startLocal[2],
                                  tipLocal[0],   tipLocal[1],   tipLocal[2]);
            sketchMgr.Insert3DSketch(false);
            //sketchMgr.AddToDB = false;

            var guideSketch = (ISketch)guideSeg.GetSketch();
            var guideFeature = (IFeature)guideSketch;
            if (guideSketch == null)
            {
                log?.Invoke($"[SketchBuilder] Profile guide sketch failed for {pipe.Name}");
                return null;
            }else
                guideFeature.Name = $"_Guide_{pipe.Name}";

            string type = guideFeature.GetTypeName2();

            //var segments      = (object[])guideSketch.GetSketchSegments();
            //var guideSeg      = (ISketchSegment)segments[0];  // for Select2
            var guideLine     = (ISketchLine)guideSeg;      // for GetStartPoint2
            var guidePoint    = (ISketchPoint)guideLine.GetStartPoint2();

            // get the component's own ModelDoc2, not the assembly's
            var partDoc = (IModelDoc2)((IAssemblyDoc)parentDoc).GetEditTarget();
            //var partDoc = (IModelDoc2)editComp.GetModelDoc2();
            /// // log?.Invoke($"[SketchBuilder] Name of the part {partDoc.GetTitle()}");

            // selections still go through the assembly while editing in-context
            var perpSD = assySelMgr.CreateSelectData(); perpSD.Mark = 0;
            var coinSD = assySelMgr.CreateSelectData(); coinSD.Mark = 1;

            parentDoc.ClearSelection2(true);
            bool ok1 = guideSeg.Select4(false, perpSD);
            bool ok2 = guidePoint.Select4(true, coinSD);

            // try the plane on the part's FeatureManager
            var planeFeature = (IFeature)partDoc.FeatureManager.InsertRefPlane(
                (int)swRefPlaneReferenceConstraints_e.swRefPlaneReferenceConstraint_Perpendicular, 0.0,
                (int)swRefPlaneReferenceConstraints_e.swRefPlaneReferenceConstraint_Coincident, 0.0,
                0, 0.0);

            if (planeFeature == null)
            {
                log?.Invoke($"[SketchBuilder] Reference plane creation failed for {pipe.Name}");
                return null;
            }
            planeFeature.Name = $"Profile_Plane_{pipe.Name}";

            // ── C: 2D profile sketch — outer circle and (if wall thickness > 0) inner bore circle.
            // CreateCircleByRadius takes model-space metres; SolidWorks constrains them to the
            // active sketch plane automatically.
            parentDoc.ClearSelection2(true);
            partDoc.ClearSelection2(true);

            var prtSketchMgr = partDoc.SketchManager;

            planeFeature.Select2(false, 0);

            prtSketchMgr.AddToDB = true;
            prtSketchMgr.InsertSketch(true);

            double outerR = pipeDiameter / 2.0 / 1000.0;
            var seg = prtSketchMgr.CreateCircleByRadius(0.0, 0.0, 0.0, outerR);

            if (wallThickness > 0f)
            {
                double innerR = (pipeDiameter / 2.0 - wallThickness) / 1000.0;
                prtSketchMgr.CreateCircleByRadius(0, 0, 0, innerR);
            }

            prtSketchMgr.InsertSketch(false);
            prtSketchMgr.AddToDB = false;

            var profileFeature = (IFeature)seg.GetSketch();
            if (profileFeature == null)
            {
                log?.Invoke($"[SketchBuilder] Profile sketch creation failed for {pipe.Name}");
                return null;
            }
            profileFeature.Name = $"Profile_{pipe.Name}";

            partDoc.EditRebuild3();


            return (ISketch)seg.GetSketch();
        }

        public static IFeature? PerformSweep(
            IModelDoc2 parentDoc,
            IModelDoc2 compDoc,
            ISketch centreLine,
            Analyser.PipeDef pipe,
            float pipeDiameter,
            float wallThickness,
            Action<string>? log = null)
        {

            var partDoc = (IModelDoc2)((IAssemblyDoc)parentDoc).GetEditTarget();
            parentDoc.ClearSelection2(true);
            partDoc.ClearSelection2(true);

            var selectManager = (SelectionMgr)partDoc.SelectionManager;

            //var profileSD = selectManager.Crea(); profileSD.Mark = 1;
            //var centreSD = selectManager.CreateSelectData(); centreSD.Mark = 4;

            var localFeature = (IFeature)centreLine;
            bool selPath = localFeature.Select2(false, 4);

            var count = selectManager.GetSelectedObjectCount2(-1);

            compDoc.ShowFeatureErrorDialog = true;
            partDoc.ShowFeatureErrorDialog = true;

            if (!selPath)
            {
                log?.Invoke($"[SketchBuilder] Sweep selection failed for {pipe.Name} " +
                            $"(path={selPath})");
                return null;
            }

            log?.Invoke($"[SketchBuilder] Sweep: selection count={count}, selPath={selPath}");

            var sweepData = (ISweepFeatureData)partDoc.FeatureManager
                .CreateDefinition((int)swFeatureNameID_e.swFmSweep);

            if (sweepData == null)
            {
                log?.Invoke($"[SketchBuilder] Sweep: CreateDefinition returned null for {pipe.Name}");
                return null;
            }

            sweepData.TangentPropagation = false;
            sweepData.AlignWithEndFaces = false;
            sweepData.TwistControlType = (int)swTwistControlType_e.swTwistControlFollowPath;
            sweepData.MaintainTangency = false;
            sweepData.AdvancedSmoothing = false;
            sweepData.StartTangencyType = 0;
            sweepData.EndTangencyType = 0;
            sweepData.ThinFeature = true;
            sweepData.SetWallThickness(true, wallThickness/1000.0);
            sweepData.SetWallThickness(false, 0.0);
            sweepData.ThinWallType = (int)swThinWallType_e.swThinWallOppDirection;
            sweepData.PathAlignmentType = 0;
            sweepData.Merge = true;
            sweepData.FeatureScope = false;
            sweepData.AutoSelect = true;
            sweepData.SetTwistAngle(0.0);
            sweepData.MergeSmoothFaces = false;
            sweepData.CircularProfile = true;
            sweepData.CircularProfileDiameter = pipeDiameter / 1000.0;
            sweepData.Direction = 0;

            log?.Invoke($"[SketchBuilder] Sweep: CircularProfile={sweepData.CircularProfile}, " +
                        $"Diameter={sweepData.CircularProfileDiameter * 1000:F2} mm, " +
                        $"ThinFeature={sweepData.ThinFeature}, " +
                        $"AutoSelect={sweepData.AutoSelect}");

            var sweepFeature = (IFeature)partDoc.FeatureManager.CreateFeature(sweepData);

            if (sweepFeature != null)
            {
                sweepFeature.Name = $"Pipe_{pipe.Name}";
                log?.Invoke($"[SketchBuilder] Sweep: created '{sweepFeature.Name}' for {pipe.Name}");
            }
            else
            {
                int errCount = partDoc.FeatureManager.GetCreateFeatureErrors(out object msgs, out string featureTypeName);
                var errArray = msgs as int[] ?? Array.Empty<int>();
                var errList = string.Join(", ", errArray.Take(errCount).Select(e => $"0x{e:X}"));
                log?.Invoke($"[SketchBuilder] Sweep: CreateFeature failed for {pipe.Name} — " +
                            $"type={featureTypeName}, errors({errCount})=[{errList}]");
            }

            return sweepFeature;
        }
    }
}
