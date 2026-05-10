using CADBooster.SolidDna;
using GeometrySolver.Solver;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Shapes;

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

            for (int i = 0; i < results.Count && i < pipes.Count; i++)
            {
                var result = results[i];
                if (result == null) continue;

                DrawPipeSketch(compDoc, mathUtil, worldToLocal, pipes[i], result, log);
                DrawPipeProfile(swApp, ParentDoc, compDoc, mathUtil, worldToLocal, pipes[i], pipeDiameter, wallThickness, log, component.Name2);
            }

            compDoc.ForceRebuild3(false);

            if (swApp.ActiveDoc is IAssemblyDoc asmDoc2)
            {
                asmDoc2.EditAssembly();
            }
        }

        private static void DrawPipeSketch(
            IModelDoc2 compDoc,
            MathUtility mathUtil,
            MathTransform worldToLocal,
            Analyser.PipeDef pipe,
            SolverResult result,
            Action<string>? log = null)
        {
            var sketchMgr = compDoc.SketchManager;

            sketchMgr.AddToDB = true;
            sketchMgr.Insert3DSketch(true);

            DrawSegments(sketchMgr, mathUtil, worldToLocal, pipe.Start, pipe.StartDir, result.Segments, pipe.End, log);

            sketchMgr.Insert3DSketch(false);
            sketchMgr.AddToDB = false;

            var feature = (IFeature)compDoc.Extension.GetLastFeatureAdded();
            if (feature != null)
                feature.Name = pipe.Name;
        }

        private static void DrawSegments(
            ISketchManager sketchMgr,
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
            int segIdx = 0;

            foreach (var seg in segments)
            {
                segIdx++;

                // Straight section
                if (seg.StraightLength > 1e-4f)
                {
                    Vector3 lineEnd = pos + dir * seg.StraightLength;
                    log?.Invoke($"[SketchBuilder] Seg {segIdx} straight end: ({lineEnd.X:F3}, {lineEnd.Y:F3}, {lineEnd.Z:F3})");
                    double[] lineEndLocal = ToLocalMeters(mathUtil, worldToLocal, lineEnd);
                    sketchMgr.CreateLine(curLocal[0], curLocal[1], curLocal[2],
                                         lineEndLocal[0], lineEndLocal[1], lineEndLocal[2]);
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
                        arcMidLocal[0], arcMidLocal[1], arcMidLocal[2]);
                    if (arc == null) throw new InvalidOperationException(
                        $"Create3PointArc returned null for pipe segment (angle={seg.Angle:F3} rad, CLR={seg.CLR:F4} m).");

                    pos = arcEnd;
                    curLocal = arcEndLocal;
                    dir = Vector3.Normalize(
                        (float)Math.Cos(fullAngle) * arcDir + (float)Math.Sin(fullAngle) * B);
                }
            }

            float endError = Vector3.Distance(pos, expectedEnd);
            log?.Invoke($"[SketchBuilder] Final pos:      ({pos.X:F3}, {pos.Y:F3}, {pos.Z:F3})");
            log?.Invoke($"[SketchBuilder] Expected end:   ({expectedEnd.X:F3}, {expectedEnd.Y:F3}, {expectedEnd.Z:F3})");
            log?.Invoke($"[SketchBuilder] Endpoint error: {endError:F3} mm");
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

        private static void DrawPipeProfile(
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
                return;
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
                return;
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
                return;
            }
            profileFeature.Name = $"Profile_{pipe.Name}";

            partDoc.EditRebuild3();

            // ── D: Swept extrude — profile sketch (mark 1) swept along centreline path (mark 4).
            parentDoc.ClearSelection2(true);
            partDoc.ClearSelection2(true);


            var profileSD = assySelMgr.CreateSelectData(); profileSD.Mark = 1;
            var centreSD = assySelMgr.CreateSelectData(); centreSD.Mark = 4;



            bool selProfile = compDoc.Extension.SelectByID2(
                $"Profile_{pipe.Name}", "SKETCH", 0, 0, 0, false, 1, null, 0);
            bool selPath = compDoc.Extension.SelectByID2(
                pipe.Name, "SKETCH", 0, 0, 0, true, 4, null, 0);

            if (!selProfile || !selPath)
            {
                log?.Invoke($"[SketchBuilder] Sweep selection failed for {pipe.Name} " +
                            $"(profile={selProfile}, path={selPath})");
                return;
            }

            var sweepFeature = (IFeature)compDoc.FeatureManager.InsertProtrusionSwept4(
                false,  // Propagate
                false,  // Alignment
                0,      // TwistCtrlOption = follow path
                false,  // KeepTangency
                false,  // BAdvancedSmoothing
                0,      // StartMatchingType = None
                0,      // EndMatchingType = None
                false,  // IsThinBody
                0.0,    // Thickness1
                0.0,    // Thickness2
                0,      // ThinType
                0,      // PathAlign
                true,   // Merge
                false,  // UseFeatScope
                true,   // UseAutoSelect
                0.0,    // TwistAngle
                false,  // BMergeSmoothFaces
                false,  // CircularProfile
                0.0,    // CircularProfileDiameter
                0       // Direction
            );

            if (sweepFeature != null)
                sweepFeature.Name = $"Pipe_{pipe.Name}";
            else
                log?.Invoke($"[SketchBuilder] Swept extrude creation failed for {pipe.Name}");
        }
    }
}
