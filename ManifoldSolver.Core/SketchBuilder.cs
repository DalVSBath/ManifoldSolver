using GeometrySolver.Solver;
using SolidWorks.Interop.sldworks;
using System;
using System.Collections.Generic;
using System.Numerics;

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
    }
}
