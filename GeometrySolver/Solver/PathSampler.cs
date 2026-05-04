using System;
using System.Collections.Generic;
using System.Numerics;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// Converts a simulated pipe path into an evenly-spaced list of 3-D centreline
    /// points. This sampled representation is consumed by every
    /// <c>ISolverCondition</c> implementation, isolating condition logic from raw
    /// path geometry.
    ///
    /// Sampling strategy:
    ///   - A "distance-to-next-sample" accumulator walks the path continuously.
    ///   - Straight sections advance the accumulator linearly.
    ///   - Arc sections use the closed-form arc parametrisation to place sample points
    ///     at exact arc-length intervals without numeric integration.
    ///   - The start and end positions of the path are always included.
    /// </summary>
    public static class PathSampler
    {
        /// <summary>
        /// Samples the centreline of the path defined by <paramref name="segments"/>.
        /// </summary>
        /// <param name="startPos">World-space start position.</param>
        /// <param name="startDir">World-space start direction (need not be normalised).</param>
        /// <param name="segments">Ordered list of bend segments (output of the solver).</param>
        /// <param name="sampleSpacing">
        /// Approximate arc-length between successive sample points.
        /// Defaults to 10 mm when not specified. Callers typically pass
        /// <c>pipeDiameter / 2</c> for obstacle clearance checks.
        /// </param>
        /// <returns>
        /// A list of 3-D points along the centreline, guaranteed to contain at
        /// least the start and end positions.
        /// </returns>
        public static List<Vector3> Sample(
            Vector3 startPos,
            Vector3 startDir,
            IReadOnlyList<BendSegment> segments,
            float sampleSpacing = 10f)
        {
            if (sampleSpacing <= 0f) sampleSpacing = 10f;

            var points  = new List<Vector3> { startPos };
            Vector3 pos = startPos;
            Vector3 dir = Vector3.Normalize(startDir);
            float   toNext = sampleSpacing;   // remaining distance until next sample

            foreach (var seg in segments)
            {
                // ── Straight section ──────────────────────────────────────────────
                float straight = seg.StraightLength;
                while (straight >= toNext)
                {
                    pos     += dir * toNext;
                    straight -= toNext;
                    toNext   = sampleSpacing;
                    points.Add(pos);
                }
                pos    += dir * straight;
                toNext -= straight;

                // ── Arc section ───────────────────────────────────────────────────
                if (seg.Angle > 1e-6f)
                {
                    PathSimulator.BuildFrame(dir, out var b0, out var b1);
                    Vector3 B = b0 * MathF.Cos(seg.Rotation)
                              + b1 * MathF.Sin(seg.Rotation);

                    float   R          = seg.CLR;
                    float   fullAngle  = seg.Angle;
                    Vector3 arcStart   = pos;
                    Vector3 arcDir     = dir;

                    // Closed-form arc parametrisation (angle measured from arc start):
                    //   position(a) = arcStart + R·sin(a)·arcDir + R·(1-cos(a))·B
                    //   direction(a) = cos(a)·arcDir + sin(a)·B

                    float arcRemaining = R * fullAngle;   // arc length left to consume
                    float angleUsed    = 0f;

                    while (arcRemaining >= toNext)
                    {
                        float da    = toNext / R;
                        angleUsed  += da;
                        arcRemaining -= toNext;
                        toNext       = sampleSpacing;

                        pos = arcStart
                            + R * MathF.Sin(angleUsed) * arcDir
                            + R * (1f - MathF.Cos(angleUsed)) * B;
                        points.Add(pos);
                    }

                    // Advance position and direction to the arc endpoint
                    pos = arcStart
                        + R * MathF.Sin(fullAngle) * arcDir
                        + R * (1f - MathF.Cos(fullAngle)) * B;
                    dir = Vector3.Normalize(
                        MathF.Cos(fullAngle) * arcDir + MathF.Sin(fullAngle) * B);
                    toNext -= arcRemaining;
                }
            }

            // Ensure end position is always present
            if ((points[^1] - pos).LengthSquared() > 1e-4f)
                points.Add(pos);

            return points;
        }
    }
}
