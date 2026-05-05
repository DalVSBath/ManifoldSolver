using System.Collections.Generic;
using System.Numerics;

namespace GeometrySolver.Solver
{
    internal static class PathSimulator
    {
        public readonly struct PathResult
        {
            public Vector3 Position    { get; init; }
            public Vector3 Direction   { get; init; }
            public float   TotalLength { get; init; }
        }

        public static PathResult Simulate(
            Vector3 startPos,
            Vector3 startDir,
            IReadOnlyList<BendSegment> segments)
        {
            Vector3 pos = startPos;
            Vector3 dir = Vector3.Normalize(startDir);
            float totalLength = 0f;

            foreach (var seg in segments)
            {
                // Walk straight section
                pos += dir * seg.StraightLength;
                totalLength += seg.StraightLength;

                // Apply bend arc (skip if angle is negligible)
                if (seg.Angle > 1e-6f)
                {
                    BuildFrame(dir, out Vector3 b0, out Vector3 b1);
                    Vector3 B = b0 * (float)Math.Cos((double)seg.Rotation)
                              + b1 * (float)Math.Sin((double)seg.Rotation);

                    float R     = seg.CLR;
                    float alpha = seg.Angle;

                    pos += R * (1f - (float)Math.Cos((double)alpha)) * B
                         + R * (float)Math.Sin((double)alpha) * dir;

                    dir = Vector3.Normalize(
                        (float)Math.Cos(alpha) * dir + (float)Math.Sin((double)alpha) * B);

                    totalLength += R * alpha;
                }
            }

            return new PathResult
            {
                Position    = pos,
                Direction   = dir,
                TotalLength = totalLength,
            };
        }

        internal static void BuildFrame(Vector3 d, out Vector3 b0, out Vector3 b1)
        {
            // Pick the world axis least aligned with d to avoid degenerate cross-product
            Vector3 arbitrary = (MathHelper.Abs(d.X) < 0.9f && MathHelper.Abs(d.Z) < 0.9f)
                ? new Vector3(1f, 0f, 0f)
                : new Vector3(0f, 1f, 0f);
            b0 = Vector3.Normalize(Vector3.Cross(d, arbitrary));
            b1 = Vector3.Cross(d, b0); // already unit length (d and b0 are orthonormal)
        }
    }
}
