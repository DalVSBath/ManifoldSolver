using System;
using System.Collections.Generic;
using System.Numerics;

namespace GeometrySolver.Conditions
{
    /// <summary>
    /// Enforces a minimum separation between a new pipe path and an already-solved
    /// reference pipe.  Used by <see cref="GeometrySolver.Solver.ManifoldSolver"/>
    /// to prevent inter-pipe collisions when routing multiple pipes in sequence.
    ///
    /// <b>IsSatisfied</b>: returns <c>true</c> when every sampled point on the new
    /// path is at least <c>minClearance + pipeDiameter</c> away from every sampled
    /// point on the reference pipe.  Both pipes occupy a tube of radius
    /// <c>diameter/2</c>, so the centre-to-centre minimum is the sum of both radii
    /// plus the explicit clearance gap.
    ///
    /// <b>Penalty</b>: returns <c>Σ max(0, required − distance)²</c> over all
    /// point-pair violations, providing a smooth gradient for the Adam optimizer to
    /// steer the new path away from the reference pipe.
    ///
    /// <b>Performance note</b>: the check is O(|new| × |ref|) brute-force.  For
    /// typical 500 mm paths at diameter/2 = 20 mm spacing this is ≤ 25 × 25 = 625
    /// comparisons per residual evaluation — negligible.  If paths are much longer
    /// a spatial grid can be added in Phase 4.
    /// </summary>
    public class PipeClearanceCondition : ISolverCondition
    {
        private readonly IReadOnlyList<Vector3> _refPoints;
        private readonly float                  _minClearance;

        /// <summary>
        /// Creates a new clearance condition against a reference pipe.
        /// </summary>
        /// <param name="refPoints">
        /// Pre-sampled centreline points of the already-solved reference pipe
        /// (typically <see cref="GeometrySolver.Solver.SolverResult.SampledPoints"/>).
        /// </param>
        /// <param name="minClearance">
        /// Required clear gap between the pipe surfaces (mm).  The total
        /// centre-to-centre minimum is <c>minClearance + pipeDiameter</c>.
        /// Passing <c>pipeDiameter</c> enforces a one-diameter surface gap.
        /// </param>
        public PipeClearanceCondition(IReadOnlyList<Vector3> refPoints, float minClearance)
        {
            _refPoints    = refPoints    ?? throw new ArgumentNullException(nameof(refPoints));
            _minClearance = minClearance;
        }

        /// <summary>
        /// Sampled points on the <em>new</em> pipe that are within this Euclidean
        /// distance of the new pipe's start or end are exempt from the clearance
        /// check.  Use this to exclude the physical connection-port stub region
        /// where the collector geometry forces the pipes into close proximity.
        /// Mirrors <c>ObstacleCondition.ExcludeEndMm</c>.  Default: 0 (no exclusion).
        /// </summary>
        public float ExcludeEndMm { get; init; } = 0f;

        /// <inheritdoc />
        public ConditionType Type => ConditionType.PipeClearance;

        /// <inheritdoc />
        public bool IsSatisfied(IReadOnlyList<Vector3> pathPoints, float pipeDiameter)
        {
            if (pathPoints.Count == 0) return true;
            float required   = _minClearance + pipeDiameter;
            float requiredSq = required * required;
            float excSq      = ExcludeEndMm * ExcludeEndMm;
            Vector3 startPt  = pathPoints[0];
            Vector3 endPt    = pathPoints[pathPoints.Count - 1];

            foreach (var p in pathPoints)
            {
                if (excSq > 0f)
                {
                    if (Vector3.DistanceSquared(p, startPt) <= excSq) continue;
                    if (Vector3.DistanceSquared(p, endPt)   <= excSq) continue;
                }
                foreach (var q in _refPoints)
                    if ((p - q).LengthSquared() < requiredSq) return false;
            }

            return true;
        }

        /// <inheritdoc />
        public double Penalty(IReadOnlyList<Vector3> pathPoints, float pipeDiameter)
        {
            if (pathPoints.Count == 0) return 0;
            float   required = _minClearance + pipeDiameter;
            float   excSq    = ExcludeEndMm * ExcludeEndMm;
            Vector3 startPt  = pathPoints[0];
            Vector3 endPt    = pathPoints[pathPoints.Count - 1];
            double  penalty  = 0;

            foreach (var p in pathPoints)
            {
                if (excSq > 0f)
                {
                    if (Vector3.DistanceSquared(p, startPt) <= excSq) continue;
                    if (Vector3.DistanceSquared(p, endPt)   <= excSq) continue;
                }

                // Find closest reference point
                float minDistSq = float.MaxValue;
                foreach (var q in _refPoints)
                {
                    float dSq = (p - q).LengthSquared();
                    if (dSq < minDistSq) minDistSq = dSq;
                }

                float dist = MathF.Sqrt(minDistSq);
                if (dist < required)
                {
                    float violation = required - dist;
                    // Weight matches the geometry non-negativity penalty (1e6) so the
                    // optimizer gives clearance violations equal priority to invalid
                    // straight-length solutions.  Previously 1e4 made clearance 100×
                    // weaker than geometry, causing Adam to always stay in the
                    // clearance-violating basin.
                    penalty += (double)(violation * violation) * 1e6;
                }
            }

            return penalty;
        }
    }
}
