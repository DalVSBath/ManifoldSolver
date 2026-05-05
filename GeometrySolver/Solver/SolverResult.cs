using System.Collections.Generic;
using System.Numerics;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// The rich return type for a solved single-pipe path. Carries everything needed
    /// for reporting, manifold coordination, and condition evaluation.
    /// </summary>
    public class SolverResult
    {
        /// <summary>Ordered list of bend segments that define the path.</summary>
        public List<BendSegment> Segments { get; init; }

        /// <summary>Simulated total path length in mm.</summary>
        public float TotalLength { get; init; }

        /// <summary>Distance between the simulated end position and the target end point (mm).</summary>
        public float PositionError { get; init; }

        /// <summary>Magnitude of the vector difference between the simulated end direction and target direction.</summary>
        public float DirectionError { get; init; }

        /// <summary>Number of bend segments (segments with Angle > 0).</summary>
        public int BendCount { get; init; }

        /// <summary>True when the path satisfied all geometric and condition constraints.</summary>
        public bool IsValid { get; init; }

        /// <summary>
        /// Evenly-spaced centreline sample points generated at <c>diameter/2</c> spacing.
        /// Pre-computed and cached here so <see cref="ManifoldSolver"/> can create
        /// <c>PipeClearanceCondition</c>s without re-sampling.
        /// </summary>
        public IReadOnlyList<Vector3>? SampledPoints { get; init; }
    }
}
