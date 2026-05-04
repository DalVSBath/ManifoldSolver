using System.Collections.Generic;
using System.Numerics;
using GeometrySolver.Conditions;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// Public facade that exposes the single-pipe solving API while maintaining full
    /// backward compatibility with existing <c>Program.cs</c> call sites.
    ///
    /// Internally delegates to <see cref="SinglePipeSolver"/>. For multi-pipe manifold
    /// solving use <see cref="ManifoldSolver"/> directly.
    ///
    /// Use <see cref="FromConfig"/> to create a fully-configured solver from a
    /// <see cref="SolverConfig"/> record in a single call.
    /// </summary>
    public class Solver
    {
        private readonly SinglePipeSolver _inner = new();

        // ── Configuration properties ──────────────────────────────────────────

        /// <summary>Maximum number of bends to attempt (default 8).</summary>
        public int MaxBends
        {
            get => _inner.MaxBends;
            set => _inner.MaxBends = value;
        }

        /// <summary>When true, prints detailed progress to the console.</summary>
        public bool Verbose
        {
            get => _inner.Verbose;
            set => _inner.Verbose = value;
        }

        /// <summary>
        /// How often ISolverCondition.Penalty is evaluated during Adam refinement.
        /// Default is 5 (every 5 iterations).
        /// </summary>
        public int ConditionSampleInterval
        {
            get => _inner.ConditionSampleInterval;
            set => _inner.ConditionSampleInterval = value;
        }

        // ── P1.1 / P1.2 performance thresholds ───────────────────────────────

        /// <inheritdoc cref="SinglePipeSolver.AdamEntryThreshold"/>
        public double AdamEntryThreshold
        {
            get => _inner.AdamEntryThreshold;
            set => _inner.AdamEntryThreshold = value;
        }

        /// <inheritdoc cref="SinglePipeSolver.AdamFastStageIters"/>
        public int AdamFastStageIters
        {
            get => _inner.AdamFastStageIters;
            set => _inner.AdamFastStageIters = value;
        }

        /// <inheritdoc cref="SinglePipeSolver.AdamFastStageThreshold"/>
        public double AdamFastStageThreshold
        {
            get => _inner.AdamFastStageThreshold;
            set => _inner.AdamFastStageThreshold = value;
        }

        /// <summary>
        /// Maximum valid solutions to collect and rank before selecting the best
        /// within a given bend count. Default is 3.
        /// </summary>
        public int MaxSolutionsToRank
        {
            get => _inner.MaxSolutionsToRank;
            set => _inner.MaxSolutionsToRank = value;
        }

        /// <summary>
        /// Minimum straight-section length (mm). Segments shorter than this are rejected.
        /// </summary>
        public float MinStraightLength
        {
            get => _inner.MinStraightLength;
            set => _inner.MinStraightLength = value;
        }

        /// <summary>
        /// Maximum allowable bend angle in degrees. Bends larger than this are rejected.
        /// </summary>
        public float MaxBendAngleDeg
        {
            get => _inner.MaxBendAngleDeg;
            set => _inner.MaxBendAngleDeg = value;
        }

        /// <inheritdoc cref="SinglePipeSolver.UseParallel"/>
        public bool UseParallel
        {
            get => _inner.UseParallel;
            set => _inner.UseParallel = value;
        }

        // ── Setup methods (mirror SinglePipeSolver API) ───────────────────────

        public void Setup(Vector3 startPoint, Vector3 startDir,
                          Vector3 endPoint,   Vector3 endDir)
            => _inner.Setup(startPoint, startDir, endPoint, endDir);

        public void SetBendRadii(float[] radii)   => _inner.SetBendRadii(radii);
        public void SetTargetLength(float length) => _inner.SetTargetLength(length);
        public void SetDiameter(float diameter)   => _inner.SetDiameter(diameter);

        public void AddCondition(ISolverCondition condition) => _inner.AddCondition(condition);
        public void ClearConditions()                        => _inner.ClearConditions();

        // ── Solve methods ─────────────────────────────────────────────────────

        /// <summary>
        /// Solves the configured pipe path, returning the full <see cref="SolverResult"/>
        /// including sampled centreline points for downstream condition use.
        /// Returns <c>null</c> if no valid solution exists within the bend budget.
        /// </summary>
        public SolverResult? Solve() => _inner.Solve();

        /// <summary>
        /// Convenience overload that returns only the segment list for simple use cases.
        /// Returns <c>null</c> if no solution is found.
        /// </summary>
        public List<BendSegment>? SolveSegments() => _inner.Solve()?.Segments;

        // ── Factory ───────────────────────────────────────────────────────────

        /// <summary>
        /// Creates a <see cref="Solver"/> with all settings applied from the supplied
        /// <see cref="SolverConfig"/>, then calls <see cref="Setup"/> with the given
        /// start/end geometry.  Conditions registered in <c>config.Conditions</c> are
        /// added in order.
        ///
        /// Example:
        /// <code>
        /// var cfg    = SolverConfig.Default(BendRadii, 500f, 41.3f);
        /// var solver = Solver.FromConfig(cfg, start, startDir, end, endDir);
        /// var result = solver.Solve();
        /// </code>
        /// </summary>
        public static Solver FromConfig(
            SolverConfig config,
            Vector3 startPoint, Vector3 startDir,
            Vector3 endPoint,   Vector3 endDir)
        {
            var s = new Solver();
            s.Setup(startPoint, startDir, endPoint, endDir);
            s.SetBendRadii(config.BendRadii);
            s.SetTargetLength(config.TargetLength);
            s.SetDiameter(config.Diameter);
            s.MaxBends                = config.MaxBends;
            s.MaxSolutionsToRank      = config.MaxSolutionsToRank;
            s.ConditionSampleInterval = config.ConditionSampleInterval;
            s.AdamEntryThreshold      = config.AdamEntryThreshold;
            s.AdamFastStageIters      = config.AdamFastStageIters;
            s.AdamFastStageThreshold  = config.AdamFastStageThreshold;
            s.MinStraightLength       = config.MinStraightLength;
            s.MaxBendAngleDeg         = config.MaxBendAngleDeg;
            s.UseParallel             = config.UseParallel;
            foreach (var cond in config.Conditions) s.AddCondition(cond);
            return s;
        }
    }
}
