using System;
using System.Collections.Generic;
using System.Text.Json.Serialization;
using GeometrySolver.Conditions;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// Immutable value describing a single bend in a pipe path.
    /// </summary>
    public struct BendSegment
    {
        /// <summary>Centre-line radius (CLR) of the bend die, in mm.</summary>
        public float CLR { get; set; }

        /// <summary>Bend angle in radians.</summary>
        public float Angle { get; set; }

        /// <summary>Rotation of the pipe in the bender (roll angle), in radians.</summary>
        public float Rotation { get; set; }

        /// <summary>Length of the straight feed before this bend, in mm.</summary>
        public float StraightLength { get; set; }
    }

    /// <summary>
    /// Serialisable configuration record that captures every setting required to run
    /// a <see cref="SinglePipeSolver"/> or <see cref="ManifoldSolver"/> solve.
    ///
    /// Pass an instance to <see cref="Solver.FromConfig"/> to configure a solver in one
    /// call, or serialise/deserialise it with <c>System.Text.Json</c> to persist die-set
    /// and tolerance settings for batch jobs.
    /// </summary>
    public class SolverConfig
    {
        // ── Die-set / geometry ───────────────────────────────────────────────

        /// <summary>Available centre-line bend radii from the die set (mm).</summary>
        public float[] BendRadii { get; set; } = Array.Empty<float>();

        /// <summary>Outer pipe diameter used for clearance calculations (mm).</summary>
        public float Diameter { get; set; }

        /// <summary>Target total path length (mm).</summary>
        public float TargetLength { get; set; }

        // ── Solver limits ────────────────────────────────────────────────────

        /// <summary>Maximum number of bends the solver may use. Default 8.</summary>
        public int MaxBends { get; set; } = 8;

        /// <summary>
        /// Maximum valid solutions to collect and rank per bend count before selecting
        /// the best one. Default 3.
        /// </summary>
        public int MaxSolutionsToRank { get; set; } = 3;

        /// <summary>
        /// Adam condition-penalty sampling interval. Conditions are evaluated every
        /// this many iterations. Default 5.
        /// </summary>
        public int ConditionSampleInterval { get; set; } = 5;

        // ── P1.1 / P1.2 performance thresholds ──────────────────────────────

        /// <summary>
        /// P1.1 — Grid residual gate.  If the best residual from the initial grid/DE
        /// search exceeds this value, Adam is skipped entirely for that radius
        /// combination.  Default <c>1e6</c>.  Set to <c>double.MaxValue</c> to disable.
        /// </summary>
        public double AdamEntryThreshold { get; set; } = 1e6;

        /// <summary>
        /// P1.2 — Iterations in the fast probe stage of Adam.  After this many
        /// iterations the residual is checked against <see cref="AdamFastStageThreshold"/>.
        /// Default 500.
        /// </summary>
        public int AdamFastStageIters { get; set; } = 500;

        /// <summary>
        /// P1.2 — Residual threshold at the end of the fast stage.  Combinations still
        /// above this value are abandoned immediately.  Default <c>1e3</c>.
        /// Set to <c>double.MaxValue</c> to disable the two-stage budget.
        /// </summary>
        public double AdamFastStageThreshold { get; set; } = 1e3;

        // ── Manufacturing tolerances ─────────────────────────────────────────

        /// <summary>
        /// Minimum allowable straight-section length (mm). Segments shorter than this
        /// are rejected. Default 0 (no minimum enforced).
        /// </summary>
        public float MinStraightLength { get; set; } = 0f;

        /// <summary>
        /// Maximum allowable bend angle in degrees. Bends larger than this are
        /// rejected. Default 180 (no maximum enforced).
        /// </summary>
        public float MaxBendAngleDeg { get; set; } = 180f;

        /// <inheritdoc cref="SinglePipeSolver.LengthToleranceFraction"/>
        public float LengthToleranceFraction { get; set; } = 0f;

        // ── Parallelism ──────────────────────────────────────────────────────

        /// <summary>
        /// P4.1 — Run the radius-combination loop in parallel using all available CPU
        /// cores.  Default <c>true</c>.  Set to <c>false</c> when debugging or when
        /// the solver is already running inside an outer parallel loop.
        /// </summary>
        public bool UseParallel { get; set; } = true;

        // ── Conditions ───────────────────────────────────────────────────────

        /// <summary>
        /// Conditions applied to every solve. Not serialised; populate at runtime.
        /// </summary>
        [JsonIgnore]
        public List<ISolverCondition> Conditions { get; set; } = new();

        // ── Factory helpers ──────────────────────────────────────────────────

        /// <summary>Creates a default config suitable for the standard die set.</summary>
        public static SolverConfig Default(float[] bendRadii, float targetLength, float diameter) =>
            new SolverConfig
            {
                BendRadii     = bendRadii,
                TargetLength  = targetLength,
                Diameter      = diameter,
            };
    }

    /// <summary>Legacy stub kept for binary compatibility. Use <see cref="SolverConfig"/>.</summary>
    internal class SolverItems { }
}
