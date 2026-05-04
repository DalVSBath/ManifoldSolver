using System;
using System.Collections.Generic;
using System.Numerics;
using GeometrySolver.Conditions;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// Solves multiple pipes simultaneously to a shared target length, enforcing
    /// inter-pipe clearance so no two pipes collide.
    ///
    /// <b>Equal-length strategy (three-step process):</b>
    ///
    /// 1. <b>Minimum-length pass</b>: each pipe is solved independently at a low
    ///    exploratory target (<c>EuclideanDist × 1.1</c>).  This finds the natural
    ///    path geometry without a tight length constraint and records the achieved
    ///    length as the pipe's <i>natural length</i>.
    ///
    /// 2. <b>Target determination</b>: if <see cref="EqualizeLength"/> is
    ///    <c>true</c>, <c>TargetLength</c> is set to
    ///    <c>max(natural lengths) + 5 mm</c>.  Otherwise the caller's explicit
    ///    <see cref="TargetLength"/> is used.
    ///
    /// 3. <b>Final solve pass</b>: each pipe is solved at <c>TargetLength</c> with
    ///    full N-bend search.  After each successful solve, the solved pipe's
    ///    sampled centreline points are registered as a
    ///    <see cref="PipeClearanceCondition"/> for all subsequent pipes.
    ///    If a pipe's natural length already equals the target (within 1 mm), the
    ///    natural-pass result is reused directly.
    ///    Otherwise, <see cref="LengthInjector"/> is tried first; if it fails the
    ///    full solver is re-run at <c>TargetLength</c>.
    /// </summary>
    public class ManifoldSolver
    {
        // ── Pipe definitions ──────────────────────────────────────────────────

        private readonly record struct PipeDef(
            Vector3 Start, Vector3 StartDir, Vector3 End, Vector3 EndDir);

        private readonly List<PipeDef>                     _pipes           = new();
        private readonly List<ISolverCondition>            _sharedConditions = new();
        private readonly Dictionary<int, List<ISolverCondition>> _pipeConditions  = new();

        // ── Configuration ─────────────────────────────────────────────────────

        /// <summary>Bend radii available from the die set (mm).</summary>
        public float[] BendRadii { get; set; } = Array.Empty<float>();

        /// <summary>
        /// Shared target total path length (mm).
        /// Ignored when <see cref="EqualizeLength"/> is <c>true</c> (auto-determined).
        /// </summary>
        public float TargetLength { get; set; }

        /// <summary>Outer pipe diameter used for clearance calculations (mm).</summary>
        public float Diameter { get; set; }

        /// <summary>Maximum number of bends the solver may use per pipe.</summary>
        public int MaxBends { get; set; } = 8;

        /// <summary>Minimum straight-section length (mm). Applied to every pipe. Default 0.</summary>
        public float MinStraightLength { get; set; } = 0f;

        /// <summary>
        /// Maximum allowable bend angle in degrees. Applied to every pipe. Default 180.
        /// </summary>
        public float MaxBendAngleDeg { get; set; } = 180f;

        // ── P1.1 / P1.2 performance thresholds (passed through to each SinglePipeSolver) ──

        /// <inheritdoc cref="SinglePipeSolver.AdamEntryThreshold"/>
        public double AdamEntryThreshold { get; set; } = 1e6;

        /// <inheritdoc cref="SinglePipeSolver.AdamFastStageIters"/>
        public int AdamFastStageIters { get; set; } = 500;

        /// <inheritdoc cref="SinglePipeSolver.AdamFastStageThreshold"/>
        public double AdamFastStageThreshold { get; set; } = 1e3;
        /// <summary>Print progress to the console.</summary>
        public bool Verbose { get; set; } = true;

        /// <summary>
        /// When <c>true</c> the manifold solver automatically determines
        /// <see cref="TargetLength"/> from the longest natural-path pipe.
        /// When <c>false</c>, <see cref="TargetLength"/> must be set explicitly.
        /// </summary>
        public bool EqualizeLength { get; set; } = true;

        /// <summary>
        /// Minimum surface-to-surface clearance gap between adjacent pipes (mm).
        /// <c>0</c> means pipes may touch surface-to-surface but not overlap.
        /// The <see cref="PipeClearanceCondition"/> always adds <see cref="Diameter"/> to
        /// this value to compute the required centre-to-centre distance.
        /// Set to a negative value to disable inter-pipe clearance checking entirely.
        /// </summary>
        public float MinClearance { get; set; } = 0f;

        // ── Pipe and condition registration ───────────────────────────────────

        /// <summary>Registers a pipe by its start/end geometry.</summary>
        public void AddPipe(Vector3 start, Vector3 startDir, Vector3 end, Vector3 endDir)
            => _pipes.Add(new PipeDef(start, Vector3.Normalize(startDir),
                                      end,   Vector3.Normalize(endDir)));

        /// <summary>Registers a condition applied to every pipe in the manifold.</summary>
        public void AddSharedCondition(ISolverCondition condition)
            => _sharedConditions.Add(condition);

        /// <summary>Registers a condition applied only to the pipe at <paramref name="pipeIndex"/>.</summary>
        public void AddPipeCondition(int pipeIndex, ISolverCondition condition)
        {
            if (!_pipeConditions.ContainsKey(pipeIndex))
                _pipeConditions[pipeIndex] = new List<ISolverCondition>();
            _pipeConditions[pipeIndex].Add(condition);
        }

        // ── Main solve ────────────────────────────────────────────────────────

        /// <summary>
        /// Solves all registered pipes.  Returns one <see cref="SolverResult"/> per
        /// pipe in registration order; entries are <c>null</c> when a pipe has no
        /// valid solution.
        /// </summary>
        public List<SolverResult?> Solve()
        {
            if (_pipes.Count == 0) return new List<SolverResult?>();

            // Progress bar: each pipe has 1 step (final solve) or 2 steps (min-length + final)
            int stepsPerPipe = EqualizeLength ? 2 : 1;
            ProgressStart(_pipes.Count * stepsPerPipe);

            // ── Step 1: minimum-length pass (only when EqualizeLength is true) ──

            SolverResult?[] naturalResults = new SolverResult?[_pipes.Count];

            if (EqualizeLength)
            {
                Log("── Minimum-length pass ──────────────────────────────────────");
                for (int i = 0; i < _pipes.Count; i++)
                {
                    var p = _pipes[i];
                    float euclidean  = (p.End - p.Start).Length();
                    float exploratory = MathF.Max(euclidean * 1.15f, euclidean + 50f);

                    Log($"  Pipe {i + 1}: Euclidean={euclidean:F1} mm, trying target={exploratory:F1} mm");
                    ProgressStep($"Pipe {i + 1}/{_pipes.Count} — min-length pass");

                    var solver = BuildSolver(i, exploratory, quiet: true);
                    var result = solver.Solve();

                    if (result == null)
                    {
                        // Retry at a larger exploratory target
                        exploratory *= 1.5f;
                        Log($"  Pipe {i + 1}: retry at {exploratory:F1} mm");
                        solver = BuildSolver(i, exploratory, quiet: true);
                        result = solver.Solve();
                    }

                    naturalResults[i] = result;
                    Log($"  Pipe {i + 1}: natural length = {result?.TotalLength.ToString("F1") ?? "FAILED"} mm");
                }
            }

            // ── Step 2: determine shared target length ─────────────────────────

            float finalTarget = TargetLength;

            if (EqualizeLength)
            {
                float maxNatural = 0f;
                for (int i = 0; i < _pipes.Count; i++)
                {
                    if (naturalResults[i] != null)
                        maxNatural = MathF.Max(maxNatural, naturalResults[i]!.TotalLength);
                }

                if (maxNatural > 0f)
                {
                    finalTarget = maxNatural + 5f;
                    Log($"\n── Target length auto-set to {finalTarget:F1} mm (max natural + 5 mm) ──");
                }
            }

            if (finalTarget <= 0f)
                throw new InvalidOperationException(
                    "TargetLength must be > 0 when EqualizeLength is false.");

            // Accumulate clearance conditions from solved pipes
            var activeClearanceConditions = new List<ISolverCondition>(_sharedConditions);
            // MinClearance is the surface-to-surface gap; PipeClearanceCondition adds
            // pipeDiameter internally.  A negative value disables clearance entirely.
            bool enableClearance = MinClearance >= 0f;
            float clearanceGap   = MinClearance;

            // ── Step 3: final solve at target length ───────────────────────────

            Log($"\n── Final solve pass at {finalTarget:F1} mm ──────────────────────────");

            var results = new List<SolverResult?>(_pipes.Count);

            for (int i = 0; i < _pipes.Count; i++)
            {
                var p = _pipes[i];
                Log($"\n{'═',0}══════════════════════════════════════════════");
                Log($"  ManifoldSolver — Pipe {i + 1}/{_pipes.Count}");
                Log("═══════════════════════════════════════════════");

                SolverResult? result = null;

                // Reuse natural-pass result if it already hits the target
                if (EqualizeLength
                    && naturalResults[i] != null
                    && MathF.Abs(naturalResults[i]!.TotalLength - finalTarget) < 1f)
                {
                    Log($"  Reusing natural-pass result (length already matches target).");
                    result = naturalResults[i];
                }
                // Try LengthInjector if natural result is shorter
                else if (EqualizeLength
                         && naturalResults[i] != null
                         && naturalResults[i]!.TotalLength < finalTarget)
                {
                    Log($"  Trying LengthInjector (deficit {finalTarget - naturalResults[i]!.TotalLength:F1} mm)...");
                    result = LengthInjector.Inject(
                        naturalResults[i]!,
                        finalTarget,
                        BendRadii,
                        p.Start, p.StartDir);

                    if (result != null)
                        Log($"  LengthInjector succeeded ({result.TotalLength:F1} mm).");
                    else
                        Log($"  LengthInjector failed — falling back to full solver.");
                }

                // Full solver if not yet solved
                if (result == null)
                {
                    ProgressStep($"Pipe {i + 1}/{_pipes.Count} — full solve ({MaxBends}-bend)");
                    var solver = BuildSolver(i, finalTarget, quiet: false,
                                             extraConditions: activeClearanceConditions);
                    result = solver.Solve();
                }
                else
                {
                    // Count the final-solve step even when reused/injected
                    ProgressStep($"Pipe {i + 1}/{_pipes.Count} — reused/injected");
                }

                if (result == null)
                {
                    Log($"\n  *** PIPE {i + 1}: NO SOLUTION FOUND — aborting manifold.");
                    ProgressEnd();
                    results.Add(null);
                    for (int j = i + 1; j < _pipes.Count; j++) results.Add(null);
                    return results;
                }

                results.Add(result);

                // Register this pipe's centreline as a clearance condition for later pipes
                if (enableClearance && result.SampledPoints != null)
                {
                    activeClearanceConditions.Add(
                        new PipeClearanceCondition(result.SampledPoints, clearanceGap));

                    Log($"  Registered clearance condition from pipe {i + 1} " +
                        $"({result.SampledPoints.Count} reference points, " +
                        $"surface-gap={clearanceGap:F1} mm, c-c min={(clearanceGap + Diameter):F1} mm).");
                }
            }

            ProgressEnd();
            return results;
        }

        private SinglePipeSolver BuildSolver(
            int              pipeIndex,
            float            targetLength,
            bool             quiet,
            List<ISolverCondition>? extraConditions = null)
        {
            var p = _pipes[pipeIndex];
            var solver = new SinglePipeSolver();
            solver.Setup(p.Start, p.StartDir, p.End, p.EndDir);
            solver.SetBendRadii(BendRadii);
            solver.SetTargetLength(targetLength);
            solver.SetDiameter(Diameter);
            solver.MaxBends             = MaxBends;
            solver.MinStraightLength    = MinStraightLength;
            solver.MaxBendAngleDeg      = MaxBendAngleDeg;
            solver.AdamEntryThreshold   = AdamEntryThreshold;
            solver.AdamFastStageIters   = AdamFastStageIters;
            solver.AdamFastStageThreshold = AdamFastStageThreshold;
            solver.Verbose              = Verbose && !quiet;
            // P4.1: each pipe solver runs its own parallel combo scan.
            // ManifoldSolver's pipe loop is sequential (each pipe depends on
            // the previous one for PipeClearanceCondition) so this is safe.
            solver.UseParallel          = true;

            // Shared + per-pipe + clearance conditions
            if (extraConditions != null)
                foreach (var c in extraConditions) solver.AddCondition(c);

            if (_pipeConditions.TryGetValue(pipeIndex, out var perPipe))
                foreach (var c in perPipe) solver.AddCondition(c);

            return solver;
        }

        private void Log(string msg)
        {
            if (Verbose) Console.WriteLine(msg);
        }

        // ── Progress bar (used when Verbose = false) ──────────────────────────

        private int  _progressDone;
        private int  _progressTotal;
        private bool _progressActive;

        private void ProgressStart(int totalSteps)
        {
            if (Verbose) return;
            _progressDone   = 0;
            _progressTotal  = totalSteps;
            _progressActive = true;
            DrawBar("Starting…");
        }

        private void ProgressStep(string label)
        {
            if (!_progressActive) return;
            _progressDone++;
            DrawBar(label);
        }

        private void ProgressEnd()
        {
            if (!_progressActive) return;
            _progressActive = false;
            DrawBar("Done", finished: true);
            Console.WriteLine();
        }

        private void DrawBar(string label, bool finished = false)
        {
            const int Width = 30;
            int filled = _progressTotal > 0
                ? (int)Math.Round((double)_progressDone / _progressTotal * Width)
                : 0;
            filled = Math.Clamp(filled, 0, Width);

            string bar  = new string('█', filled) + new string('░', Width - filled);
            string frac = $"{_progressDone}/{_progressTotal}";
            string tick = finished ? " ✓" : "  ";

            // Truncate label so the whole line stays under ~100 chars
            if (label.Length > 40) label = label[..37] + "…";

            Console.Write($"\r  [{bar}] {frac,5}{tick}  {label,-40}");
        }
    }
}
