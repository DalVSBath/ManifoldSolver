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
    public class ManifoldGeoSolver
    {
        // ── Pipe definitions ──────────────────────────────────────────────────

        public readonly record struct PipeDef(
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

        /// <summary>
        /// Sampled points on the new pipe that are within this Euclidean distance
        /// of the new pipe's start or end are exempt from the inter-pipe clearance
        /// check.  This allows the physical connection-port stub region (where the
        /// collector header geometry forces the pipes into close proximity) to be
        /// ignored without falsely rejecting otherwise valid paths.
        /// Set to <c>0</c> to disable the exclusion zone (default).
        /// A typical value is <c>2 × Diameter</c>.
        /// </summary>
        public float ClearanceExcludeEndMm { get; set; } = 0f;

        /// <summary>
        /// Fractional tolerance applied to each pipe's target length.  Passed through
        /// to each <see cref="SinglePipeSolver"/>.  See
        /// <see cref="SinglePipeSolver.LengthToleranceFraction"/> for details.
        /// Default <c>0</c> (exact length required).
        /// </summary>
        public float LengthToleranceFraction { get; set; } = 0f;

        /// <summary>
        /// Number of alternative candidate solutions to retain per pipe for
        /// depth-first back-tracking.  When a later pipe cannot be solved given the
        /// currently committed upstream paths, the solver backtracks to the previous
        /// pipe and tries its next-ranked candidate.
        /// <para>Default is <c>1</c> (greedy — no backtracking, original behaviour).</para>
        /// <para>Set to 3–5 to enable backtracking.  Higher values increase solve time
        /// but improve the chance of finding a valid manifold combination.</para>
        /// </summary>
        public int MaxBacktrackCandidates { get; set; } = 1;

        // ── Pipe and condition registration ───────────────────────────────────

        /// <summary>Registers a pipe by its start/end geometry.</summary>
        public void AddPipe(Vector3 start, Vector3 startDir, Vector3 end, Vector3 endDir)
            => _pipes.Add(new PipeDef(start, Vector3.Normalize(startDir),
                                      end,   Vector3.Normalize(endDir)));

        /// <summary>Registers a condition applied to every pipe in the manifold.</summary>
        public void AddSharedCondition(ISolverCondition condition)
            => _sharedConditions.Add(condition);

        /// <summary>Removes all shared conditions (e.g. to rebuild obstacle set).</summary>
        public void ClearSharedConditions()
            => _sharedConditions.Clear();

        /// <summary>Read-only view of the currently registered shared conditions.</summary>
        public IReadOnlyList<ISolverCondition> SharedConditions => _sharedConditions;

        /// <summary>Registers a condition applied only to the pipe at <paramref name="pipeIndex"/>.</summary>
        public void AddPipeCondition(int pipeIndex, ISolverCondition condition)
        {
            if (!_pipeConditions.ContainsKey(pipeIndex))
                _pipeConditions[pipeIndex] = new List<ISolverCondition>();
            _pipeConditions[pipeIndex].Add(condition);
        }


        public SolverResult?[] GetNaturalsV2()
        {
            SolverResult?[] naturalResults = new SolverResult?[_pipes.Count];
            Log("── Minimum-length pass ──────────────────────────────────────");
            for (int i = 0; i < _pipes.Count; i++)
            {
                var p = _pipes[i];

                var (a1, a2) = Biarc.Calculate(p.Start, p.StartDir, p.End, p.EndDir);
                
                float exploratory = a1.Length + a2.Length;

                Log($"  Pipe {i + 1}: BiArc={exploratory:F1} mm, trying target={exploratory:F1} mm");
                ProgressStep($"Pipe {i + 1}/{_pipes.Count} — min-length pass");

                // Pass shared conditions (static obstacles) so natural-length
                // results are obstacle-aware. Inter-pipe clearance conditions
                // cannot be active here because pipes are solved sequentially.
                var minLenConditions = _sharedConditions.Count > 0 ? _sharedConditions : null;
                var solver = BuildSolver(i, exploratory, quiet: true,
                                        extraConditions: minLenConditions);
                var result = solver.Solve();

                if (result == null)
                {
                    // Retry at a larger exploratory target
                    exploratory *= 1.5f;
                    Log($"  Pipe {i + 1}: retry at {exploratory:F1} mm");
                    solver = BuildSolver(i, exploratory, quiet: true,
                                        extraConditions: minLenConditions);
                    result = solver.Solve();
                }

                naturalResults[i] = result;
                Log($"  Pipe {i + 1}: natural length = {result?.TotalLength.ToString("F1") ?? "FAILED"} mm");
            }

            return naturalResults;
        }

        public SolverResult?[] GetNaturals()
        {
            SolverResult?[] naturalResults = new SolverResult?[_pipes.Count];
            Log("── Minimum-length pass ──────────────────────────────────────");
            for (int i = 0; i < _pipes.Count; i++)
            {
                var p = _pipes[i];
                float euclidean = (p.End - p.Start).Length();
                float exploratory = MathHelper.Max(euclidean * 1.15f, euclidean + 50f);

                Log($"  Pipe {i + 1}: Euclidean={euclidean:F1} mm, trying target={exploratory:F1} mm");
                ProgressStep($"Pipe {i + 1}/{_pipes.Count} — min-length pass");

                // Pass shared conditions (static obstacles) so natural-length
                // results are obstacle-aware. Inter-pipe clearance conditions
                // cannot be active here because pipes are solved sequentially.
                var minLenConditions = _sharedConditions.Count > 0 ? _sharedConditions : null;
                var solver = BuildSolver(i, exploratory, quiet: true,
                                        extraConditions: minLenConditions);
                var result = solver.Solve();

                if (result == null)
                {
                    // Retry at a larger exploratory target
                    exploratory *= 1.5f;
                    Log($"  Pipe {i + 1}: retry at {exploratory:F1} mm");
                    solver = BuildSolver(i, exploratory, quiet: true,
                                        extraConditions: minLenConditions);
                    result = solver.Solve();
                }

                naturalResults[i] = result;
                Log($"  Pipe {i + 1}: natural length = {result?.TotalLength.ToString("F1") ?? "FAILED"} mm");
            }

            return naturalResults;
        }

        public void ClearPipes(bool clearPipeConditions = true)
        {
            _pipes.Clear();
            if (clearPipeConditions)
                _pipeConditions.Clear();
        }

        // ── Main solve ────────────────────────────────────────────────────────

        /// <summary>
        /// Solves all registered pipes.  Returns one <see cref="SolverResult"/> per
        /// pipe in registration order; entries are <c>null</c> when a pipe has no
        /// valid solution.
        /// </summary>
        public List<SolverResult?> Solve(CancellationToken? ct = null)
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
                    float exploratory = MathHelper.Max(euclidean * 1.15f, euclidean + 50f);

                    Log($"  Pipe {i + 1}: Euclidean={euclidean:F1} mm, trying target={exploratory:F1} mm");
                    ProgressStep($"Pipe {i + 1}/{_pipes.Count} — min-length pass");

                    // Pass shared conditions (static obstacles) so natural-length
                    // results are obstacle-aware. Inter-pipe clearance conditions
                    // cannot be active here because pipes are solved sequentially.
                    var minLenConditions = _sharedConditions.Count > 0 ? _sharedConditions : null;
                    var solver = BuildSolver(i, exploratory, quiet: true,
                                            extraConditions: minLenConditions);
                    var result = solver.Solve();

                    if (result == null)
                    {
                        // Retry at a larger exploratory target
                        exploratory *= 1.5f;
                        Log($"  Pipe {i + 1}: retry at {exploratory:F1} mm");
                        solver = BuildSolver(i, exploratory, quiet: true,
                                            extraConditions: minLenConditions);
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
                        maxNatural = MathHelper.Max(maxNatural, naturalResults[i]!.TotalLength);
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

            // Backtracking solve: when enabled, multiple candidate paths per pipe are
            // retained so the solver can try a different upstream route when a later
            // pipe fails.  Shortcuts (LengthInjector/Reuse) are skipped in this mode;
            // the full SinglePipeSolver is always used to ensure a ranked candidate pool.
            if (MaxBacktrackCandidates > 1)
                return SolveWithBacktracking(finalTarget, enableClearance, clearanceGap);

            var results = new List<SolverResult?>(_pipes.Count);

            for (int i = 0; i < _pipes.Count; i++)
            {
                ct?.ThrowIfCancellationRequested();
                var p = _pipes[i];
                Log($"\n{'═',0}══════════════════════════════════════════════");
                Log($"  ManifoldSolver — Pipe {i + 1}/{_pipes.Count}");
                Log("═══════════════════════════════════════════════");

                SolverResult? result      = null;
                string        shortcutKind = "none";

                // Reuse natural-pass result if it already hits the target
                if (EqualizeLength
                    && naturalResults[i] != null
                    && MathHelper.Abs(naturalResults[i]!.TotalLength - finalTarget) < 1f)
                {
                    Log($"  Reusing natural-pass result (length already matches target).");
                    shortcutKind = "Reuse";
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
                    {
                        shortcutKind = "LengthInjector";
                        Log($"  LengthInjector succeeded ({result.TotalLength:F1} mm).");
                    }
                    else
                    {
                        Log($"  LengthInjector failed — falling back to full solver.");
                    }
                }

                // Validate shortcut result against all active conditions.
                // Both shortcuts derive from the unconstrained min-length pass, so
                // they may violate shared obstacle or inter-pipe clearance conditions.
                // Nullifying result causes fall-through to the full solver below.
                if (result != null && activeClearanceConditions.Count > 0
                    && result.SampledPoints != null)
                {
                    foreach (var cond in activeClearanceConditions)
                    {
                        if (!cond.IsSatisfied(result.SampledPoints, Diameter))
                        {
                            Log($"  Pipe {i + 1}: {shortcutKind} result rejected by " +
                                $"{cond.Type} condition — falling back to full solver.");
                            result = null;
                            break;
                        }
                    }
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
                    // Best-effort: skip this pipe rather than aborting the manifold.
                    // Not adding clearance from a null pipe gives subsequent pipes the
                    // widest possible routing space.
                    Log($"\n  *** PIPE {i + 1}: NO SOLUTION FOUND — skipping (best-effort).");
                    results.Add(null);
                    ProgressStep($"Pipe {i + 1}/{_pipes.Count} — no solution");
                    continue;
                }

                results.Add(result);

                // Register this pipe's centreline as a clearance condition for later pipes
                if (enableClearance && result.SampledPoints != null)
                {
                    activeClearanceConditions.Add(
                        new PipeClearanceCondition(result.SampledPoints, clearanceGap)
                        {
                            ExcludeEndMm = ClearanceExcludeEndMm,
                        });

                    Log($"  Registered clearance condition from pipe {i + 1} " +
                        $"({result.SampledPoints.Count} reference points, " +
                        $"surface-gap={clearanceGap:F1} mm, c-c min={(clearanceGap + Diameter):F1} mm" +
                        $"{(ClearanceExcludeEndMm > 0f ? $", excl-end={ClearanceExcludeEndMm:F0} mm" : "")}).");
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
            solver.AdamEntryThreshold      = AdamEntryThreshold;
            solver.AdamFastStageIters      = AdamFastStageIters;
            solver.AdamFastStageThreshold  = AdamFastStageThreshold;
            solver.LengthToleranceFraction = LengthToleranceFraction;
            solver.Verbose              = Verbose && !quiet;
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

        // ── Back-tracking solve ───────────────────────────────────────────────

        /// <summary>
        /// Final-solve pass using depth-first back-tracking: when a pipe cannot be
        /// solved with the currently committed upstream paths, the solver backtracks
        /// to the previous pipe and tries its next-ranked candidate solution.
        /// </summary>
        private List<SolverResult?> SolveWithBacktracking(
            float finalTarget, bool enableClearance, float clearanceGap)
        {
            int n = _pipes.Count;

            // candidatePools[i] is null until we first reach pipe i, and is reset to
            // null on backtrack so it will be recomputed with fresh conditions.
            var candidatePools = new List<SolverResult>?[n];
            var selectedIdx    = new int[n];
            var committed      = new SolverResult?[n];

            // conditionsAtDepth[i] = conditions passed to BuildSolver when solving pipe i:
            //   shared conditions + clearance conditions from committed[0..i-1]
            // Per-pipe conditions are added inside BuildSolver, not stored here.
            var conditionsAtDepth = new List<ISolverCondition>[n + 1];
            conditionsAtDepth[0]  = new List<ISolverCondition>(_sharedConditions);

            int pipe = 0;
            while (pipe >= 0)
            {
                if (pipe == n) break; // All pipes solved successfully

                // ── Compute candidate pool for this pipe if not yet done ──────────
                if (candidatePools[pipe] == null)
                {
                    Log($"\n  ManifoldSolver — Pipe {pipe + 1}/{n} — collecting {MaxBacktrackCandidates} 2-bend candidate(s)");

                    // Fast pass: 2-bend only — just 16 radius combos × 30×30 grid each.
                    // No tolerance retry here: the ±5 mm gate in BuildSolution already
                    // allows a small window, and retries for pipes with no valid route
                    // would waste time.  If a pipe genuinely needs 3+ bends or tolerance
                    // it will be picked up by the BestEffortGreedy fallback.
                    var fastSolver = BuildSolver(pipe, finalTarget, quiet: !Verbose,
                                                 extraConditions: conditionsAtDepth[pipe]);
                    fastSolver.MaxBends              = 2;
                    fastSolver.LengthToleranceFraction = 0f;
                    candidatePools[pipe] = fastSolver.SolveAll(MaxBacktrackCandidates);

                    selectedIdx[pipe] = 0;
                    Log($"  Pipe {pipe + 1}: {candidatePools[pipe]!.Count} candidate(s) found.");
                }

                // ── Try the current candidate index ───────────────────────────────
                if (selectedIdx[pipe] >= candidatePools[pipe]!.Count)
                {
                    // All candidates for this pipe exhausted — backtrack
                    candidatePools[pipe] = null; // Reset; will be recomputed if we return
                    pipe--;
                    if (pipe < 0) break;         // Cannot backtrack further — total failure
                    selectedIdx[pipe]++;
                    Log($"  Backtracking to pipe {pipe + 1} — trying candidate {selectedIdx[pipe] + 1}.");
                    continue;
                }

                // ── Commit the selected candidate for this pipe ───────────────────
                committed[pipe] = candidatePools[pipe]![selectedIdx[pipe]];
                Log($"  Pipe {pipe + 1}: committing candidate {selectedIdx[pipe] + 1}/{candidatePools[pipe]!.Count} " +
                    $"({committed[pipe]!.BendCount} bends, {committed[pipe]!.TotalLength:F1} mm).");

                // Build conditions for the next depth: carry forward current conditions
                // and add a clearance constraint from the just-committed pipe.
                var nextConds = new List<ISolverCondition>(conditionsAtDepth[pipe]);
                if (enableClearance && committed[pipe]!.SampledPoints != null)
                {
                    nextConds.Add(new PipeClearanceCondition(committed[pipe]!.SampledPoints!, clearanceGap)
                    {
                        ExcludeEndMm = ClearanceExcludeEndMm,
                    });
                    Log($"  Registered clearance condition from pipe {pipe + 1} " +
                        $"({committed[pipe]!.SampledPoints!.Count} pts, surface-gap={clearanceGap:F1} mm).");
                }
                conditionsAtDepth[pipe + 1] = nextConds;
                pipe++;
            }

            ProgressEnd();

            if (pipe < 0)
            {
                // Back-tracking found no valid combination across ALL candidates for
                // every pipe.  Fall back to a best-effort greedy forward pass:
                // each pipe is solved with the constraints accumulated so far; if a
                // pipe cannot be solved it is marked null and its clearance is NOT
                // added, giving subsequent pipes the best chance of routing.
                Log("\n  *** ManifoldSolver: back-tracking exhausted — running best-effort greedy pass.");
                return BestEffortGreedy(finalTarget, enableClearance, clearanceGap);
            }

            // pipe == n → all pipes successfully committed
            Log($"\n  ManifoldSolver: all {n} pipes solved via back-tracking.");
            var results = new List<SolverResult?>(_pipes.Count);
            for (int i = 0; i < n; i++) results.Add(committed[i]);
            return results;
        }

        // ── Best-effort greedy forward pass ───────────────────────────────────

        /// <summary>
        /// Sequential forward-only solve: attempts each pipe in registration order
        /// using the full solver.  When a pipe cannot be solved, it is recorded as
        /// <c>null</c> and no clearance condition is added for it, so subsequent
        /// pipes are not unnecessarily constrained.
        /// Used as a fallback when back-tracking is exhausted.
        /// </summary>
        private List<SolverResult?> BestEffortGreedy(
            float finalTarget, bool enableClearance, float clearanceGap)
        {
            int n = _pipes.Count;
            var results = new List<SolverResult?>(n);
            var activeConds = new List<ISolverCondition>(_sharedConditions);

            for (int i = 0; i < n; i++)
            {
                Log($"\n  Best-effort greedy — Pipe {i + 1}/{n}");
                ProgressStep($"Pipe {i + 1}/{n} — best-effort");

                var solver = BuildSolver(i, finalTarget, quiet: !Verbose,
                                         extraConditions: activeConds);
                // No tolerance retry in best-effort mode: tolerance retries cause the
                // solver to exhaustively search ±LengthToleranceFraction × 6 extra
                // targets for pipes that are already infeasible, wasting significant
                // time.  A pipe that cannot solve at the exact target is skipped.
                solver.LengthToleranceFraction = 0f;
                var result = solver.Solve();

                if (result == null)
                {
                    Log($"  Pipe {i + 1}: no solution found — skipping (no clearance added).");
                    results.Add(null);
                }
                else
                {
                    Log($"  Pipe {i + 1}: solved ({result.BendCount} bends, {result.TotalLength:F1} mm).");
                    results.Add(result);

                    if (enableClearance && result.SampledPoints != null)
                    {
                        activeConds.Add(new PipeClearanceCondition(
                            result.SampledPoints, clearanceGap)
                        {
                            ExcludeEndMm = ClearanceExcludeEndMm,
                        });
                        Log($"  Registered clearance condition from pipe {i + 1}.");
                    }
                }
            }

            return results;
        }


        public delegate void LogEventHandler(object sender, string e);
        public event LogEventHandler OnUpdateLog;

        private void Log(string msg, bool important = true)
        {
            if (Verbose || important) { Console.WriteLine(msg); if (OnUpdateLog != null) OnUpdateLog(this, msg); }
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
            filled = MathHelper.Clamp(filled, 0, Width);

            string bar  = new string('█', filled) + new string('░', Width - filled);
            string frac = $"{_progressDone}/{_progressTotal}";
            string tick = finished ? " ✓" : "  ";

            // Truncate label so the whole line stays under ~100 chars
            if (label.Length > 40) label = label[..37] + "…";

            Console.Write($"\r  [{bar}] {frac,5}{tick}  {label,-40}");
        }
    }
}

namespace System.Runtime.CompilerServices
{
    internal static class IsExternalInit { }
}
