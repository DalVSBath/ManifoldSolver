using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using GeometrySolver.Conditions;

namespace GeometrySolver.Solver
{
    /// <summary>
    /// Core single-pipe path solver. Kept internal; external callers use the
    /// <see cref="Solver"/> public facade or <see cref="ManifoldSolver"/>.
    /// </summary>
    internal class SinglePipeSolver
    {
        private Vector3 _startPoint, _startDir;
        private Vector3 _endPoint,   _endDir;
        private float   _targetLength;
        private float   _diameter;
        private float[] _bendRadii = Array.Empty<float>();
        private readonly List<ISolverCondition> _conditions = new();

        public SinglePipeSolver() { }

        /// <summary>Maximum number of bends to attempt before giving up (default 8).</summary>
        public int MaxBends { get; set; } = 8;

        public bool Verbose { get; set; } = true;

        /// <summary>
        /// How often ISolverCondition.Penalty is evaluated during Adam refinement.
        /// A value of 5 means condition cost is recalculated every 5 iterations,
        /// reducing overhead by ~80% when expensive conditions are active.
        /// Set to 1 to evaluate on every iteration. Default is 5.
        /// </summary>
        public int ConditionSampleInterval { get; set; } = 5;

        /// <summary>
        /// Maximum valid solutions to collect and rank before selecting the best
        /// within a given bend count. Combos are tried in descending CLR-sum order
        /// so the first valid solution already has the largest average CLR; subsequent
        /// ones are retained only to compare minimum straight length as tiebreaker.
        /// Default is 3. Set to 1 to return the very first valid solution.
        /// </summary>
        public int MaxSolutionsToRank { get; set; } = 3;

        // ── P1.1 / P1.2 performance thresholds ───────────────────────────────

        /// <summary>
        /// P1.1 — If the best grid/DE residual after <see cref="InitialSearch"/>
        /// exceeds this value, Adam refinement is skipped entirely for that radius
        /// combination.  Failed combinations typically show residuals of 1e8–1e9;
        /// converging candidates land below 1e5.  Default is <c>1e6</c>.
        /// Set to <c>double.MaxValue</c> to disable the gate and always run Adam.
        /// </summary>
        public double AdamEntryThreshold { get; set; } = 1e6;

        /// <summary>
        /// P1.2 — Number of Adam iterations in the fast (probe) stage.  After this
        /// many iterations the residual is checked; if it still exceeds
        /// <see cref="AdamFastStageThreshold"/> Adam is aborted for that combination.
        /// Default is 500.  Must be less than the total Adam budget (6 000).
        /// </summary>
        public int AdamFastStageIters { get; set; } = 500;

        /// <summary>
        /// P1.2 — Residual threshold tested after the fast stage.  Combinations
        /// whose residual is still above this value after <see cref="AdamFastStageIters"/>
        /// iterations are abandoned immediately.  Default is <c>1e3</c>.
        /// Set to <c>double.MaxValue</c> to disable the two-stage budget.
        /// </summary>
        public double AdamFastStageThreshold { get; set; } = 1e3;

        /// <summary>
        /// Minimum straight-section length (mm). Segments shorter than this are rejected.
        /// Default 0 (no minimum). Applied in BuildSolution.
        /// </summary>
        public float MinStraightLength { get; set; } = 0f;

        /// <summary>
        /// Maximum allowable bend angle in degrees. Bends larger than this are rejected.
        /// Default 180 (no maximum). Applied in BuildSolution.
        /// </summary>
        public float MaxBendAngleDeg { get; set; } = 180f;

        /// <summary>
        /// P4.1 — When <c>true</c> (the default), the radius-combination loop inside
        /// each bend-count pass is executed in parallel using <see cref="Parallel.ForEach"/>.
        /// Each combination is fully independent so there is no shared mutable state.
        /// Disable for single-threaded debugging or when the solver is already embedded
        /// inside an outer parallel loop (e.g. <see cref="ManifoldSolver"/>).
        /// </summary>
        public bool UseParallel { get; set; } = true;

        /// <summary>
        /// Fractional tolerance applied to the target pipe length.  When greater than
        /// zero and the primary solve at the exact target fails, the solver retries at
        /// lengths of <c>target ± (target × LengthToleranceFraction)</c> in three equal
        /// steps per direction.  A value of <c>0.03</c> allows ±3 % (±18 mm on a
        /// 600 mm target).  Default is <c>0</c> (exact length required).
        /// </summary>
        public float LengthToleranceFraction { get; set; } = 0f;

        public void Setup(Vector3 startPoint, Vector3 startDir,
                          Vector3 endPoint,   Vector3 endDir)
        {
            _startPoint = startPoint;
            _startDir   = Vector3.Normalize(startDir);
            _endPoint   = endPoint;
            _endDir     = Vector3.Normalize(endDir);
        }

        public void SetBendRadii(float[] radii)   => _bendRadii    = radii;
        public void SetTargetLength(float length) => _targetLength = length;
        public void SetDiameter(float diameter)   => _diameter     = diameter;

        /// <summary>Registers a constraint that every candidate path must satisfy.</summary>
        public void AddCondition(ISolverCondition condition) => _conditions.Add(condition);

        /// <summary>Removes all previously registered conditions.</summary>
        public void ClearConditions() => _conditions.Clear();

        // ── Public entry point ────────────────────────────────────────────────

        /// <summary>
        /// Iteratively tries N-bend solutions for N = 2..MaxBends, returning the first
        /// valid segment list found or null if no solution exists within the bend budget.
        /// When <see cref="LengthToleranceFraction"/> is greater than zero and the
        /// primary solve fails, retries at lengths within ±<c>LengthToleranceFraction</c>
        /// of the original target in three equal steps per direction.
        /// </summary>
        public SolverResult? Solve(CancellationToken? ct = null)
        {
            // Primary attempt at the exact target length
            var result = InnerSolve(ct);
            if (result != null || LengthToleranceFraction <= 0f)
            {
                if (result == null && Verbose)
                    Console.WriteLine("\nAll bend counts exhausted — no solution found.");
                return result;
            }

            // Retry within the ±tolerance band (3 steps per direction)
            float band  = _targetLength * LengthToleranceFraction;
            float step  = band / 3f;
            float saved = _targetLength;
            try
            {
                for (float delta = step; delta <= band + 0.01f; delta += step)
                {
                    ct?.ThrowIfCancellationRequested();

                    _targetLength = saved + delta;
                    if (Verbose) Console.WriteLine($"\n--- Retrying at {_targetLength:F1} mm (+{delta:F1}) ---");
                    result = InnerSolve(ct);
                    if (result != null) return result;

                    _targetLength = saved - delta;
                    if (Verbose) Console.WriteLine($"\n--- Retrying at {_targetLength:F1} mm (-{delta:F1}) ---");
                    result = InnerSolve(ct);
                    if (result != null) return result;
                }
            }
            finally { _targetLength = saved; }

            if (Verbose) Console.WriteLine("\nAll bend counts exhausted — no solution found.");
            return null;
        }

        /// <summary>
        /// Inner solve loop: tries all N-bend counts at the current <c>_targetLength</c>.
        /// Returns a <see cref="SolverResult"/> on first success, or <c>null</c> if all
        /// bend counts fail.  Does not modify <c>_targetLength</c>.
        /// </summary>
        private SolverResult? InnerSolve(CancellationToken? ct = null)
        {
            for (int nBends = 2; nBends <= MaxBends; nBends++)
            {
                ct?.ThrowIfCancellationRequested();
                if (Verbose) Console.WriteLine($"\n=== Trying {nBends}-bend solutions ===");

                var segs = TrySolveBends(nBends, ct);
                if (segs != null)
                {
                    var   sim       = PathSimulator.Simulate(_startPoint, _startDir, segs);
                    int   bendCount = segs.FindAll(s => s.Angle > 1e-4f).Count;
                    float spacing   = _diameter > 0f ? _diameter * 0.5f : 10f;
                    var   pts       = PathSampler.Sample(_startPoint, _startDir, segs, spacing);

                    if (Verbose)
                        Console.WriteLine($"==> Solution found: {bendCount} bends, {segs.Count} segments.");

                    return new SolverResult
                    {
                        Segments       = segs,
                        TotalLength    = sim.TotalLength,
                        PositionError  = (sim.Position - _endPoint).Length(),
                        DirectionError = (sim.Direction - _endDir).Length(),
                        BendCount      = bendCount,
                        IsValid        = true,
                        SampledPoints  = pts,
                    };
                }
            }
            return null;
        }

        // ── Multi-result search (used by ManifoldSolver backtracking) ─────────

        /// <summary>
        /// Returns up to <paramref name="maxResults"/> valid paths ranked best-first.
        /// Unlike <see cref="Solve"/> (which returns only the single best solution),
        /// this method retains multiple distinct candidates so that
        /// <see cref="ManifoldSolver"/> can back-track when a greedy commitment blocks
        /// later pipes.
        /// When <see cref="LengthToleranceFraction"/> is &gt; 0 and the exact-target
        /// search yields no candidates, the search is retried within the tolerance band.
        /// </summary>
        public List<SolverResult> SolveAll(int maxResults = 5)
        {
            var all = new List<(SolverResult r, float score)>();
            InnerSolveAll(all, maxResults);

            // Tolerance retry: only triggered when exact-target search produced nothing
            if (all.Count == 0 && LengthToleranceFraction > 0f)
            {
                float band  = _targetLength * LengthToleranceFraction;
                float step  = band / 3f;
                float saved = _targetLength;
                try
                {
                    for (float delta = step; delta <= band + 0.01f && all.Count == 0; delta += step)
                    {
                        _targetLength = saved + delta;
                        InnerSolveAll(all, maxResults);

                        if (all.Count == 0)
                        {
                            _targetLength = saved - delta;
                            InnerSolveAll(all, maxResults);
                        }
                    }
                }
                finally { _targetLength = saved; }
            }

            all.Sort((a, b) => b.score.CompareTo(a.score));
            int take = Math.Min(maxResults, all.Count);
            var out_ = new List<SolverResult>(take);
            for (int i = 0; i < take; i++) out_.Add(all[i].r);
            return out_;
        }

        /// <summary>
        /// Accumulates valid solutions across all bend counts at the current
        /// <c>_targetLength</c>, stopping when <paramref name="accumulator"/> reaches
        /// <paramref name="maxResults"/>.  Called by <see cref="SolveAll"/>.
        /// </summary>
        private void InnerSolveAll(List<(SolverResult r, float score)> accumulator, int maxResults)
        {
            for (int nBends = 2; nBends <= MaxBends && accumulator.Count < maxResults; nBends++)
            {
                if (Verbose) Console.WriteLine($"\n=== Trying {nBends}-bend solutions (ranked) ===");

                var batch = TrySolveBendsAll(nBends, maxResults - accumulator.Count);
                foreach (var (segs, score) in batch)
                {
                    var   sim       = PathSimulator.Simulate(_startPoint, _startDir, segs);
                    int   bendCount = segs.FindAll(s => s.Angle > 1e-4f).Count;
                    float spacing   = _diameter > 0f ? _diameter * 0.5f : 10f;
                    var   pts       = PathSampler.Sample(_startPoint, _startDir, segs, spacing);

                    if (Verbose)
                        Console.WriteLine(
                            $"==> Candidate: {bendCount} bends, {sim.TotalLength:F1} mm (score={score:F1}).");

                    accumulator.Add((new SolverResult
                    {
                        Segments       = segs,
                        TotalLength    = sim.TotalLength,
                        PositionError  = (sim.Position - _endPoint).Length(),
                        DirectionError = (sim.Direction - _endDir).Length(),
                        BendCount      = bendCount,
                        IsValid        = true,
                        SampledPoints  = pts,
                    }, score));
                }

                // Once any bend count produces candidates, stop — higher bend counts
                // are geometrically worse and far more expensive to search.  If all
                // candidates at this bend count ultimately fail during backtracking,
                // the ManifoldSolver will escalate to a higher bend count on the next
                // call (or the user can increase MaxBacktrackCandidates / MaxBends).
                if (batch.Count > 0) break;
            }
        }

        // ── Radius-combination loop ───────────────────────────────────────────
        //
        // TrySolveBendsAll is the core implementation: it collects up to maxCollect
        // valid solutions (sorted best-first) so ManifoldSolver can back-track across
        // candidates when a greedy commitment blocks later pipes.
        // TrySolveBends wraps it to return only the single best result (original API).

        private List<(List<BendSegment> Segs, float Score)> TrySolveBendsAll(
            int nBends, int maxCollect, CancellationToken? token = null)
        {
            var combos  = GenerateRadiusCombinations(nBends);
            int total   = combos.Count;
            var results = new List<(List<BendSegment> Segs, float Score)>();

            if (!UseParallel)
            {
                // ── Sequential path ───────────────────────────────────────────────
                int idx = 0;
                foreach (var radii in combos)
                {
                    idx++;
                    if (Verbose)
                    {
                        var rStr = string.Join(", ", Array.ConvertAll(radii, r => $"R{r:F0}"));
                        Console.WriteLine($"  [{idx}/{total}] {rStr}");
                    }

                    float minArc = 0f;
                    foreach (float r in radii) minArc += r * 0.02f;
                    if (minArc > _targetLength)
                    {
                        if (Verbose) Console.WriteLine("    Skipped (min arc > target length).");
                        continue;
                    }

                    var segs = TrySolveRadii(nBends, radii);
                    if (segs != null)
                    {
                        float score = ScoreSolution(segs);
                        results.Add((segs, score));

                        // When conditions are active, results from TrySolveRadii already
                        // satisfy them (BuildSolution gates on all conditions). Cancelling
                        // at the collection cap is always safe.
                        if (results.Count >= maxCollect)
                        {
                            if (Verbose) Console.WriteLine($"    Collection cap reached ({maxCollect}).");
                            break;
                        }
                        if (Verbose) Console.WriteLine($"    Candidate (score={score:F1}), continuing to rank...");
                    }
                    else if (Verbose)
                        Console.WriteLine("    No valid solution.");
                }
            }
            else
            {
                // ── P4.1: Parallel path ───────────────────────────────────────────
                var resultsLock = new object();
                using var cts   = new CancellationTokenSource();

                try
                {
                    Parallel.ForEach(combos,
                        new ParallelOptions { CancellationToken = cts.Token },
                        (radii, loopState) =>
                        {
                            if (cts.IsCancellationRequested || (token?.IsCancellationRequested ?? false)) return;

                            float minArc = 0f;
                            foreach (float r in radii) minArc += r * 0.02f;
                            if (minArc > _targetLength) return;

                            var segs = TrySolveRadii(nBends, radii);
                            if (segs == null) return;

                            float score = ScoreSolution(segs);
                            lock (resultsLock)
                            {
                                results.Add((segs, score));
                                // Results from TrySolveRadii already satisfy all conditions
                                // (BuildSolution gates them). Cancel at the collection cap regardless.
                                if (results.Count >= maxCollect)
                                    cts.Cancel();
                            }
                        });
                }
                catch (OperationCanceledException) { /* normal early-exit */ }
            }

            results.Sort((a, b) => b.Score.CompareTo(a.Score));
            return results;
        }

        private List<BendSegment>? TrySolveBends(int nBends, CancellationToken? ct = null)
        {
            var all = TrySolveBendsAll(nBends, MaxSolutionsToRank, ct);
            if (Verbose && all.Count > 0)
                Console.WriteLine($"    Best solution selected (score={all[0].Score:F1}).");
            return all.Count > 0 ? all[0].Segs : null;
        }

        // ── Solution scoring (higher = better) ────────────────────────────────
        //
        // Primary:   fewer bends   (weight 10 000)
        // Secondary: larger avg CLR   (mm, weight 1)
        // Tertiary:  longer min straight run   (mm, weight 0.1)

        private static float ScoreSolution(List<BendSegment> segs)
        {
            int   bends       = 0;
            float clrSum      = 0f;
            float minStraight = float.MaxValue;

            foreach (var s in segs)
            {
                if (s.Angle > 1e-4f) { bends++; clrSum += s.CLR; }
                if (s.StraightLength < minStraight) minStraight = s.StraightLength;
            }

            float avgCLR = bends > 0 ? clrSum / bends : 0f;
            return -bends * 10_000f + avgCLR + (minStraight < float.MaxValue ? minStraight * 0.1f : 0f);
        }

        // Generate all R^N radius N-tuples, sorted descending by CLR sum (best CLR first)
        private List<float[]> GenerateRadiusCombinations(int n)
        {
            var sorted = (float[])_bendRadii.Clone();
            Array.Sort(sorted);
            Array.Reverse(sorted);   // largest CLR first

            var result = new List<float[]>();
            FillCombinations(sorted, n, new float[n], 0, result);
            return result;
        }

        private static void FillCombinations(
            float[] radii, int n, float[] buf, int pos, List<float[]> result)
        {
            if (pos == n) { result.Add((float[])buf.Clone()); return; }
            foreach (float r in radii)
            {
                buf[pos] = r;
                FillCombinations(radii, n, buf, pos + 1, result);
            }
        }

        // ── Per-radii solver ──────────────────────────────────────────────────

        private List<BendSegment>? TrySolveRadii(int nBends, float[] radii)
        {
            int nParams = 2 * (nBends - 1);   // 2 angles per intermediate direction

            // 1. Grid / random initial search
            double[] bestP = InitialSearch(nBends, nParams, radii, out double bestRes);
            if (Verbose) Console.WriteLine($"    Grid best residual: {bestRes:E3}");

            // P1.1: Skip Adam entirely when the grid residual is too high.
            // Failed combos typically show 1e8–1e9; converging candidates land below 1e5.
            // Skipping saves up to 174 000 ComputeResidual calls per dead-end combination.
            if (bestRes > AdamEntryThreshold)
            {
                if (Verbose)
                    Console.WriteLine($"    Skipped Adam (grid residual exceeds entry threshold {AdamEntryThreshold:E0}).");
                return null;
            }

            // 2. Adam gradient refinement
            double[] refined = RefineDirections(bestP, radii, nBends);
            double refinedRes = ComputeResidual(refined, radii, nBends);
            if (Verbose) Console.WriteLine($"    Refined residual:   {refinedRes:E3}");

            // 3. Build and simulator-verify the solution
            return BuildSolution(refined, radii, nBends);
        }

        // ── Initial search ────────────────────────────────────────────────────

        private double[] InitialSearch(
            int nBends, int nParams, float[] radii, out double bestRes)
        {
            double[] bestP   = new double[nParams];
            double   localBest = double.MaxValue;

            void Probe(double[] p)
            {
                // Skip condition penalties during the grid sweep — they are
                // expensive (segment build + path sample) and unnecessary at
                // this coarse search stage.  Adam refinement pays the full cost.
                double r = ComputeResidual(p, radii, nBends, skipConditions: true);
                if (r < localBest) { localBest = r; bestP = (double[])p.Clone(); }
            }

            if (nBends == 2)
            {
                // 30×30 sphere grid for (θ, ψ)
                const int G = 30;
                for (int i = 0; i < G; i++)
                for (int j = 0; j < G; j++)
                    Probe(new[] { (i + 0.5) * Math.PI / G,
                                   j * 2.0  * Math.PI / G });
            }
            else if (nBends == 3)
            {
                // 12^4 = 20 736 grid for (θ1,ψ1, θ2,ψ2)
                const int G = 12;
                for (int i1 = 0; i1 < G; i1++)
                for (int j1 = 0; j1 < G; j1++)
                for (int i2 = 0; i2 < G; i2++)
                for (int j2 = 0; j2 < G; j2++)
                    Probe(new[]
                    {
                        (i1 + 0.5) * Math.PI / G,  j1 * 2.0 * Math.PI / G,
                        (i2 + 0.5) * Math.PI / G,  j2 * 2.0 * Math.PI / G
                    });
            }
            else
            {
                // Differential Evolution for N ≥ 4: population-based global search
                // avoids the local-minimum traps that random restarts miss.
                // Probe() registers the best DE individual so bestRes is set correctly.
                Probe(DifferentialEvolution(nParams, radii, nBends));
            }

            bestRes = localBest;
            return bestP;
        }

        // ── Differential Evolution (Task 4.2) ────────────────────────────────
        //
        // Population-based global search for N ≥ 4 bends where the 6+ dimensional
        // landscape has many local minima that Adam cannot escape from random starts.
        //
        // Strategy: DE/rand/1/bin with F=0.8, CR=0.9, 50 individuals, 200 generations.
        // Total evaluations: 50 × 200 = 10 000 — same order as the old random-restart
        // budget but far more effective at exploring the global landscape.

        private double[] DifferentialEvolution(int nParams, float[] radii, int nBends)
        {
            const int    popSize = 50;
            const int    maxGen  = 200;
            const double F       = 0.8;
            const double CR      = 0.9;

            // Compute a stable, combo-specific seed from nBends and the radii values so
            // that the same radius combination always explores the same DE population,
            // making the N ≥ 4 search fully deterministic across parallel threads and
            // repeated program runs (no shared counter needed).
            int deSeed = nBends * 1_000_003;
            foreach (float r in radii)
                deSeed = unchecked(deSeed * 31 + (int)BitConverter.DoubleToInt64Bits((double)r));
            var rng = new Random(deSeed);

            // Initialise population with random unit-sphere directions
            var    pop    = new double[popSize][];
            var    scores = new double[popSize];
            for (int i = 0; i < popSize; i++)
            {
                pop[i] = new double[nParams];
                for (int m = 0; m < nParams; m += 2)
                {
                    pop[i][m]     = rng.NextDouble() * (Math.PI - 0.1) + 0.05;
                    pop[i][m + 1] = rng.NextDouble() * 2.0 * Math.PI;
                }
                scores[i] = ComputeResidual(pop[i], radii, nBends, skipConditions: true);
            }

            for (int gen = 0; gen < maxGen; gen++)
            {
                // Enable condition penalties in the final quarter of generations so the
                // population converges toward clearance-satisfying regions before being
                // passed to Adam.  The first three-quarters search the pure geometry
                // landscape cheaply; the final quarter steers toward feasible space.
                // This is balanced against P0.4 in Adam (conditions only in r0) — DE
                // handles the coarse steering while Adam refines within feasible space.
                bool evalCond = _conditions.Count > 0 && gen >= maxGen * 3 / 4;

                for (int i = 0; i < popSize; i++)
                {
                    // Pick three distinct mutation bases r1, r2, r3 ≠ i
                    int r1, r2, r3;
                    do { r1 = rng.Next(popSize); } while (r1 == i);
                    do { r2 = rng.Next(popSize); } while (r2 == i || r2 == r1);
                    do { r3 = rng.Next(popSize); } while (r3 == i || r3 == r1 || r3 == r2);

                    int  jRand = rng.Next(nParams);
                    var  trial = new double[nParams];
                    for (int m = 0; m < nParams; m++)
                    {
                        trial[m] = (rng.NextDouble() < CR || m == jRand)
                            ? pop[r1][m] + F * (pop[r2][m] - pop[r3][m])
                            : pop[i][m];

                        // Clamp θ (even indices) away from the poles
                        if (m % 2 == 0)
                            trial[m] = MathHelper.Clamp(trial[m], 0.05, Math.PI - 0.05);
                    }

                    double ts = ComputeResidual(trial, radii, nBends, skipConditions: !evalCond);
                    if (ts < scores[i]) { pop[i] = trial; scores[i] = ts; }
                }

                // Re-score the full population at the transition point so scores are
                // comparable when conditions switch on.
                if (_conditions.Count > 0 && gen == maxGen * 3 / 4)
                    for (int i = 0; i < popSize; i++)
                        scores[i] = ComputeResidual(pop[i], radii, nBends, skipConditions: false);
            }

            // Final condition-aware re-score of the top-K individuals so Adam starts
            // from the best available condition-aware point in the population.
            if (_conditions.Count > 0)
            {
                const int topK = 5;
                var order = new int[popSize];
                for (int i = 0; i < popSize; i++) order[i] = i;
                Array.Sort(order, (a, b) => scores[a].CompareTo(scores[b]));
                for (int k = 0; k < Math.Min(topK, popSize); k++)
                    scores[order[k]] = ComputeResidual(pop[order[k]], radii, nBends,
                                                        skipConditions: false);
            }

            // Return the best individual from the final population
            int best = 0;
            for (int i = 1; i < popSize; i++)
                if (scores[i] < scores[best]) best = i;
            return pop[best];
        }
        //
        // For N=2: position is solved analytically (3×3 Cramer); length remains as residual.
        // For N≥3: position + length are solved analytically (4×4 or pseudoinverse);
        //          residual is only the non-negativity penalty on straight lengths.
        //
        // When ISolverConditions are registered, their Penalty() values are added as an
        // additional term so the optimizer actively steers away from violations.

        private double ComputeResidual(double[] parms, float[] radii, int nBends,
                                        bool skipConditions = false)
        {
            var   dirs    = BuildDirs(parms, nBends);
            float[]? Ls   = SolveStraightLengths(dirs, radii, out float arcTotal, out var geomCache);
            if (Ls == null) return 1e18;

            // Non-negativity penalty
            double penalty = 0;
            foreach (float L in Ls)
                if (L < 0) penalty += (double)L * L * 1e6;

            if (nBends == 2)
            {
                // Length must also be satisfied explicitly
                float T = _targetLength - arcTotal;
                if (T < 0) penalty += (double)T * T * 1e6;
                float sumL = 0; foreach (float L in Ls) sumL += L;
                double lenErr = sumL - T;
                penalty += lenErr * lenErr;
            }
            // N ≥ 3: length is baked into SolveStraightLengths — no length term needed.

            // Condition penalties (only paid when conditions are registered, to avoid
            // the overhead of BuildSegmentsRaw on every residual call otherwise)
            if (!skipConditions && _conditions.Count > 0)
            {
                var segs = BuildSegmentsRaw(dirs, radii, nBends, Ls, geomCache);
                if (segs != null)
                {
                    float   spacing = _diameter > 0f ? _diameter * 0.5f : 10f;
                    var     pts     = PathSampler.Sample(_startPoint, _startDir, segs, spacing);
                    foreach (var cond in _conditions)
                        penalty += cond.Penalty(pts, _diameter);
                }
            }

            return penalty;
        }

        // ── Per-bend geometry cache (P0.1) ───────────────────────────────────
        //
        // Holds the trig values computed once in SolveStraightLengths so that
        // BuildSegmentsRaw can reuse them without duplicating Acos/Sin/Dot work.
        // ArcDisp replaces the separate Vector3[N] array, keeping allocation count
        // the same as before while eliminating Redundancies R1 and R2.

        private readonly struct BendGeomCache
        {
            public readonly float   CosA, Alpha, SinA;
            public readonly Vector3 B;       // unit normal in the bend plane
            public readonly Vector3 ArcDisp; // CLR × arc displacement vector

            public BendGeomCache(float cosA, float alpha, float sinA,
                                  Vector3 b, Vector3 arcDisp)
            {
                CosA    = cosA;  Alpha  = alpha; SinA    = sinA;
                B       = b;     ArcDisp = arcDisp;
            }
        }

        // ── Linear system: solve for all straight lengths ─────────────────────
        //
        // Path: S →[L0·Ds]→ arc0 →[L1·D1]→ arc1 → ... →[LN·De]→ E
        //
        // Position constraint (3 equations):
        //   Σ_k L_k · dirs[k]  =  C   where C = (E-S) - Σ arcDisp_k
        //
        // Length constraint (1 equation, used for N≥3):
        //   Σ_k L_k  =  T_straight   where T_straight = targetLength - arcTotal
        //
        // For N=2 (M=3 unknowns): 3×3 Cramer on position only.
        // For N=3 (M=4 unknowns): 4×4 Gaussian (position + length).
        // For N≥4 (M≥5 unknowns): minimum-norm pseudoinverse (position + length).

        private float[]? SolveStraightLengths(
            Vector3[] dirs, float[] radii, out float arcTotal, out BendGeomCache[] cache)
        {
            arcTotal = 0f;
            int N = radii.Length;
            int M = N + 1;

            // Build arc geometry once — stored in the cache so BuildSegmentsRaw
            // never has to recompute the same Acos/Sin/Dot values (P0.1).
            cache = new BendGeomCache[N];
            for (int k = 0; k < N; k++)
            {
                float cosA = MathHelper.Clamp(Vector3.Dot(dirs[k], dirs[k + 1]), -1f, 1f);
                float alpha = (float)Math.Acos((double)cosA);
                if (alpha < 0.02f || alpha > MathHelper.PI - 0.02f)
                {
                    cache = Array.Empty<BendGeomCache>();
                    return null;
                }

                float   sinA    = (float)Math.Sin((double)alpha);
                Vector3 B       = (dirs[k + 1] - cosA * dirs[k]) / sinA;
                Vector3 arcDisp = radii[k] * ((1f - cosA) * B + sinA * dirs[k]);

                arcTotal  += radii[k] * alpha;
                cache[k]   = new BendGeomCache(cosA, alpha, sinA, B, arcDisp);
            }

            // RHS: C = (E - S) - Σ arcDisp
            Vector3 C = _endPoint - _startPoint;
            foreach (var entry in cache) C -= entry.ArcDisp;

            float T_straight = _targetLength - arcTotal;

            if (N == 2)
            {
                // 3×3 Cramer's rule (position constraints only)
                float det = Vector3.Dot(dirs[0], Vector3.Cross(dirs[1], dirs[2]));
                if (MathHelper.Abs(det) < 1e-5f) return null;

                float L0 = Vector3.Dot(C,       Vector3.Cross(dirs[1], dirs[2])) / det;
                float L1 = Vector3.Dot(dirs[0], Vector3.Cross(C,       dirs[2])) / det;
                float L2 = Vector3.Dot(dirs[0], Vector3.Cross(dirs[1], C))       / det;
                return new[] { L0, L1, L2 };
            }
            else if (N == 3)
            {
                // 4×4 Gaussian elimination [Ds|D1|D2|De] · L = [C; T_straight]
                var aug = new double[4, 5];
                for (int col = 0; col < 4; col++)
                {
                    aug[0, col] = dirs[col].X;
                    aug[1, col] = dirs[col].Y;
                    aug[2, col] = dirs[col].Z;
                    aug[3, col] = 1.0;
                }
                aug[0, 4] = C.X; aug[1, 4] = C.Y;
                aug[2, 4] = C.Z; aug[3, 4] = T_straight;

                return GaussElim4x4(aug);
            }
            else
            {
                // N ≥ 4: minimum-norm pseudoinverse: A(4×M)·L = b
                double[] b = { C.X, C.Y, C.Z, T_straight };
                var A = new double[4, M];
                for (int col = 0; col < M; col++)
                {
                    A[0, col] = dirs[col].X;
                    A[1, col] = dirs[col].Y;
                    A[2, col] = dirs[col].Z;
                    A[3, col] = 1.0;
                }
                double[]? Ld = MinNormSolve(A, b, 4, M);
                if (Ld == null) return null;

                var Lf = new float[M];
                for (int i = 0; i < M; i++) Lf[i] = (float)Ld[i];
                return Lf;
            }
        }

        // ── Gaussian elimination for 4×4 augmented matrix [4,5] ─────────────

        private static float[]? GaussElim4x4(double[,] aug)
        {
            const int n = 4;
            for (int col = 0; col < n; col++)
            {
                // Partial pivot
                int pivot = col;
                double maxV = Math.Abs(aug[col, col]);
                for (int row = col + 1; row < n; row++)
                    if (Math.Abs(aug[row, col]) > maxV)
                    {
                        maxV  = Math.Abs(aug[row, col]);
                        pivot = row;
                    }
                if (maxV < 1e-10) return null;

                if (pivot != col)
                    for (int c = 0; c <= n; c++)
                        (aug[col, c], aug[pivot, c]) = (aug[pivot, c], aug[col, c]);

                double pv = aug[col, col];
                for (int row = col + 1; row < n; row++)
                {
                    double f = aug[row, col] / pv;
                    for (int c = col; c <= n; c++) aug[row, c] -= f * aug[col, c];
                }
            }

            var x = new float[n];
            for (int row = n - 1; row >= 0; row--)
            {
                double s = aug[row, n];
                for (int col = row + 1; col < n; col++) s -= aug[row, col] * x[col];
                x[row] = (float)(s / aug[row, row]);
            }
            return x;
        }

        // ── Minimum-norm pseudoinverse: A(m×n)·x = b, n > m ──────────────────
        // Uses x = A^T (A A^T)^{-1} b

        private static double[]? MinNormSolve(double[,] A, double[] b, int m, int n)
        {
            // S = A * A^T  (m×m)
            var S = new double[m, m];
            for (int i = 0; i < m; i++)
            for (int j = 0; j < m; j++)
            {
                double s = 0;
                for (int k = 0; k < n; k++) s += A[i, k] * A[j, k];
                S[i, j] = s;
            }

            // Solve S · y = b via Gaussian elimination
            var aug = new double[m, m + 1];
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < m; j++) aug[i, j] = S[i, j];
                aug[i, m] = b[i];
            }

            for (int col = 0; col < m; col++)
            {
                int pivot = col; double maxV = Math.Abs(aug[col, col]);
                for (int row = col + 1; row < m; row++)
                    if (Math.Abs(aug[row, col]) > maxV)
                    {
                        maxV  = Math.Abs(aug[row, col]);
                        pivot = row;
                    }
                if (maxV < 1e-10) return null;
                if (pivot != col)
                    for (int c = 0; c <= m; c++)
                        (aug[col, c], aug[pivot, c]) = (aug[pivot, c], aug[col, c]);
                double pv = aug[col, col];
                for (int row = col + 1; row < m; row++)
                {
                    double f = aug[row, col] / pv;
                    for (int c = col; c <= m; c++) aug[row, c] -= f * aug[col, c];
                }
            }
            var y = new double[m];
            for (int row = m - 1; row >= 0; row--)
            {
                double s = aug[row, m];
                for (int col = row + 1; col < m; col++) s -= aug[row, col] * y[col];
                y[row] = s / aug[row, row];
            }

            // x = A^T · y
            var x = new double[n];
            for (int j = 0; j < n; j++)
            {
                double s = 0;
                for (int i = 0; i < m; i++) s += A[i, j] * y[i];
                x[j] = s;
            }
            return x;
        }

        // ── Adam optimizer over 2*(nBends-1) angular parameters ──────────────

        private double[] RefineDirections(double[] parms, float[] radii, int nBends)
        {
            const double lr      = 0.02;
            const double beta1   = 0.9;
            const double beta2   = 0.999;
            const double eps     = 1e-8;
            const double h       = 1e-3;
            const int    maxIter = 6_000;

            int      nP = parms.Length;
            double[] mM = new double[nP];
            double[] vM = new double[nP];
            double[] p  = (double[])parms.Clone();

            for (int t = 1; t <= maxIter; t++)
            {
                // P0.4: Condition penalties (PathSampler) are evaluated ONLY at the
                // base point r0, never in the perturbed gradient evaluations below.
                // On evalCond iterations r0 includes the condition penalty so Adam's
                // convergence check steers away from obstacle regions — but the per-
                // parameter perturbations use geometry-only residuals.  This cuts
                // PathSampler invocations from (1+2×nP) down to 1 per condition-
                // sample iteration, fixing redundancy R5.
                bool evalCond = _conditions.Count > 0 && (t % ConditionSampleInterval == 0);

                double r0 = ComputeResidual(p, radii, nBends, skipConditions: !evalCond);
                if (r0 < 0.01) break;

                // P1.2: Two-stage budget — after the fast probe stage, abort if still
                // far from convergence.  This abandons combos that won't converge 10×
                // faster than running the full 6 000-iteration budget.
                // Use geometry-only residual for the gate so that condition penalties
                // (which can be large even for geometrically-converging paths that are
                // temporarily inside an obstacle) do not cause false early aborts.
                // When conditions are active the fast-stage iter count and threshold are
                // doubled, since obstacle-avoidance steering slows geometry convergence.
                if (t == (_conditions.Count > 0 ? AdamFastStageIters * 2 : AdamFastStageIters))
                {
                    double geoR = evalCond
                        ? ComputeResidual(p, radii, nBends, skipConditions: true)
                        : r0;
                    double effThreshold = _conditions.Count > 0
                        ? AdamFastStageThreshold * 100.0   // 1e5 default when conditions active
                        : AdamFastStageThreshold;           // 1e3 default when no conditions
                    if (geoR > effThreshold)
                    {
                        if (Verbose)
                            Console.WriteLine($"    Adam fast-stage abort (iter={t}, geo residual={geoR:E3} > {effThreshold:E0}).");
                        break;
                    }
                }

                double mCorr = 1 - Math.Pow(beta1, t);
                double vCorr = 1 - Math.Pow(beta2, t);

                for (int i = 0; i < nP; i++)
                {
                    // Central finite-difference gradient.
                    // P0.4: Both perturbed evaluations always skip conditions — the
                    // condition penalty is not differentiated per-parameter.  This is
                    // valid because condition penalties are large flat values (1e6/point)
                    // whose gradient is near-zero except at obstacle boundaries.  The
                    // steering effect is provided by the r0 base value above.
                    p[i] += h;
                    double rf = ComputeResidual(p, radii, nBends, skipConditions: true);
                    p[i] -= 2 * h;
                    double rb = ComputeResidual(p, radii, nBends, skipConditions: true);
                    p[i] += h;

                    double g = (rf - rb) / (2 * h);

                    mM[i] = beta1 * mM[i] + (1 - beta1) * g;
                    vM[i] = beta2 * vM[i] + (1 - beta2) * g * g;

                    double step = lr * (mM[i] / mCorr) / (Math.Sqrt(vM[i] / vCorr) + eps);
                    p[i] -= step;

                    // Clamp theta (even indices) away from poles
                    if (i % 2 == 0)
                        p[i] = MathHelper.Clamp(p[i], 0.05, Math.PI - 0.05);
                }
            }

            return p;
        }

        // ── Solution builder ──────────────────────────────────────────────────

        private List<BendSegment>? BuildSolution(double[] parms, float[] radii, int nBends)
        {
            var     dirs = BuildDirs(parms, nBends);
            // P0.1: capture geomCache to pass into BuildSegmentsRaw (no trig recomputation).
            // P0.2: capture arcTotal here — eliminates the second SolveStraightLengths call
            //       that was previously needed for the N=2 length verification below.
            float[]? Ls  = SolveStraightLengths(dirs, radii, out float arcTotal, out var geomCache);
            if (Ls == null) return null;

            // All straight lengths must be non-negative (small tolerance for float drift)
            const float minStraight = -0.5f;
            foreach (float L in Ls) if (L < minStraight) return null;

            // Enforce MinStraightLength manufacturing tolerance
            if (MinStraightLength > 0f)
                foreach (float L in Ls) if (L < MinStraightLength) return null;

            // For N=2, the length constraint is not analytically guaranteed — verify it.
            // arcTotal was already computed above — no second SolveStraightLengths call needed.
            // Use the tolerance window when LengthToleranceFraction > 0 so that a path
            // that can't hit exactly _targetLength but is within the band is not rejected.
            if (nBends == 2)
            {
                float T       = _targetLength - arcTotal;
                float sumL    = 0; foreach (float L in Ls) sumL += L;
                float gate    = LengthToleranceFraction > 0f
                    ? MathHelper.Max(5f, _targetLength * LengthToleranceFraction)
                    : 5f;
                if (MathHelper.Abs(sumL - T) > gate) return null;
            }

            var segments = BuildSegmentsRaw(dirs, radii, nBends, Ls, geomCache);
            if (segments == null) return null;

            // Enforce MaxBendAngleDeg manufacturing tolerance
            if (MaxBendAngleDeg < 180f)
            {
                float maxRad = MaxBendAngleDeg * MathHelper.PI / 180f;
                foreach (var seg in segments)
                    if (seg.Angle > maxRad) return null;
            }

            // Simulator verification
            var   sim = PathSimulator.Simulate(_startPoint, _startDir, segments);
            float pe  = (sim.Position  - _endPoint).Length();
            float de  = (sim.Direction - _endDir).Length();
            float le  = MathHelper.Abs(sim.TotalLength - _targetLength);

            if (Verbose)
                Console.WriteLine($"    Sim verify: posE={pe:F3} mm  dirE={de:F5}  lenE={le:F3} mm");

            if (pe >= 1.0f || de >= 0.05f || le >= 1.0f) return null;

            // Condition gate: all registered conditions must be satisfied
            if (_conditions.Count > 0)
            {
                float   spacing = _diameter > 0f ? _diameter * 0.5f : 10f;
                var     pts     = PathSampler.Sample(_startPoint, _startDir, segments, spacing);

                foreach (var cond in _conditions)
                {
                    if (!cond.IsSatisfied(pts, _diameter))
                    {
                        if (Verbose)
                            Console.WriteLine($"    Condition {cond.Type} not satisfied — candidate rejected.");
                        return null;
                    }
                }
            }

            return segments;
        }

        // ── Shared segment builder (no simulator verification) ────────────────
        //
        // Constructs the BendSegment list from already-computed directions and lengths.
        // Used by both BuildSolution (which then adds simulator verification) and
        // ComputeResidual (which needs segments for condition penalty evaluation).

        private static List<BendSegment>? BuildSegmentsRaw(
            Vector3[] dirs, float[] radii, int nBends, float[] Ls,
            BendGeomCache[]? cache = null)
        {
            var segments = new List<BendSegment>(nBends + 1);

            for (int k = 0; k < nBends; k++)
            {
                float   cosA, alpha, sinA;
                Vector3 B;

                // P0.1: reuse cached geometry to avoid duplicating Acos/Sin/Dot (R1, R2)
                if (cache != null && k < cache.Length)
                {
                    cosA  = cache[k].CosA;
                    alpha = cache[k].Alpha;
                    sinA  = cache[k].SinA;
                    B     = cache[k].B;
                }
                else
                {
                    cosA  = MathHelper.Clamp(Vector3.Dot(dirs[k], dirs[k + 1]), -1f, 1f);
                    alpha = (float)Math.Acos((double)cosA);
                    sinA  = (float)Math.Sin((double)alpha);
                    B     = (dirs[k + 1] - cosA * dirs[k]) / sinA;
                }

                if (sinA < 1e-6f) return null;

                PathSimulator.BuildFrame(dirs[k], out var fb0, out var fb1);
                float phi = (float)Math.Atan2(Vector3.Dot(B, fb1), Vector3.Dot(B, fb0));

                segments.Add(new BendSegment
                {
                    CLR            = radii[k],
                    StraightLength = MathHelper.Max(0f, Ls[k]),
                    Angle          = alpha,
                    Rotation       = phi,
                });
            }

            segments.Add(new BendSegment
            {
                CLR            = 0f,
                StraightLength = MathHelper.Max(0f, Ls[nBends]),
                Angle          = 0f,
                Rotation       = 0f,
            });

            return segments;
        }

        // ── Helpers ───────────────────────────────────────────────────────────

        /// <summary>
        /// Build the full direction array [Ds, D1, ..., D_{N-1}, De] from the 2*(N-1)
        /// angular parameters [θ1,ψ1, θ2,ψ2, ..., θ_{N-1},ψ_{N-1}].
        /// </summary>
        private Vector3[] BuildDirs(double[] parms, int nBends)
        {
            var dirs = new Vector3[nBends + 1];
            dirs[0]      = _startDir;
            dirs[nBends] = _endDir;
            for (int k = 0; k < nBends - 1; k++)
                dirs[k + 1] = DirFromAngles(parms[2 * k], parms[2 * k + 1]);
            return dirs;
        }

        private static Vector3 DirFromAngles(double theta, double psi) => new(
            (float)(Math.Sin(theta) * Math.Cos(psi)),
            (float)(Math.Sin(theta) * Math.Sin(psi)),
            (float) Math.Cos(theta));
    }
}
