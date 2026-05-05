using GeometrySolver.Conditions;
using GeometrySolver.Solver;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;

// ── Shared settings ───────────────────────────────────────────────────────────

float Diameter = 40, BendDiameter = 41.3f;
float[] BendRadii = [BendDiameter * 1.5f, BendDiameter * 2, 102f, 127f];
float Length = 475;

// SolverConfig captures every setting in a single serialisable record.
// Solver.FromConfig() applies all settings and sets up geometry in one call.
var DefaultConfig = SolverConfig.Default(BendRadii, Length, Diameter);

// ── Locate TestPipes-V1.csv ───────────────────────────────────────────────────

string? csvPath = null;
string searchDir = AppContext.BaseDirectory;
for (int up = 0; up < 6; up++)
{
    string candidate = Path.Combine(searchDir, "TestPipes-V1.csv");
    if (File.Exists(candidate)) { csvPath = candidate; break; }
    string? parent = Path.GetDirectoryName(searchDir);
    if (parent == null) break;
    searchDir = parent;
}

if (csvPath == null)
{
    Console.WriteLine("ERROR: TestPipes-V1.csv not found.");
    return;
}

// ── Parse CSV rows ────────────────────────────────────────────────────────────

static float F(string s) => float.Parse(s.Trim(), CultureInfo.InvariantCulture);

var rows = new List<PipeRow>();
foreach (var line in File.ReadLines(csvPath).Skip(1))
{
    if (string.IsNullOrWhiteSpace(line)) continue;
    var c = line.Split(',');
    if (c.Length < 13) continue;
    rows.Add(new PipeRow(
        c[0].Trim(),
        new Vector3(F(c[1]), F(c[2]), F(c[3])),
        Vector3.Normalize(new Vector3(F(c[4]), F(c[5]), F(c[6]))),
        new Vector3(F(c[7]), F(c[8]), F(c[9])),
        Vector3.Normalize(new Vector3(F(c[10]), F(c[11]), F(c[12])))));
}

// ═════════════════════════════════════════════════════════════════════════════
// Section 1 — Individual pipe solve (all 6 rows)
// ═════════════════════════════════════════════════════════════════════════════

bool SKIP_S1 = true;

Console.WriteLine("╔══════════════════════════════════════════════╗");
Console.WriteLine("║  SECTION 1 — Individual single-pipe solve    ║");
Console.WriteLine($"╚══════════════════════════════════════════════╝");
Console.WriteLine($"  Reading: {csvPath}");
Console.WriteLine();

int solved1 = 0, failed1 = 0;

if (!SKIP_S1)
{
    foreach (var row in rows)
    {
        Console.WriteLine($"════════════════════════════════════════════════");
        Console.WriteLine($"  Pipe {row.Id}");
        Console.WriteLine($"    Start  : {row.SPoint}   Dir: {row.SDir:F4}");
        Console.WriteLine($"    End    : {row.EPoint}   Dir: {row.EDir:F4}");
        Console.WriteLine();

        var solver = Solver.FromConfig(DefaultConfig,
                         row.SPoint, row.SDir, row.EPoint, row.EDir);

        var result = solver.Solve();
        Console.WriteLine();

        if (result is null)
        {
            Console.WriteLine($"  *** PIPE {row.Id}: NO SOLUTION FOUND ***");
            failed1++;
        }
        else
        {
            Console.WriteLine($"  PIPE {row.Id} SOLVED — {result.BendCount} bends, {result.Segments.Count} segments:");
            PrintSegments(result.Segments);
            Console.WriteLine();
            Console.WriteLine($"    Total path length : {result.TotalLength,8:F2} mm  (target: {Length:F2})");
            Console.WriteLine($"    Position error    : {result.PositionError,8:F4} mm");
            Console.WriteLine($"    Direction error   : {result.DirectionError,8:F6}");
            solved1++;
        }
        Console.WriteLine();
    }

    Console.WriteLine($"════════════════════════════════════════════════");
    Console.WriteLine($"  Section 1 results: {solved1} solved, {failed1} failed  (of {rows.Count} pipes)");
}

// ═════════════════════════════════════════════════════════════════════════════
// Section 2 — Manifold solve (first 2 rows at equal length with clearance)
// ═════════════════════════════════════════════════════════════════════════════

Console.WriteLine();
Console.WriteLine("╔══════════════════════════════════════════════╗");
Console.WriteLine("║  SECTION 2 — Manifold equal-length solve     ║");
Console.WriteLine("╚══════════════════════════════════════════════╝");

// Demo with the first two CSV pipes — shows inter-pipe clearance enforcement.
// (Using only 2 pipes keeps the demo runtime reasonable.)
var demoPipes = rows.Take(6).ToList();

Console.WriteLine($"  Pipes    : {demoPipes.Count} (pipes 1 and 2 from CSV) — inner-solver output suppressed");
Console.WriteLine($"  Target   : {Length} mm (explicit)");
Console.WriteLine($"  Diameter : {Diameter} mm");
Console.WriteLine($"  Note     : end-points are ~41.6 mm apart (collector header) — MinClearance=3 mm");
Console.WriteLine();

// End-point geometry analysis:
//   Pipe 1 end: (-162.02, -119.91, 150.54)
//   Pipe 2 end: (-126.02, -114.53, 130.47)
//   Centre-to-centre at collector: ~41.6 mm (just above the 41.3 mm pipe OD).
//   Requiring MinClearance = Diameter (82.6 mm C-C) is physically impossible there.
//   MinClearance = 3 mm means 3 mm surface-to-surface gap everywhere except at the
//   collector header where geometry forces near-contact.
var manifold = new ManifoldGeoSolver
{
    BendRadii = BendRadii,
    TargetLength = Length,
    Diameter = Diameter,
    MaxBends = 7,
    Verbose = false,
    EqualizeLength = false,
    MinClearance = 0f,
    ClearanceExcludeEndMm = 0f,
    LengthToleranceFraction = 0.03f,  // ±3% = 582–618 mm
    MaxBacktrackCandidates = 5,       // 5 candidates per pipe for backtracking
};

foreach (var row in demoPipes)
    manifold.AddPipe(row.SPoint, row.SDir, row.EPoint, row.EDir);

// Manifold body obstacle — cylinder representing the collector header.
// The pipe axis points along (0, 2.9, 0.77); the body centre is offset 1 unit
// along that axis from the collector face.
// ExcludeEndMm: the pipe endpoints are connection ports that physically enter
// the manifold body, so sampled points within 2×Diameter of each endpoint are
// exempt from the obstacle check.  Only the routed (bent) portion must clear.
Vector3 obstacleAxis = new Vector3(0f, 2.9f, 0.77f);
Vector3 obstacleCentre = new Vector3(-162.02f, -109.15f, 110.39f)
                         + Vector3.Normalize(obstacleAxis);
manifold.AddSharedCondition(new ObstacleCondition(new IgnoreCylinder(
    center: obstacleCentre,
    axis: obstacleAxis,
    radius: 77.5f,
    height: 105f))
{
    ExcludeEndMm = 2f * Diameter,   // ≈82 mm — covers the connection-port stub
});

var manifoldResults = manifold.Solve();

Console.WriteLine();
Console.WriteLine("── Manifold summary ────────────────────────────");

int solved2 = 0, failed2 = 0;
for (int i = 0; i < manifoldResults.Count; i++)
{
    var r = manifoldResults[i];
    if (r is null)
    {
        Console.WriteLine($"  Pipe {demoPipes[i].Id}  : *** NO SOLUTION ***");
        failed2++;
    }
    else
    {
        Console.WriteLine($"  Pipe {demoPipes[i].Id}  : {r.BendCount} bends | " +
                          $"Length={r.TotalLength:F2} mm | " +
                          $"PosErr={r.PositionError:F4} mm | " +
                          $"DirErr={r.DirectionError:F6}");
        PrintSegments(r.Segments);
        solved2++;
    }

}

Console.WriteLine();
Console.WriteLine($"  Manifold results: {solved2} solved, {failed2} failed  (of {demoPipes.Count} pipes)");

Console.ReadLine();

// ── Helper ────────────────────────────────────────────────────────────────────

static void PrintSegments(List<BendSegment> segs)
{
    for (int i = 0; i < segs.Count; i++)
    {
        var seg = segs[i];
        Console.WriteLine($"    Segment {i + 1}:");
        Console.WriteLine($"      Straight : {seg.StraightLength,8:F2} mm");
        if (seg.Angle > 1e-4f)
        {
            Console.WriteLine($"      CLR      : {seg.CLR,8:F2} mm");
            Console.WriteLine($"      Angle    : {seg.Angle * 180f / MathHelper.PI,8:F2} deg");
            Console.WriteLine($"      Rotation : {seg.Rotation * 180f / MathHelper.PI,8:F2} deg");
            Console.WriteLine($"      Arc      : {seg.CLR * seg.Angle,8:F2} mm");
        }
    }
}

// ── Type declarations (must follow all top-level statements) ─────────────────

record PipeRow(string Id, Vector3 SPoint, Vector3 SDir, Vector3 EPoint, Vector3 EDir);


namespace System.Runtime.CompilerServices
{
    internal static class IsExternalInit { }
}