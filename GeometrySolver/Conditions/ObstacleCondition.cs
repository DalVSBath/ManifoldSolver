using System;
using System.Collections.Generic;
using System.Numerics;

namespace GeometrySolver.Conditions
{
    /// <summary>
    /// Adapts any <see cref="IIgnoreArea"/> obstacle into the <see cref="ISolverCondition"/>
    /// contract, making existing sphere/box/cylinder geometry composable with the
    /// condition-evaluation pipeline.
    ///
    /// <b>IsSatisfied</b>: returns <c>true</c> when no sampled centreline point,
    /// expanded by <c>pipeDiameter / 2</c> along the worst-case outward direction,
    /// falls inside the obstacle. Implemented conservatively: the centreline point
    /// itself is tested; the caller is expected to pass sample points at
    /// <c>pipeDiameter / 2</c> spacing so the pipe tube is fully covered.
    ///
    /// <b>Penalty</b>: returns a large flat penalty per sample point that lies
    /// inside the obstacle. This provides a coarse but reliable gradient signal that
    /// steers the Adam optimizer away from obstacle-penetrating configurations.
    /// A future refinement can replace this with a continuous penetration-depth
    /// measure once the exact geometry of each obstacle type is exposed.
    /// </summary>
    public class ObstacleCondition : ISolverCondition
    {
        private readonly IIgnoreArea _area;

        /// <summary>
        /// Creates a new <see cref="ObstacleCondition"/> wrapping the given obstacle.
        /// </summary>
        public ObstacleCondition(IIgnoreArea area)
        {
            _area = area ?? throw new ArgumentNullException(nameof(area));
        }

        /// <summary>
        /// Regions where obstacle penalties are cancelled. A path point that falls
        /// inside both an obstacle and any area in this list is treated as exempt —
        /// no penalty is applied and <see cref="IsSatisfied"/> does not reject it.
        /// Used to carve out the pipe-face zone where the pipe stub is expected to
        /// pass through the manifold body.
        /// </summary>
        public List<IIgnoreArea> AntiObstacles { get; } = new();

        private bool IsAntiExempted(Vector3 point)
        {
            foreach (var anti in AntiObstacles)
                if (anti.InArea(point)) return true;
            return false;
        }

        /// <summary>
        /// Sampled centreline points within this distance (mm) of either the pipe
        /// start or end position are excluded from the obstacle test.
        ///
        /// Use this when the pipe physically enters the obstacle at its connection
        /// port (e.g. a manifold body): the connection stub is inside the obstacle
        /// by design, so those points must be exempt.  Set to at least
        /// <c>pipeDiameter</c> to skip the last sample before each endpoint.
        /// Default is <c>0</c> (all points checked).
        /// </summary>
        public float ExcludeEndMm { get; set; } = 0f;

        /// <summary>The underlying obstacle shape.</summary>
        public IIgnoreArea Area => _area;

        /// <inheritdoc />
        public ConditionType Type => _area.AreaType switch
        {
            AreaType.Sphere          => ConditionType.ObstacleSphere,
            AreaType.Cylinder        => ConditionType.ObstacleCylinder,
            AreaType.Cube            => ConditionType.ObstacleBox,
            AreaType.TaperedCylinder => ConditionType.ObstacleTaperedCylinder,
            _                        => ConditionType.Custom,
        };

        /// <inheritdoc />
        /// <remarks>
        /// Each sampled centreline point is tested. The pipe diameter is used as the
        /// sample spacing by convention (callers use <c>diameter / 2</c>) so the
        /// full pipe tube cross-section is represented without testing each point
        /// individually in every radial direction.
        /// </remarks>
        public bool IsSatisfied(IReadOnlyList<Vector3> pathPoints, float pipeDiameter)
        {
            if (pathPoints.Count == 0) return true;
            float excSq = ExcludeEndMm * ExcludeEndMm;
            Vector3 startPt = pathPoints[0];
            Vector3 endPt   = pathPoints[pathPoints.Count - 1];

            foreach (var point in pathPoints)
            {
                if (excSq > 0f)
                {
                    if (Vector3.DistanceSquared(point, startPt) <= excSq) continue;
                    if (Vector3.DistanceSquared(point, endPt)   <= excSq) continue;
                }
                if (AntiObstacles.Count > 0 && IsAntiExempted(point)) continue;
                if (_area.DistanceTo(point) < pipeDiameter * 0.5f) return false;
            }
            return true;
        }

        /// <inheritdoc />
        /// <remarks>
        /// Returns <c>1 × 10⁶</c> per penetrating sample point. This gives the
        /// optimizer a strong penalty gradient without requiring per-obstacle
        /// penetration-depth computation in Phase 2. Phase 4 can refine this to a
        /// continuous depth-based function.
        /// </remarks>
        public double Penalty(IReadOnlyList<Vector3> pathPoints, float pipeDiameter)
        {
            if (pathPoints.Count == 0) return 0;

            // P2.1: Bounding-box pre-filter.
            // Compute the path AABB and compare against the obstacle's conservative AABB.
            // If the two boxes do not overlap, no path point can be inside the obstacle —
            // return 0 without iterating any points.  This is a constant-time rejection
            // that eliminates all per-point work for paths far from the obstacle.
            if (!PathOverlapsObstacleAABB(pathPoints, pipeDiameter)) return 0;

            float excSq = ExcludeEndMm * ExcludeEndMm;
            Vector3 startPt = pathPoints[0];
            Vector3 endPt   = pathPoints[pathPoints.Count - 1];

            double penalty = 0;
            foreach (var point in pathPoints)
            {
                if (excSq > 0f)
                {
                    if (Vector3.DistanceSquared(point, startPt) <= excSq) continue;
                    if (Vector3.DistanceSquared(point, endPt)   <= excSq) continue;
                }
                if (AntiObstacles.Count > 0 && IsAntiExempted(point)) continue;
                if (_area.DistanceTo(point) < pipeDiameter * 0.5f) penalty += 1e8;
            }
            return penalty;
        }

        // ── P2.1: AABB pre-filter ─────────────────────────────────────────────

        private bool PathOverlapsObstacleAABB(IReadOnlyList<Vector3> pathPoints, float pipeDiameter)
        {
            // Build the path AABB
            float pMinX = float.MaxValue, pMinY = float.MaxValue, pMinZ = float.MaxValue;
            float pMaxX = float.MinValue, pMaxY = float.MinValue, pMaxZ = float.MinValue;
            foreach (var pt in pathPoints)
            {
                if (pt.X < pMinX) pMinX = pt.X; if (pt.X > pMaxX) pMaxX = pt.X;
                if (pt.Y < pMinY) pMinY = pt.Y; if (pt.Y > pMaxY) pMaxY = pt.Y;
                if (pt.Z < pMinZ) pMinZ = pt.Z; if (pt.Z > pMaxZ) pMaxZ = pt.Z;
            }

            // Compute a conservative obstacle AABB from the IIgnoreArea type,
            // then expand it by the pipe radius so paths that approach within
            // pipeDiameter/2 of the obstacle surface are not prematurely rejected.
            float oMinX, oMinY, oMinZ, oMaxX, oMaxY, oMaxZ;
            ComputeObstacleAABB(out oMinX, out oMinY, out oMinZ,
                                 out oMaxX, out oMaxY, out oMaxZ);
            float r = pipeDiameter * 0.5f;

            // Separating-axis test: if separated on any axis, no overlap
            return !(pMaxX < oMinX - r || pMinX > oMaxX + r ||
                     pMaxY < oMinY - r || pMinY > oMaxY + r ||
                     pMaxZ < oMinZ - r || pMinZ > oMaxZ + r);
        }

        private void ComputeObstacleAABB(
            out float minX, out float minY, out float minZ,
            out float maxX, out float maxY, out float maxZ)
        {
            // Use reflection on the known IIgnoreArea implementations.
            // A conservative (over-approximated) AABB is sufficient for the
            // pre-filter — false positives are harmless, false negatives are not.
            switch (_area)
            {
                case IgnoreSphere s:
                {
                    // Sphere: centre ± radius in each axis
                    var c = s.Centre;
                    float r = s.Radius;
                    minX = c.X - r; maxX = c.X + r;
                    minY = c.Y - r; maxY = c.Y + r;
                    minZ = c.Z - r; maxZ = c.Z + r;
                    return;
                }
                case IgnoreRect b:
                {
                    minX = b.Min.X; maxX = b.Max.X;
                    minY = b.Min.Y; maxY = b.Max.Y;
                    minZ = b.Min.Z; maxZ = b.Max.Z;
                    return;
                }
                case IgnoreCylinder cyl:
                {
                    // Conservative AABB: centre ± (radius + halfHeight) in every axis
                    var  c = cyl.Centre;
                    float r = cyl.Radius, hh = cyl.Height * 0.5f;
                    float ext = r + hh;
                    minX = c.X - ext; maxX = c.X + ext;
                    minY = c.Y - ext; maxY = c.Y + ext;
                    minZ = c.Z - ext; maxZ = c.Z + ext;
                    return;
                }
                case IgnoreTaperedCylinder f:
                {
                    // Conservative AABB: use max radius across both faces
                    float r = Math.Max(f.RadiusBase, f.RadiusTop);
                    float hh = f.Height * 0.5f;
                    float ext = r + hh;
                    var c = f.Centre;
                    minX = c.X - ext; maxX = c.X + ext;
                    minY = c.Y - ext; maxY = c.Y + ext;
                    minZ = c.Z - ext; maxZ = c.Z + ext;
                    return;
                }
                default:
                    // Unknown type: skip the pre-filter (allow Penalty to run)
                    minX = minY = minZ = float.MinValue;
                    maxX = maxY = maxZ = float.MaxValue;
                    return;
            }
        }
    }
}
