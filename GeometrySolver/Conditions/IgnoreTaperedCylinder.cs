using System;
using System.Numerics;

namespace GeometrySolver.Conditions
{
    /// <summary>
    /// A frustum (truncated cone) obstacle. One end has <see cref="RadiusBase"/>, the other
    /// <see cref="RadiusTop"/>. The radius varies linearly along the axis between the two faces.
    /// </summary>
    public class IgnoreTaperedCylinder : IIgnoreArea
    {
        private readonly Vector3 _centre, _axis;
        private readonly float _radiusBase, _radiusTop, _height;

        /// <param name="centre">Midpoint of the frustum (halfway between the two faces).</param>
        /// <param name="axis">Normalized direction from base face toward top face.</param>
        /// <param name="radiusBase">Radius at the base face (axis × -Height/2 from centre).</param>
        /// <param name="radiusTop">Radius at the top face (axis × +Height/2 from centre).</param>
        /// <param name="height">Total height of the frustum (mm).</param>
        public IgnoreTaperedCylinder(Vector3 centre, Vector3 axis,
            float radiusBase, float radiusTop, float height)
        {
            _centre     = centre;
            _axis       = Vector3.Normalize(axis);
            _radiusBase = radiusBase;
            _radiusTop  = radiusTop;
            _height     = height;
        }

        public Vector3 Centre     => _centre;
        public Vector3 Axis       => _axis;
        public float   RadiusBase => _radiusBase;
        public float   RadiusTop  => _radiusTop;
        public float   Height     => _height;

        AreaType IIgnoreArea.AreaType => AreaType.TaperedCylinder;

        public bool InArea(Vector3 point)
        {
            var delta = point - _centre;
            float t   = Vector3.Dot(delta, _axis);
            float hh  = _height * 0.5f;
            if (t < -hh || t > hh) return false;

            float r      = (float)Math.Sqrt(Math.Max(0f, delta.LengthSquared() - t * t));
            float localR = _radiusBase + (_radiusTop - _radiusBase) * (t + hh) / _height;
            return r <= localR;
        }

        public float DistanceTo(Vector3 point)
        {
            var delta = point - _centre;
            float t  = Vector3.Dot(delta, _axis);
            float r  = (float)Math.Sqrt(Math.Max(0f, delta.LengthSquared() - t * t));
            float hh = _height * 0.5f;

            if (t >= -hh && t <= hh)
            {
                float localR = _radiusBase + (_radiusTop - _radiusBase) * (t + hh) / _height;
                if (r <= localR) return 0f;
            }

            // Reduce to 2D (t, r) space using axial symmetry.
            // The frustum's closed surface consists of three regions:
            //   slant  : line segment from (-hh, radiusBase) to (+hh, radiusTop)
            //   base cap: disk at t = -hh with radius radiusBase
            //   top cap : disk at t = +hh with radius radiusTop

            float dSlant = SegDist2D(t, r, -hh, _radiusBase, hh, _radiusTop);

            float axExBase  = Math.Max(0f, -(t + hh));
            float radExBase = Math.Max(0f, r - _radiusBase);
            float dBase     = (float)Math.Sqrt(axExBase * axExBase + radExBase * radExBase);

            float axExTop  = Math.Max(0f, t - hh);
            float radExTop = Math.Max(0f, r - _radiusTop);
            float dTop     = (float)Math.Sqrt(axExTop * axExTop + radExTop * radExTop);

            return Math.Min(dSlant, Math.Min(dBase, dTop));
        }

        private static float SegDist2D(float px, float py, float ax, float ay, float bx, float by)
        {
            float dx = bx - ax, dy = by - ay;
            float lenSq = dx * dx + dy * dy;
            if (lenSq < 1e-12f)
                return (float)Math.Sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
            float s = Math.Max(0f, Math.Min(1f, ((px - ax) * dx + (py - ay) * dy) / lenSq));
            float cx = ax + s * dx, cy = ay + s * dy;
            return (float)Math.Sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
        }
    }
}
