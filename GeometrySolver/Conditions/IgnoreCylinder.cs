using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GeometrySolver.Conditions
{
    public class IgnoreCylinder : IIgnoreArea
    {
        private Vector3 _centre, _axis;
        private float _radius, _height;

        public IgnoreCylinder(Vector3 center, Vector3 axis, float radius, float height)
        {
            _centre  = center; _axis = Vector3.Normalize(axis); _radius = radius; _height = height;
        }

        /// <summary>Centre point of the cylinder.</summary>
        public Vector3 Centre => _centre;
        /// <summary>Normalised axis direction of the cylinder.</summary>
        public Vector3 Axis   => _axis;
        /// <summary>Radius of the cylinder (mm).</summary>
        public float   Radius => _radius;
        /// <summary>Full height of the cylinder (mm).</summary>
        public float   Height => _height;

        AreaType IIgnoreArea.AreaType => AreaType.Cylinder;

        public bool InArea(Vector3 point)
        {
            Vector3 dir = _axis;
            Vector3 delta = point - _centre;

            float halfHeight = _height * 0.5f;

            // Project point onto cylinder axis
            float projection = Vector3.Dot(delta, dir);

            // Check height bounds
            if (projection < -halfHeight || projection > halfHeight)
                return false;

            // Compute perpendicular distance to axis
            Vector3 closestPointOnAxis = _centre + dir * projection;
            float sqrDistanceFromAxis = Vector3.DistanceSquared(point, closestPointOnAxis);

            return sqrDistanceFromAxis <= _radius * _radius;
        }

        public float DistanceTo(Vector3 point)
        {
            var delta      = point - _centre;
            float axial    = Vector3.Dot(delta, _axis);
            float perpDist = (float)Math.Sqrt(Math.Max(0f, delta.LengthSquared() - axial * axial));
            float radialEx = Math.Max(0f, perpDist - _radius);
            float axialEx  = Math.Max(0f, Math.Abs(axial) - _height * 0.5f);
            return (float)Math.Sqrt(radialEx * radialEx + axialEx * axialEx);
        }
    }
}
