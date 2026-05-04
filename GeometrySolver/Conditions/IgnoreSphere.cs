using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GeometrySolver.Conditions
{
    public class IgnoreSphere : IIgnoreArea
    {
        private Vector3 _centre;
        private float _radis;

        public IgnoreSphere(Vector3 centre, float radius)
        {
            _centre = centre;
            _radis = radius;
        }

        /// <summary>Centre of the sphere.</summary>
        public Vector3 Centre => _centre;
        /// <summary>Radius of the sphere (mm).</summary>
        public float   Radius => _radis;

        AreaType IIgnoreArea.AreaType => AreaType.Sphere;

        public bool InArea(Vector3 point)
        {
            float sqrDistance = Vector3.DistanceSquared(point, _centre);
            float sqrRadius = _radis * _radis;
            return sqrDistance <= sqrRadius;
        }
    }
}
