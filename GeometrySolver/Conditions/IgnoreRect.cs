using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GeometrySolver.Conditions
{
    public class IgnoreRect : IIgnoreArea
    {
        private Vector3 _min, _max;
        public IgnoreRect(Vector3 Min, Vector3 Max) 
        {
            _max = Max;
            _min = Min;
        }

        /// <summary>Minimum corner of the axis-aligned box.</summary>
        public Vector3 Min => _min;
        /// <summary>Maximum corner of the axis-aligned box.</summary>
        public Vector3 Max => _max;

        AreaType IIgnoreArea.AreaType => AreaType.Cube;

        public bool InArea(Vector3 point)
        {
            return point.X >= _min.X && point.X <= _max.X &&
           point.Y >= _min.Y && point.Y <= _max.Y &&
           point.Z >= _min.Z && point.Z <= _max.Z;
        }
    }
}
