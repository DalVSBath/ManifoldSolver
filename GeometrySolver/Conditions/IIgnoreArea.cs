using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GeometrySolver.Conditions
{
    public interface IIgnoreArea
    {
        public AreaType AreaType { get; }
        public bool InArea(Vector3 point);
        public float DistanceTo(Vector3 point);
    }
    
    public enum AreaType
    {
        Sphere,
        Cylinder,
        Cube,
        TaperedCylinder,
    }
}
