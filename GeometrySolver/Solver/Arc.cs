using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GeometrySolver.Solver
{
    public class Biarc
    {
        public struct Arc
        {
            public Vector3 Center;
            public float Radius;
            public Vector3 Start, End;
            public float Length;
        }

        public static (Arc arc1, Arc arc2) Calculate(Vector3 p1, Vector3 t1, Vector3 p2, Vector3 t2)
        {
            t1 = Vector3.Normalize(t1);
            t2 = Vector3.Normalize(t2);

            Vector3 v = p2 - p1;
            float denominator = 2f * Vector3.Dot(v, t1 + t2);

            // Handle parallel case or division by zero
            if (Math.Abs(denominator) < 1e-6f) return default;

            float d = v.LengthSquared() / denominator;

            // Join point J
            Vector3 control1 = p1 + d * t1;
            Vector3 control2 = p2 - d * t2;
            Vector3 joinPoint = (control1 + control2) * 0.5f;

            // Tangent at join point
            Vector3 tm = Vector3.Normalize(joinPoint - control1);

            return (ComputeArc(p1, t1, joinPoint, tm), ComputeArc(joinPoint, tm, p2, t2));
        }

        private static Arc ComputeArc(Vector3 pStart, Vector3 tStart, Vector3 pEnd, Vector3 tEnd)
        {
            Vector3 chord = pEnd - pStart;
            // Normal to the plane of the arc
            Vector3 normal = Vector3.Cross(chord, tStart);
            // Vector pointing from start towards center
            Vector3 side = Vector3.Cross(normal, tStart);

            float radius = chord.LengthSquared() / (2f * Vector3.Dot(chord, side));
            Vector3 center = pStart + Vector3.Normalize(side) * radius;

            float angle = (float)Math.Acos((double)Vector3.Dot(Vector3.Normalize(pStart - center), Vector3.Normalize(pEnd - center)));

            return new Arc
            {
                Center = center,
                Radius = Math.Abs(radius),
                Start = pStart,
                End = pEnd,
                Length = Math.Abs(radius) * angle
            };
        }
    }
}
