using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace GeometrySolver.Solver
{
    public class MathHelper
    {
        public const float PI = 3.14159265359f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float value, float min, float max)
        {
            if (value < min)
            {
                return min;
            }
            else if (value > max)
            {
                return max;
            }

            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Clamp(int value, int min, int max)
        {
            if (value < min)
            {
                return min;
            }
            else if (value > max)
            {
                return max;
            }

            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Clamp(double value, double min, double max)
        {
            if (value < min)
            {
                return min;
            }
            else if (value > max)
            {
                return max;
            }

            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Abs(int value)
        {
            if (value < 0)
            {
                value = -value;
            }
            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Abs(float value)
        {
            if (value < 0)
            {
                value = -value;
            }
            return value;
        }


        public static int Max(int val1, int val2)
        {
            return (val1 >= val2) ? val1 : val2;
        }
        public static float Max(float val1, float val2)
        {
            return (val1 >= val2) ? val1 : val2;
        }

    }
}
