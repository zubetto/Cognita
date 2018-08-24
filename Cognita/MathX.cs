using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Cognita
{
    public static class MathX
    {
        public struct Hyperplane
        {
            public double[] Point;
            public double[] Normal;
            public double Distance;
        }

        public struct Hyperrect
        {
            public double[] PointA;
            public double[] PointB;
        }

        public static readonly double sqrt2 = Math.Sqrt(2);
        
        public static readonly Random Rnd = new Random();

        /// <summary>
        /// Sets random hyperplane which has points in the given region
        /// </summary>
        /// <param name="region"></param>
        /// <param name="hplane"></param>
        public static void SetRandPlane(Hyperrect region, ref Hyperplane hplane)
        {
            hplane.Point.SetRandom(region);
            hplane.Normal.SetRandom(-1.0, 1.0);
            hplane.Normal.Multiply(1.0 / Math.Sqrt(hplane.Normal.SquaredNorm()));
            hplane.Distance = hplane.Point.DotProduct(hplane.Normal);
        }

        public static double LogisticFunction(double x)
        {
            return 1.0 / (1.0 + Math.Exp(x));
        }

        public static double DotProduct(this double[] r, double[] q)
        {
            double rq = 0.0;

            for (int i = 0; i < r.Length; i++)
            {
                rq += r[i] * q[i];
            }

            return rq;
        }

        public static double DotProduct(this int[] r, double[] q)
        {
            double rq = 0.0;

            for (int i = 0; i < r.Length; i++)
            {
                rq += r[i] * q[i];
            }

            return rq;
        }

        public static int[][] OuterProduct(this int[] r, int[] q)
        {
            int[][] prod = new int[r.Length][];

            for (int i = 0; i < r.Length; ++i)
            {
                int[] row = new int[q.Length];
                prod[i] = row;

                for (int j = 0, ri = r[i]; j < q.Length; ++j)
                    row[j] = ri * q[j];
            }

            return prod;
        }

        public static void OuterProduct(this int[] r, int[] q, int[][] result)
        {
            for (int i = 0; i < r.Length; ++i)
            {
                int[] row = result[i];

                for (int j = 0, ri = r[i]; j < q.Length; ++j)
                    row[j] = ri * q[j];
            }
        }

        public static void OuterProduct(this double[] r, double[] q, double[][] result)
        {
            for (int i = 0; i < r.Length; ++i)
            {
                double[] row = result[i];
                double ri = r[i];

                for (int j = 0; j < q.Length; ++j)
                    row[j] = ri * q[j];
            }
        }

        public static void OuterProduct(this double[] r, double[] q, double[,] result)
        {
            for (int i = 0; i < r.Length; ++i)
            {
                double ri = r[i];

                for (int j = 0; j < q.Length; ++j)
                    result[i, j] = ri * q[j];
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        public static void Average(this int[] input, double[] output)
        {
            if (output.Length >= input.Length)
                throw new ArgumentOutOfRangeException("output", "The length of the output should be less than length of the input");

            int batchLen = Math.DivRem(input.Length, output.Length, out int plusNum);
            int batchPlusLen = batchLen + 1;

            int start = 0;
            int stop = batchPlusLen;

            for (int j = 0; j < plusNum; ++j, start += batchPlusLen, stop += batchPlusLen)
            {
                int sum = 0;

                for (int i = start, iStop = stop; i < iStop; ++i)
                    sum += input[i];

                output[j] = sum;
                output[j] /= batchPlusLen;
            }

            stop = start + batchLen;

            for (int j = plusNum; j < output.Length; ++j, start += batchLen, stop += batchLen)
            {
                int sum = 0;

                for (int i = start, iStop = stop; i < iStop; ++i)
                    sum += input[i];

                output[j] = sum;
                output[j] /= batchLen;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="index"></param>
        /// <returns></returns>
        public static double Min(this double[] ro, out int index)
        {
            index = -1;
            double min = double.PositiveInfinity;

            for (int i = 0; i < ro.Length; ++i)
            {
                double ri = ro[i];

                if (ri < min)
                {
                    min = ri;
                    index = i;
                }
            }

            return min;
        }

        /// <summary>
        /// Maps all components of this vector to the interval [0, 1]
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="origin"></param>
        /// <param name="range"></param>
        public static void NormalizeComponents(this double[] ro, double[] origin, double[] range)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] -= origin[i];
                ro[i] /= range[i];
            }
        }

        /// <summary>
        /// Sets each component of this vector to a random value from limA to limB
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="limA"></param>
        /// <param name="limB"></param>
        public static void SetRandom(this double[] ro, double limA = 0.0, double limB = 1.0)
        {
            double delta = limB - limA;

            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] = limA + delta * Rnd.NextDouble();
            }
        }

        /// <summary>
        /// Sets each component of this vector to a random value from limA to limA + range
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="limA"></param>
        /// <param name="range"></param>
        public static void SetRandom(this double[] ro, double[] limA, double[] range)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] = limA[i] + range[i] * Rnd.NextDouble();
            }
        }

        /// <summary>
        /// Sets each component of this vector to a random value within the region
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="limA"></param>
        /// <param name="range"></param>
        public static void SetRandom(this double[] ro, Hyperrect region)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] = region.PointA[i] + (region.PointB[i] - region.PointA[i]) * Rnd.NextDouble();
            }
        }

        /// <summary>
        /// Multiplies this vector by the given scalar and stores the result in this vector
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="scalar"></param>
        public static void Multiply(this double[] ro, double scalar)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] *= scalar;
            }
        }

        /// <summary>
        /// Multiplies components of this vector by the given weights and stores the result in the result 
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="weights"></param>
        /// <param name="result"></param>
        public static void Multiply(this int[] ro, double[] weights, double[] result)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                result[i] = ro[i] * weights[i];
            }
        }

        /// <summary>
        /// Adds the vector r to this vector and stores the result in this vector
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="r"></param>
        public static void Add(this double[] ro, double[] r)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] += r[i];
            }
        }

        /// <summary>
        /// Adds the vector r to this vector and stores the result in this vector
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="r"></param>
        public static void Add(this int[] ro, int[] r)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] += r[i];
            }
        }

        /// <summary>
        /// Subtracts the vector r from this vector and stores the result in this vector
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="r"></param>
        public static void Subtract(this double[] ro, double[] r)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] -= r[i];
            }
        }

        /// <summary>
        /// Subtracts the vector r from this vector and stores result in the resuslt
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="r"></param>
        public static void Subtract(this double[] ro, double[] r, double[] result)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                result[i] = ro[i] - r[i];
            }
        }

        /// <summary>
        /// Multiplies this vector by the given scalar, adds the r
        /// and stores the result in this vector
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="scalar"></param>
        /// <param name="r"></param>
        public static void MultiplyAndAdd(this double[] ro, double scalar, double[] r)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                ro[i] *= scalar;
                ro[i] += r[i];
            }
        }

        /// <summary>
        /// Calculates vector increment from this vector to the given vector r
        /// and puts result into the dr
        /// </summary>
        /// <param name="ro"></param>
        /// <param name="r"></param>
        /// <param name="dr"></param>
        public static void VectorIncrement(this double[] ro, double[] r, double[] dr)
        {
            for (int i = 0; i < ro.Length; i++)
            {
                dr[i] = r[i] - ro[i];
            }
        }

        /// <summary>
        /// Returns squared Euclidean distance between this point and the given point
        /// </summary>
        /// <param name="center"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static double SquaredDistance(this double[] center, double[] point)
        {
            double r2 = 0;

            for (int i = 0; i < center.Length; i++)
            {
                double d = point[i] - center[i];
                r2 += d * d;
            }

            return r2;
        }

        /// <summary>
        /// Returns squared Euclidean distance between this point and the target point
        /// and puts the vector equal to the difference of the target and this point, 
        /// into the target
        /// </summary>
        /// <param name="center"></param>
        /// <param name="target"></param>
        /// <returns></returns>
        public static double SquaredDistanceAndDelta(this double[] center, double[] target)
        {
            double r2 = 0;

            for (int i = 0; i < center.Length; i++)
            {
                target[i] -= center[i];
                r2 += target[i] * target[i];
            }

            return r2;
        }

        /// <summary>
        /// Returns squared Euclidean norm of this vector
        /// </summary>
        /// <param name="r"></param>
        /// <returns></returns>
        public static double SquaredNorm(this double[] r)
        {
            double norm = 0;

            foreach (double x in r)
                norm += x * x;

            return norm;
        }

        public static double SquaredNorm(this int[] r)
        {
            double norm = 0;

            foreach (int x in r)
                norm += x * x;

            return norm;
        }

        /// <summary>
        /// Returns squared norm of this vector changed by the given increment dr;
        /// This vector and dr remain intact
        /// </summary>
        /// <param name="r"></param>
        /// <param name="dr"></param>
        /// <returns></returns>
        public static double SquaredNormIncrement(this double[] r, double[] dr)
        {
            double squaredNorm = 0.0;

            for (int i = 0; i < r.Length; i++)
            {
                double dx = r[i] + dr[i];
                squaredNorm += dx * dx;
            }

            return squaredNorm;
        }

        /// <summary>
        /// Performs soft clipping of the input value 
        /// by means of the quadratic Bezier curve
        /// resulting that output value never exceeds the specified limit
        /// </summary>
        /// <param name="input">Input value to be clipped</param>
        /// <param name="limit">Limit of the return value</param>
        /// <param name="kinkRadius">Defines softness of clipping; should be within interval [0, sqrt(2) * absLimit]</param>
        /// <returns></returns>
        public static double Clip(double input, double limit = 1.0, double kinkRadius = 1.0)
        {
            if (kinkRadius == 0)
            {
                if (input > limit) return limit;
                else return input;
            }

            double po = limit - kinkRadius / sqrt2;

            if (input <= po) return input;

            double p2x = limit + kinkRadius;

            if (input >= p2x) return limit;

            // convert input to the Bezier parameter t
            input -= po;
            input /= p2x - po;

            double inverse = 1 - input;
            double factor = input * limit;

            // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
            // P1y = P2y = threshold
            return inverse * inverse * po + 2 * inverse * factor + input * factor;
        }

        /// <summary>
        /// Performs soft clipping of the input value 
        /// by means of the quadratic Bezier curve
        /// resulting that absolute value of the output never exceeds the specified limit
        /// </summary>
        /// <param name="input">Input value to be clipped</param>
        /// <param name="absLimit">Absolute limit of the return value; should be positive</param>
        /// <param name="kinkRadius">Defines softness of clipping; should be within interval [0, sqrt(2) * absLimit]</param>
        /// <returns></returns>
        public static double ClipAbs(double input, double absLimit = 1.0, double kinkRadius = 1.0)
        {
            if (input >= 0)
            {
                if (kinkRadius == 0)
                {
                    if (input > absLimit) return absLimit;
                    else return input;
                }

                double po = absLimit - kinkRadius / sqrt2;

                if (input <= po) return input;

                double p2x = absLimit + kinkRadius;

                if (input >= p2x) return absLimit;

                // convert input to the Bezier parameter t
                input -= po;
                input /= p2x - po;

                double inverse = 1 - input;
                double factor = input * absLimit;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold
                return inverse * inverse * po + 2 * inverse * factor + input * factor;
            }
            else
            {
                double absInput = -input;

                if (kinkRadius == 0)
                {
                    if (absInput > absLimit) return -absLimit;
                    else return input;
                }

                double po = absLimit - kinkRadius / sqrt2;

                if (absInput <= po) return input;

                double p2x = absLimit + kinkRadius;

                if (absInput >= p2x) return -absLimit;

                // convert input to the Bezier parameter t
                absInput -= po;
                absInput /= p2x - po;

                double inverse = 1 - absInput;
                double factor = absInput * absLimit;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold
                absInput = inverse * inverse * po + 2 * inverse * factor + absInput * factor;
                return -absInput;
            }
        }
    }
}
