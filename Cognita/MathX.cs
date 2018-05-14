using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Cognita
{
    public static class MathX
    {
        public static readonly double sqrt2 = Math.Sqrt(2);

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
