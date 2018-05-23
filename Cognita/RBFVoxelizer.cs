using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Cognita
{
    namespace SupervisedLearning
    {
        /// <summary>
        /// f(r) = Amp / (1 + (k^2 * r^2)^exp)
        /// </summary>
        public static class RadialBasisFunction
        {
            private static double[] ranges; // { 0, r1, r2, ... rn }
            private static double[] widths; // { r1, r2 - r1, ... rn - rn_1 }

            private static double amp = 1.0;
            private static double ampAbs = 1.0;
            private static double factor = 1.0;
            private static double factorSqr = 1.0;
            private static double exp = 1.0;
            private static bool isExpOne = true;

            public static double Amplitude
            {
                get { return amp; }
                set
                {
                    amp = value;
                    ampAbs = value < 0 ? -amp : amp; 
                }
            }
            public static double Factor
            {
                get { return factor; }
                set
                {
                    if (value > 0)
                    {
                        factor = value;
                        factorSqr = value * value;
                    }
                }
            }
            public static double Exponent
            {
                get { return exp; }
                set
                {
                    if (value > 0)
                    {
                        exp = value;

                        isExpOne = (value == 1.0);
                    }
                }
            }
            public static double Threshold { get; private set; }

            public static IList<double> Ranges { get { return Array.AsReadOnly(ranges); } }
            public static IList<double> Widths { get { return Array.AsReadOnly(widths); } }

            public static double Measure(double r)
            {
                r *= r;

                if (isExpOne)
                    return amp / (1.0 + factorSqr * r);
                else
                    return amp / (1.0 + Math.Pow(factorSqr * r, Exponent));
            }

            public static double MeasureR2(double r2)
            {
                if (isExpOne)
                    return amp / (1.0 + factorSqr * r2);
                else
                    return amp / (1.0 + Math.Pow(factorSqr * r2, Exponent));
            }

            public static double MeasureAbs(double r)
            {
                r *= r;

                if (isExpOne)
                    return ampAbs / (1.0 + factorSqr * r);
                else
                    return ampAbs / (1.0 + Math.Pow(factorSqr * r, Exponent));
            }

            public static double MeasureAbsR2(double r2)
            {
                if (isExpOne)
                    return ampAbs / (1.0 + factorSqr * r2);
                else
                    return ampAbs / (1.0 + Math.Pow(factorSqr * r2, Exponent));
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="threshold"></param>
            /// <param name="rmax"></param>
            /// <param name="tolerance"></param>
            /// <param name="nlimit"></param>
            /// <returns></returns>
            public static bool CalcRanges(double threshold, double rmax, double tolerance, int nlimit = 1000)
            {
                int nmax = (int)(ampAbs / threshold);
                double h = nmax * threshold;
                double e = 0.5 / exp;
                double k = 1.0 / factor;
                double r = k * Math.Pow(h / (ampAbs - h), e);

                // Adjust nmax for the desired rmax
                while (r > rmax && --nmax > 0)
                {
                    h -= threshold;
                    r = k * Math.Pow(h / (ampAbs - h), e);
                }

                if (nmax == 0) return false;

                if (nmax > nlimit)
                    throw new Exception(string.Format("Output array length {0} exceeds the given limit {1}", nmax, nlimit));

                RadialBasisFunction.Threshold = threshold;
                RadialBasisFunction.ranges = new double[nmax + 2];
                RadialBasisFunction.widths = new double[nmax + 1];

                double[] ranges = RadialBasisFunction.ranges;
                double[] widths = RadialBasisFunction.widths;

                ranges[0] = 0;
                ranges[widths.Length] = double.PositiveInfinity;

                h = threshold;

                double prew = -1; // previous width
                bool inflectionPassed = false; // it is assumed, that RBF has no more than one inflection point

                // here widths.Length is used instead of ranges.Length 
                // due to ranges[widths.Length] = double.PositiveInfinity;
                for (int i = 1, j = 0; i < widths.Length; i++, j++)
                {
                    ranges[i] = k * Math.Pow(h / (ampAbs - h), e);

                    double tmp = ranges[i] - ranges[j];
                    widths[j] = tmp;

                    if (inflectionPassed)
                    {
                        // then RBF is convex and therefore
                        // each widths[j] is greater than the previous width prew and
                        // widths[j] is overwritten by estimation value
                        widths[j] = prew * (1.0 + tolerance * (1 - prew / tmp));
                    }
                    else if (prew > 0)
                    {
                        if (tmp > prew)
                        {
                            // then inflection point is located within the span widths[j-1];
                            // widths[j-1] stores exact difference between consecutive ranges rather
                            // then estimation value which is always greater than or equal to the difference 
                            inflectionPassed = true;

                            widths[j] = prew * (1.0 + tolerance * (1 - prew / tmp));
                        }
                        else
                        {
                            // RBF is concave and therefore
                            // each widths[j] is less than the previous width prew and
                            // widths[j-1] is overwritten by estimation value
                            widths[j - 1] = tmp * (1.0 + tolerance * (1 - tmp / prew));
                        }
                    }

                    prew = tmp;
                    h += threshold;
                }

                // recall that the last range is set to +Inf
                widths[widths.Length - 1] = prew * (1.0 + tolerance);

                return true;
            }

            /// <summary>
            /// Returns true if RBF increment within the given circle 
            /// exceeds the specified threshold;
            /// Utilises precalculated arrays for the specified threshold
            /// </summary>
            /// <param name="distance">The distance between centers of RBF and the circle</param>
            /// <param name="radius">Circle radius</param>
            /// <returns></returns>
            public static bool ThresholdExceeded(double distance, double radius)
            {
                if (distance == 0)
                {
                    return radius >= 0.5 * widths[0];
                }
                else if (distance - radius > ranges[ranges.Length - 2])
                {
                    return false;
                }

                int indH = Array.BinarySearch(ranges, distance);

                if (indH < 0)
                    indH = ~indH;

                return radius >= 0.5 * widths[--indH];
            }

            /// <summary>
            /// Returns true if RBF increment within the given circle 
            /// exceeds the given threshold;
            /// </summary>
            /// <param name="distance">The distance between centers of RBF and the circle</param>
            /// <param name="radthold">{ circle_radius, threshold }</param>
            /// <returns></returns>
            public static bool ThresholdExceeded(double distance, double[] radthold)
            {
                double Vo = MeasureAbs(distance);
                double Vfar = MeasureAbs(distance + radthold[0]);

                double h = 0.5 * radthold[1];

                // at far point
                if (Vo - Vfar >= h) return true;

                if (distance > radthold[0])
                {
                    // at near point
                    if (MeasureAbs(distance - radthold[0]) - Vo >= h) return true;
                }
                else // rbf center is within the circle
                {
                    if (ampAbs - Vo >= h) return true;
                }

                return false;
            }
        }

        public class RBFVoxelizer
        {
            private AdaptiveGrid<double> featureSpace;
        }
    }
}
