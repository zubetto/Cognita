using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;

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
            private static bool isRangesValid = false;

            private static double amp = 1.0;
            private static double ampAbs = 1.0;
            private static double[] ampsArr = new double[2] { 1.0, -1.0 };
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
            public static double[] AmplitudesArr { get { return ampsArr; } }
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
            
            public static void SetBinaryAmps(double positiveAmp, double negativeAmp)
            {
                ampsArr = new double[2] { Math.Abs(positiveAmp), -Math.Abs(negativeAmp) };
            }

            public static IList<double> Ranges { get { return Array.AsReadOnly(ranges); } }
            public static IList<double> Widths { get { return Array.AsReadOnly(widths); } }

            public static double GetEffectiveRadius(double level)
            {
                if (level < 0) level = -level;

                if (level >= ampAbs)
                    return 0;
                else
                    return Math.Pow(ampAbs / level - 1.0, 0.5 / exp) / factor;
            }

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

            public static double Measure(double[] ro, bool[] labels, double[][] rbfCenters)
            {
                double sum = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        if (labels[i])
                            sum += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                        else
                            sum -= 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                    }

                    return sum * ampAbs;
                }
                else
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        double r2 = ro.SquaredDistance(rbfCenters[i]);

                        if (labels[i])
                            sum += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                        else
                            sum -= 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }

                    return sum * ampAbs;
                }
            }

            public static void MeasureUnitAmp(double[] ro, int[] labels, double[][] rbfCenters, double[] result)
            {
                for (int i = 0; i < result.Length; ++i)
                    result[i] = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < rbfCenters.Length; ++i)
                        result[labels[i]] += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                }
                else
                {
                    for (int i = 0; i < rbfCenters.Length; ++i)
                    {
                        double r2 = ro.SquaredDistance(rbfCenters[i]);

                        result[labels[i]] += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }
                }
            }

            public static double MeasureBalanced(double[] ro, bool[] labels, double[][] rbfCenters)
            {
                double sumPos = 0.0;
                double sumNeg = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        if (labels[i])
                            sumPos += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                        else
                            sumNeg += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                    }
                }
                else
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        double r2 = ro.SquaredDistance(rbfCenters[i]);

                        if (labels[i])
                            sumPos += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                        else
                            sumNeg += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }
                }

                return sumPos * ampsArr[0] + sumNeg * ampsArr[1];
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

                if (nmax == 0 || nmax > nlimit)
                {
                    isRangesValid = false;
                    return false;
                }

                //if (nmax > nlimit)
                //    throw new Exception(string.Format("Output array length {0} exceeds the given limit {1}", nmax, nlimit));

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

                isRangesValid = true;
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
                if (!isRangesValid)
                {
                    radthold[0] = radius;
                    radthold[1] = Threshold;

                    return ThresholdExceeded(distance, radthold);
                }

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

            private static double[] radthold = new double[2];

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

        public class RadialFunction
        {
            private double amp = 1.0;
            private double ampAbs = 1.0;
            private double[] ampsArr = new double[2] { 1.0, -1.0 };
            private double factor = 1.0;
            private double factorSqr = 1.0;
            private double exp = 1.0;
            private bool isExpOne = true;

            public RadialFunction(bool copyStaticParams = true)
            {
                if (!copyStaticParams)
                    return;

                Amplitude = RadialBasisFunction.Amplitude;
                Factor = RadialBasisFunction.Factor;
                Exponent = RadialBasisFunction.Exponent;

                SetBinaryAmps(RadialBasisFunction.AmplitudesArr[0], RadialBasisFunction.AmplitudesArr[1]);
            }

            public double Amplitude
            {
                get { return amp; }
                set
                {
                    amp = value;
                    ampAbs = value < 0 ? -amp : amp;
                }
            }

            public double[] AmplitudesArr { get { return ampsArr; } }

            public double AbsAmplitude { get { return ampAbs; } }

            public double Factor
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

            public double Exponent
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

            public void SetBinaryAmps(double positiveAmp, double negativeAmp)
            {
                ampsArr = new double[2] { Math.Abs(positiveAmp), -Math.Abs(negativeAmp) };
            }

            public double GetEffectiveRadius(double level)
            {
                if (level < 0) level = -level;

                if (level >= ampAbs)
                    return 0;
                else
                    return Math.Pow(ampAbs / level - 1.0, 0.5 / exp) / factor;
            }

            #region Scalar Measurements
            public double Measure(double r)
            {
                r *= r;

                if (isExpOne)
                    return amp / (1.0 + factorSqr * r);
                else
                    return amp / (1.0 + Math.Pow(factorSqr * r, Exponent));
            }

            public double MeasureR2(double r2)
            {
                if (isExpOne)
                    return amp / (1.0 + factorSqr * r2);
                else
                    return amp / (1.0 + Math.Pow(factorSqr * r2, Exponent));
            }

            public double MeasureAbs(double r)
            {
                r *= r;

                if (isExpOne)
                    return ampAbs / (1.0 + factorSqr * r);
                else
                    return ampAbs / (1.0 + Math.Pow(factorSqr * r, Exponent));
            }

            public double MeasureAbsR2(double r2)
            {
                if (isExpOne)
                    return ampAbs / (1.0 + factorSqr * r2);
                else
                    return ampAbs / (1.0 + Math.Pow(factorSqr * r2, Exponent));
            }
            #endregion // -------------------------------------------------------


            #region Vector Measurements
            
            /// <summary>
            /// 
            /// </summary>
            /// <param name="ro"></param>
            /// <param name="rbfCenters"></param>
            /// <returns></returns>
            public double Measure(double[] ro, bool[] labels, double[][] rbfCenters)
            {
                double sum = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        if (labels[i])
                            sum += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                        else
                            sum -= 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                    }

                    return sum * ampAbs;
                }
                else
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        double r2 = ro.SquaredDistance(rbfCenters[i]);

                        if (labels[i])
                            sum += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                        else
                            sum -= 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }

                    return sum * ampAbs;
                }
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="ro"></param>
            /// <param name="labels"></param>
            /// <param name="rbfCenters"></param>
            /// <returns></returns>
            public double MeasureBalanced(double[] ro, bool[] labels, double[][] rbfCenters)
            {
                double sumPos = 0.0;
                double sumNeg = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        if (labels[i])
                            sumPos += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                        else
                            sumNeg += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                    }
                }
                else
                {
                    for (int i = 0; i < rbfCenters.Length; i++)
                    {
                        double r2 = ro.SquaredDistance(rbfCenters[i]);

                        if (labels[i])
                            sumPos += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                        else
                            sumNeg += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }
                }

                return sumPos * ampsArr[0] + sumNeg * ampsArr[1];
            }

            /// <summary>
            /// Measure with a unit amplitude
            /// </summary>
            /// <param name="ro"></param>
            /// <param name="labels"></param>
            /// <param name="rbfCenters"></param>
            /// <param name="result"></param>
            public void MeasureUnitAmp(double[] ro, int[] labels, double[][] rbfCenters, double[] result)
            {
                for (int i = 0; i < result.Length; ++i)
                    result[i] = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < rbfCenters.Length; ++i)
                        result[labels[i]] += 1.0 / (1.0 + factorSqr * ro.SquaredDistance(rbfCenters[i]));
                }
                else
                {
                    for (int i = 0; i < rbfCenters.Length; ++i)
                    {
                        double r2 = ro.SquaredDistance(rbfCenters[i]);

                        result[labels[i]] += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }
                }
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="dro"></param>
            /// <param name="dRbfCenters"></param>
            /// <returns></returns>
            public double MeasureDr(double[] dro, bool[] labels, double[][] dRbfCenters)
            {
                double sum = 0.0;

                if (isExpOne)
                {
                    for (int i = 0; i < dRbfCenters.Length; i++)
                    {
                        if (labels[i])
                            sum += 1.0 / (1.0 + factorSqr * dRbfCenters[i].SquaredNormIncrement(dro));
                        else
                            sum -= 1.0 / (1.0 + factorSqr * dRbfCenters[i].SquaredNormIncrement(dro));
                    }

                    return sum * ampAbs;
                }
                else
                {
                    for (int i = 0; i < dRbfCenters.Length; i++)
                    {
                        double r2 = dRbfCenters[i].SquaredNormIncrement(dro);

                        if (labels[i])
                            sum += 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                        else
                            sum -= 1.0 / (1.0 + Math.Pow(factorSqr * r2, Exponent));
                    }

                    return sum * ampAbs;
                }
            }

            #endregion // -------------------------------------------------------
        }

        public enum DataSet { Training, Validation, Test, None }

        public interface IPointsSource
        {
            int Dimension { get; }
            int BatchLength { get; set; }
            
            bool SeekNext(DataSet set, int index);

            int GetCurrentIndex(DataSet set);
            int GetNextIndex(DataSet set);

            int PositiveNum { get; }
            int NegativeNum { get; }

            bool GetCurrentNorm(DataSet set, ref double[][] points, ref bool[] labels);
            bool GetNextNorm(DataSet set, ref double[][] points, ref bool[] labels);

            /// <summary>
            /// If points source is based upon a some database then method should return double.NaN
            /// </summary>
            /// <param name="point"></param>
            /// <returns></returns>
            double Classify(double[] point);
        }

        /// <summary>
        /// 
        /// </summary>
        public class RBFVoxelizer
        {
            private IPointsSource dataSource;

            private AdaptiveGrid<double> featureSpace;
            private IList<IVoxel> voxelArray;
            private int tessMultp;
            private int tessMultpPlus;

            private RadialFunction radialFunction;
            private double effRadiusInv;
            private double[][] points;
            private bool[] labels;

            private int trainingInd = -1;

            private double posMax = 0.0;
            private double negMin = 0.0;

            private int posNum = 0;
            private int negNum = 0;
            
            private bool surveyComplete = false;
            private bool refineStarted = false;

            // --- Are used by the methods of smoothness check -------
            private double usedThold;

            // this is the absolute limit for handling of details
            // allowing to not waste voxels for catching information about
            // "small stones at the top of the mountain"
            private double detailsLimit; 

            private int[] tmpIndexVector;
            private double[] tmpVector;
            private int[] monotonicDirIndxs;
            private double monotonicMargin; // should be less than or equal zero

            // --- public ---------------------------------------------------------
            public AdaptiveGrid<double> FeatureSpace { get { return featureSpace; } }

            public int PositiveNum { get { return posNum; } }
            public int NegativeNum { get { return negNum; } }

            public class SyncEventArgs : EventArgs
            {
                public bool StopFlag { get; set; } = false;
            }

            public event EventHandler<SyncEventArgs> SyncEvent;
            public event EventHandler CompletionEvent;

            public bool IsRefineStarted { get { return refineStarted; } }
            public bool IsCompleted { get; set; } = false;

            /// <summary>
            /// 
            /// </summary>
            /// <param name="data"></param>
            /// <param name="rbf"></param>
            /// <param name="tessNum"></param>
            /// <param name="rootTile"></param>
            /// <param name="levelTile"></param>
            public RBFVoxelizer(IPointsSource data, RadialFunction rbf, int tessNum, int rootTile, int levelTile = 2)
            {
                // --- Assign params values -------------------------------------
                dataSource = data;
                radialFunction = rbf;
                effRadiusInv = 0.01 / radialFunction.GetEffectiveRadius(0.01);

                // --- Ini AdaptiveGrid ---------------------------------------------
                int dim = data.Dimension;
                double[,] shape = new double[dim, 3];

                for (int i = 0; i < dim; i++)
                {
                    shape[i, 0] = 0.0;
                    shape[i, 1] = 1.0;
                    shape[i, 2] = rootTile;
                }

                int[] tess = Enumerable.Repeat(levelTile, dim).ToArray();

                featureSpace = new AdaptiveGrid<double>(tessNum, shape, tess, iniValue: double.NaN);
                voxelArray = featureSpace.InternalArray;
                tessMultp = featureSpace.TessellationMultp;
                tessMultpPlus = tessMultp + 1;

                // new voxels obtained as result of tessellations should have state equal true
                featureSpace.ProcessedState = false;
                featureSpace.SetRootStates(true);

                // --- Ini vectors and grid stuff ------------------------------------
                tmpIndexVector = new int[dim];
                tmpVector = new double[dim];

                monotonicDirIndxs = featureSpace.GetDiagonalPairs(dim / 2 + 1);
            }

            /// <summary>
            /// Effective radius is used for the empirical estimation of confidence of the classification results
            /// </summary>
            /// <param name="level"></param>
            public void SetEffectiveRadius(double level)
            {
                // 0.01 is empirical value
                effRadiusInv = 0.01 / radialFunction.GetEffectiveRadius(level);
            }

            /// <summary>
            /// Returns the probability in percentage, a positive or negative value, 
            /// that the given point belongs to a positive or negative class
            /// </summary>
            /// <param name="point"></param>
            /// <returns></returns>
            public double Classify(double[] point)
            {
                if (!featureSpace.GetInnermost(point, out IVoxel voxel))
                    return double.NaN;

                // --- Empirical estimation of the classification confidence ---
                // https://www.desmos.com/calculator/djvlmfzorq

                // the more disbalance of presented points the less confidence of classification
                double classRatio = posNum > negNum ? (1.0 * negNum / posNum) : (1.0 * posNum / negNum);

                // the smaller the effective radius the more confidence of classification;
                // if the effective radius is tends to zero 
                // then the overall confidence coefficient tends to one;
                // if the effective radius is tends to infinity 
                // then the overall confidence coefficient tends to the classRatio
                classRatio = Math.Sqrt((1.0 + classRatio) / (1.0 - classRatio));

                double confCoef = effRadiusInv + classRatio;
                confCoef *= confCoef;
                confCoef = (confCoef - 1.0) / (confCoef + 1.0); 

                // normalized voxel value
                double val = voxel.Data > 0 ? voxel.Data / posMax : voxel.Data / -negMin;

                return confCoef * val;
            }

            /// <summary>
            /// 
            /// </summary>
            private void DefineMax()
            {
                posMax = 0.0;
                negMin = 0.0;

                int level = -1;

                while (++level < featureSpace.LevelsNumber)
                {
                    featureSpace.FirstAtLevel(level, out IVoxel voxel);

                    do
                    {
                        if (voxel.ContentSI > 0) continue;

                        if (voxel.Data > posMax)
                            posMax = voxel.Data;
                        else if (voxel.Data < negMin)
                            negMin = voxel.Data;
                    }
                    while (featureSpace.NextAtLevel(ref voxel));
                }
            }

            /// <summary>
            /// The main refinement process of the feature space
            /// </summary>
            public void Voxelize(double threshold, double batchCoef = 1.0)
            {
                if (IsCompleted)
                    throw new InvalidOperationException("IsCompleted property is set to true, perhaps due to the source depletion.");

                if (threshold <= 0.0)
                    throw new ArgumentOutOfRangeException("threshold", "threshold should be positive value");

                if (batchCoef <= 0.0)
                    throw new ArgumentOutOfRangeException("batchCoef", "batchCoef should be positive value");

                refineStarted = true;

                if (!surveyComplete)
                {
                    for (int i = 0; i < featureSpace.RootsNumber; i++)
                        featureSpace.InternalArray[i].Data = 0.0;

                    surveyComplete = true;
                }
                
                if (trainingInd < 0)
                    trainingInd = dataSource.GetNextIndex(DataSet.Training);
                
                dataSource.SeekNext(DataSet.Training, trainingInd);

                usedThold = threshold;
                detailsLimit = points.Length * batchCoef * radialFunction.AbsAmplitude;

                while (dataSource.GetNextNorm(DataSet.Training, ref points, ref labels))
                {
                    trainingInd = dataSource.GetNextIndex(DataSet.Training);

                    posNum += dataSource.PositiveNum;
                    negNum += dataSource.NegativeNum;

                    AggregateRefine();

                    // --- Sync event ------------
                    var SEA = new SyncEventArgs();
                    SyncEvent?.Invoke(this, SEA);

                    if (SEA.StopFlag)
                    {
                        DefineMax();
                        return;
                    }
                }

                DefineMax();
                IsCompleted = true;

                CompletionEvent?.Invoke(this, EventArgs.Empty);
            }

            /// <summary>
            /// Preliminary refines the grid with a few points 
            /// for reducing the chance of loss of the fine details
            /// </summary>
            public void Survey(double threshold, double monotonicTol = 0.25, double batchCoef = 1.0)
            {
                if (threshold <= 0.0)
                    throw new ArgumentOutOfRangeException("threshold", "threshold should be positive value");

                if (batchCoef <= 0.0)
                    throw new ArgumentOutOfRangeException("batchCoef", "batchCoef should be positive value");

                if (refineStarted)
                    throw new InvalidOperationException("The Survey method should not be called after the Voxelize was called at least once");

                if (surveyComplete)
                {
                    foreach (var v in featureSpace.InternalArray)
                    {
                        v.Merge(true, iniValue: double.NaN);
                        v.State = true;
                    }
                }

                if (!dataSource.GetCurrentNorm(DataSet.Training, ref points, ref labels))
                    return; // >>>>>>> the source is depleted >>>>>>>

                posNum = dataSource.PositiveNum;
                negNum = dataSource.NegativeNum;

                usedThold = threshold;
                detailsLimit = points.Length * batchCoef * radialFunction.AbsAmplitude;

                if (detailsLimit < usedThold) detailsLimit = usedThold;

                monotonicMargin = monotonicTol > 0 ? -monotonicTol : monotonicTol;

                // --- Tessellation pass ---------------------------------------------
                // traversal all voxels at the 0 level
                for (int i = 0, num = featureSpace.RootsNumber; i < num; i++)
                {
                    IVoxel voxel = voxelArray[i];

                    voxel.GetCenter(tmpVector);
                    voxel.Data = radialFunction.MeasureBalanced(tmpVector, labels, points);

                    voxel.Tessellate(true, LeafNotSmooth);
                }

                int level = 0;

                // continue tree traversal downwards
                while (++level < featureSpace.LevelsNumber)
                {
                    if (!featureSpace.FirstAtLevel(level, out IVoxel voxel))
                        break;

                    // traversal all voxels at the current level
                    do
                    {
                        if (voxel.Tessellate(true, LeafNotSmooth))
                        {
                            // The ancestor becomes complex (neither a leaf nor a crown)
                            if (voxel.Ancestor != null)
                                voxel.Ancestor.State = false;
                        }
                    }
                    while (featureSpace.NextAtLevel(ref voxel));
                }

                DefineMax();

                trainingInd = dataSource.GetNextIndex(DataSet.Training);
                surveyComplete = true;
            }

            /// <summary>
            /// Superimposes the new batch of points with the existing grid
            /// </summary>
            private void AggregateRefine()
            {
                // --- Merge pass --------------------------------------
                int level = featureSpace.LevelsNumber - 1;

                // tree traversal upwards starting from the penultimate level
                while (--level >= 0)
                {
                    featureSpace.FirstAtLevel(level, out IVoxel voxel);

                    do
                    {
                        if (voxel.State) // then voxel is leaf or crown
                        {
                            voxel.GetCenter(tmpVector);
                            voxel.Data += radialFunction.MeasureBalanced(tmpVector, labels, points);

                            if (voxel.ContentSI > 0) // then voxel is crown
                            {
                                // check for smoothness and as a result updating the leafs data
                                if (CrownNotSmooth_Agg(voxel))
                                    continue;

                                // crown has become smooth
                                voxel.Merge(true);

                                // prevent considiration of this voxel during the tess-pass
                                voxel.State = false;

                                // check if the ancestor has become the crown after the merge
                                if (voxel.Ancestor != null)
                                {
                                    voxel.Ancestor.State = true;

                                    for (int i = voxel.Ancestor.ContentSI, stop = i + tessMultp; i < stop; i++)
                                    {
                                        if (voxelArray[i].ContentSI > 0)
                                        {
                                            voxel.Ancestor.State = false;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            // voxel.Data is set to average of the all data of its direct children
                            // and this average is fresh due to the upwards traversal;
                            // this average value is intended for visualization in cases
                            // when voxels at the next levels cannot be visibly resolved
                            double avg = 0.0;
                            for (int i = voxel.ContentSI, stop = i + tessMultp; i < stop; i++)
                                avg += voxelArray[i].Data;

                            voxel.Data = avg / tessMultp;
                        }
                    }
                    while (featureSpace.NextAtLevel(ref voxel));
                }

                // --- Tessellation pass --------------------------------------------------------------
                level = -1;

                // tree traversal downwards
                while (++level < featureSpace.LevelsNumber)
                {
                    if (!featureSpace.FirstAtLevel(level, out IVoxel voxel))
                        break;

                    // traversal all voxels at the current level
                    do
                    {
                        if (voxel.ContentSI < 0)
                        {
                            if (voxel.State) // then voxel is leaf and was not merged
                            {
                                if (voxel.Tessellate(true, LeafNotSmooth_Agg))
                                {
                                    // The ancestor becomes complex (neither a leaf nor a crown)
                                    if (voxel.Ancestor != null)
                                        voxel.Ancestor.State = false;
                                }
                            }
                            else voxel.State = true; // replace state to true to indicate that voxel is leaf
                        }
                    }
                    while (featureSpace.NextAtLevel(ref voxel));
                }
            }

            /// <summary>
            /// OBSOLETE
            /// </summary>
            /// <param name="voxel"></param>
            /// <returns></returns>
            //private bool LeafNotSmooth(IVoxel voxel)
            //{
            //    double center = voxel.Data;
            //    double min = center;
            //    double max = min;

            //    // ini the iterators for the traversal,
            //    // the tmpIndexVector should be already set to zero-index
            //    voxel.GetFirstCenter(tmpVector);

            //    int index0 = voxel.ContentSI; // should be reserved in the Tessellation method
            //    int index = index0;
            //    bool isTholdExceeded = false;

            //    // threshold exceedance check
            //    do
            //    {
            //        double val = radialFunction.MeasureBalanced(tmpVector, labels, points);

            //        voxelArray[index++].Data = val;

            //        if (val < min)
            //        {
            //            min = val;

            //            if (max - min >= usedThold)
            //            {
            //                isTholdExceeded = true;
            //                break;
            //            }
            //        }
            //        else if (val > max)
            //        {
            //            max = val;

            //            if (max - min >= usedThold)
            //            {
            //                isTholdExceeded = true;
            //                break;
            //            }
            //        }
            //    }
            //    while (voxel.NextCenter(tmpIndexVector, tmpVector));

            //    // finish calculation of data for voxels to be obtained as a result of tessellation
            //    if (isTholdExceeded)
            //    {
            //        while (voxel.NextCenter(tmpIndexVector, tmpVector))
            //            voxelArray[index++].Data = radialFunction.Measure(tmpVector, labels, points);

            //        return true;
            //    }

            //    // monotonicity check
            //    if (double.IsInfinity(monotonicMargin))
            //        return false;

            //    for (int i = 0; i < monotonicDirIndxs.Length; i += 2)
            //    {
            //        double d1 = center - voxelArray[index0 + monotonicDirIndxs[i]].Data;
            //        double d2 = voxelArray[index0 + monotonicDirIndxs[i + 1]].Data - center;
            //        double margin = monotonicMargin * d1;

            //        if ((d1 > 0.0 && d2 < margin) || (d1 < 0.0 && d2 > margin))
            //            return true;
            //    }

            //    return false;
            //}

            /// <summary>
            /// 
            /// </summary>
            /// <param name="voxel"></param>
            /// <returns></returns>
            private bool LeafNotSmooth(IVoxel voxel)
            {
                double center = voxel.Data;
                double min = center;
                double max = min;
                double avg = center;
                bool isNegative = center < 0.0;

                // ini the iterators for the traversal,
                // the tmpIndexVector should be already set to zero-index
                voxel.GetFirstCenter(tmpVector);

                int index0 = voxel.ContentSI; // should be reserved in the Tessellation method
                int index = index0;
                bool isUnipolar = true;

                do // calc avg and set voxel data
                {
                    double val = radialFunction.MeasureBalanced(tmpVector, labels, points);
                    
                    voxelArray[index++].Data = val;
                    avg += val;

                    // sign change check
                    if (isUnipolar && (isNegative ^ val < 0.0))
                        isUnipolar = false;
                    
                    if (val < min)
                        min = val;
                    else if (val > max)
                        max = val;
                }
                while (voxel.NextCenter(tmpIndexVector, tmpVector));

                avg /= tessMultpPlus;
                voxel.Data = avg;
                
                double delta = max - min;

                // threshold exceedance check
                if (delta >= usedThold)
                {
                    if (!isUnipolar)
                        return true;

                    if (avg < 0.0) avg = -avg;

                    // check if relative increment exceeds the allowable relative error
                    // (deta / avg) >= relativeErr equivalent to
                    // avg < (delta / relativeErr) equivalent to
                    // avg < (usedThold / relativeErr) where 
                    // relativeErr is defined as (usedThold / detailsLimit)
                    if (avg < detailsLimit)
                        return true;
                }

                // monotonicity check
                if (double.IsInfinity(monotonicMargin))
                    return false;

                for (int i = 0; i < monotonicDirIndxs.Length; i += 2)
                {
                    double d1 = center - voxelArray[index0 + monotonicDirIndxs[i]].Data;
                    double d2 = voxelArray[index0 + monotonicDirIndxs[i + 1]].Data - center;
                    double margin = monotonicMargin * d1;

                    if ((d1 > 0.0 && d2 < margin) || (d1 < 0.0 && d2 > margin))
                        return true;
                }

                return false;
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="voxel"></param>
            /// <returns></returns>
            private bool LeafNotSmooth_Agg(IVoxel voxel)
            {
                // ini the iterators for the traversal,
                // the tmpIndexVector should be already set to zero-index
                voxel.GetFirstCenter(tmpVector);

                double avg = radialFunction.MeasureBalanced(tmpVector, labels, points);
                double min = avg;
                double max = min;

                int index0 = voxel.ContentSI; // should be reserved in the Tessellation method
                int index = index0;

                voxelArray[index].Data = avg;

                while (voxel.NextCenter(tmpIndexVector, tmpVector))
                {
                    double val = radialFunction.MeasureBalanced(tmpVector, labels, points);

                    voxelArray[++index].Data = val;
                    avg += val;
                    
                    if (val < min)
                        min = val;
                    else if (val > max)
                        max = val;
                }
                
                double delta = max - min;

                // here is assumed that value of the rbf at the voxel center is within delta
                if (delta >= usedThold)
                {
                    // then one more calculation of the rbf value at the same point
                    // as during the merge pass is needed to restore previous value of the voxel
                    voxel.GetCenter(tmpVector);

                    double center = radialFunction.MeasureBalanced(tmpVector, labels, points);
                    double prev = voxel.Data - center;

                    bool isNegative = voxel.Data < 0.0;
                    bool isUnipolar = true;

                    for (int i = index0, stop = i + tessMultp; i < stop; i++)
                    {
                        if (isNegative ^ (voxelArray[i].Data += prev) < 0.0)
                            isUnipolar = false;
                    }

                    avg = prev + (avg + center) / tessMultpPlus;

                    voxel.Data = avg;

                    if (!isUnipolar)
                        return true;

                    if (avg < 0.0) avg = -avg;

                    // check if relative increment exceeds the allowable relative error
                    // (deta / avg) >= relativeErr equivalent to
                    // avg < (delta / relativeErr) equivalent to
                    // avg < (usedThold / relativeErr) where 
                    // relativeErr is defined as (usedThold / detailsLimit)
                    if (avg < detailsLimit)
                        return true;
                }

                return false;
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="voxel"></param>
            /// <returns></returns>
            private bool CrownNotSmooth_Agg(IVoxel voxel)
            {
                double center = voxel.Data;
                double min = center;
                double max = min;
                double avg = center;
                bool isNegative = center < 0.0;

                // ini the iterators for the traversal,
                // the tmpIndexVector should be already set to zero-index
                voxel.GetFirstCenter(tmpVector);

                int index0 = voxel.ContentSI; // voxel should be the crown
                int index = index0;
                bool isUnipolar = true;

                do // calc avg and aggregate voxel data
                {
                    var iV = voxelArray[index++];

                    iV.Data += radialFunction.MeasureBalanced(tmpVector, labels, points);

                    double val = iV.Data;

                    avg += val;

                    // sign change check
                    if (isUnipolar && (isNegative ^ val < 0.0))
                        isUnipolar = false;

                    if (val < min)
                        min = val;
                    else if (val > max)
                        max = val;
                }
                while (voxel.NextCenter(tmpIndexVector, tmpVector));

                avg /= tessMultpPlus;
                voxel.Data = avg;

                double delta = max - min;

                // threshold exceedance check
                if (delta >= usedThold)
                {
                    if (!isUnipolar)
                        return true;

                    if (avg < 0.0) avg = -avg;

                    // check if relative increment exceeds the allowable relative error
                    // (deta / avg) >= relativeErr equivalent to
                    // avg < (delta / relativeErr) equivalent to
                    // avg < (usedThold / relativeErr) where 
                    // relativeErr is defined as (usedThold / detailsLimit)
                    if (avg < detailsLimit)
                        return true;
                }
                
                return false;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        public class RBFAggregator
        {
            private int[][] labelsBatch;
            private double[][][] pointsBatch;
            private double[][] resultsBatch;
            private double[] result;
            private double[] coeffs;

            private TasksDispenser dispenser;

            public RadialFunction RBF { get; private set; }
            
            /// <summary>
            /// 
            /// </summary>
            /// <param name="classesNum"></param>
            /// <param name="labels"></param>
            /// <param name="points"></param>
            /// <param name="rbf"></param>
            /// <param name="trainNum"></param>
            /// <param name="fractions"></param>
            /// <param name="threadMultp"></param>
            public RBFAggregator(int classesNum, int[] labels, double[][] points, RadialFunction rbf, 
                                 int trainNum = int.MaxValue, double[] fractions = null, int threadMultp = 1)
            {
                if (classesNum < 2)
                    throw new ArgumentOutOfRangeException("classesNum", "The min allowed number of classes is two");

                if (labels.Length != points.Length)
                    throw new ArgumentException("The labels and points arrays should be the same length");

                if (trainNum <= 0)
                    throw new ArgumentOutOfRangeException("trainNum", "The trainNum should be positive");

                RBF = rbf ?? throw new ArgumentNullException("rbf");

                if (trainNum > labels.Length)
                    trainNum = labels.Length;
                
                dispenser = new TasksDispenser(trainNum, threadMultp);

                labelsBatch = new int[dispenser.ThreadsNum][];
                pointsBatch = new double[dispenser.ThreadsNum][][];
                resultsBatch = new double[dispenser.ThreadsNum][];
                result = new double[classesNum];
                coeffs = new double[classesNum];

                // --- calc balance coeffs ---
                if (fractions == null)
                {
                    for (int i = 0; i < trainNum; ++i)
                        ++coeffs[labels[i]];

                    for (int i = 0; i < classesNum; ++i)
                        coeffs[i] /= trainNum;
                }
                else
                {
                    Buffer.BlockCopy(fractions, 0, coeffs, 0, classesNum * sizeof(double));
                }

                double min = coeffs.Min(out int imin);
                double exp = 1.0 / points[0].Length;

                for (int i = 0; i < imin; ++i)
                    coeffs[i] = Math.Pow(min / coeffs[i], exp);

                coeffs[imin] = 1.0;

                for (int i = imin; i < classesNum; ++i)
                    coeffs[i] = Math.Pow(min / coeffs[i], exp);

                // --- set batches for each thread ---
                dispenser.ResetDispenser();

                int start = 0;
                int index = 0;

                while (dispenser.DispenseNext(out int length))
                {
                    // copy the labels
                    labelsBatch[index] = new int[length];

                    Buffer.BlockCopy(labels, start * sizeof(int), labelsBatch[index], 0, length * sizeof(int));

                    // reference the points
                    double[][] tmp = new double[length][];
                    pointsBatch[index] = tmp;

                    for (int i = 0, k = start; i < length; ++i, ++k)
                        tmp[i] = points[k];

                    ++index;
                    start += length;
                }

                // ini the resultsBatch
                for (int i = 0; i < resultsBatch.Length; ++i)
                    resultsBatch[i] = new double[classesNum];
            }
            
            /// <summary>
            /// Returns predicted class for the given point
            /// </summary>
            /// <param name="point"></param>
            /// <param name="balanced"></param>
            /// <returns></returns>
            public int Classify_parallel(double[] point, bool balanced = true)
            {
                for (int i = 0; i < result.Length; ++i)
                    result[i] = 0.0;

                int counter = labelsBatch.Length;
                var RelayComplete = new AutoResetEvent(false);

                for (int i = 0; i < labelsBatch.Length; ++i)
                {
                    ThreadPool.QueueUserWorkItem((o) => 
                    {
                        int index = (int)o;
                        double[] iResult = resultsBatch[index];

                        RBF.MeasureUnitAmp(point, labelsBatch[index], pointsBatch[index], iResult);

                        lock (result)
                        {
                            for (int k = 0; k < result.Length; ++k)
                                result[k] += iResult[k];
                        }
                        
                        if (Interlocked.Decrement(ref counter) == 0)
                            RelayComplete.Set();

                    }, i);
                }

                RelayComplete.WaitOne();
                
                if (balanced)
                {
                    double max = result[0] * coeffs[0];
                    int imax = 0;

                    for (int i = 1; i < result.Length; ++i)
                    {
                        double ri = result[i] * coeffs[i];

                        if (ri > max)
                        {
                            max = ri;
                            imax = i;
                        }
                    }

                    return imax;
                }
                else
                {
                    double max = result[0];
                    int imax = 0;

                    for (int i = 1; i < result.Length; ++i)
                    {
                        if (result[i] > max)
                        {
                            max = result[i];
                            imax = i;
                        }
                    }

                    return imax;
                }
            }
        }
    }
}
