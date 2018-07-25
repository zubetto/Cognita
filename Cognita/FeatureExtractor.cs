using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Cognita
{
    public class ImageFeatureExtractor
    {
        private int width;
        private int height;
        private int pixNum;

        private double inLevel = 0.5;
        private double outLevel = 0.5;

        private bool[] tmpRow;

        public ImageFeatureExtractor(int width, int height)
        {
            if (width <= 0)
                throw new ArgumentOutOfRangeException("width", "The width should be positive");

            if (height <= 0)
                throw new ArgumentOutOfRangeException("height", "The height should be positive");

            this.width = width;
            this.height = height;
            pixNum = width * height;

            tmpRow = new bool[width];
        }

        public void SetInOutLevels(double inLevel, double outLevel)
        {
            if (inLevel < outLevel)
                throw new ArgumentException("The inLevel should be greater than or equal to the outLevel");

            this.inLevel = inLevel;
            this.outLevel = outLevel;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="input"></param>
        /// <param name="max"></param>
        /// <param name="output"></param>
        public void GetRisingEdgesNum(int[] input, int max, int[][] output)
        {
            // --- process the first row --------------------
            output[0][0] = 0;

            bool isActive = false;
            
            for (int i = 0; i < width; ++i)
            {
                double value = input[i];
                value /= max;

                // cacl rising edges in the first row
                if (isActive)
                {
                    if (value < outLevel)
                        isActive = false;
                }
                else
                {
                    if (value > inLevel)
                    {
                        isActive = true;
                        ++output[0][0];
                    }
                }

                // cacl rising edges in i-th column
                if (value > inLevel)
                {
                    tmpRow[i] = true;
                    output[1][i] = 1;
                }
                else
                {
                    tmpRow[i] = false;
                    output[1][i] = 0;
                }
            }

            // --- process the remain rows --------------------
            for (int n = 1; n < height; ++n)
            {
                output[0][n] = 0;
                isActive = false;

                for (int i = 0; i < width; ++i)
                {
                    double value = input[i + n * width];
                    value /= max;

                    // cacl rising edges in n-th row
                    if (isActive)
                    {
                        if (value < outLevel)
                            isActive = false;
                    }
                    else
                    {
                        if (value > inLevel)
                        {
                            isActive = true;
                            ++output[0][n];
                        }
                    }

                    // cacl rising edges in i-th column
                    if (tmpRow[i])
                    {
                        if (value < outLevel)
                            tmpRow[i] = false;
                    }
                    else
                    {
                        if (value > inLevel)
                        {
                            tmpRow[i] = true;
                            ++output[1][i];
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="linesNum"></param>
        /// <param name="ratio">Ratio of softLines and linesNum</param>
        /// <param name="threshold"></param>
        /// <returns></returns>
        public double[] GetSymmetryWeights(int linesNum, double ratio, double threshold = 0.99)
        {
            if (linesNum <= 0)
                throw new ArgumentOutOfRangeException("linesNum", "The linesNum should be positive");

            if (ratio < 0.0)
                throw new ArgumentOutOfRangeException("ratio", "The ratio should be greater than or equal zero");

            if (threshold < 0.0 || threshold > 1.0)
                throw new ArgumentOutOfRangeException("threshold", "The threshold should lies within [0, 1] interval");

            double[] weights = new double[linesNum];

            int center = linesNum / 2;
            int softNum = (int)(0.5 * ratio * linesNum);

            double k = Math.Log((1.0 + threshold) / (1.0 - threshold)) / softNum;

            if (double.IsPositiveInfinity(k))
                k = double.MaxValue;

            for (int i = 0; i < weights.Length; ++i)
            {
                weights[i] = Math.Tanh(k * (i - center));
            }

            return weights;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="linesNum"></param>
        /// <param name="ratio">Ratio of centerLines and linesNum</param>
        /// <param name="exponent"></param>
        /// <param name="threshold"></param>
        /// <returns></returns>
        public double[] GetCenterWeights(int linesNum, double ratio, double exponent = 1.0, double threshold = 0.01)
        {
            if (linesNum <= 0)
                throw new ArgumentOutOfRangeException("linesNum", "The linesNum should be positive");

            if (ratio < 0.0)
                throw new ArgumentOutOfRangeException("ratio", "The ratio should be greater than or equal zero");

            if (threshold < 0.0 || threshold > 1.0)
                throw new ArgumentOutOfRangeException("threshold", "The threshold should lies within [0, 1] interval");

            double[] weights = new double[linesNum];

            int center = linesNum / 2;
            int centerNum = (int)(0.5 * ratio * linesNum);

            double k = Math.Pow(1.0 / threshold - 1.0, 0.5 / exponent) / centerNum;

            k *= k;

            if (double.IsPositiveInfinity(k))
                k = double.MaxValue;

            for (int i = 0; i < weights.Length; ++i)
            {
                int x = i - center;
                x *= x;

                weights[i] = 1.0 / (1.0 + Math.Pow(k * x, exponent));
            }

            return weights;
        }

        /// <summary>
        /// Range (-0.5, 0.5)
        /// </summary>
        /// <returns></returns>
        public double GetSymmetryFactor(int[] input, double[] weights, out int sum)
        {
            sum = 0;
            double weightedSum = 0.0;

            for (int i = 0; i < input.Length; ++i)
            {
                var value = input[i];

                sum += value;
                weightedSum += value * weights[i];
            }

            return weightedSum / sum;
        }
    }
}
