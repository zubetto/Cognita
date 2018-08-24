using Cognita;
using Cognita.SupervisedLearning;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ModelFunctions
{
    public static class CsvConverter
    {
        public static void Read(string line, ref ValueTuple<int, int[], int> LabelPointMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;

            // the first column contains labels of digits
            do
                ind = line.IndexOf(delim, ++pos);
            while (ind == pos);

            int val = int.Parse(line.Substring(pos, ind - pos));

            LabelPointMax.Item1 = val;
            LabelPointMax.Item3 = int.MinValue;

            pos = ind;

            int endex = LabelPointMax.Item2.Length - 1;

            // the remaining columns contain features
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = int.Parse(line.Substring(pos, ind - pos));

                LabelPointMax.Item2[j] = val;

                if (val > LabelPointMax.Item3)
                    LabelPointMax.Item3 = val;

                pos = ind;
            }

            // the last column
            val = int.Parse(line.Substring(++pos));

            LabelPointMax.Item2[endex] = val;

            if (val > LabelPointMax.Item3)
                LabelPointMax.Item3 = val;

            return;
        }

        public static void Read(string line, ref ValueTuple<int[], int> PointMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;
            int val;
            
            PointMax.Item2 = int.MinValue;

            int endex = PointMax.Item1.Length - 1;

            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = int.Parse(line.Substring(pos, ind - pos));

                PointMax.Item1[j] = val;

                if (val > PointMax.Item2)
                    PointMax.Item2 = val;

                pos = ind;
            }

            // the last column
            val = int.Parse(line.Substring(++pos));

            PointMax.Item1[endex] = val;

            if (val > PointMax.Item2)
                PointMax.Item2 = val;

            return;
        }

        public static void Read(string line, int[] point, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;
            int val;

            int endex = point.Length - 1;

            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = int.Parse(line.Substring(pos, ind - pos));

                point[j] = val;

                pos = ind;
            }

            // the last column
            val = int.Parse(line.Substring(++pos));

            point[endex] = val;

            return;
        }

        public static void Read(string line, ref ValueTuple<int, double[], double> LabelPointMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;

            // the first column contains labels of digits
            do
                ind = line.IndexOf(delim, ++pos);
            while (ind == pos);

            double val;

            LabelPointMax.Item1 = int.Parse(line.Substring(pos, ind - pos));
            LabelPointMax.Item3 = double.NegativeInfinity;

            pos = ind;

            int endex = LabelPointMax.Item2.Length - 1;

            // the remaining columns contain features
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                LabelPointMax.Item2[j] = val;

                if (val > LabelPointMax.Item3)
                    LabelPointMax.Item3 = val;

                pos = ind;
            }

            // the last column
            val = double.Parse(line.Substring(++pos));

            LabelPointMax.Item2[endex] = val;

            if (val > LabelPointMax.Item3)
                LabelPointMax.Item3 = val;

            return;
        }

        public static void Read(string line, ref ValueTuple<double[], double> PointMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;
            double val;

            PointMax.Item2 = double.NegativeInfinity;

            int endex = PointMax.Item1.Length - 1;

            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                PointMax.Item1[j] = val;

                if (val > PointMax.Item2)
                    PointMax.Item2 = val;

                pos = ind;
            }

            // the last column
            val = double.Parse(line.Substring(++pos));

            PointMax.Item1[endex] = val;

            if (val > PointMax.Item2)
                PointMax.Item2 = val;

            return;
        }

        public static void Read(string line, double[] point, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;
            double val;

            int endex = point.Length - 1;

            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                point[j] = val;

                pos = ind;
            }

            // the last column
            val = double.Parse(line.Substring(++pos));

            point[endex] = val;

            return;
        }

        public static void Read(string line, ref ValueTuple<int, double[], double[], double[]> LabelPointMinMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;

            // --- the first column contains labels of digits --------------
            do
                ind = line.IndexOf(delim, ++pos);
            while (ind == pos);

            LabelPointMinMax.Item1 = int.Parse(line.Substring(pos, ind - pos));

            pos = ind;

            double val;
            int endex = LabelPointMinMax.Item2.Length - 1;

            // --- the remaining columns contain features -------------------
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                LabelPointMinMax.Item2[j] = val;

                if (val < LabelPointMinMax.Item3[j])
                    LabelPointMinMax.Item3[j] = val;
                else if (val > LabelPointMinMax.Item4[j])
                    LabelPointMinMax.Item4[j] = val;

                pos = ind;
            }

            // read the last column
            val = double.Parse(line.Substring(++pos));

            LabelPointMinMax.Item2[endex] = val;

            if (val < LabelPointMinMax.Item3[endex])
                LabelPointMinMax.Item3[endex] = val;
            else if (val > LabelPointMinMax.Item4[endex])
                LabelPointMinMax.Item4[endex] = val;

            return;
        }

        public static void ReadIniMinMax(string line, ref ValueTuple<int, double[], double[], double[]> LabelPointMinMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;

            // --- the first column contains labels of digits --------------
            do
                ind = line.IndexOf(delim, ++pos);
            while (ind == pos);

            LabelPointMinMax.Item1 = int.Parse(line.Substring(pos, ind - pos));

            pos = ind;

            double val;
            int endex = LabelPointMinMax.Item2.Length - 1;

            // --- the remaining columns contain features -------------------
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                LabelPointMinMax.Item2[j] = val;
                LabelPointMinMax.Item3[j] = val;
                LabelPointMinMax.Item4[j] = val;

                pos = ind;
            }

            // read the last column
            val = double.Parse(line.Substring(++pos));

            LabelPointMinMax.Item2[endex] = val;
            LabelPointMinMax.Item3[endex] = val;
            LabelPointMinMax.Item4[endex] = val;

            return;
        }

        public static void Read(string line, ref ValueTuple<double[], double[], double[]> PointMinMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;

            double val;
            int endex = PointMinMax.Item1.Length - 1;

            // --- the remaining columns contain features -------------------
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                PointMinMax.Item1[j] = val;

                if (val < PointMinMax.Item2[j])
                    PointMinMax.Item2[j] = val;
                else if (val > PointMinMax.Item3[j])
                    PointMinMax.Item3[j] = val;

                pos = ind;
            }

            // read the last column
            val = double.Parse(line.Substring(++pos));

            PointMinMax.Item1[endex] = val;

            if (val < PointMinMax.Item2[endex])
                PointMinMax.Item2[endex] = val;
            else if (val > PointMinMax.Item3[endex])
                PointMinMax.Item3[endex] = val;

            return;
        }

        public static void ReadIniMinMax(string line, ref ValueTuple<double[], double[], double[]> PointMinMax, char delim = ',')
        {
            int ind = 0;
            int pos = line[0] == '\uFEFF' ? 0 : -1;

            double val;
            int endex = PointMinMax.Item1.Length - 1;

            // --- the remaining columns contain features -------------------
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(delim, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                PointMinMax.Item1[j] = val;
                PointMinMax.Item2[j] = val;
                PointMinMax.Item3[j] = val;

                pos = ind;
            }

            // read the last column
            val = double.Parse(line.Substring(++pos));

            PointMinMax.Item1[endex] = val;
            PointMinMax.Item2[endex] = val;
            PointMinMax.Item3[endex] = val;

            return;
        }

        public static void Write(StringBuilder line, ValueTuple<int, double[]> LabelPoint, char delim = '|')
        {
            line.Clear();

            line.Append(LabelPoint.Item1);

            for (int i = 0; i < LabelPoint.Item2.Length; ++i)
                line.AppendFormat("{0}{1}", delim, LabelPoint.Item2[i]);
        }

        public static void Write(StringBuilder line, ValueTuple<int, int[][]> LabelPoints, char delim = '|')
        {
            line.Clear();

            line.Append(LabelPoints.Item1);

            for (int i = 0; i < LabelPoints.Item2.Length; ++i)
            {
                int[] tmp = LabelPoints.Item2[i];

                for (int j = 0; j < tmp.Length; ++j)
                    line.AppendFormat("{0}{1}", delim, tmp[j]);
            }
        }

        public static void Write(StringBuilder line, ValueTuple<int, double[][]> LabelPoints, char delim = '|')
        {
            line.Clear();

            line.Append(LabelPoints.Item1);

            for (int i = 0; i < LabelPoints.Item2.Length; ++i)
            {
                double[] tmp = LabelPoints.Item2[i];

                for (int j = 0; j < tmp.Length; ++j)
                    line.AppendFormat("{0}{1}", delim, tmp[j]);
            }
        }
    }

    /// <summary>
    /// All lines of a file are parsed and loaded to the RAM
    /// </summary>
    public class PostalServiceZipCode : IPointsSource
    {
        private string dataFilename;

        // --- Data config ------------
        private int dimension;
        private int posDigit;
        private int negDigit;
        private char separator;
        private double testNumRatio;
        
        private int endex;
        private bool isOneVsAllconfig;
        private int startPos;

        // --- Set size ---------------
        private int totalNum;
        private int trainingNum;
        private int testNum;

        // --- Data range -------------
        private double[] pointO;
        private double[] pointR;
        private double[] deltaRO;

        private void ReadConfig(string filename)
        {
            /* Config file structure:
             * dimension                // 0 - int
             * positiveDigit            // 1 - digit that should be labled with true
             * negativeDigit            // 2 - digit that should be labled with false; set -1 for the "one-vs-all"
             * 'separator'              // 3 - character enclosed in single quotes
             * testNumRatio             // 4 - defines the portion of points used for testing
             */

            const int cfgNum = 5;

            StreamReader SR = null;

            try
            {
                SR = new StreamReader(filename);

                for (int i = 0; i < cfgNum; i++)
                {
                    string line = SR.ReadLine();

                    if (line == null)
                        throw new FileFormatException("Insufficient number of lines in the data config file");

                    int trimInd = line.IndexOf("//");

                    if (trimInd > 0)
                        line = line.Substring(0, trimInd);
                    
                    line = line.Trim(' ', '\t');

                    char[] charr = line.ToCharArray();

                    switch (i)
                    {
                        case 0:
                            dimension = int.Parse(line);
                            endex = dimension - 1;
                            break;

                        case 1:
                            posDigit = int.Parse(line);
                            break;

                        case 2:
                            negDigit = int.Parse(line);
                            isOneVsAllconfig = negDigit < 0;
                            break;

                        case 3:
                            line = line.Trim('\'');
                            separator = char.Parse(line);
                            break;

                        case 4:
                            testNumRatio = double.Parse(line);

                            if (testNumRatio < 0.0 || testNumRatio >= 1.0)
                                throw new ArgumentException("The ratio should lie in interval (0, 1)");
                            break;
                    }
                }
            }
            finally
            {
                if (SR != null) SR.Close();
            }
        }

        public PostalServiceZipCode(string configFile, string dataFile)
        {
            ReadConfig(configFile);

            dataFilename = dataFile;

            StreamReader SR = null;

            try
            {
                SR = new StreamReader(dataFile);

                // --- define size and range -------------------------
                pointO = Enumerable.Repeat(0.0, dimension).ToArray();
                pointR = Enumerable.Repeat(0.0, dimension).ToArray();
                deltaRO = new double[dimension];

                string line;

                while ((line = SR.ReadLine()) != null && !SurveyFirstLine(line)) ;

                while ((line = SR.ReadLine()) != null)
                    SurveyLine(line);

                pointR.Subtract(pointO, deltaRO);

                testNum = (int)(testNumRatio * totalNum);
                trainingNum = totalNum - testNum;

                // --- load all valid lines ---------------------
                sourceTrainPoints = new double[trainingNum][];
                sourceTrainLabels = new bool[trainingNum];

                sourceTestPoints = new double[testNum][];
                sourceTestLabels = new bool[testNum];

                //SR.DiscardBufferedData();
                //SR.BaseStream.Seek(0, SeekOrigin.Begin);
                if (SR != null) SR.Close();

                SR = new StreamReader(dataFile);

                int i = 0;
                bool nextFlag = false;

                // --- load training data ---
                sourceTrainPoints[0] = new double[dimension];

                while (!nextFlag || ++i < trainingNum)
                {
                    if (nextFlag)
                        sourceTrainPoints[i] = new double[dimension];

                    nextFlag = LoadLine(SR.ReadLine(), ref sourceTrainLabels[i], sourceTrainPoints[i]);
                }

                // --- load test data ---
                i = 0;
                nextFlag = false;
                sourceTestPoints[0] = new double[dimension];

                while (!nextFlag || ++i < testNum)
                {
                    if (nextFlag)
                        sourceTestPoints[i] = new double[dimension];

                    nextFlag = LoadLine(SR.ReadLine(), ref sourceTestLabels[i], sourceTestPoints[i]);
                }

                //// ### Debug
                //double min0 = Math.Min(sourceTrainPoints.Min(p => p[0]), sourceTestPoints.Min(p => p[0]));
                //double max0 = Math.Max(sourceTrainPoints.Max(p => p[0]), sourceTestPoints.Max(p => p[0]));

                //double min1 = Math.Min(sourceTrainPoints.Min(p => p[1]), sourceTestPoints.Min(p => p[1]));
                //double max1 = Math.Max(sourceTrainPoints.Max(p => p[1]), sourceTestPoints.Max(p => p[1]));
            }
            finally
            {
                if (SR != null) SR.Close();
            }
        }

        // --- Line Survey ---
        private bool SurveyFirstLine(string line)
        {
            startPos = line[0] == '\uFEFF' ? 0 : -1;

            int ind = 0;
            int pos = startPos;

            // the first column contains digits
            do
                ind = line.IndexOf(separator, ++pos);
            while (ind == pos);
            
            if (!double.TryParse(line.Substring(pos, ind - pos), out double val))
                return false;

            if (!isOneVsAllconfig)
            {
                int digit = (int)val;

                if (digit != posDigit && digit != negDigit)
                    return false;
            }

            ++totalNum;

            pos = ind;

            // the remaining columns contain features
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(separator, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                pointO[j] = val;
                pointR[j] = val;

                pos = ind;
            }

            // the last column
            val = double.Parse(line.Substring(++pos));

            pointO[endex] = val;
            pointR[endex] = val;

            return true;
        }

        private void SurveyLine(string line)
        {
            int ind = 0;
            int pos = startPos;

            // the first column contains digits
            do
                ind = line.IndexOf(separator, ++pos);
            while (ind == pos);

            if (!double.TryParse(line.Substring(pos, ind - pos), out double val))
                return;

            if (!isOneVsAllconfig)
            {
                int digit = (int)val;

                if (digit != posDigit && digit != negDigit)
                    return;
            }

            ++totalNum;

            pos = ind;

            // the remaining columns contain features
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(separator, ++pos);
                while (ind == pos);
                
                val = double.Parse(line.Substring(pos, ind - pos));

                if (val < pointO[j])
                    pointO[j] = val;
                else if (val > pointR[j])
                    pointR[j] = val;

                pos = ind;
            }

            // the last column
            val = double.Parse(line.Substring(++pos));

            if (val < pointO[endex])
                pointO[endex] = val;
            else if (val > pointR[endex])
                pointR[endex] = val;
        }

        // --- Load Line ---
        private bool LoadLine(string line, ref bool label, double[] point)
        {
            int ind = 0;
            int pos = startPos;

            // the first column contains digits
            do
                ind = line.IndexOf(separator, ++pos);
            while (ind == pos);

            if (!double.TryParse(line.Substring(pos, ind - pos), out double val))
                return false;

            int digit = (int)val;

            if (digit == posDigit)
                label = true;
            else if (isOneVsAllconfig || digit == negDigit)
                label = false;
            else
                return false;

            pos = ind;

            // the remaining columns contain features
            for (int j = 0; j < endex; j++)
            {
                do
                    ind = line.IndexOf(separator, ++pos);
                while (ind == pos);

                val = double.Parse(line.Substring(pos, ind - pos));

                point[j] = (val - pointO[j]) / deltaRO[j];

                pos = ind;
            }

            // the last column
            val = double.Parse(line.Substring(++pos));

            point[endex] = (val - pointO[endex]) / deltaRO[endex];

            return true;
        }
        
        private void CopyTrainingData(int srcShift, int batchShift)
        {
            int num = batchLength - batchShift;

            // copy points
            for (int n = srcShift, i = batchShift; i < loadedPoints.Length; ++n, ++i)
                loadedPoints[i] = sourceTrainPoints[n];

            // copy labels
            num *= sizeof(bool);
            srcShift *= sizeof(bool);
            batchShift *= sizeof(bool);

            Buffer.BlockCopy(sourceTrainLabels, srcShift, loadedLabels, batchShift, num);
        }

        private void CopyTestData(int srcShift, int batchShift)
        {
            int num = batchLength - batchShift;

            // copy points
            for (int n = srcShift, i = batchShift; i < loadedPoints.Length; ++n, ++i)
                loadedPoints[i] = sourceTestPoints[n];

            // copy labels
            num *= sizeof(bool);
            srcShift *= sizeof(bool);
            batchShift *= sizeof(bool);

            Buffer.BlockCopy(sourceTestLabels, srcShift, loadedLabels, batchShift, num);
        }

        //--------------------------------------------------------------------
        #region IPointsSource Implementation

        private int batchLength;

        private double[][] sourceTrainPoints;
        private bool[] sourceTrainLabels;

        private double[][] sourceTestPoints;
        private bool[] sourceTestLabels;

        private double[][] loadedPoints;
        private bool[] loadedLabels;
        
        private int posNum;
        private int negNum;

        private int loadedLength = 0;
        private DataSet loadedSet = DataSet.None;

        // indexes { curr, next }
        private int[] indsTraining = new int[2] { -1, 0 };
        private int[] indsTest = new int[2] { -1, 0 };

        private void SetBatchLength(int value)
        {
            bool[] labelsTmp = new bool[value];
            double[][] pointsTmp = new double[value][];

            if (batchLength == 0)
            {
                for (int i = 0; i < pointsTmp.Length; i++)
                    pointsTmp[i] = new double[dimension];
            }
            else if (value < batchLength)
            {
                Buffer.BlockCopy(loadedLabels, 0, labelsTmp, 0, value * sizeof(bool));

                for (int i = 0; i < pointsTmp.Length; i++)
                    pointsTmp[i] = loadedPoints[i];
            }
            else if (value > batchLength)
            {
                Buffer.BlockCopy(loadedLabels, 0, labelsTmp, 0, batchLength * sizeof(bool));

                for (int i = 0; i < loadedPoints.Length; i++)
                    pointsTmp[i] = loadedPoints[i];

                for (int i = batchLength; i < pointsTmp.Length; i++)
                    pointsTmp[i] = new double[dimension];
            }

            loadedLabels = labelsTmp;
            loadedPoints = pointsTmp;

            batchLength = value;

            GC.Collect();
        }

        int IPointsSource.Dimension { get { return dimension; } }

        int IPointsSource.BatchLength
        {
            get { return batchLength; }
            set
            {
                if (value <= 0 || value == batchLength)
                    return;

                SetBatchLength(value);
            }
        }

        bool IPointsSource.SeekNext(DataSet set, int index)
        {
            if (index < 0)
                throw new ArgumentOutOfRangeException("index", "The index should be non negative");

            switch (set)
            {
                case DataSet.Training:
                    if (index >= trainingNum)
                        return false;

                    indsTraining[1] = index;
                    return true;

                case DataSet.Test:
                    if (index >= testNum)
                        return false;

                    indsTest[1] = index;
                    return true;

                default:
                    return false;
            }
        }

        int IPointsSource.GetCurrentIndex(DataSet set)
        {
            switch (set)
            {
                case DataSet.Training:
                    return indsTraining[0];

                case DataSet.Test:
                    return indsTest[0];

                default:
                    return -1;
            }
        }

        int IPointsSource.GetNextIndex(DataSet set)
        {
            switch (set)
            {
                case DataSet.Training:
                    return indsTraining[1];

                case DataSet.Test:
                    return indsTest[1];

                default:
                    return 0;
            }
        }

        int IPointsSource.PositiveNum { get { return posNum; } }
        int IPointsSource.NegativeNum { get { return negNum; } }

        bool IPointsSource.GetCurrentNorm(DataSet set, ref double[][] points, ref bool[] labels)
        {
            if (batchLength == 0)
                return false;
            
            bool isFresh = false;

            switch (set)
            {
                case DataSet.Training:
                    if (indsTraining[0] < 0)
                        return false;
                    
                    int maxLength = trainingNum - indsTraining[0];

                    if (batchLength > maxLength)
                        SetBatchLength(maxLength);

                    if (set != loadedSet)
                        CopyTrainingData(indsTraining[0], 0);
                    else if (batchLength > loadedLength)
                        CopyTrainingData(indsTraining[0] + loadedLength, loadedLength);
                    else if (batchLength == loadedLength)
                        isFresh = true;

                    indsTraining[1] = indsTraining[0] + batchLength;
                    break;

                case DataSet.Test:
                    if (indsTest[0] < 0)
                        return false;

                    maxLength = testNum - indsTest[0];

                    if (batchLength > maxLength)
                        SetBatchLength(maxLength);

                    if (set != loadedSet)
                        CopyTestData(indsTest[0], 0);
                    else if (batchLength > loadedLength)
                        CopyTestData(indsTest[0] + loadedLength, loadedLength);
                    else if (batchLength == loadedLength)
                        isFresh = true;

                    indsTest[1] = indsTest[0] + batchLength;
                    break;

                default:
                    return false;
            }

            if (!isFresh)
            {
                posNum = 0;
                negNum = 0;

                for (int i = 0; i < loadedLabels.Length; i++)
                {
                    if (loadedLabels[i])
                        ++posNum;
                    else
                        ++negNum;
                }
            }
            
            points = loadedPoints;
            labels = loadedLabels;

            loadedSet = set;
            loadedLength = batchLength;
            
            return true;
        }

        bool IPointsSource.GetNextNorm(DataSet set, ref double[][] points, ref bool[] labels)
        {
            int maxLength;

            switch (set)
            {
                case DataSet.Training:
                    if (indsTraining[1] >= trainingNum)
                        return false;

                    maxLength = trainingNum - indsTraining[1];

                    if (batchLength > maxLength)
                        SetBatchLength(maxLength);

                    CopyTrainingData(indsTraining[1], 0);

                    indsTraining[0] = indsTraining[1];
                    indsTraining[1] += batchLength;
                    break;

                case DataSet.Test:
                    if (indsTest[1] >= testNum)
                        return false;

                    maxLength = testNum - indsTest[1];

                    if (batchLength > maxLength)
                        SetBatchLength(maxLength);

                    CopyTestData(indsTest[1], 0);

                    indsTest[0] = indsTest[1];
                    indsTest[1] += batchLength;
                    break;

                default:
                    return false;
            }

            posNum = 0;
            negNum = 0;

            for (int i = 0; i < loadedLabels.Length; i++)
            {
                if (loadedLabels[i])
                    ++posNum;
                else
                    ++negNum;
            }

            points = loadedPoints;
            labels = loadedLabels;

            loadedSet = set;
            loadedLength = batchLength;

            return true;
        }

        double IPointsSource.Classify(double[] point) { return double.NaN; }

        #endregion
    }
}
