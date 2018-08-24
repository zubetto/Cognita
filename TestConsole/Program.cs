using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Cognita;
using Cognita.SupervisedLearning;
using ModelFunctions;
using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;
using RBF = Cognita.SupervisedLearning.RadialBasisFunction;

namespace TestConsole
{
    public static class ConsoleIO
    {
        public static void IONumeric(string inputDescription, ref int iniValue, Predicate<int> Constraint = null)
        {
            Console.Write(inputDescription);

            string input = Console.ReadLine();
            int inputNum;

            if (int.TryParse(input, out inputNum) && (Constraint == null || Constraint(inputNum)))
                iniValue = inputNum;

            Console.SetCursorPosition(inputDescription.Length, Console.CursorTop - 1);
            Console.Write(new string(' ', input.Length));
            Console.CursorLeft -= input.Length;
            Console.WriteLine(iniValue);
        }

        public static void IONumeric(string inputDescription, ref double iniValue, Predicate<double> Constraint = null)
        {
            Console.Write(inputDescription);

            string input = Console.ReadLine();
            double inputNum;

            if (double.TryParse(input, out inputNum) && (Constraint == null || Constraint(inputNum)))
                iniValue = inputNum;

            Console.SetCursorPosition(inputDescription.Length, Console.CursorTop - 1);
            Console.Write(new string(' ', input.Length));
            Console.CursorLeft -= input.Length;
            Console.WriteLine(iniValue);
        }

        public static void IOString(string inputDescription, ref char iniValue, Predicate<string> Constraint = null)
        {
            Console.Write(inputDescription);

            string input = Console.ReadLine();

            if (Constraint == null || Constraint(input))
                iniValue = input[0];

            Console.SetCursorPosition(inputDescription.Length, Console.CursorTop - 1);
            Console.Write(new string(' ', input.Length));
            Console.CursorLeft -= input.Length;
            Console.WriteLine(iniValue);
        }

        public static void IOString(string inputDescription, ref string iniValue, Predicate<string> Constraint = null)
        {
            Console.Write(inputDescription);

            string input = Console.ReadLine();

            if (Constraint == null || Constraint(input))
                iniValue = input;

            Console.SetCursorPosition(inputDescription.Length, Console.CursorTop - 1);
            Console.Write(new string(' ', input.Length));
            Console.CursorLeft -= input.Length;
            Console.WriteLine(iniValue);
        }
    }

    class Program
    {
        static void Main(string[] args)
        {
            double Dx = 128;
            double Dy = 10;
            double xo = 64;
            double yo = 5;
            double a = 10;
            double k = 10;
            double kx = k / Dx;
            double ky = k / Dy;
            double px = 1;
            double py = 1;

            Func<AdaptiveGrid<double[]>.IVoxel, double[]> ignitor =
                (V) => new double[] { V.Origin[0] + 0.5 * V.GridStep[0], V.Origin[1] + 0.5 * V.GridStep[1] };

            Func<IVoxel, double> pulse = delegate (IVoxel V)
            {
                double x = V.Origin[0] + 0.5 * V.GridStep[0];
                double y = V.Origin[1] + 0.5 * V.GridStep[1];

                x = kx * Math.Pow(x - xo, 2 * px);
                y = ky * Math.Pow(y - yo, 2 * py);

                return a / (x + y + 1);
            };

            int TESTnO = 11;

            #region 0_Simple_2dGrid
            if (TESTnO == 0)
            {
                double[,] shape = new double[2, 3] { { 0, Dx, 4 }, { 0, Dy, 4 } };
                var agrid = new AdaptiveGrid<double>(32, shape, new int[] { 2, 2 }, 4, pulse);

                // --- Traversal ---
                var iV = agrid.GetInnermost(agrid.InternalArray[0]);
                int index = 0;

                Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6}", "x", "y", "value", "ind");

                do
                {
                    Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6:N2}", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextInnermost(ref iV));

                Console.WriteLine("----------------------------------------\n\r");


                // --- Tessellation ---
                agrid.Tessellate(new double[] { 60, 4 }, initializer: pulse);

                iV = agrid.GetInnermost(agrid.InternalArray[0]);
                index = 0;

                do
                {
                    Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6:N2}", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextInnermost(ref iV));

                Console.WriteLine("----------------------------------------\n\r");


                // --- Tessellation ---
                agrid.Tessellate(new double[] { 100, 6 }, initializer: pulse);

                iV = agrid.GetInnermost(agrid.InternalArray[0]);
                index = 0;

                do
                {
                    Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6:N2}", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextInnermost(ref iV));

                Console.WriteLine("----------------------------------------\n\r");


                // --- Tessellation ---
                agrid.Tessellate(new double[] { 60, 4 }, initializer: pulse);
                agrid.Tessellate(new double[] { 60, 4 }, initializer: pulse);
                agrid.Tessellate(new double[] { 60, 4 }, initializer: pulse);
                agrid.Tessellate(new double[] { 60, 6 }, initializer: pulse);

                iV = agrid.GetInnermost(agrid.InternalArray[0]);
                index = 0;

                do
                {
                    Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6:N2}", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextInnermost(ref iV));

                Console.WriteLine("----------------------------------------\n\r");


                // --- Merge ---
                agrid.InternalArray[5].Merge_R(true, pulse);

                iV = agrid.GetInnermost(agrid.InternalArray[0]);
                index = 0;

                do
                {
                    Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6:N2}", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextInnermost(ref iV));

                Console.WriteLine("----------------------------------------\n\r");


                // --- Traversal of the specified level ---
                bool tessFlag = false;
                tessFlag = agrid.Tessellate(new double[] { 0, 0 }, initializer: pulse);
                tessFlag = agrid.Tessellate(new double[] { 0, 0 }, initializer: pulse);
                tessFlag = agrid.Tessellate(new double[] { 0, 0 }, initializer: pulse);
                tessFlag = agrid.Tessellate(new double[] { 0, 0 }, initializer: pulse);

                tessFlag = agrid.Tessellate(new double[] { 97, 8 }, initializer: pulse);
                tessFlag = agrid.Tessellate(new double[] { 97, 8 }, initializer: pulse);
                tessFlag = agrid.Tessellate(new double[] { 97, 8 }, initializer: pulse);
                tessFlag = agrid.Tessellate(new double[] { 97, 8 }, initializer: pulse);

                int ilevel = 4;
                index = 0;

                if (!agrid.FirstAtLevel(ilevel, out iV))
                {
                    Console.WriteLine("There are no voxels at {0} level", ilevel);
                    Console.WriteLine("----------------------------------------\n\r");
                    goto END;
                }

                do
                {
                    Console.WriteLine("{3,3},   {0,4}, {1,4}, {2,6:N2}", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextAtLevel(ref iV));

                Console.WriteLine("----------------------------------------\n\r");
            }
            #endregion


            #region 1_Dense_2dGrid
            if (TESTnO == 1)
            {
                double[,] shape = new double[2, 3] { { 0, Dx, 16 }, { 0, Dy, 16 } };
                var agrid = new AdaptiveGrid<double>(2*16*16, shape, null, 4, pulse);

                // --- Tessellation ---
                agrid.Tessellate(new double[] { 62, 4.5 }, initializer: pulse);
                agrid.Tessellate(new double[] { 65, 4.5 }, initializer: pulse);
                agrid.Tessellate(new double[] { 62, 5.1 }, initializer: pulse);
                agrid.Tessellate(new double[] { 65, 5.1 }, initializer: pulse);

                var iV = agrid.GetInnermost(agrid.InternalArray[0]);
                int index = 0;

                StringBuilder SB = new StringBuilder(33 * 1000);
                SB.AppendFormat("{3,3},   {0,8}, {1,8}, {2,8}\n\r", "x", "y", "value", "ind");

                do
                {
                    SB.AppendFormat("{3,3},   {0,8:G6}, {1,8:G6}, {2,8:G6}\n\r", iV.Origin[0], iV.Origin[1], iV.Data, index++);
                }
                while (agrid.NextInnermost(ref iV));

                SB.AppendLine("----------------------------------------");
                Console.WriteLine(SB.ToString());
            }
            #endregion


            #region 2_RadialFunction
            if (TESTnO == 2)
            {
                RBF.Amplitude = 0.1;
                RBF.Factor = 25;
                RBF.Exponent = 1.0;

                StringBuilder strOut = new StringBuilder(256);
                string input = "";
                double tol = 0.5;

                while (input != "y")
                {
                    Console.Write("Enter tolerance: ");
                    input = Console.ReadLine();

                    if (!double.TryParse(input, out tol)) continue;

                    RBF.CalcRanges(0.0085, 1.0, tol);

                    strOut.Clear();
                    strOut.AppendLine("  Ranges:");

                    for (int i = 0; i < RBF.Ranges.Count; i++)
                    {
                        strOut.AppendFormat("{0,8:N6}, ", RBF.Ranges[i]);
                    }

                    strOut.AppendLine("\n\n\r  Deltas:");
                    int inflectInd = -1;
                    double prev = RBF.Ranges[1];

                    for (int i = 1; i < RBF.Ranges.Count; i++)
                    {
                        double d = RBF.Ranges[i] - RBF.Ranges[i - 1];
                        strOut.AppendFormat("{0,8:N6}, ", d);

                        if (inflectInd < 0 && d > prev)
                            inflectInd = i - 2;

                        prev = d;
                    }

                    strOut.AppendFormat("\n\rinflection: {0}\n\r", inflectInd);

                    strOut.AppendLine("\n\r  Widths:");
                    inflectInd = -1;
                    prev = RBF.Widths[0];

                    for (int i = 0; i < RBF.Widths.Count; i++)
                    {
                        strOut.AppendFormat("{0,8:N6}, ", RBF.Widths[i]);

                        if (inflectInd < 0 && RBF.Widths[i] > prev)
                            inflectInd = i - 1;

                        prev = RBF.Widths[i];
                    }

                    strOut.AppendFormat("\n\rinflection: {0}\n\r", inflectInd);

                    Console.WriteLine(strOut.ToString());
                    Console.Write("\n\rexit?: ");
                    input = Console.ReadLine();
                    Console.WriteLine("------------------\n\r");
                }
            }
            #endregion


            #region 3_NormIncrement
            if (TESTnO == 3)
            {
                const int dim = 7;

                double[,] shape = new double[dim, 3];

                for (int i = 0; i < dim; i++)
                {
                    shape[i, 0] = 0.0;
                    shape[i, 1] = 1.0;
                    shape[i, 2] = 2.0;
                }
                
                int[] tess = Enumerable.Repeat(2, dim).ToArray();

                AdaptiveGrid<double> agrid = new AdaptiveGrid<double>(1, shape, tess);

                var increment = agrid.GetIncrements(0);
                var serialIncr = agrid.GetSerialIncrements(0);
                var point = new double[dim];
                var center0 = new double[dim];
                var center1 = new double[dim];
                var isTessellated = new bool[agrid.RootsNumber];

                // --- ini diagnostic tools -----------------
                var diagTimerStraight = new Stopwatch();
                var diagTimerIncr = new Stopwatch();
                var outputTimer = new Stopwatch();
                var rnd = new Random();
                var outputSB = new StringBuilder(200);

                void GetPoint(double[] r)
                {
                    for (int i = 0; i < r.Length; i++)
                        r[i] = shape[i, 0] + rnd.NextDouble() * (shape[i, 1] - shape[i, 0]);
                }

                int GetIndexRoot(int indMin = 0, int indMax = -1)
                {
                    indMax = indMax < 0 ? agrid.RootsNumber : indMax;

                    return rnd.Next(indMin, indMax);
                }

                //int GetIndex(int indMin = 0, int indMax = -1)
                //{
                //    indMax = indMax < 0 ? agrid.RootsNumber : indMax;

                //    return rnd.Next(indMin, agrid.TessellationMultp);
                //}

                // --- ini diag params ----------------------
                int outputLap = 333;
                int warmingNum = 10;
                int batchSize = 1000;

                double tessThold = 1.0;

                long avgTimeCounter = 0;
                long avgDeltaCounter = 0;

                bool straightNotIni = true;
                long straightMin = long.MinValue;
                long straightMax = long.MaxValue;
                long straightCur = 0;
                double straightAvg = 0.0;

                bool incrNotIni = true;
                long incrMin = long.MinValue;
                long incrMax = long.MaxValue;
                long incrCur = 0;
                double incrAvg = 0.0;

                bool deltaNotIni = true;
                double deltaMin = double.NegativeInfinity;
                double deltaMax = double.PositiveInfinity;
                double deltaCur = 0.0;
                double deltaAvg = 0.0;

                bool resumeFlag = true;

                // --- Test Methods --------------------------------------------------------
                int selectedMethod = 4;
                string[] methodNames = new string[5]
                {
                    "TestMethod_Rand", "TestMethod_RandDirect", "TestMethod_RandTrue", "TestMethod_SerialDirect", "TestMethod_SerialTrue"
                };

                // 0
                void TestMethod_Rand()
                {
                    for (int i = 0; i < batchSize; i++)
                    {
                        int SI0 = GetIndexRoot(indMax: agrid.RootsNumber - 2);
                        int SI1 = GetIndexRoot(indMin: SI0 + 1);

                        // --- straight calculation ----------------------------
                        diagTimerStraight.Start();
                        agrid.InternalArray[SI1].GetCenter(center1);
                        double straightD = point.SquaredDistance(center1);
                        diagTimerStraight.Stop();

                        // --- calculations with increment data in use ---------
                        agrid.InternalArray[SI0].GetCenter(center0);
                        point.VectorIncrement(center0, center0);
                        double incrD;

                        diagTimerIncr.Start();
                        incrD = center0.SquaredNormIncrement(increment[SI1][SI0]);
                        diagTimerIncr.Stop();

                        // calc difference between two methods
                        deltaCur = incrD - straightD;
                        deltaCur = deltaCur < 0 ? -deltaCur : deltaCur;

                        if (deltaCur < deltaMin) deltaMin = deltaCur;
                        else if (deltaCur > deltaMax) deltaMax = deltaCur;
                        else if (deltaNotIni)
                        {
                            deltaMin = deltaCur;
                            deltaMax = deltaCur;
                            deltaNotIni = false;
                        }

                        deltaAvg += (deltaCur - deltaAvg) / ++avgDeltaCounter;

                        // interrupt handling
                        if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                        {
                            Console.WriteLine("\n\r PAUSED\n\r");
                            Console.Write("Enter \"y\" to stop >>: ");

                            if (Console.ReadLine() == "y")
                            {
                                resumeFlag = false;
                                break;
                            }
                        }
                    }
                }

                // 1
                void TestMethod_RandDirect()
                {
                    for (int i = 0; i < batchSize; i++)
                    {
                        int SI0 = GetIndexRoot(indMax: agrid.RootsNumber - 2);
                        int SI1 = GetIndexRoot(indMin: SI0 + 1);

                        // --- straight calculation ----------------------------
                        agrid.InternalArray[SI1].GetCenter(center1);

                        diagTimerStraight.Start();
                        double straightD = point.SquaredDistance(center1);
                        diagTimerStraight.Stop();

                        // --- calculations with increment data in use ---------
                        agrid.InternalArray[SI0].GetCenter(center0);
                        point.VectorIncrement(center0, center0);
                        double incrD;
                        double[] deltaVector = increment[SI1][SI0];

                        diagTimerIncr.Start();
                        incrD = center0.SquaredNormIncrement(deltaVector);
                        diagTimerIncr.Stop();

                        // calc difference between two methods
                        deltaCur = incrD - straightD;
                        deltaCur = deltaCur < 0 ? -deltaCur : deltaCur;

                        if (deltaCur < deltaMin) deltaMin = deltaCur;
                        else if (deltaCur > deltaMax) deltaMax = deltaCur;
                        else if (deltaNotIni)
                        {
                            deltaMin = deltaCur;
                            deltaMax = deltaCur;
                            deltaNotIni = false;
                        }

                        deltaAvg += (deltaCur - deltaAvg) / ++avgDeltaCounter;

                        // interrupt handling
                        if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                        {
                            Console.WriteLine("\n\r PAUSED\n\r");
                            Console.Write("Enter \"y\" to stop >>: ");

                            if (Console.ReadLine() == "y")
                            {
                                resumeFlag = false;
                                break;
                            }
                        }
                    }
                }

                // 2
                void TestMethod_RandTrue()
                {
                    for (int i = 0; i < batchSize; i++)
                    {
                        // set random tess flags
                        for (int j = 0; j < isTessellated.Length; j++)
                        {
                            if (rnd.NextDouble() > tessThold) isTessellated[j] = true;
                            else isTessellated[j] = false;
                        }

                        // at least one should not be tessellated
                        isTessellated[rnd.Next(isTessellated.Length)] = false;

                        int SI0 = -1;

                        for (int SI1 = 0; SI1 < agrid.RootsNumber; SI1++)
                        {
                            if (isTessellated[SI1]) continue;

                            // --- straight calculation ----------------------------
                            diagTimerStraight.Start();

                            agrid.InternalArray[SI1].GetCenter(center1);
                            double straightD = point.SquaredDistance(center1);

                            diagTimerStraight.Stop();

                            // --- calculations with increment data in use ---------
                            double incrD;

                            if (SI0 < 0)
                            {
                                SI0 = SI1;

                                diagTimerIncr.Start();

                                agrid.InternalArray[SI0].GetCenter(center0);
                                incrD = point.SquaredDistanceAndDelta(center0);

                                diagTimerIncr.Stop();
                            }
                            else
                            {
                                diagTimerIncr.Start();
                                incrD = center0.SquaredNormIncrement(increment[SI1][SI0]);
                                diagTimerIncr.Stop();
                            }

                            // --- calc difference between two methods --------------------
                            deltaCur = incrD - straightD;
                            deltaCur = deltaCur < 0 ? -deltaCur : deltaCur;

                            if (deltaCur < deltaMin) deltaMin = deltaCur;
                            else if (deltaCur > deltaMax) deltaMax = deltaCur;
                            else if (deltaNotIni)
                            {
                                deltaMin = deltaCur;
                                deltaMax = deltaCur;
                                deltaNotIni = false;
                            }

                            deltaAvg += (deltaCur - deltaAvg) / ++avgDeltaCounter;
                        }
                        
                        // interrupt handling
                        if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                        {
                            Console.WriteLine("\n\r PAUSED\n\r");
                            Console.Write("Enter \"y\" to stop >>: ");

                            if (Console.ReadLine() == "y")
                            {
                                resumeFlag = false;
                                break;
                            }
                        }
                    }
                }

                // 3
                void TestMethod_SerialDirect()
                {
                    for (int i = 0; i < batchSize; i++)
                    {
                        const int SI0 = 0;

                        agrid.InternalArray[SI0].GetCenter(center0);
                        point.VectorIncrement(center0, center0);

                        for (int SI1 = 1; SI1 < agrid.RootsNumber; SI1++)
                        {
                            // --- straight calculation --------------------------
                            double straightD;
                            agrid.InternalArray[SI1].GetCenter(center1);

                            diagTimerStraight.Start();
                            straightD = point.SquaredDistance(center1);
                            diagTimerStraight.Stop();

                            // --- calculations with increment data in use -------
                            double incrD;

                            diagTimerIncr.Start();
                            incrD = center0.SquaredNormIncrement(serialIncr[SI1]);
                            diagTimerIncr.Stop();

                            // --- calc difference between two methods ------------
                            deltaCur = incrD - straightD;
                            deltaCur = deltaCur < 0 ? -deltaCur : deltaCur;

                            if (deltaCur < deltaMin) deltaMin = deltaCur;
                            else if (deltaCur > deltaMax) deltaMax = deltaCur;
                            else if (deltaNotIni)
                            {
                                deltaMin = deltaCur;
                                deltaMax = deltaCur;
                                deltaNotIni = false;
                            }

                            deltaAvg += (deltaCur - deltaAvg) / ++avgDeltaCounter;
                        }
                        
                        // interrupt handling
                        if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                        {
                            Console.WriteLine("\n\r PAUSED\n\r");
                            Console.Write("Enter \"y\" to stop >>: ");

                            if (Console.ReadLine() == "y")
                            {
                                resumeFlag = false;
                                break;
                            }
                        }
                    }
                }

                // 4
                void TestMethod_SerialTrue()
                {
                    for (int i = 0; i < batchSize; i++)
                    {
                        // set random tess flags
                        for (int j = 0; j < isTessellated.Length; j++)
                        {
                            if (rnd.NextDouble() > tessThold) isTessellated[j] = true;
                            else isTessellated[j] = false;
                        }

                        // at least one should not be tessellated
                        isTessellated[rnd.Next(isTessellated.Length)] = false;

                        // first portion of calculations with increment data in use
                        diagTimerIncr.Start();
                        agrid.InternalArray[0].GetCenter(center0);
                        point.VectorIncrement(center0, center0);
                        diagTimerIncr.Stop();

                        for (int SI1 = 0; SI1 < agrid.RootsNumber; SI1++)
                        {
                            if (isTessellated[SI1]) continue;

                            // --- straight calculation --------------------------
                            double straightD;

                            diagTimerStraight.Start();

                            agrid.InternalArray[SI1].GetCenter(center1);
                            straightD = point.SquaredDistance(center1);

                            diagTimerStraight.Stop();

                            // --- calculations with increment data in use -------
                            double incrD;
                            
                            if (SI1 == 0)
                            {
                                diagTimerIncr.Start();
                                incrD = center0.SquaredNorm();
                                diagTimerIncr.Stop();
                            }
                            else
                            {
                                diagTimerIncr.Start();
                                incrD = center0.SquaredNormIncrement(serialIncr[SI1]);
                                diagTimerIncr.Stop();
                            }
                            
                            // --- calc difference between two methods ------------
                            deltaCur = incrD - straightD;
                            deltaCur = deltaCur < 0 ? -deltaCur : deltaCur;

                            if (deltaCur < deltaMin) deltaMin = deltaCur;
                            else if (deltaCur > deltaMax) deltaMax = deltaCur;
                            else if (deltaNotIni)
                            {
                                deltaMin = deltaCur;
                                deltaMax = deltaCur;
                                deltaNotIni = false;
                            }

                            deltaAvg += (deltaCur - deltaAvg) / ++avgDeltaCounter;
                        }

                        // interrupt handling
                        if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                        {
                            Console.WriteLine("\n\r PAUSED\n\r");
                            Console.Write("Enter \"y\" to stop >>: ");

                            if (Console.ReadLine() == "y")
                            {
                                resumeFlag = false;
                                break;
                            }
                        }
                    }
                }

                // --- Start ---------------------------------------------------------------
                string header = string.Format("Dimension = {0} | TessMultp = {1} | Batch size = {2} | Method: {3}\n\n\r", 
                                               dim, agrid.RootsNumber, batchSize, methodNames[selectedMethod]);
                string footer = "\n\r------------\n\r RUNNING";
                
                Console.Write(header);
                Console.Write("Press any key to sart ; press esc to pause\n\r");
                Console.ReadKey(true);
                Console.Write(footer);
                
                outputTimer.Restart();

                while (resumeFlag)
                {
                    GetPoint(point);

                    straightCur = 0;
                    incrCur = 0;

                    switch (selectedMethod)
                    {
                        case 0:
                            TestMethod_Rand();
                            break;

                        case 1:
                            TestMethod_RandDirect();
                            break;

                        case 2:
                            TestMethod_RandTrue();
                            break;

                        case 3:
                            TestMethod_SerialDirect();
                            break;

                        case 4:
                            TestMethod_SerialTrue();
                            break;
                    }

                    if (warmingNum > 0)
                    {
                        --warmingNum;

                        diagTimerStraight.Reset();
                        diagTimerIncr.Reset();

                        continue;
                    }

                    avgTimeCounter++;

                    // straight timing
                    straightCur = diagTimerStraight.ElapsedMilliseconds;
                    diagTimerStraight.Reset();

                    straightAvg += (straightCur - straightAvg) / avgTimeCounter;

                    if (straightCur < straightMin) straightMin = straightCur;
                    else if (straightCur > straightMax) straightMax = straightCur;
                    else if (straightNotIni)
                    {
                        straightMin = straightCur;
                        straightMax = straightCur;
                        straightNotIni = false;
                    }
                    
                    // incr timing
                    incrCur = diagTimerIncr.ElapsedMilliseconds;
                    diagTimerIncr.Reset();

                    incrAvg += (incrCur - incrAvg) / avgTimeCounter;

                    if (incrCur < incrMin) incrMin = incrCur;
                    else if (incrCur > incrMax) incrMax = incrCur;
                    else if (incrNotIni)
                    {
                        incrMin = incrCur;
                        incrMax = incrCur;
                        incrNotIni = false;
                    }

                    // --- output results -------------------------------------
                    if (outputTimer.ElapsedMilliseconds > outputLap)
                    {
                        outputSB.Clear();
                        outputSB.Append(header);
                        outputSB.AppendFormat("               {0,11} | {1,11} | {2,11}\n\r", "min", "max", "avg");
                        outputSB.AppendFormat("Straight  ms:  {0,11:N} | {1,11:N} | {2,11:e4}\n\r", straightMin, straightMax, straightAvg);
                        outputSB.AppendFormat("Increment ms:  {0,11:N} | {1,11:N} | {2,11:e4}\n\r", incrMin, incrMax, incrAvg);
                        outputSB.AppendFormat("Delta       :  {0,11:e4} | {1,11:e4} | {2,11:e4}\n\r", deltaMin, deltaMax, deltaAvg);
                        outputSB.Append(footer);

                        Console.Clear();
                        Console.WriteLine(outputSB.ToString());

                        outputTimer.Restart();
                    }
                }
            }
            #endregion


            #region 4_ArrayTest
            if (TESTnO == 4)
            {
                int rowNum = 3000;
                int clmNum = 3000;
                int passesNum = rowNum * clmNum;

                // --- Ini arrays -----------------------------------------------------
                int[] single = new int[rowNum * clmNum];

                int[,] multi = new int[rowNum, clmNum];

                int[][] jagged = new int[rowNum][];
                for (int i = 0; i < rowNum; i++) jagged[i] = new int[clmNum];

                // --- Traversal of the arrays -----------------------------------------
                Stopwatch diagTimer = new Stopwatch();
                Random rnd = new Random();

                // --- Warming up ------------------
                for (int n = 0; n < 100; n++)
                {
                    int i = rnd.Next(0, rowNum);
                    int j = rnd.Next(0, clmNum);
                    
                    int tmp = single[i * clmNum + j];
                    tmp = multi[i, j];
                    tmp = jagged[i][j];
                }

                // --- single serial access ---
                int iterNum = rowNum * clmNum; // for verification
                diagTimer.Restart();

                for (int i = 0; i < single.Length; i += clmNum)
                {
                    for (int j = i, stop = i + clmNum; j < stop; j++)
                    {
                        int tmp = single[j];
                        //iterNum++;
                    }
                }

                diagTimer.Stop();

                long tSingleSerial = diagTimer.ElapsedMilliseconds;

                // --- single random access ---
                diagTimer.Reset();

                for (int n = 0; n < passesNum; n++)
                {
                    int i = rnd.Next(0, rowNum);
                    int j = rnd.Next(0, clmNum);

                    diagTimer.Start();
                    int tmp = single[i * clmNum + j];
                    diagTimer.Stop();
                }

                long tSingleRandom = diagTimer.ElapsedMilliseconds;

                Console.WriteLine("Single : serial = {0,6:N0} ; rnd = {1,6:N0} ; iters = {2,6:N0}", tSingleSerial, tSingleRandom, iterNum);

                // --- multi serial access ---
                //iterNum = 0;
                diagTimer.Restart();

                for (int i = 0; i < rowNum; i++)
                {
                    for (int j = 0; j < clmNum; j++)
                    {
                        int tmp = multi[i, j];
                        //iterNum++;
                    }
                }

                diagTimer.Stop();

                long tMultiSerial = diagTimer.ElapsedMilliseconds;

                // --- multi random access ---
                diagTimer.Reset();

                for (int n = 0; n < passesNum; n++)
                {
                    int i = rnd.Next(0, rowNum);
                    int j = rnd.Next(0, clmNum);

                    diagTimer.Start();
                    int tmp = multi[i, j];
                    diagTimer.Stop();
                }

                long tMultiRandom = diagTimer.ElapsedMilliseconds;
                
                Console.WriteLine("Multi  : serial = {0,6:N0} ; rnd = {1,6:N0} ; iters = {2,6:N0}", tMultiSerial, tMultiRandom, iterNum);

                // --- jagged serial access ---
                //iterNum = 0;
                diagTimer.Restart();

                for (int i = 0; i < rowNum; i++)
                {
                    for (int j = 0; j < clmNum; j++)
                    {
                        int tmp = jagged[i][j];
                        //iterNum++;
                    }
                }

                diagTimer.Stop();

                long tJaggedSerial = diagTimer.ElapsedMilliseconds;

                // --- multi random access ---
                diagTimer.Reset();

                for (int n = 0; n < passesNum; n++)
                {
                    int i = rnd.Next(0, rowNum);
                    int j = rnd.Next(0, clmNum);

                    diagTimer.Start();
                    int tmp = jagged[i][j];
                    diagTimer.Stop();
                }

                long tJaggedRandom = diagTimer.ElapsedMilliseconds;
                
                Console.WriteLine("Jagged : serial = {0,6:N0} ; rnd = {1,6:N0} ; iters = {2,6:N0}", tJaggedSerial, tJaggedRandom, iterNum);
            }
            #endregion


            #region 5_DigonalIndexes
            if (TESTnO == 5)
            {
                const int dim = 4;

                double[,] shape = new double[dim, 3];

                for (int i = 0; i < dim; i++)
                {
                    shape[i, 0] = 0.0;
                    shape[i, 1] = 1.0;
                    shape[i, 2] = 2.0;
                }

                int[] tess = Enumerable.Repeat(2, dim).ToArray();
                tess[0] = 3;
                tess[2] = 4;

                AdaptiveGrid<double> agrid = new AdaptiveGrid<double>(1, shape, tess);

                int num = -5;
                int start = 0*30;

                int[] pairs = agrid.GetDiagonalPairs(num, start);
                //int[] pairs_S = agrid.GetDiagonalPairs_simple(num, start);

                for (int i = 0; i < pairs.Length; i += 2)
                {
                    Console.WriteLine("{0,3} - {1,3}", pairs[i], pairs[i + 1]);
                    //Console.WriteLine("{0,3} - {1,3}  |  {2,3} - {3,3}", pairs[i], pairs[i+1], pairs_S[i], pairs_S[i+1]);
                }
            }
            #endregion


            #region 6_Caching_ArrayAccess
            if (TESTnO == 6)
            {
                int length;
                int num;
                string input;

                double[] dArr;
                double[] d2Arr;
                bool[] bArr;
                
                var diagTimer = new Stopwatch();

                long twoArrTime;
                long oneArrTime;
                long thrArrTime;

                double sum;

                START:

                Console.Write("Length: ");
                input = Console.ReadLine();

                if (!int.TryParse(input, out length) || length == 0)
                    goto START;

                Console.Write("Num: ");
                input = Console.ReadLine();

                if (!int.TryParse(input, out num) || num == 0)
                    goto START;

                dArr = new double[length];
                d2Arr = new double[length];
                bArr = new bool[length];

                sum = 0.0;
                diagTimer.Restart();

                for (int n = 0; n < num; n++)
                {
                    for (int i = 0; i < dArr.Length; i++)
                    {
                        sum += (1.0 + d2Arr[i]) * (1.0 + dArr[i]);
                    }
                }

                diagTimer.Stop();

                oneArrTime = diagTimer.ElapsedMilliseconds;

                Console.WriteLine("No-convert : sum = {0} ; t = {1} ms", sum, oneArrTime);


                sum = 0.0;
                diagTimer.Restart();

                for (int n = 0; n < num; n++)
                {
                    for (int i = 0; i < dArr.Length; i++)
                    {
                        sum += (1.0 + Convert.ToDouble(bArr[i])) * (1.0 + dArr[i]);
                    }
                }

                diagTimer.Stop();

                twoArrTime = diagTimer.ElapsedMilliseconds;

                Console.WriteLine("Sys-convert: sum = {0} ; t = {1} ms", sum, twoArrTime);


                sum = 0.0;
                diagTimer.Restart();

                for (int n = 0; n < num; n++)
                {
                    for (int i = 0; i < dArr.Length; i++)
                    {
                        sum += (1.0 + (bArr[i] ? 1.0 : 0.0)) * (1.0 + dArr[i]);
                    }
                }

                diagTimer.Stop();

                thrArrTime = diagTimer.ElapsedMilliseconds;

                Console.WriteLine("InL-convert: sum = {0} ; t = {1} ms", sum, thrArrTime);


                Console.WriteLine("{0} %", 100.0 * oneArrTime / twoArrTime);
                Console.WriteLine("{0} %", 100.0 * oneArrTime / thrArrTime);
                Console.WriteLine("------------------------------\n\n\r");

                Console.Write("Enter \"y\" to repeat: ");
                if (Console.ReadLine() == "y")
                    goto START;
            }
            #endregion


            #region 7_MNIST_Preprocess
            if (TESTnO == 7)
            {
                Console.WriteLine("******* MNIST Preprocess *******\n\r");

                START:

                string loadFile = "";
                string savePath = "";

                while(!File.Exists(loadFile))
                {
                    if (loadFile != "")
                        Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFile);

                    Console.Write("Open file: ");
                    loadFile = Console.ReadLine();
                    Console.WriteLine();
                }

                while (!Directory.Exists(savePath))
                {
                    if (savePath != "")
                        Console.WriteLine("Unable to find directory: \"{0}\"\n\r", savePath);

                    Console.Write("Save folder: ");
                    savePath = Console.ReadLine();
                    Console.WriteLine();
                }
                
                string saveFile = Path.Combine(savePath, 
                                    string.Format("{0}_prep-{1}.{2}", 
                                        Path.GetFileNameWithoutExtension(loadFile), 
                                        DateTime.Now.ToString("yyMMdd-HHmm"),
                                        "csv"));
                
                int width = 28;
                ConsoleIO.IONumeric("Width : ", ref width, w => w > 0);

                int height = width;
                ConsoleIO.IONumeric("Height: ", ref height, h => h > 0);

                //int centerW = width / 2;
                //int centerH = height / 2;

                var fex = new ImageFeatureExtractor(width, height);
                var risingEdges = new int[2][] { new int[height], new int[width] };

                int widthAvg = width / 2;
                int heightAvg = height / 2;

                if (widthAvg == 0) ++widthAvg;
                if (heightAvg == 0) ++heightAvg;

                ConsoleIO.IONumeric("Avg width : ", ref widthAvg, w => w > 0 && w < width);
                ConsoleIO.IONumeric("Avg height: ", ref heightAvg, h => h > 0 && h < height);

                double inLevel = 0.5;
                double outLevel = 0.5;

                ConsoleIO.IONumeric("In Level : ", ref inLevel);

                if (outLevel > inLevel)
                    outLevel = inLevel;

                ConsoleIO.IONumeric("Out Level: ", ref outLevel, v => v <= inLevel);

                fex.SetInOutLevels(inLevel, outLevel);

                //double ratioS = 1.5;

                //ConsoleIO.IONumeric("Symmetry ratio : ", ref ratioS, r => r >= 0.0);

                //var weightsS = fex.GetSymmetryWeights(height, ratioS);

                //double ratioC = 1.5;
                //double exp = 3.0;

                //ConsoleIO.IONumeric("Center ratio   : ", ref ratioC, r => r >= 0.0);
                //ConsoleIO.IONumeric("Center exponent: ", ref exp, r => r > 0.0);

                //var weightsC = fex.GetCenterWeights(height, ratioC, exp);

                StreamReader SR = null;
                StreamWriter SW = null;

                try
                {
                    SR = new StreamReader(loadFile);
                    SW = new StreamWriter(saveFile);

                    // --- The file preview ---
                    Console.WriteLine("\n\r PREVIEW:\n\r");

                    string line;
                    int index = 0;

                    while ((line = SR.ReadLine()) != null && index++ < 7)
                        Console.WriteLine(line.Substring(0, Math.Min(64, line.Length)));

                    Console.WriteLine();

                    char delim = ',';

                    ConsoleIO.IOString("Delimiter: ", ref delim, s => s.Length == 1);
                    
                    // --- Feature Extraction ---
                    if (SR != null)
                        SR.Close();

                    SR = new StreamReader(loadFile);
                    index = 0;

                    Console.Write("Enter 'n' if the first line is NOT a header: ");
                    
                    if (Console.ReadLine() != "n")
                        SR.ReadLine();

                    Console.WriteLine("\n\r STARTED >>> Press 'Esc' to cancel <<<\n\r");

                    var risingEdgesAvg = new double[2][] { new double[heightAvg], new double[widthAvg] };
                    //var risingEdgesMx = new double[heightAvg, heightAvg];

                    //var tmpW = new double[width];
                    //var tmpH = new double[height];

                    //double[] aggregated = new double[4];

                    int[] point = new int[width * height];
                    var lineDataRead = ValueTuple.Create(0, point, int.MinValue);
                    var lineDataWrite = ValueTuple.Create(0, risingEdgesAvg);
                    var lineSb = new StringBuilder(256);

                    while ((line = SR.ReadLine()) != null)
                    {
                        CsvConverter.Read(line, ref lineDataRead, delim);

                        fex.GetRisingEdgesNum(lineDataRead.Item2, lineDataRead.Item3, risingEdges);

                        //double horizontal;
                        //double vertical;

                        risingEdges[0].Average(risingEdgesAvg[0]);
                        risingEdges[1].Average(risingEdgesAvg[1]);

                        lineDataWrite.Item1 = lineDataRead.Item1;
                        //aggregated[0] = risingEdges[0].Sum() + risingEdges[1].Sum();
                        //aggregated[1] = risingEdgesAvg[0].DotProduct(risingEdgesAvg[1]);
                        //aggregated[2] = risingEdges[0].DotProduct(weightsS);
                        //aggregated[3] = risingEdges[1].DotProduct(weightsS);

                        CsvConverter.Write(lineSb, lineDataWrite);

                        SW.WriteLine(lineSb.ToString());

                        //horizontal = risingEdgesAvg[0].Sum() * risingEdges[1].DotProduct(weightsC);

                        //vertical = risingEdges[0].Where((v, i) => i >= centerH).Max();
                        //vertical -= risingEdges[0].Where((v, i) => i < centerH).Min();
                        //vertical *= risingEdgesAvg[1].Sum();

                        //SW.WriteLine(string.Format("{0:F0}|{1:E9}|{2:E9}", lineData.Item1, horizontal, vertical));

                        ++index;

                        if (index % 1000 == 0)
                        {
                            Console.WriteLine("Lines processed: {0,12:N0}", index);

                            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                            {
                                Console.WriteLine("\n\r CANCELED");
                                break;
                            }

                            Console.CursorLeft = 0;
                            --Console.CursorTop;
                        }
                    }

                    Console.WriteLine("\n\r DONE {0} lines processed", index);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("\n\r!!! FAULT !!!\n\r{0}", ex.ToString());
                }
                finally
                {
                    if (SR != null)
                        SR.Close();

                    if (SW != null)
                        SW.Close();
                }

                Console.Write("\n\rEnter 'x' to exit: ");

                if (Console.ReadLine() != "x")
                {
                    Console.WriteLine("----------------------------------\n\n\r");
                    goto START;
                }
            }
            #endregion


            #region 8_MNIST_Weights
            if (TESTnO == 8)
            {
                Console.WriteLine("******* MNIST Weights *******\n\r");

                START:

                string dateStr = DateTime.Now.ToString("yyMMdd-HHmm");
                string loadFileCsv = "";
                string loadFileWbin = "";
                string savePath = "";

                StreamWriter SW = null;
                StreamReader SR = null;
                BinaryWriter BW = null;
                BinaryReader BR = null;
                Bitmap bmap = null;

                bool isCalcMode = true;
                Console.Write("Enter \"apply\" to apply calculated weights: ");

                string inputStr = Console.ReadLine();

                if (inputStr != "apply" && inputStr != "app" && inputStr != "a")
                {
                    isCalcMode = true;
                    Console.WriteLine("\n\r *** CALCULATION OF WEIGHTS ***\n\r");
                }
                else
                {
                    isCalcMode = false;
                    Console.WriteLine("\n\r *** APPLICATION OF WEIGHTS ***\n\r");

                    while (!File.Exists(loadFileWbin))
                    {
                        if (loadFileWbin != "")
                            Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFileWbin);

                        Console.Write("Open file *.wbin: ");
                        loadFileWbin = Console.ReadLine();
                        Console.WriteLine();
                    }
                }

                while (!File.Exists(loadFileCsv))
                {
                    if (loadFileCsv != "")
                        Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFileCsv);

                    Console.Write("Open file *.csv: ");
                    loadFileCsv = Console.ReadLine();
                    Console.WriteLine();
                }
                
                try
                {
                    // --- The file preview ----------------------------------------------------------
                    Console.WriteLine("\n\r PREVIEW:\n\r");

                    SR = new StreamReader(loadFileCsv);
                    
                    string line;
                    int linesNumTrain = 0;
                    int linesNumTest = 0;
                    char delim = ',';
                    bool skipFirst = true;

                    while ((line = SR.ReadLine()) != null && ++linesNumTrain < 7)
                        Console.WriteLine(line.Substring(0, Math.Min(64, line.Length)));

                    while (SR.ReadLine() != null)
                        ++linesNumTrain;

                    SR.Close();

                    Console.WriteLine("---------\n\rTotal lines: {0}\n\r", linesNumTrain);
                    
                    ConsoleIO.IOString("Delimiter: ", ref delim, s => s.Length == 1);

                    Console.Write("Enter 'y' if the first line is a header: ");
                    
                    if (Console.ReadLine() != "y")
                        skipFirst = false;
                    else
                        --linesNumTrain;
                    
                    double ratio = 0.1;
                    ConsoleIO.IONumeric("Test lines ratio [0, 1]: ", ref ratio, r => r >= 0.0 && r < 1.0);

                    linesNumTest = (int)(ratio * linesNumTrain);
                    linesNumTrain -= linesNumTest;
                    
                    int width = 28;
                    ConsoleIO.IONumeric("Width : ", ref width, w => w > 0);

                    int height = width;
                    ConsoleIO.IONumeric("Height: ", ref height, h => h > 0);

                    int length = width * height;

                    // --- Calculation Mode -----------------------------------------------------------------------------------
                    if (isCalcMode)
                    {
                        double ratioNeg = -0.1;

                        ConsoleIO.IONumeric("Denying factor [-1, 0] : ", ref ratioNeg, r => r >= -1.0 && r <= 1.0);

                        if (ratioNeg > 0.0)
                            ratioNeg = -ratioNeg;

                        savePath = string.Format("{0}_weights-{1}-{2}-{3}",
                                             Path.GetFileNameWithoutExtension(loadFileCsv),
                                             linesNumTrain,
                                             width * height,
                                             dateStr);

                        savePath = Path.Combine(Path.GetDirectoryName(loadFileCsv), savePath);

                        Console.WriteLine("\n\r SAVE PATH:\n\r{0}", savePath);

                        Directory.CreateDirectory(savePath);


                        // --- Weights accumulation ------------------------------------------------------
                        double[][] weights = new double[10][];
                        double[] tmpPos = new double[length];
                        double[] tmpNeg = new double[length];

                        for (int i = 0; i < 10; ++i)
                            weights[i] = new double[length];

                        SR = new StreamReader(loadFileCsv);

                        if (skipFirst)
                            SR.ReadLine();

                        int[] point = new int[length];
                        var lineData = ValueTuple.Create(0, point, int.MinValue);

                        int num = 0;
                        bool isCanceled = false;
                        Console.WriteLine("\n\r STARTED >>> Press 'Esc' to cancel <<<\n\r");

                        while (linesNumTrain-- > 0)
                        {
                            line = SR.ReadLine();

                            CsvConverter.Read(line, ref lineData, delim);

                            double factorPos = 1.0 / lineData.Item3;
                            double factorNeg = ratioNeg / lineData.Item3;

                            for (int i = 0; i < point.Length; ++i)
                            {
                                tmpPos[i] = factorPos * point[i];
                                tmpNeg[i] = factorNeg * point[i];
                            }

                            for (int i = 0; i < 10; ++i)
                            {
                                if (i == lineData.Item1)
                                    weights[i].Add(tmpPos);
                                else
                                    weights[i].Add(tmpNeg);
                            }

                            if (++num % 1000 == 0)
                            {
                                Console.WriteLine("Lines processed: {0,12:N0}", num);

                                if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                                {
                                    Console.WriteLine("\n\r CANCELED");
                                    isCanceled = true;
                                    break;
                                }

                                Console.CursorLeft = 0;
                                --Console.CursorTop;
                            }
                            else if (linesNumTrain == 0)
                            {
                                Console.WriteLine("Lines processed: {0,12:N0}", num);
                            }
                        }

                        if (!isCanceled)
                        {
                            Console.WriteLine("\n\r DONE");

                            // --- Normalize weights ---
                            for (int i = 0; i < 10; ++i)
                            {
                                double factor = weights[i].Max(v => v < 0.0 ? -v : v); // abs max
                                factor = 1.0 / factor;

                                weights[i].Multiply(factor);
                            }

                            Console.WriteLine("\n\r SAVING:");

                            // --- Save weights and bitmaps ---
                            string saveFileBin = Path.Combine(savePath, "Weights.wbin");

                            BW = new BinaryWriter(File.Open(saveFileBin, FileMode.Create));
                            BW.Write(width);
                            BW.Write(height);

                            Color colorNeg = Color.FromArgb(66, 244, 206);
                            Color colorPos = Color.FromArgb(244, 66, 137);
                            Color colorZero = Color.Black;

                            int[][] coeff = new int[2][]
                            {
                                new int[3] { colorZero.B - colorNeg.B, colorZero.G - colorNeg.G, colorZero.R - colorNeg.R },
                                new int[3] { colorPos.B - colorZero.B, colorPos.G - colorZero.G, colorPos.R - colorZero.R }
                            };

                            // --- Set bitmap ---
                            bmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
                            const int BPP = 3;
                            var bdat = bmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
                            IntPtr ptr = bdat.Scan0;
                            int stride = Math.Abs(bdat.Stride);
                            byte[] bgrArr = new byte[stride * bdat.Height];

                            for (int n = 0; n < 10; ++n)
                            {
                                double[] wVector = weights[n];

                                for (int i = 0, bj = 0; i < length; ++i, bj += BPP)
                                {
                                    double wi = wVector[i];

                                    BW.Write(wi);

                                    int[] ci = wi < 0.0 ? coeff[0] : coeff[1];

                                    bgrArr[bj] = (byte)(colorZero.B + wi * ci[0]);
                                    bgrArr[bj + 1] = (byte)(colorZero.G + wi * ci[1]);
                                    bgrArr[bj + 2] = (byte)(colorZero.R + wi * ci[2]);
                                }

                                System.Runtime.InteropServices.Marshal.Copy(bgrArr, 0, ptr, bgrArr.Length);

                                string saveFileBitmap = Path.Combine(savePath, string.Format("WeightsMap-{0}.png", n));

                                bmap.Save(saveFileBitmap, ImageFormat.Png);
                            }

                            Console.WriteLine("\n\r DONE");
                        }
                    }
                    else // --- Application Mode ------------------------------------------------------------------------------
                    {
                        string saveFileCsv = string.Format("{0}_wapplied-{1}.{2}",
                                             Path.GetFileNameWithoutExtension(loadFileCsv),
                                             dateStr,
                                             "csv");

                        saveFileCsv = Path.Combine(Path.GetDirectoryName(loadFileWbin), saveFileCsv);

                        // --- Load Weights ---
                        Console.WriteLine("\n\r LOADING WEIGHTS:");
                        BR = new BinaryReader(File.Open(loadFileWbin, FileMode.Open));

                        int wWidth = BR.ReadInt32();
                        int wHeight = BR.ReadInt32();

                        if (width != wWidth || height != wHeight)
                            throw new Exception("Mismatch of the dimension of loaded weight-vector with the specified width and height");

                        double[][] weights = new double[10][];
                        double[] vector = new double[length];
                        double[] products = new double[10];

                        for (int n = 0; n < weights.Length; ++n)
                        {
                            double[] tmpVector = new double[length];
                            weights[n] = tmpVector;

                            for (int i = 0; i < tmpVector.Length; ++i)
                                tmpVector[i] = BR.ReadDouble();
                        }

                        BR.Close();
                        Console.WriteLine(" DONE");

                        Console.WriteLine("\n\r APPLYING WEIGHTS:");

                        SW = new StreamWriter(saveFileCsv);
                        SR = new StreamReader(loadFileCsv);

                        if (skipFirst)
                            SR.ReadLine();

                        int errIn = 0;
                        int errOut = 0;

                        int[] point = new int[length];
                        var lineDataRead = ValueTuple.Create(0, point, int.MinValue);
                        var lineDataWrite = ValueTuple.Create(0, products);

                        var sbline = new StringBuilder(200);

                        int num = 0;
                        Console.WriteLine("\n\r STARTED >>> Press 'Esc' to cancel <<<\n\r");

                        while (num < linesNumTrain)
                        {
                            line = SR.ReadLine();

                            CsvConverter.Read(line, ref lineDataRead, delim);

                            //double factor = 1.0 / lineDataRead.Item3;
                            double factor = 1.0 / Math.Sqrt(point.SquaredNorm());
                            double max = double.NegativeInfinity;
                            int maxInd = -1;

                            for (int n = 0; n < 10; ++n)
                            {
                                // normalize point
                                for (int i = 0; i < point.Length; ++i)
                                    vector[i] = factor * point[i];

                                double p = vector.DotProduct(weights[n]);

                                products[n] = p;

                                if (p > max)
                                {
                                    max = p;
                                    maxInd = n;
                                }
                            }

                            lineDataWrite.Item1 = lineDataRead.Item1;
                            CsvConverter.Write(sbline, lineDataWrite);

                            SW.WriteLine(sbline.ToString());

                            if (maxInd != lineDataRead.Item1)
                                ++errIn;

                            if (++num % 1000 == 0)
                            {
                                Console.WriteLine("Train lines processed: {0,12:N0} | Error In: {1:N0}", num, errIn);

                                if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                                {
                                    Console.WriteLine("\n\r CANCELED");
                                    break;
                                }

                                Console.CursorLeft = 0;
                                --Console.CursorTop;
                            }
                            else if (num == linesNumTrain)
                            {
                                Console.WriteLine("Train lines processed: {0,12:N0} | Error In: {1:N0}", num, errIn);
                            }
                        }

                        num = 0;

                        while (num < linesNumTest)
                        {
                            line = SR.ReadLine();

                            CsvConverter.Read(line, ref lineDataRead, delim);

                            //double factor = 1.0 / lineDataRead.Item3;
                            double factor = 1.0 / Math.Sqrt(point.SquaredNorm());
                            double max = double.NegativeInfinity;
                            int maxInd = -1;

                            for (int n = 0; n < 10; ++n)
                            {
                                // normalize point
                                for (int i = 0; i < point.Length; ++i)
                                    vector[i] = factor * point[i];

                                double p = vector.DotProduct(weights[n]);

                                products[n] = p;

                                if (p > max)
                                {
                                    max = p;
                                    maxInd = n;
                                }
                            }

                            lineDataWrite.Item1 = lineDataRead.Item1;
                            CsvConverter.Write(sbline, lineDataWrite);

                            SW.WriteLine(sbline.ToString());

                            if (maxInd != lineDataRead.Item1)
                                ++errOut;

                            if (++num % 1000 == 0)
                            {
                                Console.WriteLine("Test lines processed: {0,12:N0} | Error In: {1:N0}", num, errOut);

                                if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                                {
                                    Console.WriteLine("\n\r CANCELED");
                                    break;
                                }

                                Console.CursorLeft = 0;
                                --Console.CursorTop;
                            }
                            else if (num == linesNumTest)
                            {
                                Console.WriteLine("Test lines processed: {0,12:N0} | Error In: {1:N0}", num, errOut);
                            }
                        }

                        Console.WriteLine("\n\r DONE:");

                        double errInPct = 100.0 * errIn / linesNumTrain;
                        double errOutPct = 100.0 * errOut / linesNumTest;

                        Console.WriteLine("Err In: {0} %  |  Err Out: {1} %", errInPct, errOutPct);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("\n\r!!! FAULT !!!\n\r{0}", ex.ToString());
                }
                finally
                {
                    if (SW != null)
                        SW.Close();

                    if (SR != null)
                        SR.Close();

                    if (BW != null)
                        BW.Close();

                    if (BR != null)
                        BR.Close();

                    if (bmap != null)
                        bmap.Dispose();
                }

                Console.Write("\n\rEnter 'y' to continue: ");

                if (Console.ReadLine() == "y")
                {
                    Console.WriteLine("----------------------------------\n\n\r");
                    goto START;
                }
            }
            #endregion


            #region 9_MNIST_Optimal_Weights
            if (TESTnO == 9)
            {
                Console.WriteLine("******* MNIST Optimal Weights *******\n\r");

                START:

                string dateStr = DateTime.Now.ToString("yyMMdd-HHmm");
                string loadFileCsv = "";
                string savePath = "";

                StreamWriter SW = null;
                StreamReader SR = null;
                BinaryWriter BW = null;
                Bitmap bmap = null;

                while (!File.Exists(loadFileCsv))
                {
                    if (loadFileCsv != "")
                        Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFileCsv);

                    Console.Write("Open file *.csv: ");
                    loadFileCsv = Console.ReadLine();
                    Console.WriteLine();
                }

                try
                {
                    // --- The csv file preview ----------------------------------------------------------
                    Console.WriteLine("\n\r PREVIEW:\n\r");

                    SR = new StreamReader(loadFileCsv);

                    string line;
                    int linesNumTrain = 0;
                    int linesNumTest = 0;
                    char delim = ',';
                    bool skipFirst = true;

                    while ((line = SR.ReadLine()) != null && ++linesNumTrain < 7)
                        Console.WriteLine(line.Substring(0, Math.Min(64, line.Length)));

                    while (SR.ReadLine() != null)
                        ++linesNumTrain;

                    SR.Close();

                    Console.WriteLine("---------\n\rTotal lines: {0}\n\r", linesNumTrain);

                    ConsoleIO.IOString("Delimiter: ", ref delim, s => s.Length == 1);

                    Console.Write("Enter 'n' if the first line is NOT a header: ");

                    if (Console.ReadLine() == "n")
                        skipFirst = false;
                    else
                        --linesNumTrain;

                    double ratio = 0.1;
                    ConsoleIO.IONumeric("Test lines ratio [0, 1]: ", ref ratio, r => r >= 0.0 && r < 1.0);

                    linesNumTest = (int)(ratio * linesNumTrain);
                    linesNumTrain -= linesNumTest;

                    int width = 28;
                    ConsoleIO.IONumeric("Width : ", ref width, w => w > 0);

                    int height = width;
                    ConsoleIO.IONumeric("Height: ", ref height, h => h > 0);

                    int length = width * height;
                    int vectorSize = length * sizeof(double);

                    double ratioNeg = -0.1;
                    ConsoleIO.IONumeric("Ini negative factor [-1, 1] : ", ref ratioNeg, r => r >= -1.0 && r <= 1.0);

                    if (ratioNeg > 0.0)
                        ratioNeg = -ratioNeg;

                    double ratioStep = 0.5 * ratioNeg;
                    ConsoleIO.IONumeric("Negative factor step [-1, 1]: ", ref ratioStep, r => r >= -1.0 && r <= 1.0);

                    double stepShrink = 2.1;
                    ConsoleIO.IONumeric("Step contraction (1, +inf)  : ", ref stepShrink, r => r > 1.0);


                    // --- Dataset loading ------------------------------------------------------------------------------------
                    Console.WriteLine("\n\r DATASET LOADING:");
                    
                    int[] labels = new int[linesNumTest + linesNumTrain];
                    double[][] points = new double[labels.Length][];
                    double[] fractions = new double[10];
                    double[][] discord = new double[10][];
                    double[][] weights = new double[10][];
                    double[] tmpNeg = new double[length];
                    double[] zeros = new double[length];

                    for (int i = 0; i < 10; ++i)
                        weights[i] = new double[length];

                    SR = new StreamReader(loadFileCsv);

                    if (skipFirst)
                        SR.ReadLine();

                    int[] point = new int[length];
                    var lineData = ValueTuple.Create(0, point, int.MinValue);

                    int index = 0;
                    bool isCanceled = false;

                    while ((line = SR.ReadLine()) != null)
                    {
                        CsvConverter.Read(line, ref lineData, delim);

                        labels[index] = lineData.Item1;
                        
                        double[] tmp = new double[length];
                        points[index] = tmp;

                        //double factor = 1.0 / lineData.Item3;
                        double factor = 1.0 / Math.Sqrt(point.SquaredNorm());

                        for (int i = 0; i < point.Length; ++i)
                            tmp[i] = factor * point[i];

                        if (index < linesNumTrain)
                        {
                            weights[lineData.Item1].Add(tmp);
                            ++fractions[lineData.Item1];
                        }

                        if (++index == points.Length)
                        {
                            // averaging images of each digit
                            for (int i = 0; i < 10; ++i)
                                weights[i].Multiply(1.0 / fractions[i]);

                            double max = double.NegativeInfinity;

                            // calc the discord as Euclidean distance between avg images
                            for (int i = 1; i < 10; ++i)
                            {
                                double[] vector = new double[i];
                                discord[i] = vector;

                                for (int j = 0; j < vector.Length; ++j)
                                {
                                    double dist = Math.Sqrt(weights[j].SquaredDistance(weights[i]));

                                    vector[j] = dist;

                                    if (dist > max)
                                        max = dist;
                                }
                            }
                            
                            // normalize discord
                            for (int i = 1; i < 10; ++i)
                                discord[i].Multiply(1.0 / max);
                            
                            fractions.Multiply(1.0 / linesNumTrain);

                            // --- output results ---
                            Console.WriteLine("Lines loaded: {0,12:N0}", index);

                            for (int i = 0; i < 10; ++i)
                                Console.WriteLine("{0}: {1, 11:F6} %\n\r", i, 100.0 * fractions[i]);

                            StringBuilder discordStr = new StringBuilder(256);

                            for (int i = 0; i < 10; ++i)
                                discordStr.AppendFormat(" {0,8}", i);

                            discordStr.AppendLine();

                            discordStr.AppendFormat("0 {0,8:F4}\n\r", 0.0);

                            for (int i = 1; i < 10; ++i)
                            {
                                discordStr.Append(i);

                                double[] vector = discord[i];

                                for (int j = 0; j < vector.Length; ++j)
                                    discordStr.AppendFormat(" {0,8:F4}", vector[j]);

                                discordStr.AppendFormat(" {0,8:F4}\n\r", 0.0);
                            }

                            Console.WriteLine(discordStr.ToString());
                            Console.WriteLine(" DONE");
                        }
                        else if (index % 1000 == 0)
                        {
                            Console.WriteLine("Lines loaded: {0,12:N0}", index);

                            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                            {
                                Console.WriteLine("\n\r CANCELED");
                                isCanceled = true;
                                break;
                            }

                            Console.CursorLeft = 0;
                            --Console.CursorTop;
                        }
                    }

                    SR.Close();


                    // --- Weights calculation with iteration of the ratioNeg -----------------------------------------------------------------
                    if (!isCanceled)
                    {
                        Console.WriteLine("\n\r CALCULATION OF WEIGHTS >>> Press 'Esc' to stop <<<: \n\r");
                        
                        int errInPrev = linesNumTrain;
                        int errOutPrev = linesNumTest;

                        string outStr = "";

                        Console.Write("step = {0,12:E4}  |  factor = {1,17:E9}  |  errIn = {2,8:F4} %  |  errOut = {3,8:F4} %", 
                                      ratioStep, ratioNeg, double.NaN, double.NaN);
                        Console.CursorLeft = 0;
                        

                        void WeightsAccumulation()
                        {
                            // --- Weights resetting ---
                            for (int i = 0; i < 10; ++i)
                                Buffer.BlockCopy(zeros, 0, weights[i], 0, vectorSize);

                            for (int n = 0; n < linesNumTrain; ++n)
                            {
                                double[] tmpPos = points[n];
                                int label = labels[n];
                                double pi = fractions[label];

                                for (int li = 0; li < label; ++li)
                                {
                                    double ci = ratioNeg; // discord[label][li];

                                    for (int i = 0; i < point.Length; ++i)
                                        tmpNeg[i] = ci * tmpPos[i];

                                    weights[li].Add(tmpNeg);
                                }

                                weights[label].Add(tmpPos);

                                for (int li = label + 1; li < 10; ++li)
                                {
                                    double ci = ratioNeg; // discord[li][label];

                                    for (int i = 0; i < point.Length; ++i)
                                        tmpNeg[i] = ci * tmpPos[i];

                                    weights[li].Add(tmpNeg);
                                }
                            }

                            // --- Weights normalization ---
                            for (int i = 0; i < 10; ++i)
                            {
                                double factor = 1.0 / Math.Sqrt(weights[i].SquaredNorm());

                                weights[i].Multiply(factor);
                            }
                        }


                        void ErrorsEvaluation(out int errIn, out int errOut)
                        {
                            errIn = 0;
                            errOut = 0;

                            for (int n = 0; n < linesNumTrain; ++n)
                            {
                                double[] tmp = points[n];
                                double max = double.NegativeInfinity;
                                int maxInd = -1;

                                for (int i = 0; i < 10; ++i)
                                {
                                    double p = tmp.DotProduct(weights[i]);

                                    if (p > max)
                                    {
                                        max = p;
                                        maxInd = i;
                                    }
                                }

                                if (maxInd != labels[n])
                                    ++errIn;
                            }

                            for (int n = linesNumTrain; n < points.Length; ++n)
                            {
                                double[] tmp = points[n];
                                double max = double.NegativeInfinity;
                                int maxInd = -1;

                                for (int i = 0; i < 10; ++i)
                                {
                                    double p = tmp.DotProduct(weights[i]);

                                    if (p > max)
                                    {
                                        max = p;
                                        maxInd = i;
                                    }
                                }

                                if (maxInd != labels[n])
                                    ++errOut;
                            }
                        }

                        while (true)
                        {
                            WeightsAccumulation();

                            ErrorsEvaluation(out int errIn, out int errOut);

                            outStr = string.Format("step = {0,12:E4}  |  factor = {1,17:E9}  |  errIn = {2,8:F4} %  |  errOut = {3,8:F4} %",
                                          ratioStep, ratioNeg, 100.0 * errIn / linesNumTrain, 100.0 * errOut / linesNumTest);

                            Console.Write(outStr);
                            Console.CursorLeft = 0;

                            int sum = errIn + errOut;
                            int preSum = errInPrev + errOutPrev;

                            if (sum > preSum)
                                ratioStep /= -stepShrink;

                            ratioNeg += ratioStep;

                            errInPrev = errIn;
                            errOutPrev = errOut;

                            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                            {
                                Console.WriteLine("\n\n\r STOPPED");
                                break;
                            }
                        }

                        // --- Saving the results -----------------------------------------------------------------------------------------
                        if (!string.IsNullOrEmpty(outStr))
                        {
                            Console.Write("Enter 'y' to save the results: ");

                            if (Console.ReadLine() == "y")
                            {
                                savePath = string.Format("{0}_weights-opt-{1}-{2}-{3}",
                                             Path.GetFileNameWithoutExtension(loadFileCsv),
                                             linesNumTrain,
                                             length,
                                             dateStr);

                                savePath = Path.Combine(Path.GetDirectoryName(loadFileCsv), savePath);

                                Console.WriteLine("\n\r SAVE PATH:\n\r{0}", savePath);

                                Directory.CreateDirectory(savePath);

                                Console.WriteLine("\n\r SAVING:");

                                // --- Save weights and bitmaps ---
                                string saveFileBin = Path.Combine(savePath, "Weights.wbin");

                                BW = new BinaryWriter(File.Open(saveFileBin, FileMode.Create));
                                BW.Write(width);
                                BW.Write(height);

                                Color colorNeg = Color.FromArgb(66, 244, 206);
                                Color colorPos = Color.FromArgb(244, 66, 137);
                                Color colorZero = Color.Black;

                                int[][] coeff = new int[2][]
                                {
                                    new int[3] { colorZero.B - colorNeg.B, colorZero.G - colorNeg.G, colorZero.R - colorNeg.R },
                                    new int[3] { colorPos.B - colorZero.B, colorPos.G - colorZero.G, colorPos.R - colorZero.R }
                                };

                                // Set bitmap
                                bmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
                                const int BPP = 3;
                                var bdat = bmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
                                IntPtr ptr = bdat.Scan0;
                                int stride = Math.Abs(bdat.Stride);
                                byte[] bgrArr = new byte[stride * bdat.Height];

                                for (int n = 0; n < 10; ++n)
                                {
                                    double[] wVector = weights[n];

                                    double factor = 1.0 / wVector.Max(v => v < 0.0 ? -v : v); // 1.0 / abs max

                                    for (int i = 0, bj = 0; i < length; ++i, bj += BPP)
                                    {
                                        double wi = wVector[i];

                                        BW.Write(wi);

                                        wi *= factor;

                                        int[] ci = wi < 0.0 ? coeff[0] : coeff[1];

                                        bgrArr[bj] = (byte)(colorZero.B + wi * ci[0]);
                                        bgrArr[bj + 1] = (byte)(colorZero.B + wi * ci[1]);
                                        bgrArr[bj + 2] = (byte)(colorZero.B + wi * ci[2]);
                                    }

                                    System.Runtime.InteropServices.Marshal.Copy(bgrArr, 0, ptr, bgrArr.Length);

                                    string saveFileBitmap = Path.Combine(savePath, string.Format("WeightsMap-{0}.png", n));

                                    bmap.Save(saveFileBitmap, ImageFormat.Png);
                                }

                                // --- Save the last outStr -----------------------------------------------------------------
                                SW = new StreamWriter(File.OpenWrite(Path.Combine(savePath, "NegativeFactor.txt")));
                                SW.WriteLine(outStr);
                                SW.Close();

                                // --- Save Errors Chart --------------------------------------------------------------------
                                Console.Write("Enter 'y' to save chart of errors against the negative factor: ");

                                if (Console.ReadLine() == "y")
                                {
                                    double lower = -1.0;
                                    ConsoleIO.IONumeric("Lower limit: ", ref lower, v => v < 0.0);

                                    double upper = 0.0;
                                    ConsoleIO.IONumeric("Upper limit: ", ref upper, v => v > lower);

                                    double step = (upper - lower) / 1000.0;
                                    ConsoleIO.IONumeric("Step       : ", ref step, v => v > 0.0);

                                    delim = ',';
                                    ConsoleIO.IOString("Delimiter  : ", ref delim, s => s.Length == 1);

                                    Console.WriteLine("\n\r WRITING CHART >>> Press 'Esc' to stop <<<: \n\r");

                                    SW = new StreamWriter(File.OpenWrite(Path.Combine(savePath, "ErrorsChart.csv")));
                                    SW.WriteLine("Factor{0}ErrIn_%{0}ErrOut_%", delim);

                                    int num = (int)((upper - lower) / step);
                                    ratioNeg = lower;

                                    while (ratioNeg <= upper)
                                    {
                                        WeightsAccumulation();

                                        ErrorsEvaluation(out int errIn, out int errOut);

                                        SW.WriteLine("{1}{0}{2}{0}{3}", delim, ratioNeg, 100.0 * errIn / linesNumTrain, 100.0 * errOut / linesNumTest);

                                        Console.Write("factor: {0,12:E4}  |  remains: {1,4}", ratioNeg, --num);
                                        Console.CursorLeft = 0;

                                        if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                                        {
                                            Console.WriteLine("\n\n\r STOPPED");
                                            break;
                                        }

                                        ratioNeg += step;
                                    }

                                    Console.WriteLine();
                                }

                                Console.WriteLine("\n\r DONE");
                            }
                        }

                        points = null;
                        labels = null;

                        GC.Collect();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("\n\r!!! FAULT !!!\n\r{0}", ex.ToString());
                }
                finally
                {
                    if (SW != null)
                        SW.Close();

                    if (SR != null)
                        SR.Close();

                    if (BW != null)
                        BW.Close();

                    if (bmap != null)
                        bmap.Dispose();

                    GC.Collect();
                }

                Console.Write("\n\rEnter 'y' to continue: ");

                if (Console.ReadLine() == "y")
                {
                    Console.WriteLine("----------------------------------\n\n\r");
                    goto START;
                }
            }
            #endregion


            #region 10_MNIST_RBFA
            if (TESTnO == 10)
            {
                Console.WriteLine("******* MNIST RBF Aggregation *******\n\r");

                string dateStr = DateTime.Now.ToString("yyMMdd-HHmm");
                string loadFileCsv = "";

                StreamWriter SW = null;
                StreamReader SR = null;

                while (!File.Exists(loadFileCsv))
                {
                    if (loadFileCsv != "")
                        Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFileCsv);

                    Console.Write("Open file *.csv: ");
                    loadFileCsv = Console.ReadLine();
                    Console.WriteLine();
                }
                
                try
                {
                    // --- The csv file preview ----------------------------------------------------------
                    Console.WriteLine("\n\r PREVIEW:\n\r");

                    SR = new StreamReader(loadFileCsv);

                    string line;
                    int linesNumTrain = 0;
                    int linesNumTest = 0;
                    char delim = ',';
                    bool skipFirst = true;

                    while ((line = SR.ReadLine()) != null && ++linesNumTrain < 7)
                        Console.WriteLine(line.Substring(0, Math.Min(64, line.Length)));

                    while (SR.ReadLine() != null)
                        ++linesNumTrain;

                    SR.Close();

                    Console.WriteLine("---------\n\rTotal lines: {0}\n\r", linesNumTrain);

                    ConsoleIO.IOString("Delimiter: ", ref delim, s => s.Length == 1);

                    Console.Write("Enter 'n' if the first line is NOT a header: ");

                    if (Console.ReadLine() == "n")
                        skipFirst = false;
                    else
                        --linesNumTrain;

                    double ratio = 0.1;
                    ConsoleIO.IONumeric("Test lines ratio [0, 1]: ", ref ratio, r => r >= 0.0 && r < 1.0);

                    linesNumTest = (int)(ratio * linesNumTrain);
                    linesNumTrain -= linesNumTest;
                    
                    int length = 28 * 28;
                    ConsoleIO.IONumeric("Space dimension: ", ref length, d => d > 0);

                    int vectorSize = length * sizeof(double);
                    
                    Console.Write("Enter 'y' to manually set range for all components: ");
                    bool autoNormFlag = Console.ReadLine() != "y";

                    double lowerLim = 0.0;
                    double upperLim = 1.0;

                    if (!autoNormFlag)
                    {
                        ConsoleIO.IONumeric("Lower limit: ", ref lowerLim);

                        upperLim = lowerLim + 1.0;
                        ConsoleIO.IONumeric("Upper limit: ", ref upperLim, v => v > lowerLim);
                    }

                    double range = upperLim - lowerLim;

                    // --- Dataset loading ------------------------------------------------------------------------------------
                    Console.WriteLine("\n\r DATASET LOADING:");

                    int[] labels = new int[linesNumTest + linesNumTrain];
                    double[][] points = new double[labels.Length][];
                    double[] lowerLims = null;
                    double[] spans = null;
                    double[] fractions = new double[10];
                    double[] tmpNeg = new double[length];
                    double[] zeros = new double[length];
                    double[] point = new double[length];

                    var lineData = ValueTuple.Create(0, point, 0.0); // LabelPointMax
                    var lineDataAuto = ValueTuple.Create(0, point, lowerLims, spans); // LabelPointMinMax

                    SR = new StreamReader(loadFileCsv);

                    if (skipFirst)
                        SR.ReadLine();
                    
                    int index = 0;
                    bool isCanceled = false;

                    if (autoNormFlag)
                    {
                        lowerLims = new double[length];
                        spans = new double[length];
                        lineDataAuto = ValueTuple.Create(0, point, lowerLims, spans); // LabelPointMinMax

                        // --- read the first data line to init bounds ---
                        CsvConverter.ReadIniMinMax(SR.ReadLine(), ref lineDataAuto, delim);

                        labels[0] = lineDataAuto.Item1;
                        points[0] = new double[length];
                        Buffer.BlockCopy(point, 0, points[0], 0, vectorSize);

                        ++fractions[lineDataAuto.Item1];
                        ++index;
                    }
                    
                    while ((line = SR.ReadLine()) != null)
                    {
                        int label;

                        if (autoNormFlag)
                        {
                            CsvConverter.Read(line, ref lineDataAuto, delim);
                            label = lineDataAuto.Item1;
                        }
                        else
                        {
                            CsvConverter.Read(line, ref lineData, delim);
                            label = lineData.Item1;
                        }

                        labels[index] = label;

                        double[] tmp = new double[length];
                        points[index] = tmp;

                        Buffer.BlockCopy(point, 0, tmp, 0, vectorSize);

                        if (index < linesNumTrain)
                            ++fractions[label];

                        if (++index == points.Length)
                        {
                            // --- output results ---
                            Console.WriteLine("Lines loaded: {0,12:N0}", index);
                            
                            // --- calc fractions ---
                            fractions.Multiply(1.0 / linesNumTrain);

                            for (int i = 0; i < 10; ++i)
                                Console.WriteLine("{0}: {1, 11:F6} %", i, 100.0 * fractions[i]);

                            // --- normalize points ---
                            Console.WriteLine("\n\r NORMALIZATION:");

                            if (autoNormFlag)
                            {
                                spans.Subtract(lowerLims);

                                for (int n = 0; n < points.Length; ++n)
                                    points[n].NormalizeComponents(lowerLims, spans);
                            }
                            else
                            {
                                foreach (var vector in points)
                                {
                                    for (int i = 0; i < vector.Length; ++i)
                                    {
                                        vector[i] -= lowerLim;
                                        vector[i] /= range;
                                    }
                                }
                            }

                            Console.WriteLine("\n\r DONE");
                        }
                        else if (index % 5000 == 0)
                        {
                            Console.Write("Lines loaded: {0,12:N0}", index);

                            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                            {
                                Console.WriteLine("\n\n\r CANCELED");
                                isCanceled = true;
                                break;
                            }

                            Console.CursorLeft = 0;
                        }
                    }

                    SR.Close();
                    Console.WriteLine();

                    int multp = 1;
                    ConsoleIO.IONumeric("Thread multiplier...: ", ref multp, m => m > 0);

                    var rbfa = new RBFAggregator(10, labels, points, new RadialFunction(), linesNumTrain, threadMultp: multp);

                    while (!isCanceled)
                    {
                        // --- RBF settings ---
                        double fLower = 10.0;
                        ConsoleIO.IONumeric("RBF factor lower lim: ", ref fLower, f => f > 0.0);

                        double fUpper = fLower;
                        ConsoleIO.IONumeric("RBF factor upper lim: ", ref fUpper, f => f >= fLower);

                        double fStep = 10.0;
                        ConsoleIO.IONumeric("RBF factor step.....: ", ref fStep, f => f > 0.0);

                        double exp = 1.0;
                        ConsoleIO.IONumeric("RBF exponent........: ", ref exp, e => e > 0.0);
                        
                        int bflag = 0;
                        ConsoleIO.IONumeric("Balanced flag.......: ", ref bflag, f => f == 0 || f == 1);
                        bool balanceFlag = bflag == 1 ? true : false;

                        rbfa.RBF.Factor = fLower;
                        rbfa.RBF.Exponent = exp;
                        
                        Console.Write("Enter 'y' to save the chart of errors: ");
                        bool saveFlag = Console.ReadLine() == "y";

                        Console.WriteLine();

                        if (saveFlag)
                        {
                            string saveFile = string.Format("{0}_rbfa-errors-{1}{2}.csv",
                                                            Path.GetFileNameWithoutExtension(loadFileCsv),
                                                            fLower,
                                                            (fLower + fStep <= fUpper) ? "-" + fUpper.ToString("F0") : "");

                            saveFile = Path.Combine(Path.GetDirectoryName(loadFileCsv), saveFile);

                            SW = new StreamWriter(File.OpenWrite(saveFile));
                        }

                        Console.WriteLine("\n\r ERRORS COUNTING >>> Press 'Space' to skip or 'Esc' to cancel <<< :\n\r");

                        while (fLower <= fUpper)
                        {
                            int errIn = 0;

                            for (int i = 0, num = 1; i < linesNumTrain; ++i, ++num)
                            {
                                int predicted = rbfa.Classify_parallel(points[i], balanceFlag);

                                if (predicted != labels[i])
                                    ++errIn;

                                if (num % 500 != 0)
                                    continue;

                                Console.Write("Factor: {0,10:F2}  |  Points num: {1,7}  |  errIn : {2,6} ", fLower, num, errIn);
                                Console.CursorLeft = 0;

                                if (Console.KeyAvailable)
                                {
                                    var cki = Console.ReadKey(true);

                                    if (cki.Key == ConsoleKey.Escape)
                                    {
                                        isCanceled = true;
                                        goto LOOP_EXIT;
                                    }
                                    else if (cki.Key == ConsoleKey.Spacebar)
                                    {
                                        isCanceled = true;
                                        break;
                                    }
                                }
                            }

                            int errOut = 0;

                            for (int i = linesNumTrain, num = 1; i < points.Length; ++i, ++num)
                            {
                                int predicted = rbfa.Classify_parallel(points[i], balanceFlag);

                                if (predicted != labels[i])
                                    ++errOut;

                                if (num % 500 != 0)
                                    continue;

                                Console.Write("Factor: {0,10:F2}  |  Points num: {1,7}  |  errOut: {2,6} ", fLower, num, errOut);
                                Console.CursorLeft = 0;

                                if (Console.KeyAvailable)
                                {
                                    var cki = Console.ReadKey(true);

                                    if (cki.Key == ConsoleKey.Escape)
                                    {
                                        isCanceled = true;
                                        goto LOOP_EXIT;
                                    }
                                    else if (cki.Key == ConsoleKey.Spacebar)
                                    {
                                        isCanceled = true;
                                        break;
                                    }
                                }
                            }
                            
                            double errInProc = 100.0 * errIn / linesNumTrain;
                            double errOutProc = 100.0 * errOut / linesNumTest;

                            Console.WriteLine("Factor: {0,10:F2}  |  errIn: {1,10:F6} %  |  errOut: {2,10:F6} %",
                                               fLower, errInProc, errOutProc);

                            if (saveFlag && !isCanceled)
                                SW.WriteLine("{1}{0}{2}{0}{3}", ',', fLower, errInProc, errOutProc);

                            isCanceled = false;
                            fLower += fStep;
                            rbfa.RBF.Factor = fLower;
                        }

                        LOOP_EXIT:

                        if (SW != null)
                            SW.Close();

                        Console.WriteLine("{0}\n\r", isCanceled ? "\n\n\r CANCELED" : "\n\r DONE");
                        Console.Write("Enter 'x' to exit: ");

                        isCanceled = Console.ReadLine() == "x";
                        Console.WriteLine("---------\n\r");
                    }


                    // --- Classification -----------------------------------------------------------------------------------
                    Console.Write("Enter 'y' to classify data: ");

                    isCanceled = Console.ReadLine() != "y";

                    Console.WriteLine("\n\r");

                    while (!isCanceled)
                    {
                        loadFileCsv = "";

                        while (!File.Exists(loadFileCsv))
                        {
                            if (loadFileCsv != "")
                                Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFileCsv);

                            Console.Write("Open file *.csv: ");
                            loadFileCsv = Console.ReadLine();
                            Console.WriteLine();
                        }

                        // --- The csv file preview -----------------------------------------
                        Console.WriteLine("\n\r PREVIEW:\n\r");

                        SR = new StreamReader(loadFileCsv);

                        int lineId = 0;

                        while ((line = SR.ReadLine()) != null && ++lineId < 7)
                            Console.WriteLine(line.Substring(0, Math.Min(64, line.Length)));
                        
                        SR.Close();

                        Console.WriteLine("---------\n\r");

                        ConsoleIO.IOString("The read file delimiter: ", ref delim, s => s.Length == 1);
                        
                        Console.Write("Enter 'n' if the first line is NOT a header: ");
                        skipFirst = Console.ReadLine() != "n";

                        // --- Classified file settings ---
                        char delimW = ',';
                        ConsoleIO.IOString("The write file delimiter: ", ref delimW, s => s.Length == 1);

                        int cols = 2;
                        ConsoleIO.IONumeric("The number of columns: ", ref cols, n => n > 0);

                        StringBuilder lineSb = new StringBuilder(64);
                        string header = "ImageId";

                        for (int n = 0, final = cols - 1; n < cols; ++n)
                        {
                            if (n == 1)
                                header = "Label";
                            else if (n > 1)
                                header = n.ToString();

                            ConsoleIO.IOString(string.Format("Header #{0}: ", n), ref header, s => !string.IsNullOrWhiteSpace(s) && !s.Contains(delimW));

                            if (n != final)
                                lineSb.Append(header + delimW);
                            else
                                lineSb.Append(header);
                        }

                        Console.WriteLine();

                        // --- RBF settings ---
                        double factor = 10.0;
                        ConsoleIO.IONumeric("RBF factor..........: ", ref factor, f => f > 0.0);

                        double exp = 1.0;
                        ConsoleIO.IONumeric("RBF exponent........: ", ref exp, e => e > 0.0);

                        int bflag = 0;
                        ConsoleIO.IONumeric("Balanced flag.......: ", ref bflag, f => f == 0 || f == 1);
                        bool balanceFlag = bflag == 1 ? true : false;

                        rbfa.RBF.Factor = factor;
                        rbfa.RBF.Exponent = exp;

                        string saveFile = string.Format("{0}_rbfa-{1}-f{2:E0}-e{3:E0}.csv",
                                                        Path.GetFileNameWithoutExtension(loadFileCsv),
                                                        linesNumTrain,
                                                        factor,
                                                        exp);

                        saveFile = Path.Combine(Path.GetDirectoryName(loadFileCsv), saveFile);

                        SW = new StreamWriter(File.Open(saveFile, FileMode.CreateNew));
                        SR = new StreamReader(loadFileCsv);

                        if (skipFirst)
                            SR.ReadLine();

                        SW.WriteLine(lineSb.ToString());
                        
                        Console.WriteLine("\n\r CLASSIFICATION >>> Press 'Esc' to cancel <<< :\n\r");

                        lineId = 1;

                        while ((line = SR.ReadLine()) != null)
                        {
                            CsvConverter.Read(line, point, delim);

                            if (autoNormFlag)
                            {
                                point.NormalizeComponents(lowerLims, spans);
                            }
                            else
                            {
                                for (int i = 0; i < point.Length; ++i)
                                {
                                    point[i] -= lowerLim;
                                    point[i] /= range;
                                }
                            }

                            int digit = rbfa.Classify_parallel(point, balanceFlag);
                            
                            line = string.Format("{1}{0}{2}", delimW, lineId, digit);

                            SW.WriteLine(line);

                            if (lineId % 100 == 0)
                            {
                                Console.Write("{1,9}{0}{2,4}", delimW, lineId, digit);
                                Console.CursorLeft = 0;

                                if (Console.KeyAvailable && Console.ReadKey().Key == ConsoleKey.Escape)
                                {
                                    isCanceled = true;
                                    break;
                                }
                            }

                            ++lineId;
                        }

                        Console.WriteLine(line);
                        Console.WriteLine("\n\r {0}\n\r", isCanceled ? "CANCELED" : "DONE");

                        SW.Close();
                        SR.Close();

                        Console.Write("Enter 'x' to exit: ");

                        isCanceled = Console.ReadLine() == "x";
                        Console.WriteLine("---------\n\r");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("\n\r!!! FAULT !!!\n\r{0}", ex.ToString());
                }
                finally
                {
                    if (SW != null)
                        SW.Close();

                    if (SR != null)
                        SR.Close();

                    GC.Collect();
                }
            }
            #endregion


            #region 11_MNIST_Significant_Pixels
            if (TESTnO == 11)
            {
                Console.WriteLine("******* MNIST Significant_Pixels *******\n\r");

                START:
                
                string loadFileCsv = "";

                StreamWriter SW = null;
                StreamReader SR = null;
                BinaryWriter BW = null;
                BinaryReader BR = null;
                Bitmap bmap = null;

                while (!File.Exists(loadFileCsv))
                {
                    if (loadFileCsv != "")
                        Console.WriteLine("Unable to open file: \"{0}\"\n\r", loadFileCsv);

                    Console.Write("Open file *.csv: ");
                    loadFileCsv = Console.ReadLine();
                    Console.WriteLine();
                }

                Color colorPos = Color.FromArgb(255, 255, 255); //Color.FromArgb(200, 244, 66);
                Color colorZero = Color.Black;

                Color[] colorDigits = new Color[10]
                {
                    Color.FromArgb(255, 0, 0),
                    Color.FromArgb(255, 128, 0),
                    Color.FromArgb(255, 255, 0),
                    Color.FromArgb(185, 255, 0),
                    Color.FromArgb(0, 255, 0),
                    Color.FromArgb(0, 255, 127),
                    Color.FromArgb(0, 255, 255),
                    Color.FromArgb(0, 150, 255),
                    Color.FromArgb(0, 0, 255),
                    Color.FromArgb(255, 0, 255)
                };

                int[] colorDeltas = new int[3] { colorPos.B - colorZero.B, colorPos.G - colorZero.G, colorPos.R - colorZero.R };

                try
                {
                    // --- The file preview ----------------------------------------------------------
                    Console.WriteLine("\n\r PREVIEW:\n\r");

                    SR = new StreamReader(loadFileCsv);

                    string line;
                    int linesNumTrain = 0;
                    int linesNumTest = 0;
                    char delim = ',';
                    bool skipFirst = true;

                    while ((line = SR.ReadLine()) != null && ++linesNumTrain < 7)
                        Console.WriteLine(line.Substring(0, Math.Min(64, line.Length)));

                    while (SR.ReadLine() != null)
                        ++linesNumTrain;

                    SR.Close();

                    Console.WriteLine("---------\n\rTotal lines: {0}\n\r", linesNumTrain);

                    ConsoleIO.IOString("Delimiter: ", ref delim, s => s.Length == 1);

                    Console.Write("Enter 'n' if the first line is NOT a header: ");

                    if (Console.ReadLine() == "n")
                        skipFirst = false;
                    else
                        --linesNumTrain;

                    double ratio = 0.1;
                    ConsoleIO.IONumeric("Test lines ratio [0, 1]: ", ref ratio, r => r >= 0.0 && r < 1.0);

                    linesNumTest = (int)(ratio * linesNumTrain);
                    linesNumTrain -= linesNumTest;
                    int linesNum = linesNumTrain + linesNumTest;

                    int width = 28;
                    ConsoleIO.IONumeric("Width : ", ref width, w => w > 0);

                    int height = width;
                    ConsoleIO.IONumeric("Height: ", ref height, h => h > 0);

                    int length = width * height;
                    

                    // --- Shapes accumulation -------------------------------------------------------------------------------------
                    double[][] shapes = new double[10][];

                    for (int i = 0; i < 10; ++i)
                        shapes[i] = new double[length];

                    SR = new StreamReader(loadFileCsv);

                    if (skipFirst)
                        SR.ReadLine();

                    // it is assumed that the range for each pixel is [0, 255]
                    double pxmax = 255 * linesNumTrain; 
                    double[] point = new double[length];
                    var lineDataRead = ValueTuple.Create(0, point, double.MinValue);

                    int num = 0;
                    bool isCanceled = false;
                    Console.WriteLine("\n\r SHAPES ACCUMULATION >>> Press 'Esc' to cancel <<<\n\r");

                    while (++num <= linesNumTrain)
                    {
                        line = SR.ReadLine();

                        CsvConverter.Read(line, ref lineDataRead, delim);

                        shapes[lineDataRead.Item1].Add(point);

                        if (num == linesNumTrain)
                        {
                            Console.WriteLine("Lines processed: {0,12:N0}", num);
                        }
                        else if (num % 1000 == 0)
                        {
                            Console.WriteLine("Lines processed: {0,12:N0}", num);

                            if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                            {
                                Console.WriteLine("\n\r CANCELED");
                                isCanceled = true;
                                break;
                            }

                            Console.CursorLeft = 0;
                            --Console.CursorTop;
                        }
                    }

                    SR.Close();
                    Console.WriteLine("\n\r DONE\n\r");

                    // shapes normalization
                    for (int n = 0; n < 10; ++n)
                    {
                        double[] tmp = shapes[n];
                        double max = tmp.Max();
                        tmp.Multiply(1.0 / max);
                    }

                    double[] pxsense = null;
                    int[] imaxes = null;

                    if (!isCanceled)
                    {
                        pxsense = new double[length];
                        imaxes = new int[length];
                    }

                    while (!isCanceled)
                    {
                        double senseThold = 0.05;
                        ConsoleIO.IONumeric("Sense threshold [0, 1)    : ", ref senseThold, v => v >= 0.0 && v < 1.0);

                        double senseAggro = 2.0;
                        ConsoleIO.IONumeric("Sense agg ratio [1, +inf) : ", ref senseAggro, v => v >= 1.0);

                        int directNum = 0; // the number of original pixels not subject to any modifications

                        // this array stores the number of pixels in each aggregation group
                        int[] aggGroups = new int[10];

                        double senseMax = double.NegativeInfinity;

                        // --- Calculation of the pixels sense ---
                        for (int i = 0; i < pxsense.Length; ++i)
                        {
                            double avg = shapes[0][i];
                            double min = avg;
                            double max = min;
                            int imax = 0;

                            for (int n = 1; n < 10; ++n)
                            {
                                double pxval = shapes[n][i];
                                avg += pxval;

                                if (pxval < min)
                                {
                                    min = pxval;
                                }
                                else if (pxval > max)
                                {
                                    max = pxval;
                                    imax = n;
                                }
                            }

                            double d = max - min;
                            pxsense[i] = d;

                            if (d > senseMax)
                                senseMax = d;

                            if (d > senseThold)
                            {
                                avg /= 10.0;

                                if ((max - avg) > senseAggro * (avg - min))
                                {
                                    imaxes[i] = imax;
                                    ++aggGroups[imax];
                                }
                                else
                                {
                                    ++directNum;
                                    imaxes[i] = -1;
                                }
                            }
                            else
                                imaxes[i] = int.MinValue;
                        }

                        pxsense.Multiply(1.0 / senseMax);
                        

                        Console.WriteLine("\n\r SAVING THE SENSE BITMAP:");

                        // --- Save weights and bitmaps ---
                        //string saveFileBin = Path.Combine(savePath, "Weights.wbin");

                        //BW = new BinaryWriter(File.Open(saveFileBin, FileMode.Create));
                        //BW.Write(width);
                        //BW.Write(height);
                        
                        // --- Set bitmap ---
                        bmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
                        const int BPP = 3;
                        var bdat = bmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
                        IntPtr ptr = bdat.Scan0;
                        int stride = Math.Abs(bdat.Stride);
                        byte[] bgrArr = new byte[stride * bdat.Height];

                        for (int i = 0, bj = 0; i < length; ++i, bj += BPP)
                        {
                            double pxi = pxsense[i];
                            int ci = imaxes[i];

                            if (ci < -1)
                            {
                                bgrArr[bj] = colorZero.B;
                                bgrArr[bj + 1] = colorZero.G;
                                bgrArr[bj + 2] = colorZero.R;
                            }
                            else if (ci < 0)
                            {
                                bgrArr[bj] = (byte)(colorZero.B + pxi * colorDeltas[0]);
                                bgrArr[bj + 1] = (byte)(colorZero.G + pxi * colorDeltas[1]);
                                bgrArr[bj + 2] = (byte)(colorZero.R + pxi * colorDeltas[2]);
                            }
                            else
                            {
                                Color color = colorDigits[ci];

                                bgrArr[bj] = (byte)(colorZero.B + pxi * color.B);
                                bgrArr[bj + 1] = (byte)(colorZero.G + pxi * color.G);
                                bgrArr[bj + 2] = (byte)(colorZero.R + pxi * color.R);
                            }
                        }

                        System.Runtime.InteropServices.Marshal.Copy(bgrArr, 0, ptr, bgrArr.Length);

                        string dateStr = DateTime.Now.ToString("yyMMdd-HHmmss");
                        string savePath = string.Format("{0}_pixsense-{1}-{2}-{3}",
                                                 Path.GetFileNameWithoutExtension(loadFileCsv),
                                                 linesNumTrain,
                                                 width * height,
                                                 dateStr);

                        savePath = Path.Combine(Path.GetDirectoryName(loadFileCsv), savePath);

                        Directory.CreateDirectory(savePath);

                        string saveBitmap = string.Format("PixSense-{0}-_{1}-{2}.png",
                                                           linesNumTrain,
                                                           senseThold.ToString("g4").Substring(2),
                                                           senseAggro.ToString("g4").Replace('.', '_'));

                        saveBitmap = Path.Combine(savePath, saveBitmap);
                        bmap.Save(saveBitmap, ImageFormat.Png);

                        Console.WriteLine("\n\r DONE\n\r");


                        // calc the number of new dimensions
                        int aggNum = aggGroups.Count(b => b > 0);
                        int dim = directNum + aggNum;
                        
                        Console.WriteLine("\n\rDirect: {0} | Agg.: {1} | Total: {2}\n\r", directNum, aggNum, dim);

                        Console.Write("Enter 'y' to apply transformation: ");
                        
                        // transformation to the new space
                        if (Console.ReadLine() == "y")
                        {
                            int[] aggIndMap = null; // forward mapping
                            int[] aggIndInvMap = null; // inverse mapping
                            int outInd = 0;

                            // --- define mapping for the pixels belonging to the aggregation groups ---
                            if (aggNum > 0)
                            {
                                aggIndMap = Enumerable.Repeat(-1, aggGroups.Length).ToArray(); // forward mapping
                                aggIndInvMap = new int[aggNum]; // inverse mapping


                                for (int i = 0; i < aggGroups.Length; ++i)
                                {
                                    if (aggGroups[i] != 0)
                                    {
                                        aggIndMap[i] = outInd;
                                        aggIndInvMap[outInd] = i;
                                        ++outInd;
                                    }
                                }
                            }
                            
                            // --- define mapping for all input pixels ---
                            outInd = aggNum;

                            for (int i = 0; i < length; ++i)
                            {
                                ref int imax = ref imaxes[i];

                                if (imax < -1)
                                    continue;

                                if (imax < 0) // direct pixels mapping
                                {
                                    imax = outInd;
                                    ++outInd;
                                }
                                else // agg group pixels mapping
                                {
                                    imax = aggIndMap[imax];
                                }
                            }

                            // --- read and apply transformation to all points -------------------------------
                            string saveFileCsv = string.Format("{0}-{1}-{2}_{3}-{4}.csv",
                                                           Path.GetFileNameWithoutExtension(loadFileCsv),
                                                           linesNumTrain,
                                                           dim,
                                                           senseThold.ToString("g4").Substring(2),
                                                           senseAggro.ToString("g4").Replace('.', '_'));

                            saveFileCsv = Path.Combine(savePath, saveFileCsv);

                            SR = new StreamReader(loadFileCsv);
                            SW = new StreamWriter(saveFileCsv);

                            if (skipFirst)
                                SR.ReadLine();
                            
                            double[] trpoint = new double[dim];
                            double[] zeroarr = new double[dim];
                            int trsize = dim * sizeof(double);

                            var lineDataWrite = ValueTuple.Create(0, trpoint);
                            var lineBld = new StringBuilder(256);

                            num = 0;
                            isCanceled = false;
                            Console.WriteLine("\n\r TRANSFORMATION >>> Press 'Esc' to cancel <<<\n\r");

                            while (++num <= linesNum)
                            {
                                // READ
                                line = SR.ReadLine();
                                CsvConverter.Read(line, ref lineDataRead, delim);

                                // --- TRANSFORM ---
                                for (int i = 0; i < length; ++i)
                                {
                                    int ind = imaxes[i];

                                    if (ind < 0)
                                        continue;

                                    trpoint[ind] += point[i];
                                }

                                // Normalize aggregated components
                                for (int i = 0; i < aggNum; ++i)
                                    trpoint[i] /= aggGroups[aggIndInvMap[i]];

                                // WRITE
                                lineDataWrite.Item1 = lineDataRead.Item1;
                                CsvConverter.Write(lineBld, lineDataWrite);
                                SW.WriteLine(lineBld.ToString());

                                // erase trpoint
                                Buffer.BlockCopy(zeroarr, 0, trpoint, 0, trsize);

                                // Sync
                                if (num == linesNum)
                                {
                                    Console.WriteLine("Lines processed: {0,12:N0}", num);
                                }
                                else if (num % 1000 == 0)
                                {
                                    Console.WriteLine("Lines processed: {0,12:N0}", num);

                                    if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                                    {
                                        Console.WriteLine("\n\r CANCELED");
                                        isCanceled = true;
                                        break;
                                    }

                                    Console.CursorLeft = 0;
                                    --Console.CursorTop;
                                }
                            }

                            SR.Close();
                            SW.Close();
                            Console.WriteLine("\n\r DONE\n\r");
                        }

                        // --- loop exit ------------------------------------
                        Console.Write("Enter 'x' to load another data: ");
                        isCanceled = Console.ReadLine() == "x";
                        Console.WriteLine("\n\r");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("\n\r!!! FAULT !!!\n\r{0}", ex.ToString());
                }
                finally
                {
                    if (SW != null)
                        SW.Close();

                    if (SR != null)
                        SR.Close();

                    if (BW != null)
                        BW.Close();

                    if (BR != null)
                        BR.Close();

                    if (bmap != null)
                        bmap.Dispose();
                }

                Console.Write("\n\rEnter 'x' to exit: ");

                if (Console.ReadLine() != "x")
                {
                    Console.WriteLine("----------------------------------\n\n\r");
                    goto START;
                }
            }
            #endregion

            END:
            Console.WriteLine(">> Press any key to exit <<");
            Console.ReadKey();
        }
    }
}
