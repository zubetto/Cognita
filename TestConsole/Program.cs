using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Cognita;
using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;
using RBF = Cognita.SupervisedLearning.RadialBasisFunction;

namespace TestConsole
{
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

            int TESTnO = 3;

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

                int GetIndex(int indMin = 0, int indMax = -1)
                {
                    indMax = indMax < 0 ? agrid.RootsNumber : indMax;

                    return rnd.Next(indMin, agrid.TessellationMultp);
                }

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

            END:
            Console.WriteLine(">> Press any key to exit <<");
            Console.ReadKey();
        }
    }
}
