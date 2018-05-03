using System;
using System.Windows;
using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;
using Alea;
using Alea.CSharp;
using gmath = Alea.DeviceFunction;
using System.Windows.Media;

namespace ModelFunctions
{
    public static class MathX
    {
        public static readonly double sqrt2 = Math.Sqrt(2);

        /// <summary>
        /// Performs clipping by means of the quadratic Bezier curve
        /// </summary>
        /// <param name="input"></param>
        /// <param name="threshold"></param>
        /// <param name="kinkRadius"></param>
        /// <returns></returns>
        public static double Clip(double input, double threshold, double kinkRadius)
        {
            if (kinkRadius == 0)
            {
                if (input > threshold) return threshold;
                else return input;
            }

            if (kinkRadius < 0) kinkRadius = -kinkRadius;

            double po = threshold - kinkRadius / sqrt2;

            if (input <= po) return input;

            double p2x = threshold + kinkRadius;

            if (input >= p2x) return threshold;

            // convert input to the Bezier parameter t
            input -= po;
            input /= p2x - po;

            double inverse = 1 - input;
            double factor = input * threshold;

            // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
            // P1y = P2y = threshold
            return inverse * inverse * po + 2 * inverse * factor + input * factor;
        }
    }

    public interface IModel
    {
        double Time { get; set; }
        double TimeStep { get; }
        double Xo { get; set; }
        double Yo { get; set; }
        double RangeX { get; set; }
        double RangeY { get; set; }

        double Measure(double x, double y);
        double Measure(double[] point);
        double Measure(Point point);
        double Measure(IVoxel voxel);

        double Normalize(double value);

        void ResetSimulation();
        void IncrementTime(double dt);

        void SetDrawing(int width, int height, byte[] pixelArray, byte[] gradColorsBgra, Matrix toModelTransform);
        void FreeMemory();
        void DrawFrame();
    }

    /// <summary>
    /// 
    /// </summary>
    public class Speck : IModel
    {
        private double t;
        private double xo, yo;
        private double Dx = 10, Dy = 10;
        private double xC, yC;
        private double A = 1;
        private double kx = 1, ky = 1;
        private double exp = 1;

        private double[] modelParams = new double[11];

        private void FillModelParams()
        {
            modelParams[0] = t;
            modelParams[1] = xo;
            modelParams[2] = yo;
            modelParams[3] = Dx;
            modelParams[4] = Dy;
            modelParams[5] = xC;
            modelParams[6] = yC;
            modelParams[7] = A;
            modelParams[8] = kx;
            modelParams[9] = ky;
            modelParams[10] = exp;
        }

        private double xoIni, yoIni;
        private double DxIni = 10, DyIni = 10;
        private double xCini, yCini;
        private double Aini = 1;
        private double kxIni = 1, kyIni = 1;
        private double expIni = 1;

        private byte[] pxArray;
        private int pxW, pxH;
        private int rowStride;

        // --- GPU variables ---
        private Gpu gpu = Gpu.Default;
        private LaunchParam LP;

        private byte[] gPxArray;

        private static readonly GlobalVariableSymbol<int> gPxW = Gpu.DefineConstantVariableSymbol<int>();
        private static readonly GlobalVariableSymbol<int> gPxH = Gpu.DefineConstantVariableSymbol<int>();
        private static readonly GlobalVariableSymbol<int> gPxStride = Gpu.DefineConstantVariableSymbol<int>();

        private static readonly GlobalArraySymbol<byte> gGradColors = Gpu.DefineConstantArraySymbol<byte>(5 * 4);
        private static readonly GlobalArraySymbol<double> gGradOffsets = Gpu.DefineConstantArraySymbol<double>(5);
        private static readonly GlobalVariableSymbol<double> gGradStep = Gpu.DefineConstantVariableSymbol<double>();

        private static readonly GlobalVariableSymbol<int> gStrideX = Gpu.DefineConstantVariableSymbol<int>();
        private static readonly GlobalVariableSymbol<int> gStrideY = Gpu.DefineConstantVariableSymbol<int>();

        private static readonly GlobalArraySymbol<double> gIniModelParams = Gpu.DefineConstantArraySymbol<double>(10);
        private static readonly GlobalArraySymbol<double> gMP = Gpu.DefineConstantArraySymbol<double>(11);

        private static readonly GlobalArraySymbol<double> gToModel = Gpu.DefineConstantArraySymbol<double>(4);

        #region Public Props
        public double Time { get { return t; } set { t = value; } }
        public double TimeStep { get; set; }

        public double Xo { get { return xo; } set { xo = value; xoIni = value; } }
        public double Yo { get { return yo; } set { yo = value; yoIni = value; } }

        public double RangeX { get { return Dx; } set { if (value > 0) { Dx = value; DxIni = value; } } }
        public double RangeY { get { return Dy; } set { if (value > 0) { Dy = value; DyIni = value; } } }

        public double CenterX { get { return xC; } set { xC = value; xCini = value; } }
        public double CenterY { get { return yC; } set { yC = value; yCini = value; } }

        public double Amp { get { return A; } set { if (value > 0) { A = value; Aini = value; } } }

        public double Kx { get { return kx; } set { kx = value; kxIni = value; } }
        public double Ky { get { return ky; } set { ky = value; kyIni = value; } }

        public double Exp { get { return exp; } set { exp = Math.Abs(value); expIni = exp; } }
        #endregion

        public double Measure(double x, double y)
        {
            double dx = kx * (x - xC);
            double dy = ky * (y - yC);

            double r2 = dx * dx + dy * dy;

            return A / (Math.Pow(r2, exp) + 1);
        }

        public double Measure(double[] point)
        {
            double dx = kx * (point[0] - xC);
            double dy = ky * (point[1] - yC);

            double r2 = dx * dx + dy * dy;

            return A / (Math.Pow(r2, exp) + 1);
        }

        public double Measure(Point point)
        {
            double dx = kx * (point.X - xC);
            double dy = ky * (point.Y - yC);

            double r2 = dx * dx + dy * dy;

            return A / (Math.Pow(r2, exp) + 1);
        }

        public double Measure(IVoxel V)
        {
            double x = V.Origin[0] + 0.5 * V.GridStep[0];
            double y = V.Origin[1] + 0.5 * V.GridStep[1];

            double dx = kx * (x - xC);
            double dy = ky * (y - yC);

            double r2 = dx * dx + dy * dy;

            return A / (Math.Pow(r2, exp) + 1);
        }

        public double Normalize(double value)
        {
            return value / A;
        }

        public void ResetSimulation()
        {
            t = 0;
            xo = xoIni; yo = yoIni;
            Dx = DxIni; Dy = DyIni;
            xC = xCini; yC = yCini;
            A = Aini;
            kx = kxIni; ky = kyIni;
            exp = expIni;
        }

        private const double W = 2 * Math.PI;

        public void IncrementTime(double dt)
        {
            TimeStep = dt;

            t += dt;

            xC = xCini + 500 * Math.Sin(W * t / 50000);
            yC = yCini + 100 * Math.Cos(W * t / 50000);
            exp = 7 * (1 - Math.Cos(W * t / 90000)) + 0.25;
        }

        public void SetDrawing(int width, int height, byte[] pixelArray, byte[] gradColorsBgra, Matrix trmM)
        {
            pxW = width;
            pxH = height;
            pxArray = pixelArray;

            rowStride = pxW * 4;

            if (pxArray.Length / rowStride != pxH)
                throw new ArgumentException("Inconsistent parameters values");

            if (gradColorsBgra.Length != 20)
                throw new ArgumentException("gradColorsBgra length should be equal to 4*5");

            // --- Set GPU params ------------------
            var block = new dim3(16, 16);
            var grid = new dim3(pxW / block.x, pxH / block.y);

            LP = new LaunchParam(grid, block);

            // allocate result array
            gPxArray = gpu.Allocate<byte>(pxArray.Length);

            // --- copy constants ---
            gpu.Copy(grid.x * block.x, gStrideX);
            gpu.Copy(grid.y * block.y, gStrideY);

            gpu.Copy(new double[10] { xoIni, yoIni, DxIni, DyIni, xCini, yCini, Aini, kxIni, kyIni, expIni }, gIniModelParams);
            gpu.Copy(pxW, gPxW);
            gpu.Copy(pxH, gPxH);
            gpu.Copy(rowStride, gPxStride);

            double[] gradStops = new double[5] { 0, 0.25, 0.5, 0.75, 1 };
            gpu.Copy(gradColorsBgra, gGradColors);
            gpu.Copy(gradStops, gGradOffsets);
            gpu.Copy(gradStops[1] - gradStops[0], gGradStep);

            // set canvas-to-model-transform
            double[] trm = new double[4] { trmM.M11, trmM.M22, trmM.OffsetX, trmM.OffsetY };
            gpu.Copy(trm, gToModel);
        }

        public void FreeMemory()
        {
            if (gPxArray != null) Gpu.Free(gPxArray);

            Gpu.FreeAllImplicitMemory(true);
        }

        public void DrawFrame()
        {
            FillModelParams();

            gpu.Copy(modelParams, gMP);

            gpu.Launch(Kernel, LP, gPxArray);

            Gpu.Copy(gPxArray, pxArray);
        }

        public static void Kernel(byte[] gPxArray)
        {
            var px = blockIdx.x * blockDim.x + threadIdx.x;
            var py = blockIdx.y * blockDim.y + threadIdx.y;

            for (int jy = py; jy < gPxH.Value; jy += gStrideY.Value)
            {
                for (int ix = px; ix < gPxW.Value; ix += gStrideX.Value)
                {
                    // --- Calc the model value at the pixel (ix, jy) -----------------------------
                    // { t, xo, yo, Dx, Dy, xC, yC, A, kx, ky, exp }
                    // { 0, 1,  2,  3,  4,  5,  6,  7, 8,  9,  10  }

                    // transform pixel coords to model space
                    double dx = gToModel[0] * ix + gToModel[2];
                    double dy = gToModel[1] * jy + gToModel[3];

                    dx = gMP[8] * (dx - gMP[5]);
                    dy = gMP[9] * (dy - gMP[6]);

                    double val = dx * dx + dy * dy;

                    val = gMP[7] / (gmath.Pow(val, gMP[10]) + 1);

                    // normalize val
                    val /= gMP[7];

                    // convert val to BGRA pixel
                    byte B = 0, G = 0, R = 0, A = 0;

                    for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
                    {
                        double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                        B += (byte)(factor * gGradColors[ci]);
                        G += (byte)(factor * gGradColors[ci + 1]);
                        R += (byte)(factor * gGradColors[ci + 2]);
                        A += (byte)(factor * gGradColors[ci + 3]);
                    }

                    // define index of the pixel (ix, jy) blue component in the gPxArray
                    int pxInd = jy * gPxStride.Value + ix * 4;

                    // write BGRA to the gPxArray
                    gPxArray[pxInd] = B;
                    gPxArray[++pxInd] = G;
                    gPxArray[++pxInd] = R;
                    gPxArray[++pxInd] = A;
                }
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class RepulsiveSpecks : IModel
    {
        private double t;
        private double _dt;
        private double xo, yo;
        private double xb, yb;
        private double Dx = 10, Dy = 10;

        private int speckNumber = 3;

        private double[,] speckCenter;
        private double[,] speckVelocity;
        private double[,] speckAcceleration;
        private double[] speckMass;
        private double[][] speckParams; // { k, exp, expIni, expAmp, expOmega, expPhase }
        private double[,] speckVisParams; // k, exp

        private const double ClipKink = 0.2;
        private double A = 1;
        private double massFactor = 1;
        private double Vmax, dRmax;
        private double damping = 0.05;

        private byte[] pxArray;
        private int pxW, pxH;
        private int rowStride;

        #region GPU variables
        // --- GPU variables ---
        private const int TILE_COLS = 32; // !!! Should be equal to the threads in warp !!!
        private const int TILE_ROWS = 16;
        private static readonly dim3 block = new dim3(TILE_COLS, TILE_ROWS);
        private Gpu gpu = Gpu.Default;
        private LaunchParam LP;

        // --- canvas params ---
        private byte[] gPxArray;

        private static readonly GlobalVariableSymbol<int> gPxStride = Gpu.DefineConstantVariableSymbol<int>();

        private static readonly GlobalArraySymbol<byte> gGradColors = Gpu.DefineConstantArraySymbol<byte>(5 * 4);
        private static readonly GlobalArraySymbol<double> gGradOffsets = Gpu.DefineConstantArraySymbol<double>(5);
        private static readonly GlobalVariableSymbol<double> gGradStep = Gpu.DefineConstantVariableSymbol<double>();

        // Canvas-To-Model Transform
        private static readonly GlobalArraySymbol<double> gToModel = Gpu.DefineConstantArraySymbol<double>(4);

        // The Model params
        public enum MemoryUsage { GMEM, SMEM, CMEM, Constants }
        private MemoryUsage allocatedMem = MemoryUsage.GMEM;

        private double[] gCenters;
        private double[] gMP;
        private double[,] Tmp;
        private static readonly GlobalArraySymbol<double> gModelParams = Gpu.DefineConstantArraySymbol<double>(64);
        private static readonly GlobalArraySymbol<double> gSpeckCenter = Gpu.DefineConstantArraySymbol<double>(64);
        private static readonly GlobalVariableSymbol<int> gSpeckNum = Gpu.DefineConstantVariableSymbol<int>();
        private static readonly GlobalVariableSymbol<int> gPadNum = Gpu.DefineConstantVariableSymbol<int>();

        // clipping params
        private static readonly GlobalArraySymbol<double> gCP = Gpu.DefineConstantArraySymbol<double>(2);

        #endregion

        #region Public Props
        public double Time { get { return t; } set { t = value; } }
        public double TimeStep { get { return _dt; } }

        public double Xo { get { return xo; } set { xo = value; xb = xo + Dx; } }
        public double Yo { get { return yo; } set { yo = value; yb = yo + Dy; } }

        public double RangeX { get { return Dx; } set { if (value > 0) { Dx = value; xb = xo + Dx; } } }
        public double RangeY { get { return Dy; } set { if (value > 0) { Dy = value; yb = yo + Dy; } } }
        
        public double MassFactor { get { return massFactor; } set { if (value > 0) massFactor = value; } }
        public double MaxIniSpeed { get; set; }
        public double BoundsDamping { get { return damping; } set { if (value >= 0) damping = value; } }
        public double MaxStep { get; set; }
        public double Amp { get { return A; } set { if (value >= 1) { A = value; } } }

        public int SpecksNumber
        {
            get { return speckNumber; }
            set
            {
                if (speckNumber != value && value > 0)
                {
                    GenerateRndSpecks(value, MaxIniSpeed);
                    FreeMemoryModelParams();
                    SetGpuMemory();
                }
            }
        }

        public MemoryUsage MemoryAccess
        {
            get { return allocatedMem; }
            set
            {
                if (value != allocatedMem)
                {
                    allocatedMem = value;
                    FreeMemoryModelParams();
                    SetGpuMemory();
                }
            }
        }
        #endregion


        private void GenerateRndSpecks(int num = 0, double maxIniSpeed = 0)
        {
            if (num <= 0) num = speckNumber;
            else speckNumber = num;

            Vmax = Math.Abs(maxIniSpeed);

            // ini speck arrays
            speckCenter = new double[2, num];
            speckVelocity = new double[2, num];
            speckAcceleration = new double[2, num];
            speckMass = new double[num];
            speckParams = new double[num][];
            speckVisParams = new double[2, num];

            Random rnd = new Random();

            double startX = 0.5 * (xo + xb);
            double startY = 0.5 * (yo + yb);
            double range = Math.Min(Dx, Dy);
            double startR = 0.125 * range;

            for (int i = 0; i < num; i++)
            {
                // ini position
                speckCenter[0, i] = startX + startR * (2 * rnd.NextDouble() - 1);
                speckCenter[1, i] = startY + startR * (2 * rnd.NextDouble() - 1);

                // ini speed
                double v = Vmax * rnd.NextDouble();

                speckVelocity[0, i] = v * (2 * rnd.NextDouble() - 1);
                speckVelocity[1, i] = v * (2 * rnd.NextDouble() - 1);

                // ini params
                double k = (85 * rnd.NextDouble() + 15) / range;
                double expMin = 0.1;
                double expIni = 10 * rnd.NextDouble() + 5 * expMin;
                double expAmp = (expIni - expMin) * rnd.NextDouble();
                double expOmega = 2 * Math.PI / (15000 * (expAmp + 1));
                double expPhase = 2 * Math.PI * rnd.NextDouble();

                double exp = expIni + expAmp * Math.Sin(expPhase);

                speckParams[i] = new double[] { k, exp, expIni, expAmp, expOmega, expPhase };

                speckVisParams[0, i] = k;
                speckVisParams[1, i] = exp;

                // define speck inertia
                speckMass[i] = massFactor * Math.Pow(4, 1.0 / expIni) / k;
            }
        }

        public void ResetSimulation()
        {
            t = 0;
            _dt = 0;

            GenerateRndSpecks(maxIniSpeed: MaxIniSpeed);
        }

        private void SetGpuMemory()
        {
            // --- Set GPU params ------------------
            // threads/block = 32*16 = 512
            // shared_memory/block = 8*4*512 = 16384
            // max blocks/SM = 3
            var grid = new dim3(pxW / block.x, pxH / block.y);
            
            // --- allocate arrays in global memory ---
            switch (allocatedMem)
            {
                case MemoryUsage.GMEM:
                    LP = new LaunchParam(grid, block);

                    gCenters = gpu.Allocate<double>(2 * speckNumber);
                    gMP = gpu.Allocate<double>(speckVisParams.GetLength(1) * speckNumber);
                    gpu.Copy(speckNumber, gSpeckNum);

                    break;

                case MemoryUsage.SMEM:
                    LP = new LaunchParam(grid, block);

                    // calc padded array length
                    int padnum = TILE_COLS * ((speckNumber + TILE_COLS - 1) / TILE_COLS);

                    if (padnum == speckNumber)
                    {
                        gCenters = gpu.Allocate<double>(2 * speckNumber);
                        gMP = gpu.Allocate<double>(2 * speckNumber);
                    }
                    else
                    {
                        double[] pad = new double[2 * padnum];

                        for (int i = 0; i < pad.Length; i++) pad[i] = double.PositiveInfinity;

                        gCenters = gpu.Allocate(pad);
                        gMP = gpu.Allocate(pad);
                    }

                    gpu.Copy(speckNumber, gSpeckNum);
                    gpu.Copy(padnum, gPadNum);

                    break;

                case MemoryUsage.CMEM:
                    LP = new LaunchParam(grid, block);

                    gpu.Copy(Math.Min(32, speckNumber), gSpeckNum);

                    if (speckNumber > 32) Tmp = new double[2, 32];

                    break;

                case MemoryUsage.Constants:
                    LP = new LaunchParam(grid, block);

                    gCenters = gpu.Allocate<double>(2 * speckNumber);
                    gpu.Copy(speckNumber, gSpeckNum);

                    break;
            }
        }

        public void SetDrawing(int width, int height, byte[] pixelArray, byte[] gradColorsBgra, Matrix trmM)
        {
            // define max simulation time step
            if (MaxStep > 0) dRmax = MaxStep;
            else
            {
                double M11 = Math.Abs(trmM.M11);
                double M22 = Math.Abs(trmM.M22);

                dRmax = Math.Min(M11, M22);
            }
            
            // --- Set canvas and colors ---------------------------
            pxW = width;
            pxH = height;
            pxArray = pixelArray;

            rowStride = pxW * 4;

            if (pxArray.Length / rowStride != pxH)
                throw new ArgumentException("Inconsistent parameters values");

            if (width % block.x != 0 || height % block.y != 0)
                throw new ArgumentException(string.Format("Width and height should be multiples of {0} and {1} respectively", block.x, block.y));

            if (gradColorsBgra.Length != 20)
                throw new ArgumentException("gradColorsBgra length should be equal to 4*5");

            SetGpuMemory();

            gPxArray = gpu.Allocate<byte>(pxArray.Length);
            
            // --- copy constants ---
            gpu.Copy(rowStride, gPxStride);
            gpu.Copy(new double[2] { 1 - ClipKink / Math.Sqrt(2), 1 + ClipKink }, gCP);

            double[] gradStops = new double[5] { 0, 0.25, 0.5, 0.75, 1 };
            gpu.Copy(gradColorsBgra, gGradColors);
            gpu.Copy(gradStops, gGradOffsets);
            gpu.Copy(gradStops[1] - gradStops[0], gGradStep);

            // set to-model-transform
            double[] trm = new double[4] { trmM.M11, trmM.M22, trmM.OffsetX, trmM.OffsetY };
            gpu.Copy(trm, gToModel);
        }

        private void FreeMemoryModelParams()
        {
            if (gCenters != null) Gpu.Free(gCenters);
            if (gMP != null) Gpu.Free(gMP);

            Gpu.FreeAllImplicitMemory(true);
        }

        public void FreeMemory()
        {
            if (gPxArray != null) Gpu.Free(gPxArray);

            if (gCenters != null) Gpu.Free(gCenters);
            if (gMP != null) Gpu.Free(gMP);

            Gpu.FreeAllImplicitMemory(true);
        }

        public double Measure(double x, double y)
        {
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        private double Measure(int n1, int n2, double x, double y)
        {
            double sum = 0;

            // n1
            double dx = speckParams[n1][0] * (x - speckCenter[0, n1]);
            double dy = speckParams[n1][0] * (y - speckCenter[1, n1]);

            double r2 = dx * dx + dy * dy;

            sum += 1 / (1 + Math.Pow(r2, speckParams[n1][1]));

            // n2
            dx = speckParams[n2][0] * (x - speckCenter[0, n2]);
            dy = speckParams[n2][0] * (y - speckCenter[1, n2]);

            r2 = dx * dx + dy * dy;

            sum += 1 / (1 + Math.Pow(r2, speckParams[n2][1]));

            return A * sum;
        }

        private double Measure(int index, double x, double y)
        {
            double dx = speckParams[index][0] * (x - speckCenter[0, index]);
            double dy = speckParams[index][0] * (y - speckCenter[1, index]);

            double r2 = dx * dx + dy * dy;

            return A / (1 + Math.Pow(r2, speckParams[index][1]));
        }

        public double Measure(double[] point)
        {
            double sum = 0;
            double x = point[0];
            double y = point[1];

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Measure(Point point)
        {
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (point.X - speckCenter[0, i]);
                double dy = speckParams[i][0] * (point.Y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Measure(IVoxel V)
        {
            double x = V.Origin[0] + 0.5 * V.GridStep[0];
            double y = V.Origin[1] + 0.5 * V.GridStep[1];
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Normalize(double value)
        {
            //return Math.Tanh(value);

            return MathX.Clip(value, A, ClipKink);
        }

        private void AddBoundsImpact(int index)
        {
            // X
            if (speckCenter[0, index] < xo)
            {
                double impact = A * (1 + xo - speckCenter[0, index]);

                speckAcceleration[0, index] += impact / speckMass[index];
                speckAcceleration[0, index] -= damping * speckVelocity[0, index];
            }
            else if (speckCenter[0, index] > xb)
            {
                double impact = A * (1 + speckCenter[0, index] - xb);

                speckAcceleration[0, index] -= impact / speckMass[index];
                speckAcceleration[0, index] -= damping * speckVelocity[0, index];
            }
            else
            {
                speckAcceleration[0, index] += Measure(index, xo, speckCenter[1, index]) / speckMass[index];
                speckAcceleration[0, index] -= Measure(index, xb, speckCenter[1, index]) / speckMass[index];
            }

            // Y
            if (speckCenter[1, index] < yo)
            {
                double impact = A * (1 + yo - speckCenter[1, index]);

                speckAcceleration[1, index] += impact / speckMass[index];
                speckAcceleration[1, index] -= damping * speckVelocity[1, index];
            }
            else if (speckCenter[1, index] > yb)
            {
                double impact = A * (1 + speckCenter[1, index] - yb);

                speckAcceleration[1, index] -= impact / speckMass[index];
                speckAcceleration[1, index] -= damping * speckVelocity[1, index];
            }
            else
            {
                speckAcceleration[1, index] += Measure(index, speckCenter[0, index], yo) / speckMass[index];
                speckAcceleration[1, index] -= Measure(index, speckCenter[0, index], yb) / speckMass[index];
            }
        }

        public void IncrementTime(double timeSpan)
        {
            if (Vmax == 0) Vmax = Math.Max(Dx, Dy) / (1000 / 60);

            double dt = dRmax / Vmax;

            if (_dt != 0 && dt / _dt > 10) dt = 10 * _dt;

            _dt = dt; // expose used dt

            int iterNum = 1;

            if (timeSpan != 0) // then use the given timeSpan
            {
                if (timeSpan > 0) iterNum = (int)Math.Ceiling(timeSpan / dt);
                else iterNum = (int)Math.Ceiling(-timeSpan / dt);

                dt = timeSpan / iterNum;
            }

            double dt2 = 0.5 * dt * dt;

            int endex = speckNumber;
            endex--;

            for (int i = 0; i < iterNum; i++)
            {
                t += dt;

                // calc acceleration of each speck
                for (int n1 = 0; n1 < endex; n1++)
                {
                    for (int n2 = n1 + 1; n2 < speckNumber; n2++)
                    {
                        double dRx = speckCenter[0, n2] - speckCenter[0, n1];
                        double dRy = speckCenter[1, n2] - speckCenter[1, n1];

                        double dR = dRx * dRx + dRy * dRy;
                        dR = Math.Sqrt(dR);

                        dRx /= dR;
                        dRy /= dR;

                        double midRx = 0.5 * (speckCenter[0, n2] + speckCenter[0, n1]);
                        double midRy = 0.5 * (speckCenter[1, n2] + speckCenter[1, n1]);

                        // calc mutual impact between n1 and n2 specks
                        double impact = Measure(n1, n2, midRx, midRy);

                        dRx *= impact;
                        dRy *= impact;

                        // add mutual impact
                        speckAcceleration[0, n1] -= dRx / speckMass[n1];
                        speckAcceleration[1, n1] -= dRy / speckMass[n1];

                        speckAcceleration[0, n2] += dRx / speckMass[n2];
                        speckAcceleration[1, n2] += dRy / speckMass[n2];
                    }

                    // add bounds impact on speck n1
                    AddBoundsImpact(n1);
                }

                // add bounds impact on the last speck
                AddBoundsImpact(endex);

                // reset Vmax
                Vmax = -1;

                // advance params of each speck
                for (int n = 0; n < speckNumber; n++)
                {
                    // advance position
                    speckCenter[0, n] += dt * speckVelocity[0, n] + dt2 * speckAcceleration[0, n];
                    speckCenter[1, n] += dt * speckVelocity[1, n] + dt2 * speckAcceleration[1, n];

                    // advance velocity
                    speckVelocity[0, n] += dt * speckAcceleration[0, n];
                    speckVelocity[1, n] += dt * speckAcceleration[1, n];

                    // define Vmax
                    double v = speckVelocity[0, n] * speckVelocity[0, n];
                    v += speckVelocity[1, n] * speckVelocity[1, n];
                    v = Math.Sqrt(v);

                    if (v > Vmax) Vmax = v;

                    // reset acceleration
                    speckAcceleration[0, n] = 0;
                    speckAcceleration[1, n] = 0;

                    // advance speck exp
                    // { 0,   1,      2,      3,        4,        5 }
                    // { k, exp, expIni, expAmp, expOmega, expPhase }

                    double[] P = speckParams[n];

                    P[1] = P[2] + P[3] * Math.Sin(t * P[4] + P[5]);

                    speckVisParams[1, n] = P[1];
                }
            }
        }

        public void DrawFrame()
        {
            switch (allocatedMem)
            {
                case MemoryUsage.GMEM:
                    Gpu.Copy(speckCenter, gCenters);
                    Gpu.Copy(speckVisParams, gMP);

                    gpu.Launch(Kernel_GMEM, LP, gPxArray, gCenters, gMP);
                    break;

                case MemoryUsage.SMEM:
                    Gpu.Copy(speckCenter, gCenters);
                    Gpu.Copy(speckVisParams, gMP);

                    gpu.Launch(Kernel_SMEM, LP, gPxArray, gCenters, gMP);
                    break;

                case MemoryUsage.CMEM:
                    if (speckNumber > 32)
                    {
                        int n = 32 * sizeof(double);
                        int s = speckNumber * sizeof(double);

                        Buffer.BlockCopy(speckCenter, 0, Tmp, 0, n);
                        Buffer.BlockCopy(speckCenter, s, Tmp, n, n);
                        gpu.Copy(Tmp, gSpeckCenter);

                        Buffer.BlockCopy(speckVisParams, 0, Tmp, 0, n);
                        Buffer.BlockCopy(speckVisParams, s, Tmp, n, n);
                        gpu.Copy(Tmp, gModelParams);
                    }
                    else
                    {
                        gpu.Copy(speckCenter, gSpeckCenter);
                        gpu.Copy(speckVisParams, gModelParams);
                    }

                    gpu.Launch(Kernel_CMEM, LP, gPxArray);
                    break;

                case MemoryUsage.Constants:
                    Gpu.Copy(speckCenter, gCenters);

                    gpu.Launch(Kernel_const, LP, gPxArray, gCenters);
                    break;
            }

            Gpu.Copy(gPxArray, pxArray);
        }

        [SharedMemConfig(SharedMemConfig.EightBytesBankSize)]
        public static void Kernel_SMEM(byte[] gPxArray, double[] gCenters, double[] gMP)
        {
            var shared = __shared__.Array2D<double>(TILE_ROWS, 4 * TILE_COLS);

            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            int offset_y = TILE_COLS;
            int offset_k = offset_y + TILE_COLS;
            int offset_e = offset_k + TILE_COLS;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;

            for (int n = 0, num = gPadNum.Value; n < num; n += TILE_COLS)
            {
                int i0 = n + threadIdx.x;
                int i1 = i0 + gSpeckNum.Value;

                shared[threadIdx.y, threadIdx.x] = gCenters[i0];
                shared[threadIdx.y, offset_y + threadIdx.x] = gCenters[i1];

                shared[threadIdx.y, offset_k + threadIdx.x] = gMP[i0];
                shared[threadIdx.y, offset_e + threadIdx.x] = gMP[i1];

                for (int i = 0; i < TILE_COLS; i++)
                {
                    if (double.IsInfinity(shared[threadIdx.y, i])) break;

                    double dx = x - shared[threadIdx.y, i];
                    double dy = y - shared[threadIdx.y, offset_y + i];

                    double k = shared[threadIdx.y, offset_k + i];

                    dx *= k;
                    dy *= k;

                    dx = dx * dx + dy * dy;

                    val += 1.0 / (1.0 + gmath.Pow(dx, shared[threadIdx.y, offset_e + i]));
                }
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_GMEM(byte[] gPxArray, double[] gCenters, double[] gMP)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double k = gMP[i];
                double dx = k * (x - gCenters[i]);
                double dy = k * (y - gCenters[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, gMP[offset + i]));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_const(byte[] gPxArray, double[] gCenters)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double dx = 0.25 * (x - gCenters[i]);
                double dy = 0.25 * (y - gCenters[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, 2));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_CMEM(byte[] gPxArray)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double k = gModelParams[i];
                double dx = k * (x - gSpeckCenter[i]);
                double dy = k * (y - gSpeckCenter[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, gModelParams[offset + i]));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class SolarSpecks : IModel
    {
        private double t;
        private double _dt;
        private double xo, yo;
        private double xb, yb;
        private double Dx = 10, Dy = 10;

        private int speckNumber = 3;

        private double[,] speckCenter;
        private double[,] speckVelocity;
        private double[,] speckAcceleration;
        private double[] speckMass;
        private double[][] speckParams; // { k, exp }
        private double[,] speckVisParams; // k, exp

        private const double ClipKink = 0.2;
        private double A = 1;
        private double massFactor = 1;
        private double attractF = 1;
        private double repulseF = 1;
        private double tempF = 1e-003;
        private double bndDamping = 0.05;
        private double envDamping = 1e-004;
        private double Vmax, dRmax;

        private byte[] pxArray;
        private int pxW, pxH;
        private int rowStride;

        #region GPU variables
        // --- GPU variables ---
        private const int TILE_COLS = 32; // !!! Should be equal to the threads in warp !!!
        private const int TILE_ROWS = 16;
        private static readonly dim3 block = new dim3(TILE_COLS, TILE_ROWS);
        private Gpu gpu = Gpu.Default;
        private LaunchParam LP;

        // --- canvas params ---
        private byte[] gPxArray;

        private static readonly GlobalVariableSymbol<int> gPxStride = Gpu.DefineConstantVariableSymbol<int>();

        private static readonly GlobalArraySymbol<byte> gGradColors = Gpu.DefineConstantArraySymbol<byte>(5 * 4);
        private static readonly GlobalArraySymbol<double> gGradOffsets = Gpu.DefineConstantArraySymbol<double>(5);
        private static readonly GlobalVariableSymbol<double> gGradStep = Gpu.DefineConstantVariableSymbol<double>();

        // Canvas-To-Model Transform
        private static readonly GlobalArraySymbol<double> gToModel = Gpu.DefineConstantArraySymbol<double>(4);

        // The Model params
        public enum MemoryUsage { GMEM, SMEM, CMEM, Constants }
        private MemoryUsage allocatedMem = MemoryUsage.GMEM;

        private double[] gCenters;
        private double[] gMP;
        private double[,] Tmp;
        private static readonly GlobalArraySymbol<double> gModelParams = Gpu.DefineConstantArraySymbol<double>(64);
        private static readonly GlobalArraySymbol<double> gSpeckCenter = Gpu.DefineConstantArraySymbol<double>(64);
        private static readonly GlobalVariableSymbol<int> gSpeckNum = Gpu.DefineConstantVariableSymbol<int>();
        private static readonly GlobalVariableSymbol<int> gPadNum = Gpu.DefineConstantVariableSymbol<int>();

        // clipping params
        private static readonly GlobalArraySymbol<double> gCP = Gpu.DefineConstantArraySymbol<double>(2);

        #endregion

        #region Public Props
        public double Time { get { return t; } set { t = value; } }
        public double TimeStep { get { return _dt; } }

        public double Xo { get { return xo; } set { xo = value; xb = xo + Dx; } }
        public double Yo { get { return yo; } set { yo = value; yb = yo + Dy; } }

        public double RangeX { get { return Dx; } set { if (value > 0) { Dx = value; xb = xo + Dx; } } }
        public double RangeY { get { return Dy; } set { if (value > 0) { Dy = value; yb = yo + Dy; } } }
        
        public double MassFactor { get { return massFactor; } set { if (value > 0) massFactor = value; } }
        public double AttractionFactor { get { return attractF; } set { if (value >= 0) attractF = value; } }
        public double RepulsionFactor { get { return repulseF; } set { if (value >= 0) repulseF = value; } }
        public double TemperatureFactor { get { return tempF; } set { if (value > 0) tempF = value; } }
        public double MaxIniSpeed { get; set; }
        public double DragFactor { get { return envDamping; } set { if (value >= 0) envDamping = value; } }
        public double BoundsDamping { get { return bndDamping; } set { if (value >= 0) bndDamping = value; } }
        public double MaxStep { get; set; }
        public double Amp { get { return A; } set { if (value >= 1) { A = value; } } }

        public int SpecksNumber
        {
            get { return speckNumber; }
            set
            {
                if (speckNumber != value && value > 0)
                {
                    GenerateRndSpecks(value, MaxIniSpeed);
                    FreeMemoryModelParams();
                    SetGpuMemory();
                }
            }
        }

        public MemoryUsage MemoryAccess
        {
            get { return allocatedMem; }
            set
            {
                if (value != allocatedMem)
                {
                    allocatedMem = value;
                    FreeMemoryModelParams();
                    SetGpuMemory();
                }
            }
        }
        #endregion


        private void GenerateRndSpecks(int num = 0, double maxIniSpeed = 0)
        {
            if (num <= 0) num = speckNumber;
            else speckNumber = num;

            Vmax = Math.Abs(maxIniSpeed);

            // ini speck arrays
            speckCenter = new double[2, num];
            speckVelocity = new double[2, num];
            speckAcceleration = new double[2, num];
            speckMass = new double[num];
            speckParams = new double[num][];
            speckVisParams = new double[2, num];

            Random rnd = new Random();

            double startX = 0.5 * (xo + xb);
            double startY = 0.5 * (yo + yb);
            double range = Math.Min(Dx, Dy);
            double startR = 0.45 * range;

            for (int i = 0; i < num; i++)
            {
                // ini position
                speckCenter[0, i] = startX + startR * (2 * rnd.NextDouble() - 1);
                speckCenter[1, i] = startY + startR * (2 * rnd.NextDouble() - 1);

                // ini speed
                double v = Vmax * rnd.NextDouble();

                speckVelocity[0, i] = v * (2 * rnd.NextDouble() - 1);
                speckVelocity[1, i] = v * (2 * rnd.NextDouble() - 1);

                // ini params
                double k = (85 * rnd.NextDouble() + 15) / range;
                double exp = 0.3 + 15 / (1 + tempF * v * v);

                speckParams[i] = new double[] { k, exp };

                speckVisParams[0, i] = k;
                speckVisParams[1, i] = exp;

                // define speck inertia
                speckMass[i] = massFactor * Math.Pow(4, 1.0 / exp) / k;
            }
        }

        public void ResetSimulation()
        {
            t = 0;
            _dt = 0;

            GenerateRndSpecks(maxIniSpeed: MaxIniSpeed);
        }

        private void SetGpuMemory()
        {
            // --- Set GPU params ------------------
            // threads/block = 32*16 = 512
            // shared_memory/block = 8*4*512 = 16384
            // max blocks/SM = 3
            var grid = new dim3(pxW / block.x, pxH / block.y);

            // --- allocate arrays in global memory ---
            switch (allocatedMem)
            {
                case MemoryUsage.GMEM:
                    LP = new LaunchParam(grid, block);

                    gCenters = gpu.Allocate<double>(2 * speckNumber);
                    gMP = gpu.Allocate<double>(speckVisParams.GetLength(1) * speckNumber);
                    gpu.Copy(speckNumber, gSpeckNum);

                    break;

                case MemoryUsage.SMEM:
                    LP = new LaunchParam(grid, block);

                    // calc padded array length
                    int padnum = TILE_COLS * ((speckNumber + TILE_COLS - 1) / TILE_COLS);

                    if (padnum == speckNumber)
                    {
                        gCenters = gpu.Allocate<double>(2 * speckNumber);
                        gMP = gpu.Allocate<double>(2 * speckNumber);
                    }
                    else
                    {
                        double[] pad = new double[2 * padnum];

                        for (int i = 0; i < pad.Length; i++) pad[i] = double.PositiveInfinity;

                        gCenters = gpu.Allocate(pad);
                        gMP = gpu.Allocate(pad);
                    }

                    gpu.Copy(speckNumber, gSpeckNum);
                    gpu.Copy(padnum, gPadNum);

                    break;

                case MemoryUsage.CMEM:
                    LP = new LaunchParam(grid, block);

                    gpu.Copy(Math.Min(32, speckNumber), gSpeckNum);

                    if (speckNumber > 32) Tmp = new double[2, 32];

                    break;

                case MemoryUsage.Constants:
                    LP = new LaunchParam(grid, block);

                    gCenters = gpu.Allocate<double>(2 * speckNumber);
                    gpu.Copy(speckNumber, gSpeckNum);

                    break;
            }
        }

        public void SetDrawing(int width, int height, byte[] pixelArray, byte[] gradColorsBgra, Matrix trmM)
        {
            // define max simulation time step
            if (MaxStep > 0) dRmax = MaxStep;
            else
            {
                double M11 = Math.Abs(trmM.M11);
                double M22 = Math.Abs(trmM.M22);

                dRmax = Math.Min(M11, M22);
            }

            // --- Set canvas and colors ---------------------------
            pxW = width;
            pxH = height;
            pxArray = pixelArray;

            rowStride = pxW * 4;

            if (pxArray.Length / rowStride != pxH)
                throw new ArgumentException("Inconsistent parameters values");

            if (width % block.x != 0 || height % block.y != 0)
                throw new ArgumentException(string.Format("Width and height should be multiples of {0} and {1} respectively", block.x, block.y));

            if (gradColorsBgra.Length != 20)
                throw new ArgumentException("gradColorsBgra length should be equal to 4*5");

            SetGpuMemory();

            gPxArray = gpu.Allocate<byte>(pxArray.Length);
            
            // --- copy constants ---
            gpu.Copy(rowStride, gPxStride);
            gpu.Copy(new double[2] { 1 - ClipKink / Math.Sqrt(2), 1 + ClipKink }, gCP);

            double[] gradStops = new double[5] { 0, 0.25, 0.5, 0.75, 1 };
            gpu.Copy(gradColorsBgra, gGradColors);
            gpu.Copy(gradStops, gGradOffsets);
            gpu.Copy(gradStops[1] - gradStops[0], gGradStep);

            // set to-model-transform
            double[] trm = new double[4] { trmM.M11, trmM.M22, trmM.OffsetX, trmM.OffsetY };
            gpu.Copy(trm, gToModel);
        }

        private void FreeMemoryModelParams()
        {
            if (gCenters != null) Gpu.Free(gCenters);
            if (gMP != null) Gpu.Free(gMP);

            Gpu.FreeAllImplicitMemory(true);
        }

        public void FreeMemory()
        {
            if (gPxArray != null) Gpu.Free(gPxArray);

            if (gCenters != null) Gpu.Free(gCenters);
            if (gMP != null) Gpu.Free(gMP);

            Gpu.FreeAllImplicitMemory(true);
        }

        public double Measure(double x, double y)
        {
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        private double Measure(int n1, int n2, double x, double y)
        {
            double sum = 0;

            // n1
            double dx = speckParams[n1][0] * (x - speckCenter[0, n1]);
            double dy = speckParams[n1][0] * (y - speckCenter[1, n1]);

            double r2 = dx * dx + dy * dy;

            sum += 1 / (1 + Math.Pow(r2, speckParams[n1][1]));

            // n2
            dx = speckParams[n2][0] * (x - speckCenter[0, n2]);
            dy = speckParams[n2][0] * (y - speckCenter[1, n2]);

            r2 = dx * dx + dy * dy;

            sum += 1 / (1 + Math.Pow(r2, speckParams[n2][1]));

            return A * sum;
        }

        private double Measure(int index, double x, double y)
        {
            double dx = speckParams[index][0] * (x - speckCenter[0, index]);
            double dy = speckParams[index][0] * (y - speckCenter[1, index]);

            double r2 = dx * dx + dy * dy;

            return A / (1 + Math.Pow(r2, speckParams[index][1]));
        }

        public double Measure(double[] point)
        {
            double sum = 0;
            double x = point[0];
            double y = point[1];

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Measure(Point point)
        {
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (point.X - speckCenter[0, i]);
                double dy = speckParams[i][0] * (point.Y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Measure(IVoxel V)
        {
            double x = V.Origin[0] + 0.5 * V.GridStep[0];
            double y = V.Origin[1] + 0.5 * V.GridStep[1];
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Normalize(double value)
        {
            //return Math.Tanh(value);

            return MathX.Clip(value, A, ClipKink);
        }

        private void AddBoundsImpact(int index)
        {
            // X
            if (speckCenter[0, index] < xo)
            {
                double impact = A * (1 + xo - speckCenter[0, index]);

                speckAcceleration[0, index] += impact / speckMass[index];
                speckAcceleration[0, index] -= bndDamping * speckVelocity[0, index];
            }
            else if (speckCenter[0, index] > xb)
            {
                double impact = A * (1 + speckCenter[0, index] - xb);

                speckAcceleration[0, index] -= impact / speckMass[index];
                speckAcceleration[0, index] -= bndDamping * speckVelocity[0, index];
            }
            else
            {
                speckAcceleration[0, index] += Measure(index, xo, speckCenter[1, index]) / speckMass[index];
                speckAcceleration[0, index] -= Measure(index, xb, speckCenter[1, index]) / speckMass[index];
            }

            // Y
            if (speckCenter[1, index] < yo)
            {
                double impact = A * (1 + yo - speckCenter[1, index]);

                speckAcceleration[1, index] += impact / speckMass[index];
                speckAcceleration[1, index] -= bndDamping * speckVelocity[1, index];
            }
            else if (speckCenter[1, index] > yb)
            {
                double impact = A * (1 + speckCenter[1, index] - yb);

                speckAcceleration[1, index] -= impact / speckMass[index];
                speckAcceleration[1, index] -= bndDamping * speckVelocity[1, index];
            }
            else
            {
                speckAcceleration[1, index] += Measure(index, speckCenter[0, index], yo) / speckMass[index];
                speckAcceleration[1, index] -= Measure(index, speckCenter[0, index], yb) / speckMass[index];
            }
        }

        public void IncrementTime(double timeSpan)
        {
            if (Vmax == 0) Vmax = Math.Max(Dx, Dy) / (1000 / 60);

            double dt = dRmax / Vmax;

            if (_dt != 0 && dt / _dt > 10) dt = 10 * _dt;

            _dt = dt; // expose used dt

            int iterNum = 1;

            if (timeSpan != 0) // then use the given timeSpan
            {
                if (timeSpan > 0) iterNum = (int)Math.Ceiling(timeSpan / dt);
                else iterNum = (int)Math.Ceiling(-timeSpan / dt);

                dt = timeSpan / iterNum;
            }

            double dt2 = 0.5 * dt * dt;

            int endex = speckNumber;
            endex--;

            for (int i = 0; i < iterNum; i++)
            {
                t += dt;

                // calc acceleration of each speck
                for (int n1 = 0; n1 < endex; n1++)
                {
                    for (int n2 = n1 + 1; n2 < speckNumber; n2++)
                    {
                        double dRx = speckCenter[0, n2] - speckCenter[0, n1];
                        double dRy = speckCenter[1, n2] - speckCenter[1, n1];

                        double dR = dRx * dRx + dRy * dRy;
                        dR = Math.Sqrt(dR);

                        dRx /= dR;
                        dRy /= dR;

                        double midRx = 0.5 * (speckCenter[0, n2] + speckCenter[0, n1]);
                        double midRy = 0.5 * (speckCenter[1, n2] + speckCenter[1, n1]);

                        // calc mutual impact between n1 and n2 specks
                        double impact = A - Measure(n1, n2, midRx, midRy);

                        impact *= impact > 0 ? attractF : repulseF;

                        dRx *= impact;
                        dRy *= impact;

                        // add mutual impact
                        speckAcceleration[0, n1] += dRx / speckMass[n1];
                        speckAcceleration[1, n1] += dRy / speckMass[n1];

                        speckAcceleration[0, n2] -= dRx / speckMass[n2];
                        speckAcceleration[1, n2] -= dRy / speckMass[n2];
                    }

                    // add drag force
                    speckAcceleration[0, n1] -= envDamping * speckVelocity[0, n1] / speckMass[n1];
                    speckAcceleration[1, n1] -= envDamping * speckVelocity[1, n1] / speckMass[n1];

                    // add bounds impact on speck n1
                    AddBoundsImpact(n1);
                }

                // add drag force
                speckAcceleration[0, endex] -= envDamping * speckVelocity[0, endex] / speckMass[endex];
                speckAcceleration[1, endex] -= envDamping * speckVelocity[1, endex] / speckMass[endex];

                // add bounds impact on the last speck
                AddBoundsImpact(endex);

                // reset Vmax
                Vmax = -1;

                // advance params of each speck
                for (int n = 0; n < speckNumber; n++)
                {
                    // advance position
                    speckCenter[0, n] += dt * speckVelocity[0, n] + dt2 * speckAcceleration[0, n];
                    speckCenter[1, n] += dt * speckVelocity[1, n] + dt2 * speckAcceleration[1, n];

                    // advance velocity
                    speckVelocity[0, n] += dt * speckAcceleration[0, n];
                    speckVelocity[1, n] += dt * speckAcceleration[1, n];

                    // define Vmax
                    double v2 = speckVelocity[0, n] * speckVelocity[0, n];
                    v2 += speckVelocity[1, n] * speckVelocity[1, n];

                    double v = Math.Sqrt(v2);

                    if (v > Vmax) Vmax = v;

                    // reset acceleration
                    speckAcceleration[0, n] = 0;
                    speckAcceleration[1, n] = 0;

                    // advance speck exp
                    double exp = 0.3 + 15 / (1 + tempF * v2);

                    speckParams[n][1] = exp;
                    speckVisParams[1, n] = exp;
                }
            }
        }

        public void DrawFrame()
        {
            switch (allocatedMem)
            {
                case MemoryUsage.GMEM:
                    Gpu.Copy(speckCenter, gCenters);
                    Gpu.Copy(speckVisParams, gMP);

                    gpu.Launch(Kernel_GMEM, LP, gPxArray, gCenters, gMP);
                    break;

                case MemoryUsage.SMEM:
                    Gpu.Copy(speckCenter, gCenters);
                    Gpu.Copy(speckVisParams, gMP);

                    gpu.Launch(Kernel_SMEM, LP, gPxArray, gCenters, gMP);
                    break;

                case MemoryUsage.CMEM:
                    if (speckNumber > 32)
                    {
                        int n = 32 * sizeof(double);
                        int s = speckNumber * sizeof(double);

                        Buffer.BlockCopy(speckCenter, 0, Tmp, 0, n);
                        Buffer.BlockCopy(speckCenter, s, Tmp, n, n);
                        gpu.Copy(Tmp, gSpeckCenter);

                        Buffer.BlockCopy(speckVisParams, 0, Tmp, 0, n);
                        Buffer.BlockCopy(speckVisParams, s, Tmp, n, n);
                        gpu.Copy(Tmp, gModelParams);
                    }
                    else
                    {
                        gpu.Copy(speckCenter, gSpeckCenter);
                        gpu.Copy(speckVisParams, gModelParams);
                    }

                    gpu.Launch(Kernel_CMEM, LP, gPxArray);
                    break;

                case MemoryUsage.Constants:
                    Gpu.Copy(speckCenter, gCenters);

                    gpu.Launch(Kernel_const, LP, gPxArray, gCenters);
                    break;
            }

            Gpu.Copy(gPxArray, pxArray);
        }

        [SharedMemConfig(SharedMemConfig.EightBytesBankSize)]
        public static void Kernel_SMEM(byte[] gPxArray, double[] gCenters, double[] gMP)
        {
            var shared = __shared__.Array2D<double>(TILE_ROWS, 4 * TILE_COLS);

            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            int offset_y = TILE_COLS;
            int offset_k = offset_y + TILE_COLS;
            int offset_e = offset_k + TILE_COLS;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;

            for (int n = 0, num = gPadNum.Value; n < num; n += TILE_COLS)
            {
                int i0 = n + threadIdx.x;
                int i1 = i0 + gSpeckNum.Value;

                shared[threadIdx.y, threadIdx.x] = gCenters[i0];
                shared[threadIdx.y, offset_y + threadIdx.x] = gCenters[i1];

                shared[threadIdx.y, offset_k + threadIdx.x] = gMP[i0];
                shared[threadIdx.y, offset_e + threadIdx.x] = gMP[i1];

                for (int i = 0; i < TILE_COLS; i++)
                {
                    if (double.IsInfinity(shared[threadIdx.y, i])) break;

                    double dx = x - shared[threadIdx.y, i];
                    double dy = y - shared[threadIdx.y, offset_y + i];

                    double k = shared[threadIdx.y, offset_k + i];

                    dx *= k;
                    dy *= k;

                    dx = dx * dx + dy * dy;

                    val += 1.0 / (1.0 + gmath.Pow(dx, shared[threadIdx.y, offset_e + i]));
                }
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_GMEM(byte[] gPxArray, double[] gCenters, double[] gMP)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double k = gMP[i];
                double dx = k * (x - gCenters[i]);
                double dy = k * (y - gCenters[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, gMP[offset + i]));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_const(byte[] gPxArray, double[] gCenters)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double dx = 0.25 * (x - gCenters[i]);
                double dy = 0.25 * (y - gCenters[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, 2));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_CMEM(byte[] gPxArray)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double k = gModelParams[i];
                double dx = k * (x - gSpeckCenter[i]);
                double dy = k * (y - gSpeckCenter[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, gModelParams[offset + i]));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class GravitySpecks : IModel
    {
        private double t;
        private double _dt;
        private double xo, yo;
        private double xb, yb;
        private double Dx = 10, Dy = 10;

        private int speckNumber = 3;

        private double[,] speckCenter;
        private double[,] speckVelocity;
        private double[,] speckAcceleration;
        private double[] speckMass;
        private double[][] speckParams; // { k, exp }
        private double[,] speckVisParams; // k, exp

        private const double ClipKink = 0.2;
        private double A = 1;
        private double massFactor = 1;
        private double attractF = 1;
        private double repulseF = 1;
        private double frictionF = 1;
        private double tempF = 1e-003;
        private double bndDamping = 0.05;
        private double envDamping = 1e-004;
        private double Vmax, dRmax;

        private byte[] pxArray;
        private int pxW, pxH;
        private int rowStride;

        #region GPU variables
        // --- GPU variables ---
        private const int TILE_COLS = 32; // !!! Should be equal to the threads in warp !!!
        private const int TILE_ROWS = 16;
        private static readonly dim3 block = new dim3(TILE_COLS, TILE_ROWS);
        private Gpu gpu = Gpu.Default;
        private LaunchParam LP;

        // --- canvas params ---
        private byte[] gPxArray;

        private static readonly GlobalVariableSymbol<int> gPxStride = Gpu.DefineConstantVariableSymbol<int>();

        private static readonly GlobalArraySymbol<byte> gGradColors = Gpu.DefineConstantArraySymbol<byte>(5 * 4);
        private static readonly GlobalArraySymbol<double> gGradOffsets = Gpu.DefineConstantArraySymbol<double>(5);
        private static readonly GlobalVariableSymbol<double> gGradStep = Gpu.DefineConstantVariableSymbol<double>();

        // Canvas-To-Model Transform
        private static readonly GlobalArraySymbol<double> gToModel = Gpu.DefineConstantArraySymbol<double>(4);

        // The Model params
        public enum MemoryUsage { GMEM, SMEM, CMEM, Constants }
        private MemoryUsage allocatedMem = MemoryUsage.GMEM;

        private double[] gCenters;
        private double[] gMP;
        private double[,] Tmp;
        private static readonly GlobalArraySymbol<double> gModelParams = Gpu.DefineConstantArraySymbol<double>(64);
        private static readonly GlobalArraySymbol<double> gSpeckCenter = Gpu.DefineConstantArraySymbol<double>(64);
        private static readonly GlobalVariableSymbol<int> gSpeckNum = Gpu.DefineConstantVariableSymbol<int>();
        private static readonly GlobalVariableSymbol<int> gPadNum = Gpu.DefineConstantVariableSymbol<int>();

        // clipping params
        private static readonly GlobalArraySymbol<double> gCP = Gpu.DefineConstantArraySymbol<double>(2);

        #endregion

        #region Public Props
        public double Time { get { return t; } set { t = value; } }
        public double TimeStep { get { return _dt; } }

        public double Xo { get { return xo; } set { xo = value; xb = xo + Dx; } }
        public double Yo { get { return yo; } set { yo = value; yb = yo + Dy; } }

        public double RangeX { get { return Dx; } set { if (value > 0) { Dx = value; xb = xo + Dx; } } }
        public double RangeY { get { return Dy; } set { if (value > 0) { Dy = value; yb = yo + Dy; } } }
        
        public double MassFactor { get { return massFactor; } set { if (value > 0) massFactor = value; } }
        public double AttractionFactor { get { return attractF; } set { if (value >= 0) attractF = value; } }
        public double RepulsionFactor { get { return repulseF; } set { if (value >= 0) repulseF = value; } }
        public double FrictionFactor { get { return frictionF; } set { if (value >= 0) frictionF = value; } }
        public double TemperatureFactor { get { return tempF; } set { if (value > 0) tempF = value; } }
        public double MaxIniSpeed { get; set; }
        public double DragFactor { get { return envDamping; } set { if (value >= 0) envDamping = value; } }
        public double BoundsDamping { get { return bndDamping; } set { if (value >= 0) bndDamping = value; } }
        public double MaxStep { get { return dRmax; } set { if (value > 0) dRmax = value; } }
        public double Amp { get { return A; } set { if (value >= 1) { A = value; } } }

        public int SpecksNumber
        {
            get { return speckNumber; }
            set
            {
                if (speckNumber != value && value > 0)
                {
                    GenerateRndSpecks(value, MaxIniSpeed);
                    FreeMemoryModelParams();
                    SetGpuMemory();
                }
            }
        }

        public MemoryUsage MemoryAccess
        {
            get { return allocatedMem; }
            set
            {
                if (value != allocatedMem)
                {
                    allocatedMem = value;
                    FreeMemoryModelParams();
                    SetGpuMemory();
                }
            }
        }
        #endregion


        private void GenerateRndSpecks(int num = 0, double maxIniSpeed = 0)
        {
            if (num <= 0) num = speckNumber;
            else speckNumber = num;

            Vmax = Math.Abs(maxIniSpeed);

            // ini speck arrays
            speckCenter = new double[2, num];
            speckVelocity = new double[2, num];
            speckAcceleration = new double[2, num];
            speckMass = new double[num];
            speckParams = new double[num][];
            speckVisParams = new double[2, num];

            Random rnd = new Random();

            double startX = 0.5 * (xo + xb);
            double startY = 0.5 * (yo + yb);
            double range = Math.Min(Dx, Dy);
            double startR = 0.45 * range;

            for (int i = 0; i < num; i++)
            {
                // ini position
                speckCenter[0, i] = startX + startR * (2 * rnd.NextDouble() - 1);
                speckCenter[1, i] = startY + startR * (2 * rnd.NextDouble() - 1);

                // ini speed
                double v = Vmax * rnd.NextDouble();

                speckVelocity[0, i] = v * (2 * rnd.NextDouble() - 1);
                speckVelocity[1, i] = v * (2 * rnd.NextDouble() - 1);

                // ini params
                double k = (85 * rnd.NextDouble() + 15) / range;
                double exp = 0.85 + 15 / (1 + tempF * v * v);
                double r = Math.Pow(3, 1.0 / exp) / k;

                speckParams[i] = new double[] { k, exp, r };

                speckVisParams[0, i] = k;
                speckVisParams[1, i] = exp;

                // define speck inertia
                speckMass[i] = massFactor * r * r;
            }
        }

        public void ResetSimulation()
        {
            t = 0;
            _dt = 0;

            GenerateRndSpecks(maxIniSpeed: MaxIniSpeed);
        }

        private void SetGpuMemory()
        {
            // --- Set GPU params ------------------
            // threads/block = 32*16 = 512
            // shared_memory/block = 8*4*512 = 16384
            // max blocks/SM = 3
            var grid = new dim3(pxW / block.x, pxH / block.y);

            // --- allocate arrays in global memory ---
            switch (allocatedMem)
            {
                case MemoryUsage.GMEM:
                    LP = new LaunchParam(grid, block);

                    gCenters = gpu.Allocate<double>(2 * speckNumber);
                    gMP = gpu.Allocate<double>(speckVisParams.GetLength(1) * speckNumber);
                    gpu.Copy(speckNumber, gSpeckNum);

                    break;

                case MemoryUsage.SMEM:
                    LP = new LaunchParam(grid, block);

                    // calc padded array length
                    int padnum = TILE_COLS * ((speckNumber + TILE_COLS - 1) / TILE_COLS);

                    if (padnum == speckNumber)
                    {
                        gCenters = gpu.Allocate<double>(2 * speckNumber);
                        gMP = gpu.Allocate<double>(2 * speckNumber);
                    }
                    else
                    {
                        double[] pad = new double[2 * padnum];

                        for (int i = 0; i < pad.Length; i++) pad[i] = double.PositiveInfinity;

                        gCenters = gpu.Allocate(pad);
                        gMP = gpu.Allocate(pad);
                    }

                    gpu.Copy(speckNumber, gSpeckNum);
                    gpu.Copy(padnum, gPadNum);

                    break;

                case MemoryUsage.CMEM:
                    LP = new LaunchParam(grid, block);

                    gpu.Copy(Math.Min(32, speckNumber), gSpeckNum);

                    if (speckNumber > 32) Tmp = new double[2, 32];

                    break;

                case MemoryUsage.Constants:
                    LP = new LaunchParam(grid, block);

                    gCenters = gpu.Allocate<double>(2 * speckNumber);
                    gpu.Copy(speckNumber, gSpeckNum);

                    break;
            }
        }

        public void SetDrawing(int width, int height, byte[] pixelArray, byte[] gradColorsBgra, Matrix trmM)
        {
            // define max simulation time step
            if (MaxStep > 0) dRmax = MaxStep;
            else
            {
                double M11 = Math.Abs(trmM.M11);
                double M22 = Math.Abs(trmM.M22);

                dRmax = Math.Min(M11, M22);
            }

            // --- Set canvas and colors ---------------------------
            pxW = width;
            pxH = height;
            pxArray = pixelArray;

            rowStride = pxW * 4;

            if (pxArray.Length / rowStride != pxH)
                throw new ArgumentException("Inconsistent parameters values");

            if (width % block.x != 0 || height % block.y != 0)
                throw new ArgumentException(string.Format("Width and height should be multiples of {0} and {1} respectively", block.x, block.y));

            if (gradColorsBgra.Length != 20)
                throw new ArgumentException("gradColorsBgra length should be equal to 4*5");

            SetGpuMemory();

            gPxArray = gpu.Allocate<byte>(pxArray.Length);
            
            // --- copy constants ---
            gpu.Copy(rowStride, gPxStride);
            gpu.Copy(new double[2] { 1 - ClipKink / Math.Sqrt(2), 1 + ClipKink }, gCP);

            double[] gradStops = new double[5] { 0, 0.25, 0.5, 0.75, 1 };
            gpu.Copy(gradColorsBgra, gGradColors);
            gpu.Copy(gradStops, gGradOffsets);
            gpu.Copy(gradStops[1] - gradStops[0], gGradStep);

            // set to-model-transform
            double[] trm = new double[4] { trmM.M11, trmM.M22, trmM.OffsetX, trmM.OffsetY };
            gpu.Copy(trm, gToModel);
        }

        private void FreeMemoryModelParams()
        {
            if (gCenters != null) Gpu.Free(gCenters);
            if (gMP != null) Gpu.Free(gMP);

            Gpu.FreeAllImplicitMemory(true);
        }

        public void FreeMemory()
        {
            if (gPxArray != null) Gpu.Free(gPxArray);
            if (gCenters != null) Gpu.Free(gCenters);
            if (gMP != null) Gpu.Free(gMP);

            Gpu.FreeAllImplicitMemory(true);
        }

        public double Measure(double x, double y)
        {
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        private double Measure(int n1, int n2, double x, double y)
        {
            double sum = 0;

            // n1
            double dx = speckParams[n1][0] * (x - speckCenter[0, n1]);
            double dy = speckParams[n1][0] * (y - speckCenter[1, n1]);

            double r2 = dx * dx + dy * dy;

            sum += 1 / (1 + Math.Pow(r2, speckParams[n1][1]));

            // n2
            dx = speckParams[n2][0] * (x - speckCenter[0, n2]);
            dy = speckParams[n2][0] * (y - speckCenter[1, n2]);

            r2 = dx * dx + dy * dy;

            sum += 1 / (1 + Math.Pow(r2, speckParams[n2][1]));

            return A * sum;
        }

        private double Measure(int index, double x, double y)
        {
            double dx = speckParams[index][0] * (x - speckCenter[0, index]);
            double dy = speckParams[index][0] * (y - speckCenter[1, index]);

            double r2 = dx * dx + dy * dy;

            return A / (1 + Math.Pow(r2, speckParams[index][1]));
        }

        private double Measure(int sourceInd, int recipientInd)
        {
            double dx = speckParams[sourceInd][0] * (speckCenter[0, recipientInd] - speckCenter[0, sourceInd]);
            double dy = speckParams[sourceInd][0] * (speckCenter[1, recipientInd] - speckCenter[1, sourceInd]);

            double r2 = dx * dx + dy * dy;

            return A / (1 + Math.Pow(r2, speckParams[sourceInd][1]));
        }

        public double Measure(double[] point)
        {
            double sum = 0;
            double x = point[0];
            double y = point[1];

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Measure(Point point)
        {
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (point.X - speckCenter[0, i]);
                double dy = speckParams[i][0] * (point.Y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Measure(IVoxel V)
        {
            double x = V.Origin[0] + 0.5 * V.GridStep[0];
            double y = V.Origin[1] + 0.5 * V.GridStep[1];
            double sum = 0;

            for (int i = 0; i < speckNumber; i++)
            {
                double dx = speckParams[i][0] * (x - speckCenter[0, i]);
                double dy = speckParams[i][0] * (y - speckCenter[1, i]);

                double r2 = dx * dx + dy * dy;

                sum += 1 / (1 + Math.Pow(r2, speckParams[i][1]));
            }

            return A * sum;
        }

        public double Normalize(double value)
        {
            //return Math.Tanh(value);

            return MathX.Clip(value, A, ClipKink);
        }

        private void AddBoundsImpact(int index)
        {
            // X
            if (speckCenter[0, index] < xo)
            {
                double impact = A * (1 + xo - speckCenter[0, index]);

                speckAcceleration[0, index] += impact / speckMass[index];
                speckAcceleration[0, index] -= bndDamping * speckVelocity[0, index];
            }
            else if (speckCenter[0, index] > xb)
            {
                double impact = A * (1 + speckCenter[0, index] - xb);

                speckAcceleration[0, index] -= impact / speckMass[index];
                speckAcceleration[0, index] -= bndDamping * speckVelocity[0, index];
            }
            else
            {
                speckAcceleration[0, index] += Measure(index, xo, speckCenter[1, index]) / speckMass[index];
                speckAcceleration[0, index] -= Measure(index, xb, speckCenter[1, index]) / speckMass[index];
            }

            // Y
            if (speckCenter[1, index] < yo)
            {
                double impact = A * (1 + yo - speckCenter[1, index]);

                speckAcceleration[1, index] += impact / speckMass[index];
                speckAcceleration[1, index] -= bndDamping * speckVelocity[1, index];
            }
            else if (speckCenter[1, index] > yb)
            {
                double impact = A * (1 + speckCenter[1, index] - yb);

                speckAcceleration[1, index] -= impact / speckMass[index];
                speckAcceleration[1, index] -= bndDamping * speckVelocity[1, index];
            }
            else
            {
                speckAcceleration[1, index] += Measure(index, speckCenter[0, index], yo) / speckMass[index];
                speckAcceleration[1, index] -= Measure(index, speckCenter[0, index], yb) / speckMass[index];
            }
        }

        public void IncrementTime(double timeSpan)
        {
            if (Vmax == 0) Vmax = Math.Max(Dx, Dy) / (1000 / 60);

            double dt = dRmax / Vmax;

            if (_dt != 0 && dt / _dt > 10) dt = 10 * _dt;

            _dt = dt; // expose used dt

            int iterNum = 1;

            if (timeSpan != 0) // then use the given timeSpan
            {
                if (timeSpan > 0) iterNum = (int)Math.Ceiling(timeSpan / dt);
                else iterNum = (int)Math.Ceiling(-timeSpan / dt);

                dt = timeSpan / iterNum;
            }

            double dt2 = 0.5 * dt * dt;

            int endex = speckNumber;
            endex--;

            for (int i = 0; i < iterNum; i++)
            {
                t += dt;

                // calc acceleration of each speck
                for (int n1 = 0; n1 < endex; n1++)
                {
                    for (int n2 = n1 + 1; n2 < speckNumber; n2++)
                    {
                        double dRx = speckCenter[0, n2] - speckCenter[0, n1];
                        double dRy = speckCenter[1, n2] - speckCenter[1, n1];

                        double dR2 = dRx * dRx + dRy * dRy;
                        double dR = Math.Sqrt(dR2);

                        dRx /= dR;
                        dRy /= dR;

                        // calc mutual Gravity impact
                        double g;

                        if (dR < speckParams[n1][2])
                        {
                            double r1 = speckParams[n1][2];
                            r1 *= r1 * r1;

                            g = dR * attractF / r1;
                        }
                        else if (dR < speckParams[n2][2])
                        {
                            double r2 = speckParams[n2][2];
                            r2 *= r2 * r2;

                            g = dR * attractF / r2;
                        }
                        else
                        {
                            g = attractF / dR2;
                        }
                        
                        double gx = dRx * g;
                        double gy = dRy * g;

                        // add mutual Gravity impact
                        speckAcceleration[0, n1] += gx * speckMass[n2];
                        speckAcceleration[1, n1] += gy * speckMass[n2];

                        speckAcceleration[0, n2] -= gx * speckMass[n1];
                        speckAcceleration[1, n2] -= gy * speckMass[n1];
                        
                        // calc mutual Repulsion impact between n1 and n2 specks
                        double midRx = 0.5 * (speckCenter[0, n2] + speckCenter[0, n1]);
                        double midRy = 0.5 * (speckCenter[1, n2] + speckCenter[1, n1]);

                        g = Math.Pow(Measure(n1, n2, midRx, midRy), 5);
                        g *= repulseF;
                        gx = dRx * g;
                        gy = dRy * g;

                        // add mutual Repulsion impact
                        speckAcceleration[0, n1] -= gx / speckMass[n1];
                        speckAcceleration[1, n1] -= gy / speckMass[n1];

                        speckAcceleration[0, n2] += gx / speckMass[n2];
                        speckAcceleration[1, n2] += gy / speckMass[n2];

                        // calc mutual friction
                        g = frictionF * Measure(n2, n1) / speckMass[n1];
                        gx = speckVelocity[0, n1];
                        gy = speckVelocity[1, n1];

                        gx *= gx > 0 ? gx : -gx;
                        gy *= gy > 0 ? gy : -gy;

                        // add mutual friction impact
                        speckAcceleration[0, n1] -= g * gx;
                        speckAcceleration[1, n1] -= g * gy;

                        g = frictionF * Measure(n1, n2) / speckMass[n2];
                        gx = speckVelocity[0, n2];
                        gy = speckVelocity[1, n2];

                        gx *= gx > 0 ? gx : -gx;
                        gy *= gy > 0 ? gy : -gy;

                        speckAcceleration[0, n2] -= g * gx;
                        speckAcceleration[1, n2] -= g * gy;
                    }

                    // add enviro drag force
                    double ux = speckVelocity[0, n1];
                    double uy = speckVelocity[1, n1];

                    ux *= ux > 0 ? ux : -ux;
                    uy *= uy > 0 ? uy : -uy;

                    speckAcceleration[0, n1] -= envDamping * ux / speckMass[n1];
                    speckAcceleration[1, n1] -= envDamping * uy / speckMass[n1];

                    // add bounds impact on speck n1
                    AddBoundsImpact(n1);
                }

                // add enviro drag force
                double vx = speckVelocity[0, endex];
                double vy = speckVelocity[1, endex];

                vx *= vx > 0 ? vx : -vx;
                vy *= vy > 0 ? vy : -vy;

                speckAcceleration[0, endex] -= envDamping * vx / speckMass[endex];
                speckAcceleration[1, endex] -= envDamping * vy / speckMass[endex];

                // add bounds impact on the last speck
                AddBoundsImpact(endex);

                // reset Vmax
                Vmax = -1;

                // advance params of each speck
                for (int n = 0; n < speckNumber; n++)
                {
                    // advance position
                    speckCenter[0, n] += dt * speckVelocity[0, n] + dt2 * speckAcceleration[0, n];
                    speckCenter[1, n] += dt * speckVelocity[1, n] + dt2 * speckAcceleration[1, n];

                    // advance velocity
                    speckVelocity[0, n] += dt * speckAcceleration[0, n];
                    speckVelocity[1, n] += dt * speckAcceleration[1, n];

                    // define Vmax
                    double v2 = speckVelocity[0, n] * speckVelocity[0, n];
                    v2 += speckVelocity[1, n] * speckVelocity[1, n];

                    double v = Math.Sqrt(v2);

                    if (v > Vmax) Vmax = v;

                    // reset acceleration
                    speckAcceleration[0, n] = 0;
                    speckAcceleration[1, n] = 0;

                    // advance speck exp
                    double exp = 0.85 + 15 / (1 + tempF * v2);

                    speckParams[n][1] = exp;
                    speckVisParams[1, n] = exp;
                }
            }
        }

        public void DrawFrame()
        {
            switch (allocatedMem)
            {
                case MemoryUsage.GMEM:
                    Gpu.Copy(speckCenter, gCenters);
                    Gpu.Copy(speckVisParams, gMP);

                    gpu.Launch(Kernel_GMEM, LP, gPxArray, gCenters, gMP);
                    break;

                case MemoryUsage.SMEM:
                    Gpu.Copy(speckCenter, gCenters);
                    Gpu.Copy(speckVisParams, gMP);

                    gpu.Launch(Kernel_SMEM, LP, gPxArray, gCenters, gMP);
                    break;

                case MemoryUsage.CMEM:
                    if (speckNumber > 32)
                    {
                        int n = 32 * sizeof(double);
                        int s = speckNumber * sizeof(double);

                        Buffer.BlockCopy(speckCenter, 0, Tmp, 0, n);
                        Buffer.BlockCopy(speckCenter, s, Tmp, n, n);
                        gpu.Copy(Tmp, gSpeckCenter);

                        Buffer.BlockCopy(speckVisParams, 0, Tmp, 0, n);
                        Buffer.BlockCopy(speckVisParams, s, Tmp, n, n);
                        gpu.Copy(Tmp, gModelParams);
                    }
                    else
                    {
                        gpu.Copy(speckCenter, gSpeckCenter);
                        gpu.Copy(speckVisParams, gModelParams);
                    }

                    gpu.Launch(Kernel_CMEM, LP, gPxArray);
                    break;

                case MemoryUsage.Constants:
                    Gpu.Copy(speckCenter, gCenters);

                    gpu.Launch(Kernel_const, LP, gPxArray, gCenters);
                    break;
            }

            Gpu.Copy(gPxArray, pxArray);
        }

        [SharedMemConfig(SharedMemConfig.EightBytesBankSize)]
        public static void Kernel_SMEM(byte[] gPxArray, double[] gCenters, double[] gMP)
        {
            var shared = __shared__.Array2D<double>(TILE_ROWS, 4 * TILE_COLS);

            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            int offset_y = TILE_COLS;
            int offset_k = offset_y + TILE_COLS;
            int offset_e = offset_k + TILE_COLS;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;

            for (int n = 0, num = gPadNum.Value; n < num; n += TILE_COLS)
            {
                int i0 = n + threadIdx.x;
                int i1 = i0 + gSpeckNum.Value;

                shared[threadIdx.y, threadIdx.x] = gCenters[i0];
                shared[threadIdx.y, offset_y + threadIdx.x] = gCenters[i1];

                shared[threadIdx.y, offset_k + threadIdx.x] = gMP[i0];
                shared[threadIdx.y, offset_e + threadIdx.x] = gMP[i1];

                for (int i = 0; i < TILE_COLS; i++)
                {
                    if (double.IsInfinity(shared[threadIdx.y, i])) break;

                    double dx = x - shared[threadIdx.y, i];
                    double dy = y - shared[threadIdx.y, offset_y + i];

                    double k = shared[threadIdx.y, offset_k + i];

                    dx *= k;
                    dy *= k;

                    dx = dx * dx + dy * dy;

                    val += 1.0 / (1.0 + gmath.Pow(dx, shared[threadIdx.y, offset_e + i]));
                }
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_GMEM(byte[] gPxArray, double[] gCenters, double[] gMP)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double k = gMP[i];
                double dx = k * (x - gCenters[i]);
                double dy = k * (y - gCenters[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, gMP[offset + i]));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_const(byte[] gPxArray, double[] gCenters)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double dx = 0.25 * (x - gCenters[i]);
                double dy = 0.25 * (y - gCenters[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, 2));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }

        public static void Kernel_CMEM(byte[] gPxArray)
        {
            int px = blockIdx.x * blockDim.x + threadIdx.x;
            int py = blockIdx.y * blockDim.y + threadIdx.y;

            // transform pixel coords to model space
            double x = gToModel[0] * px + gToModel[2];
            double y = gToModel[1] * py + gToModel[3];

            double val = 0;
            int offset = gSpeckNum.Value;

            for (int i = offset - 1; i >= 0; i--)
            {
                double k = gModelParams[i];
                double dx = k * (x - gSpeckCenter[i]);
                double dy = k * (y - gSpeckCenter[offset + i]);

                dx = dx * dx + dy * dy;

                val += 1.0 / (1.0 + gmath.Pow(dx, gModelParams[offset + i]));
            }

            //val = gmath.Tanh(2.5 * val);

            // perform soft clipping
            if (val >= gCP[1]) val = 1;
            else if (val > gCP[0])
            {
                val -= gCP[0];
                val /= gCP[1] - gCP[0];

                double inv = 1 - val;

                // By = (1-t)^2 * P0y + 2*(1-t)*t*P1y + t^2 * P2y
                // P1y = P2y = threshold = 1
                val = inv * inv * gCP[0] + 2 * inv * val + val * val;
            }

            // convert val to BGRA pixel
            byte B = 0, G = 0, R = 0, A = 0;

            for (int oi = 0, ci = 0; oi < gGradOffsets.Length; oi++, ci += 4)
            {
                double factor = gmath.Max(0, 1 - gmath.Abs(val - gGradOffsets[oi]) / gGradStep.Value);

                B += (byte)(factor * gGradColors[ci]);
                G += (byte)(factor * gGradColors[ci + 1]);
                R += (byte)(factor * gGradColors[ci + 2]);
                A += (byte)(factor * gGradColors[ci + 3]);
            }

            // define index of the pixel (ix, jy) blue component in the gPxArray
            int pxInd = py * gPxStride.Value + px * 4;

            // write BGRA to the gPxArray
            gPxArray[pxInd] = B;
            gPxArray[++pxInd] = G;
            gPxArray[++pxInd] = R;
            gPxArray[++pxInd] = A;
        }
    }
}
