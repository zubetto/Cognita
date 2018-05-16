using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using Cognita;
using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;
using RBF = Cognita.SupervisedLearning.RadialBasisFunction;

namespace TestWpf_RBFV
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private static int _pW, _pH;
        private static double _addW, _addH, _minH;
        private static double _toDevM11, _toDevM22;

        private double[] SpaceDim = new double[4] { 0.0, 0.0, 1.0, 1.0 }; // {X0, Y0, Xmax, Ymax}
        private int[] rootGridShape = new int[4] { 3, 3, 960, 960 };
        private int[] tessFactors = new int[2] { 2, 2 };

        private AdaptiveGrid<double> adaptiveGrid;
        private IList<IVoxel> voxelArray;
        private int tessMultp;
        private int tessNum = 3000;

        private static Matrix ModelToCanvas = Matrix.Identity;
        private static Matrix CanvasToModel = Matrix.Identity;

        private static readonly PixelFormat _pF = PixelFormats.Pbgra32;
        public const int BYTESPP = 4;

        private int pixel_4B;
        private byte[] pixelTmp1;
        private byte[] pixelTmp2;
        private byte[] pixelArrTmp1;
        private byte[] pixelArrTmp2;

        private Color colorLine;
        private Color colorLimpid;
        private Color[] orderedGradColors;
        private double[] orderedGradOffsets;

        private WriteableBitmap gridBitmap;
        private WriteableBitmap signalBitmap;
        private byte[] gridPixelArray;
        private byte[] modelPixelArray;
        private int rowStride;
        private double[] placementTmp = new double[4]; // { pointX, pointY, stepX, stepY }
        private double[] vectorTmp = new double[2]; // { pointX, pointY }

        public MainWindow()
        {
            InitializeComponent();
            Loaded += SetDPI;

            // store values specified in designer
            _addW = Width - MainCanvas.Width;
            _addH = Height - MainCanvas.Height;
            _minH = Height;

            // set the model
            SetModel();

            // ini settings-block text
            tessNum *= rootGridShape[0] * rootGridShape[1];

            //RootShapeTxtBox.Text = ArrayToString(rootGridShape);
            //TessShapeTxtBox.Text = ArrayToString(tessFactors);
            //TessNumTxtBox.Text = tessNum.ToString();
            //ThresholdTxtBox.Text = refinerThold.ToString();
        }

        private void SetDPI(object sender, RoutedEventArgs e)
        {
            HwndSource WinHS = PresentationSource.FromVisual(this) as HwndSource;

            var ToDev = WinHS.CompositionTarget.TransformToDevice;

            _toDevM11 = ToDev.M11;
            _toDevM22 = ToDev.M22;

            if (ToDev.M11 != 1 || ToDev.M22 != 1)
            {
                ScaleTransform dpiTransform = new ScaleTransform(1 / ToDev.M11, 1 / ToDev.M22);
                if (dpiTransform.CanFreeze) dpiTransform.Freeze();
                RootGrid.LayoutTransform = dpiTransform;

                _minH /= _toDevM22;
            }

            Loaded -= SetDPI;

            SetModelToCanvasTransform(true);
            SetColors();
            IniBitmaps();
            IniGrid();
            //SetGPU();

            //ResetAvg();
            UpdateGI();
            UpdateInfo();
        }

        private void SetModel()
        {
            double dx = SpaceDim[2] - SpaceDim[0];
            double dy = SpaceDim[3] - SpaceDim[1];

            dx *= dx;
            dy *= dy;

            double dr = Math.Sqrt(dx + dy);

            RBF.Amplitude = 1.0;
            RBF.Factor = 33.0;
            RBF.Exponent = 1;
            RBF.CalcRanges(0.01, dr, 1.0);
        }

        private void SetModelToCanvasTransform(bool fit)
        {
            // Set dimensions
            _pW = rootGridShape[2];
            _pH = rootGridShape[3];

            Width = (_pW + _addW) / _toDevM11;
            Height = (_pH + _addH) / _toDevM22;

            if (Height < _minH) Height = _minH;

            MainCanvas.Width = _pW;
            MainCanvas.Height = _pH;

            // Set matrix
            if (fit)
            {
                ModelToCanvas.M11 = _pW / (SpaceDim[2] - SpaceDim[0]);
                ModelToCanvas.M22 = -_pH / (SpaceDim[3] - SpaceDim[1]);
            }
            else
            {
                ModelToCanvas.M11 = 1.0;
                ModelToCanvas.M22 = -1.0;
            }

            ModelToCanvas.OffsetX = -SpaceDim[0] * ModelToCanvas.M11; // -Model.Xo * ModelToCanvas.M11;
            ModelToCanvas.OffsetY = _pH - SpaceDim[1] * ModelToCanvas.M22; // _pH - Model.Yo * ModelToCanvas.M22;

            // Set inverse transform
            CanvasToModel.M11 = 1 / ModelToCanvas.M11;
            CanvasToModel.M22 = 1 / ModelToCanvas.M22;
            CanvasToModel.OffsetX = -ModelToCanvas.OffsetX / ModelToCanvas.M11;
            CanvasToModel.OffsetY = -ModelToCanvas.OffsetY / ModelToCanvas.M22;
        }

        #region Space Transformation Methods
        private static void PointToCanvas(ref double x, ref double y)
        {
            x *= ModelToCanvas.M11;
            x += ModelToCanvas.OffsetX;

            y *= ModelToCanvas.M22;
            y += ModelToCanvas.OffsetY;
        }

        private static void PointToCanvas(double[] point)
        {
            point[0] *= ModelToCanvas.M11;
            point[0] += ModelToCanvas.OffsetX;

            point[1] *= ModelToCanvas.M22;
            point[1] += ModelToCanvas.OffsetY;
        }

        private static void StepToCanvas(double[] step)
        {
            step[0] *= ModelToCanvas.M11;
            step[1] *= -ModelToCanvas.M22;
        }

        private static void PlacementToCanvas(double[] placement)
        {
            // point transformation
            placement[0] *= ModelToCanvas.M11;
            placement[0] += ModelToCanvas.OffsetX;

            placement[1] *= ModelToCanvas.M22;
            placement[1] += ModelToCanvas.OffsetY;

            // step transformation
            placement[2] *= ModelToCanvas.M11;
            placement[3] *= -ModelToCanvas.M22;
        }

        private static void PointToModel(ref Point point)
        {
            point.X = (point.X - ModelToCanvas.OffsetX) / ModelToCanvas.M11;

            point.Y = (point.Y - ModelToCanvas.OffsetY) / ModelToCanvas.M22;
        }

        private static void PointToModel(Point pointCanvas, double[] pointModel)
        {
            pointModel[0] = pointCanvas.X / ModelToCanvas.M11 - ModelToCanvas.OffsetX;

            pointModel[1] = (pointCanvas.Y - ModelToCanvas.OffsetY) / ModelToCanvas.M22;
        }

        private static void PointToModel(double[] point)
        {
            point[0] -= ModelToCanvas.OffsetX;
            point[0] /= ModelToCanvas.M11;

            point[1] -= ModelToCanvas.OffsetY;
            point[1] /= ModelToCanvas.M22;
        }
        #endregion

        private void SetColors()
        {
            if (BYTESPP != 4)
                throw new Exception("The pixel_4B is intended to use only with the format of four bytes per pixel");

            // grid lines color
            colorLine = (GridBorderRect.Stroke as SolidColorBrush).Color;
            double mult = colorLine.A / 255.0;

            colorLine.R = (byte)(colorLine.R * mult);
            colorLine.G = (byte)(colorLine.G * mult);
            colorLine.B = (byte)(colorLine.B * mult);

            // limpid color
            colorLimpid = Color.FromArgb(0, 0, 0, 0);
            pixel_4B = 0;

            // get gradient stops in ascending order
            var gstops = SignalGradient.GradientStops.OrderBy(s => s.Offset);

            orderedGradColors = gstops.Select(s => s.Color).ToArray();
            orderedGradOffsets = gstops.Select(s => s.Offset).ToArray();

            // hide template controls
            GradientRect.Visibility = Visibility.Collapsed;
        }

        private void IniBitmaps()
        {
            // Set grid bitmap
            rowStride = _pW * BYTESPP;

            gridPixelArray = new byte[rowStride * _pH];
            modelPixelArray = new byte[rowStride * _pH];
            pixelTmp1 = new byte[BYTESPP];
            pixelTmp2 = new byte[BYTESPP];
            pixelArrTmp1 = new byte[rowStride];
            pixelArrTmp2 = new byte[rowStride];

            gridBitmap = new WriteableBitmap(_pW, _pH, 96, 96, _pF, null);
            signalBitmap = new WriteableBitmap(_pW, _pH, 96, 96, _pF, null);

            GridImage.Source = gridBitmap;
            ModelImage.Source = signalBitmap;
        }

        private void IniGrid()
        {
            // Set grid params
            int numX = rootGridShape[0];
            int numY = rootGridShape[1];
            
            double[,] shape = new double[2, 3]
            {
                { SpaceDim[0], SpaceDim[2], numX },
                { SpaceDim[1], SpaceDim[3], numY }
            };

            adaptiveGrid = new AdaptiveGrid<double>(tessNum, shape, tessFactors);
            voxelArray = adaptiveGrid.InternalArray;
            tessMultp = adaptiveGrid.TessellationMultp;

            // Ini drawings
            DrawRootGrid();
        }

        private static double Normalize(double input)
        {
            input = MathX.ClipAbs(input, 0.5, 0.5);
            input += 0.5;

            return input;
        }

        private static void SetPixelBgra(byte[] pixel, Color color)
        {
            pixel[0] = color.B;
            pixel[1] = color.G;
            pixel[2] = color.R;
            pixel[3] = color.A;
        }

        private void SetPixelBgra(byte[] pixel, double value)
        {
            if (value <= 0)
            {
                Color color = orderedGradColors[0];

                pixel[0] = color.B;
                pixel[1] = color.G;
                pixel[2] = color.R;
                pixel[3] = color.A;
            }
            else if (value >= 1)
            {
                Color color = orderedGradColors[orderedGradColors.Length - 1];

                pixel[0] = color.B;
                pixel[1] = color.G;
                pixel[2] = color.R;
                pixel[3] = color.A;
            }
            else
            {
                for (int i = 1; i < orderedGradOffsets.Length; i++)
                {
                    Color after = orderedGradColors[i];
                    double offset = orderedGradOffsets[i];

                    if (value < offset)
                    {
                        i--;
                        double offset0 = orderedGradOffsets[i];
                        Color before = orderedGradColors[i];

                        double ratio = (value - offset0) / (offset - offset0);

                        pixel[0] = (byte)(before.B + ratio * (after.B - before.B));
                        pixel[1] = (byte)(before.G + ratio * (after.G - before.G));
                        pixel[2] = (byte)(before.R + ratio * (after.R - before.R));
                        pixel[3] = (byte)(before.A + ratio * (after.A - before.A));

                        return;
                    }
                    else if (value == offset)
                    {
                        pixel[0] = after.B;
                        pixel[1] = after.G;
                        pixel[2] = after.R;
                        pixel[3] = after.A;

                        return;
                    }
                }
            }
        }

        private void DrawRootGrid(bool updateBitmap = true)
        {
            SetPixelBgra(pixelTmp1, colorLine);
            SetPixelBgra(pixelTmp2, colorLimpid);

            double dw = 1.0 * _pW / rootGridShape[0];
            double dh = 1.0 * _pH / rootGridShape[1];

            int cellW = 0;
            int cellH = 0;

            double x = 0;

            // set solid and dotted lines
            for (int i = 0, clm = 0; i < rowStride; i += BYTESPP, clm++)
            {
                // solid line
                Buffer.BlockCopy(pixelTmp1, 0, pixelArrTmp1, i, BYTESPP);

                // dotted line
                if (clm == cellW)
                {
                    x += dw;
                    cellW = (int)Math.Round(x);

                    if (cellW >= _pW)
                        cellW = _pW - 1;

                    Buffer.BlockCopy(pixelTmp1, 0, pixelArrTmp2, i, BYTESPP);
                }
                else Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP);
            }

            double y = 0;

            // build the root grid layout formed from the solid and dotted lines
            for (int i = 0, row = 0; i < gridPixelArray.Length; i += rowStride, row++)
            {
                if (row == cellH)
                {
                    y += dh;
                    cellH = (int)Math.Round(y);

                    if (cellH >= _pH)
                        cellH = _pH - 1;

                    Buffer.BlockCopy(pixelArrTmp1, 0, gridPixelArray, i, rowStride); // draw solid line
                }
                else Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, i, rowStride); // draw dotted line
            }

            // write pixels to the bitmap
            if (updateBitmap)
                gridBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), gridPixelArray, rowStride, 0, 0);
        }

        private Stopwatch funcTimer = new Stopwatch();
        private long timeRefine, timeDrawGrid, timeDrawVoxs;

        [Flags]
        private enum GItems { None = 0, Grid = 1, Voxels = 2, Model = 4, ModelGPU = 8, All = 15 }
        
        private void UpdateGI(GItems gi = GItems.All)
        {
            // ### Diagnostic
            if (funcTimer.IsRunning)
            {
                funcTimer.Stop();
                timeRefine = funcTimer.ElapsedMilliseconds;
            }

            if (ShowGridButton.IsChecked == true && (gi & GItems.Grid) == GItems.Grid)
            {
                // ### Diagnostic
                funcTimer.Restart();

                DrawCells();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawGrid = funcTimer.ElapsedMilliseconds;
            }
            else timeDrawGrid = 0;

            if (ShowVoxsButton.IsChecked == true && (gi & GItems.Voxels) == GItems.Voxels)
            {
                // ### Diagnostic
                funcTimer.Restart();

                DrawVoxels();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxs = funcTimer.ElapsedMilliseconds;
            }
            else if (ShowModelButton.IsChecked == true && (gi & GItems.Model) == GItems.Model)
            {
                // ### Diagnostic
                funcTimer.Restart();

                //DrawModelParallel();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxs = funcTimer.ElapsedMilliseconds;
            }
            else timeDrawVoxs = 0;
        }

        private void UpdateInfo()
        {
            // count active, reserve and levels
            var VArray = adaptiveGrid.InternalArray;
            int voxels = adaptiveGrid.ActiveCount;
            int reserve = VArray.Count - voxels;
            int deepestLev = adaptiveGrid.LevelsNumber - 1;

            // output results
            VoxelsTxtBlock.Text = voxels.ToString("N0");
            SlotsTxtBlock.Text = reserve.ToString("N0");
            LevelsTxtBlock.Text = deepestLev.ToString("N0");

            //// voxel count
            //if (focusedVoxel != null)
            //{
            //    if (focusedVoxel.SerialIndex != focusedVoxelSI)
            //    {
            //        focusedVoxelSI = focusedVoxel.SerialIndex;

            //        var Vc = GetVoxelCount(focusedVoxel);

            //        VoxelSerialTxtBlock.Text = focusedVoxelSI.ToString();
            //        VoxelLevelTxtBlock.Text = focusedVoxel.TessLevel.ToString();
            //        InnerVoxelsTxtBlock.Text = string.Format("{0,6}, {1,6}", Vc.Item1, Vc.Item2);
            //    }
            //}
            //else
            //{
            //    VoxelSerialTxtBlock.Text = "_";
            //    VoxelLevelTxtBlock.Text = "_";
            //    InnerVoxelsTxtBlock.Text = "_";
            //}

            //// output performance 
            //FPSTxtBlock.Text = string.Format("{0,4:N0}, {1,4:N0} avg", fps, fpsAvg);
            //SimTxtBlock.Text = timeAdvModel == 0 ? "<1" : timeAdvModel.ToString();
            //RefineTxtBlock.Text = string.Format("{0,4:N0}, {1,4:N0} avg", timeRefine, timeRefineAvg);
            //DrawGridVoxsTxtBlock.Text = string.Format("{0,4:N0}, {1,4:N0}", timeDrawGrid, timeDrawVoxs);

            //// output simulation time
            //ModelTimeTxtBlock.Text = TimeSpan.FromMilliseconds(Model.Time).ToString("c");

            //if (modeldt == 0)
            //{
            //    if (Model.TimeStep == 0) ModelStepTxtBox.Text = "variable";
            //    else ModelStepTxtBox.Text = Model.TimeStep.ToString("G8");
            //}
            //else modeldt.ToString("G8");
        }

        private void DrawCells()
        {
            DrawRootGrid(false);

            // here after the DrawRootGrid method
            // the pixelTmp1 has colorLine color
            // and the pixelTmp2 has colorLimpid color;

            // --- Draw tessellation lines, level by level -------------------------
            IVoxel ivoxel;
            bool isResolvable = true;
            bool isLevelCached = false;
            int level = 0;

            int countX = adaptiveGrid.TesselationFactors[0];
            int countY = adaptiveGrid.TesselationFactors[0];

            countX--;
            countY--;

            while (isResolvable && adaptiveGrid.FirstAtLevel(level++, out ivoxel))
            {
                isLevelCached = false;

                ivoxel.GetPlacement(placementTmp);
                PlacementToCanvas(placementTmp);
                
                if ((int)Math.Round(placementTmp[2]) <= 4 ||
                    (int)Math.Round(placementTmp[3]) <= 4)
                    break;
                
                do
                {
                    ivoxel.CopyOrigin(placementTmp);
                    PointToCanvas(placementTmp);

                    double Yod = placementTmp[1] - placementTmp[3];

                    int xo = (int)Math.Round(placementTmp[0]);
                    int yo = (int)Math.Round(Yod);

                    // start index in the gridPixelArray
                    int index = xo * BYTESPP + yo * rowStride;
                    
                    if (BitConverter.ToInt32(gridPixelArray, index) != pixel_4B)
                    {
                        if (BitConverter.ToInt32(gridPixelArray, index + rowStride) != pixel_4B)
                            xo++;

                        if (BitConverter.ToInt32(gridPixelArray, index + BYTESPP) != pixel_4B)
                            yo++;

                        index = xo * BYTESPP + yo * rowStride;
                    }
                    
                    if (ivoxel.ContentSI > 0) // then draw tessellation lines
                    {
                        if (!isLevelCached)
                        {
                            isLevelCached = true;

                            adaptiveGrid.CopyGridStep(level, vectorTmp);
                            StepToCanvas(vectorTmp);

                            int tessW = (int)Math.Round(vectorTmp[0]);
                            int tessH = (int)Math.Round(vectorTmp[1]);

                            if (tessW <= 2 || tessH <= 2) { isResolvable = false; break; }
                        }

                        double d = placementTmp[0] + vectorTmp[0];
                        int notch = (int)Math.Round(d);
                        int count = countX;

                        int byteLength = 0;
                        int pos = xo;
                        
                        // set dotted line buffer
                        do
                        {
                            if (count > 0 && pos == notch)
                            {
                                count--;

                                d += vectorTmp[0];
                                notch = (int)Math.Round(d);

                                Buffer.BlockCopy(pixelTmp1, 0, pixelArrTmp2, byteLength, BYTESPP); // dot
                            }
                            else Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, byteLength, BYTESPP); // limpid
                            
                            byteLength += BYTESPP;
                            pos++;
                        }
                        while (pos < _pW && BitConverter.ToInt32(gridPixelArray, index + byteLength) == pixel_4B);
                        
                        d = Yod + vectorTmp[1];
                        notch = (int)Math.Round(d);
                        count = countY;

                        pos = yo;

                        // build the voxel grid layout formed from the solid and dotted lines
                        do
                        {
                            if (count > 0 && pos == notch)
                            {
                                count--;

                                d += vectorTmp[1];
                                notch = (int)Math.Round(d);

                                Buffer.BlockCopy(pixelArrTmp1, 0, gridPixelArray, index, byteLength); // draw solid line
                            }
                            else Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, index, byteLength); // draw dotted line

                            index += rowStride;
                            pos++;
                        }
                        while (pos < _pH && BitConverter.ToInt32(gridPixelArray, index) == pixel_4B);
                    }
                }
                while (adaptiveGrid.NextAtLevel(ref ivoxel));
            }

            // write pixels to the bitmap
            gridBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), gridPixelArray, rowStride, 0, 0);
        }

        private void DrawVoxels(int stopLevel = int.MaxValue)
        {
            IVoxel ivoxel;
            int level = 0;
            bool isResolvable = true;

            while (isResolvable && adaptiveGrid.FirstAtLevel(level++, out ivoxel))
            {
                ivoxel.GetPlacement(placementTmp);
                PlacementToCanvas(placementTmp);

                int W = (int)Math.Ceiling(placementTmp[2]);
                int H = (int)Math.Ceiling(placementTmp[3]);

                int byteLength = W * BYTESPP;

                if (W == 1 || H == 1 || level > stopLevel) isResolvable = false;

                do
                {
                    if (isResolvable && ivoxel.ContentSI > 0) continue;

                    ivoxel.CopyOrigin(placementTmp);
                    PointToCanvas(placementTmp);

                    double Yod = placementTmp[1] - placementTmp[3];

                    int xo = (int)Math.Round(placementTmp[0]);
                    int yo = (int)Math.Round(Yod);

                    if (xo + W > _pW) xo--;
                    if (yo + H > _pH) yo--;

                    SetPixelBgra(pixelTmp2, Normalize(ivoxel.Data));

                    // set color line buffer
                    for (int i = 0; i < byteLength; i += BYTESPP)
                    {
                        Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP);
                    }

                    int index0 = xo * BYTESPP + yo * rowStride;

                    // shade voxel by the color line
                    for (int i = index0, row = 0; row < H; i += rowStride, row++)
                    {
                        Buffer.BlockCopy(pixelArrTmp2, 0, modelPixelArray, i, byteLength);
                    }
                }
                while (adaptiveGrid.NextAtLevel(ref ivoxel));
            }

            // write pixels to the bitmap
            signalBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), modelPixelArray, rowStride, 0, 0);
        }

        private double[] rbfCenter = new double[2];

        private void AddRbf(double[] center, bool isNegative)
        {
            // --- Reset Grid ----------------------------
            int rootNum = adaptiveGrid.LevelsCounts[0];
            var voxelArr = adaptiveGrid.InternalArray;

            for (int i = 0; i < rootNum; i++)
                voxelArr[i].Merge(true);

            // --- Refine Grid ---------------------------
            if (isNegative && RBF.Amplitude > 0)
                RBF.Amplitude = -RBF.Amplitude;

            int level = -1;
            bool notFull = true;

            // level-loop
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                IVoxel voxel;
                if (!adaptiveGrid.FirstAtLevel(level, out voxel)) continue;

                placementTmp[2] = voxel.GridStep[0];
                placementTmp[3] = voxel.GridStep[1];

                placementTmp[0] = 0.5 * placementTmp[2];
                placementTmp[1] = 0.5 * placementTmp[3];

                placementTmp[2] *= placementTmp[2];
                placementTmp[3] *= placementTmp[3];

                double radius = 0.5 * Math.Sqrt(placementTmp[2] + placementTmp[3]);

                // traversal all voxels at the current level
                do
                {
                    // calc voxel center coords.
                    vectorTmp[0] = voxel.Origin[0] + placementTmp[0];
                    vectorTmp[1] = voxel.Origin[1] + placementTmp[1];

                    // calc distance between voxel center and rbf center
                    double dr2 = center.SquaredDistance(vectorTmp);

                    if (notFull && RBF.ThresholdExceeded(Math.Sqrt(dr2), radius))
                    {
                        if (!voxel.Tessellate(true))
                        {
                            notFull = false;
                            voxel.Data = RBF.MeasureR2(dr2);
                        }
                    }
                    else
                    {
                        voxel.Data = RBF.MeasureR2(dr2);
                    }
                }
                while (adaptiveGrid.NextAtLevel(ref voxel));
            }
        }

        /// <summary>
        /// Fills with average values the voxels which have not been assigned data 
        /// as result of refinement process performed by the AddRbf method;
        /// this method is required for proper visualisation of voxels by the DrawVoxels method
        /// </summary>
        private void FillWithAverages()
        {
            var voxelArr = adaptiveGrid.InternalArray;
            int level = adaptiveGrid.LevelsNumber;
            int tessNum = adaptiveGrid.TessellationMultp;

            // to avoid start from leafs
            --level;

            while (--level >= 0)
            {
                adaptiveGrid.FirstAtLevel(level, out IVoxel voxel);

                do
                {
                    if (voxel.ContentSI < 0) continue;

                    double avg = 0;
                    int stop = voxel.ContentSI + tessNum;

                    // calc average
                    for (int i = voxel.ContentSI; i < stop; i++)
                    {
                        avg += voxelArr[i].Data;
                    }

                    voxel.Data = avg / tessNum;
                }
                while (adaptiveGrid.NextAtLevel(ref voxel));
            }
        }

        #region Interface Event Handlers

        private void RenderBttsStack_Click(object sender, RoutedEventArgs e)
        {
            var sourceBtt = (FrameworkElement)e.OriginalSource;
            bool isSimActive = false; // StartStopButton.IsChecked == true;

            switch (sourceBtt.Name)
            {
                case "ShowGridButton":
                    if (ShowGridButton.IsChecked == true)
                    {
                        if (!isSimActive) UpdateGI(GItems.Grid);

                        GridImage.Visibility = Visibility.Visible;
                    }
                    else GridImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowVoxsButton":
                    if (ShowVoxsButton.IsChecked == true)
                    {
                        ShowModelButton.IsChecked = false;
                        ShowModelGPUButton.IsChecked = false;

                        //LDrawGridVoxsTxtBlock.Text = "Draw grid,voxels ms:";

                        if (!isSimActive) UpdateGI(GItems.Voxels);

                        ModelImage.Visibility = Visibility.Visible;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowModelButton":
                    if (ShowModelButton.IsChecked == true)
                    {
                        ShowVoxsButton.IsChecked = false;
                        ShowModelGPUButton.IsChecked = false;

                        //LDrawGridVoxsTxtBlock.Text = "Draw grid,model ms:";

                        if (!isSimActive) UpdateGI(GItems.Model);

                        ModelImage.Visibility = Visibility.Visible;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;
            }
        }

        private void MainCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point p = e.GetPosition(MainCanvas);

            PointToModel(p, rbfCenter);

            AddRbf(rbfCenter, true);
            FillWithAverages();

            UpdateGI();
            UpdateInfo();
        }

        #endregion
    }
}
