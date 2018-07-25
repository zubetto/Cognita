using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using IOPath = System.IO.Path;
using System.Linq;
using System.Text;
using System.Threading;
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
using Cognita.SupervisedLearning;
using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;
using RBF = Cognita.SupervisedLearning.RadialBasisFunction;
using ModelFunctions;

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
        private double spaceDiagonal;

        private AdaptiveGrid<double> adaptiveGrid;
        private IList<IVoxel> voxelArray;
        private int tessMultp;
        private int tessNum = 1000;

        private static Matrix ModelToCanvas = Matrix.Identity;
        private static Matrix CanvasToModel = Matrix.Identity;

        private static readonly PixelFormat _pF = PixelFormats.Pbgra32;
        public const int BYTESPP = 4;
        public const int BITMAPWF = 32;
        public const int BITMAPHF = 32;
        public const int TILEMIN = 10;

        private int pixel_4B;
        private byte[] pixelTmp1;
        private byte[] pixelTmp2;
        private byte[] pixelArrTmp1;
        private byte[] pixelArrTmp2;

        private Color colorLine;
        private Color colorLimpid;
        private Color[] orderedGradColors;
        private double[] orderedGradOffsets;
        private double outputGain = 1.0;

        private WriteableBitmap gridBitmap; // only grid lines
        private WriteableBitmap signalBitmap; // voxelizer, rbf-voxels, model
        private WriteableBitmap modelRbfBitmap; // only model-rbf
        private byte[] gridPixelArray;
        private byte[] modelPixelArray;
        private int rowStride;
        private double[] placementTmp = new double[4]; // { pointX, pointY, stepX, stepY }
        private double[] vectorTmp = new double[2]; // { pointX, pointY }

        private TasksDispenser TPSlines;

        private RadialFunction usedRbf;
        private double noiseFactor = 0.01;

        public double NoiseFactor { get { return noiseFactor; } set { if (value >= 0.0) noiseFactor = value; } }

        private int batchLength = 100;
        private double batchRatio = 1.0;

        private double[][] rbfPoints = new double[1][] { new double[2] };
        private bool[] rbfLabels = new bool[1];

        private bool isModelRbfFresh = false;

        private IModel model;
        private IPointsSource dataSource;
        
        private RBFVoxelizer voxelizer;
        private double threshold = 0.005;
        private double monotonicTol = double.NegativeInfinity;

        private int trainingNum;
        private int testNum;

        SolidColorBrush txtEditableFore;
        SolidColorBrush txtRedFore;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += SetDPI;

            // --- nset static rbf -------------------
            double dx = SpaceDim[2] - SpaceDim[0];
            double dy = SpaceDim[3] - SpaceDim[1];

            dx *= dx;
            dy *= dy;

            spaceDiagonal = Math.Sqrt(dx + dy);

            RBF.Amplitude = 1.0;
            RBF.Factor = 25;
            RBF.Exponent = 1.0;
            RBF.CalcRanges(threshold, spaceDiagonal, 1.0);

            // store values specified in designer
            _addW = Width - MainCanvas.Width;
            _addH = Height - MainCanvas.Height;
            _minH = Height;
            
            // set model list
            ModelList.ItemsSource = Enum.GetValues(typeof(Models));
            ModelList.SelectedIndex = 1;
            ModelList.SelectionChanged += SetModel;
            SetModel();

            // ini settings-block text
            tessNum *= rootGridShape[0] * rootGridShape[1];

            RootShapeTxtBox.Text = ArrayToString(rootGridShape);
            TessShapeTxtBox.Text = ArrayToString(tessFactors);
            TessNumTxtBox.Text = tessNum.ToString();
            ThresholdTxtBox.Text = threshold.ToString();
            DetailsFactorTxtBox.Text = batchRatio.ToString();
            MonotonicityTxtBox.Text = double.IsInfinity(monotonicTol) ? "infinity" : monotonicTol.ToString();

            // ini brushes
            txtEditableFore = FindResource("TxtEditableFore") as SolidColorBrush;
            txtRedFore = FindResource("TxtRedFore") as SolidColorBrush;
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

        private string[] GetFilenames(string filter)
        {
            string[] filenames = null;

            Microsoft.Win32.OpenFileDialog dlgOpenFile = new Microsoft.Win32.OpenFileDialog
            {
                Multiselect = true,
                Filter = filter,
                RestoreDirectory = true
            };

            try
            {
                if (dlgOpenFile.ShowDialog() == true)
                    filenames = dlgOpenFile.FileNames;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "Unexpected error", MessageBoxButton.OK, MessageBoxImage.Exclamation);
            }

            return filenames;
        }

        public enum Models { None, Linear2D, Wave2D, WaveHF2D, PostalZips }

        private void SetModel()
        {
            // --- Set DataModel --------------------------------------
            var region = new MathX.Hyperrect()
            {
                PointA = new double[2] { SpaceDim[0], SpaceDim[1] },
                PointB = new double[2] { SpaceDim[2], SpaceDim[3] }
            };

            Models selected = (Models)ModelList.SelectedIndex;
            bool errFlag = false;

            switch (selected)
            {
                case Models.Linear2D:
                    var linearModel = new LinearModel2D(region, 1.0 / noiseFactor);

                    model = linearModel;
                    dataSource = linearModel;
                    
                    break;

                case Models.Wave2D:
                    var waveModel = new Wave2D(region, 1.0 / noiseFactor);

                    model = waveModel;
                    dataSource = waveModel;

                    break;

                case Models.WaveHF2D:
                    waveModel = new Wave2D(region, 1.0 / noiseFactor, 15);

                    model = waveModel;
                    dataSource = waveModel;

                    break;

                case Models.PostalZips:
                    string filter = "CSV Data and Config|*.csv;*.dcfg";
                    string configFile = null;
                    string dataFile = null;

                    string[] filenames = GetFilenames(filter);

                    if (filenames == null || filenames.Length != 2)
                    {
                        errFlag = true;
                    }
                    else if (IOPath.GetExtension(filenames[0]) == ".csv")
                    {
                        dataFile = filenames[0];

                        if (IOPath.GetExtension(filenames[1]) == ".dcfg")
                            configFile = filenames[1];
                        else
                            errFlag = true;
                    }
                    else if (IOPath.GetExtension(filenames[1]) == ".csv")
                    {
                        dataFile = filenames[1];

                        if (IOPath.GetExtension(filenames[0]) == ".dcfg")
                            configFile = filenames[0];
                        else
                            errFlag = true;
                    }
                    else errFlag = true;

                    if (!errFlag)
                    {
                        try
                        {
                            var zipCodes = new PostalServiceZipCode(configFile, dataFile);

                            model = null;
                            dataSource = zipCodes;
                        }
                        catch (Exception ex)
                        {
                            errFlag = true;
                            MessageBox.Show(ex.ToString(), "Error during loading", MessageBoxButton.OK, MessageBoxImage.Exclamation);
                        }
                    }
                    else
                    {
                        MessageBox.Show("Invalid couple of data and config were provided", 
                                        "Filenames error", MessageBoxButton.OK, MessageBoxImage.Exclamation);
                    }

                    break;

                default:
                    model = null;
                    dataSource = null;

                    DataBttStack.IsEnabled = false;

                    break;
            }

            if (!errFlag && selected != Models.None)
            {
                DataBttStack.IsEnabled = true;
                PreviewNextButton.IsEnabled = true;
                PreviewButton.IsEnabled = false;
            }
            else
            {
                model = null;
                dataSource = null;

                DataBttStack.IsEnabled = false;
            }

            if (model == null)
            {
                ShowModelButton.IsChecked = false;
                ShowModelButton.IsEnabled = false;

                ErrorInButton.Content = "Err In";
                LErrorInTxtBlock.Text = "Error In:";
            }
            else
            {
                ShowModelButton.IsEnabled = true;

                UpdateGI(GItems.Model);

                ErrorInButton.Content = "Err Model";
                LErrorInTxtBlock.Text = "Error Model:";
            }

            if (dataSource == null)
            {
                ShowModelRbfButton.IsChecked = false;
                ShowModelRbfButton.IsEnabled = false;
            }
            else
            {
                ShowModelRbfButton.IsEnabled = true;
            }

            InitButton.IsEnabled = false;
        }

        private void SetModel(object sender, SelectionChangedEventArgs e)
        {
            SetModel();
            SetModelToCanvasTransform(true);
        }

        private void SetVoxelizer()
        {
            usedRbf = new RadialFunction();

            int rootTile = rootGridShape[0];
            int levelTile = tessFactors[0];

            rootGridShape[1] = rootTile;
            tessFactors[1] = levelTile;

            voxelizer = new RBFVoxelizer(dataSource, usedRbf, tessNum, rootTile, levelTile);
            voxelizer.SyncEvent += UpdateGrid;
            voxelizer.CompletionEvent += CompletionHandler;
        }

        private void GetNextDataBatch()
        {
            if (dataSource == null) return;

            dataSource.BatchLength = batchLength;

            if (!dataSource.GetNextNorm(DataSet.Training, ref rbfPoints, ref rbfLabels))
            {
                dataSource.SeekNext(DataSet.Training, 0);
                dataSource.GetNextNorm(DataSet.Training, ref rbfPoints, ref rbfLabels);
            }

            batchLength = dataSource.BatchLength;
        }

        private void GetCurrentDataBatch()
        {
            if (dataSource == null) return;

            dataSource.BatchLength = batchLength;

            dataSource.GetCurrentNorm(DataSet.Training, ref rbfPoints, ref rbfLabels);

            batchLength = dataSource.BatchLength;
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
            PointerRect.Visibility = Visibility.Collapsed;
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
            modelRbfBitmap = new WriteableBitmap(_pW, _pH, 96, 96, _pF, null);

            GridImage.Source = gridBitmap;
            ModelImage.Source = signalBitmap;
            ModelRbfImage.Source = modelRbfBitmap;

            // switch visibility of the bitmaps
            RenderBttsStack_Switch(ShowVoxsButton, false);

            // Set threads
            TPSlines = new TasksDispenser(_pH);
        }
        
        private void IniGrid()
        {
            if (voxelizer == null)
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
            }
            else
            {
                adaptiveGrid = voxelizer.FeatureSpace;
            }

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
        private long timeRefine, timeDrawGrid, timeDrawVoxsModel;

        [Flags]
        private enum GItems { None = 0, Grid = 1, Voxels = 2, Model = 4, RBF = 8, ModelRBF = 16, All = 31 }

        private delegate void GUpdater(GItems gi = GItems.All);
        private delegate void InfoUpdater();

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

                DrawVoxels(gain: voxelizer == null ? 1.0 : outputGain);

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;
            }
            else if (ShowModelButton.IsChecked == true && (gi & GItems.Model) == GItems.Model)
            {
                // ### Diagnostic
                funcTimer.Restart();

                DrawModel_Parallel();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;
            }
            else if (ShowRbfButton.IsChecked == true && (gi & GItems.RBF) == GItems.RBF)
            {
                // ### Diagnostic
                funcTimer.Restart();

                DrawRbf_Parallel();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;
            }
            else if (ShowModelRbfButton.IsChecked == true && (gi & GItems.ModelRBF) == GItems.ModelRBF)
            {
                if (isModelRbfFresh) return;

                // ### Diagnostic
                funcTimer.Restart();

                DrawModelRbf_Parallel();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;

                isModelRbfFresh = true;
            }
            else timeDrawVoxsModel = 0;
        }

        private void UpdateInfo()
        {
            // count active, reserve and levels
            var VArray = adaptiveGrid.InternalArray;
            int voxels = adaptiveGrid.ActiveCount;
            int reserve = VArray.Count - voxels;
            int deepestLev = adaptiveGrid.LevelsNumber - 1;

            double posP = 0.0;
            double negP = 0.0;

            if (voxelizer != null)
            {
                posP = 100.0 * voxelizer.PositiveNum / trainingNum;
                negP = 100.0 * voxelizer.NegativeNum / trainingNum;
            }
            else
            {
                ErrorInTxtBlock.Text = "_ %";
                ErrorOutTxtBlock.Text = "_ %";
                VoxelValueTxtBlock.Text = "_ %";
            }

            if (dataSource != null)
            {
                posP = 100.0 * dataSource.PositiveNum / dataSource.BatchLength;
                negP = 100.0 * dataSource.NegativeNum / dataSource.BatchLength;
            }

            // output Grid Info results
            VoxelsTxtBlock.Text = voxels.ToString("N0");
            SlotsTxtBlock.Text = reserve.ToString("N0");
            LevelsTxtBlock.Text = deepestLev.ToString("N0");

            // output Performance
            TrainingNumTxtBlock.Text = trainingNum.ToString("N0");
            TrainingRatioTxtBlock.Text = string.Format("{0,5:F1}, {1,5:F1} %", posP, negP);

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

        private void DrawModel_Parallel()
        {
            if (model == null) return;

            int lineInd = 0;
            int linelNum;

            TPSlines.ResetDispenser();

            var RelayComplete = new AutoResetEvent(false);
            int numOut = TPSlines.ThreadsNum;

            while (TPSlines.DispenseNext(out linelNum))
            {
                linelNum += lineInd;

                var interval = new Tuple<int, int>(lineInd, linelNum);

                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    var segment = (Tuple<int, int>)o;

                    int rowStart = segment.Item1;
                    int rowNum = segment.Item2;

                    byte[] pixel = new byte[4];
                    double[] point = new double[2];

                    for (int row = rowStart; row < rowNum; row++)
                    {
                        int start = row * rowStride;
                        int stop = start + rowStride;

                        for (int i = start, clm = 0; i < stop; i += BYTESPP, clm++)
                        {
                            point[0] = clm;
                            point[1] = row;
                            PointToModel(point);

                            SetPixelBgra(pixel, model.Measure(point));

                            Buffer.BlockCopy(pixel, 0, modelPixelArray, i, BYTESPP);
                        }
                    }

                    if (Interlocked.Decrement(ref numOut) == 0) RelayComplete.Set();
                }), interval);

                lineInd = linelNum;
            }

            RelayComplete.WaitOne();

            // write pixels to the bitmap
            signalBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), modelPixelArray, rowStride, 0, 0);
        }

        private void DrawModelRbf_Parallel()
        {
            int lineInd = 0;
            int linelNum;

            TPSlines.ResetDispenser();

            var RelayComplete = new AutoResetEvent(false);
            int numOut = TPSlines.ThreadsNum;

            while (TPSlines.DispenseNext(out linelNum))
            {
                linelNum += lineInd;

                var interval = new Tuple<int, int>(lineInd, linelNum);

                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    var segment = (Tuple<int, int>)o;

                    int rowStart = segment.Item1;
                    int rowNum = segment.Item2;

                    byte[] pixel = new byte[4];
                    double[] point = new double[2];

                    for (int row = rowStart; row < rowNum; row++)
                    {
                        int start = row * rowStride;
                        int stop = start + rowStride;

                        for (int i = start, clm = 0; i < stop; i += BYTESPP, clm++)
                        {
                            point[0] = clm;
                            point[1] = row;
                            PointToModel(point);

                            SetPixelBgra(pixel, Normalize(outputGain * RBF.MeasureBalanced(point, rbfLabels, rbfPoints)));

                            Buffer.BlockCopy(pixel, 0, modelPixelArray, i, BYTESPP);
                        }
                    }

                    if (Interlocked.Decrement(ref numOut) == 0) RelayComplete.Set();
                }), interval);

                lineInd = linelNum;
            }

            RelayComplete.WaitOne();

            // write pixels to the bitmap
            modelRbfBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), modelPixelArray, rowStride, 0, 0);
        }

        private void DrawRbf_Parallel()
        {
            int lineInd = 0;
            int linelNum;

            TPSlines.ResetDispenser();

            var RelayComplete = new AutoResetEvent(false);
            int numOut = TPSlines.ThreadsNum;

            while (TPSlines.DispenseNext(out linelNum))
            {
                linelNum += lineInd;

                var interval = new Tuple<int, int>(lineInd, linelNum);

                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    var segment = (Tuple<int, int>)o;

                    int rowStart = segment.Item1;
                    int rowNum = segment.Item2;

                    byte[] pixel = new byte[4];
                    double[] point = new double[2];

                    for (int row = rowStart; row < rowNum; row++)
                    {
                        int start = row * rowStride;
                        int stop = start + rowStride;

                        for (int i = start, clm = 0; i < stop; i += BYTESPP, clm++)
                        {
                            point[0] = clm;
                            point[1] = row;
                            PointToModel(point);

                            SetPixelBgra(pixel, Normalize(RBF.MeasureR2(rbfCenter.SquaredDistance(point))));

                            Buffer.BlockCopy(pixel, 0, modelPixelArray, i, BYTESPP);
                        }
                    }

                    if (Interlocked.Decrement(ref numOut) == 0) RelayComplete.Set();
                }), interval);

                lineInd = linelNum;
            }

            RelayComplete.WaitOne();

            // write pixels to the bitmap
            signalBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), modelPixelArray, rowStride, 0, 0);
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

        private void DrawVoxels(int stopLevel = int.MaxValue, double gain = 1.0)
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

                    SetPixelBgra(pixelTmp2, Normalize(gain * ivoxel.Data));

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

            for (int i = 0; i < rootNum; i++)
                voxelArray[i].Merge(true);

            // --- Refine Grid ---------------------------
            if (isNegative && RBF.Amplitude > 0)
                RBF.Amplitude = -RBF.Amplitude;

            int level = -1;
            bool notFull = true;

            // level-loop
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                IVoxel voxel;
                if (!adaptiveGrid.FirstAtLevel(level, out voxel))
                    break;

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
            int level = adaptiveGrid.LevelsNumber - 1;

            // tree traversal upwards starting from the penultimate level
            while (--level >= 0)
            {
                adaptiveGrid.FirstAtLevel(level, out IVoxel voxel);

                do
                {
                    if (voxel.ContentSI < 0) continue;

                    double avg = 0.0;
                    int stop = voxel.ContentSI + tessMultp;

                    // calc average
                    for (int i = voxel.ContentSI; i < stop; i++)
                    {
                        avg += voxelArray[i].Data;
                    }

                    voxel.Data = avg / tessMultp;
                }
                while (adaptiveGrid.NextAtLevel(ref voxel));
            }
        }

        private void UpdateGrid(object sender, RBFVoxelizer.SyncEventArgs sea)
        {
            trainingNum += dataSource.BatchLength;

            Dispatcher.Invoke(new GUpdater(UpdateGI), GItems.All);
            Dispatcher.Invoke(new InfoUpdater(UpdateInfo));

            if (!RefineButtonIsChecked)
            {
                sea.StopFlag = true;

                Dispatcher.Invoke(() =>
                {
                    RefineButtonIsChecked = false;
                    RefineButton.IsEnabled = true;
                    RefineButton.Content = "Refine";

                    InitButton.IsEnabled = true;
                    EvaluationStack.IsEnabled = true;
                    DataStack.IsEnabled = true;

                    batchLength = dataSource.BatchLength;
                    BatchNumTextBox.Text = batchLength.ToString("N0");
                });
            }
        }

        private void CompletionHandler(object sender, EventArgs e)
        {
            Dispatcher.Invoke(() => 
            {
                RefineButtonIsChecked = false;
                RefineButton.IsChecked = false;
                RefineButton.IsEnabled = false;
                RefineButton.Content = "Refine";

                InitButton.IsEnabled = true;
                EvaluationStack.IsEnabled = true;
                DataStack.IsEnabled = true;

                batchLength = dataSource.BatchLength;
                BatchNumTextBox.Text = batchLength.ToString("N0");
            });
        }

        //-------------------------------------------------------------------------------
        #region GUI Event Handlers
        //-------------------------------------------------------------------------------
        private bool RbfAmplitudesInput()
        {
            string[] inputStr = RBFamplitudeTextBox.Text.Split(',');

            if (inputStr.Length != 2)
                return false;

            double[] inputVal = new double[2];

            for (int i = 0; i < 2; i++)
            {
                if (!double.TryParse(inputStr[i], out double tmp))
                    return false;

                inputVal[i] = tmp;
            }

            RBF.Amplitude = inputVal[0];
            RBF.SetBinaryAmps(inputVal[0], inputVal[1]);
            RBF.CalcRanges(threshold, spaceDiagonal, 1);

            UpdateGI(GItems.RBF);

            return true;
        }

        private void RBFamplitudeTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                RbfAmplitudesInput();

                RBFamplitudeTextBox.Text = string.Format("{0:G6}, {1:G6}", RBF.AmplitudesArr[0], RBF.AmplitudesArr[1]);
            }
        }
        private void RBFamplitudeTextBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            RbfAmplitudesInput();

            RBFamplitudeTextBox.Text = string.Format("{0:G6}, {1:G6}", RBF.AmplitudesArr[0], RBF.AmplitudesArr[1]);
        }

        private void RBFfactorTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (double.TryParse(RBFfactorTextBox.Text, out double val))
                {
                    RBF.Factor = val;
                    RBF.CalcRanges(threshold, spaceDiagonal, 1);

                    UpdateGI(GItems.RBF);
                }

                RBFfactorTextBox.Text = RBF.Factor.ToString();
            }
        }
        private void RBFfactorTextBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (double.TryParse(RBFfactorTextBox.Text, out double val))
            {
                RBF.Factor = val;
                RBF.CalcRanges(threshold, spaceDiagonal, 1);

                UpdateGI(GItems.RBF);
            }

            RBFfactorTextBox.Text = RBF.Factor.ToString();
        }

        private void RBFexponentTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (double.TryParse(RBFexponentTextBox.Text, out double val))
                {
                    RBF.Exponent = val;
                    RBF.CalcRanges(threshold, spaceDiagonal, 1);

                    UpdateGI(GItems.RBF);
                }

                RBFexponentTextBox.Text = RBF.Exponent.ToString();
            }
        }
        private void RBFexponentTextBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (double.TryParse(RBFexponentTextBox.Text, out double val))
            {
                RBF.Exponent = val;
                RBF.CalcRanges(threshold, spaceDiagonal, 1);

                UpdateGI(GItems.RBF);
            }

            RBFexponentTextBox.Text = RBF.Exponent.ToString();
        }

        private void BatchNumTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (int.TryParse(BatchNumTextBox.Text, out int num) && num > 0 && num < 1000000)
                {
                    batchLength = num;
                    GetCurrentDataBatch();
                }

                BatchNumTextBox.Text = batchLength.ToString("N0");
            }
        }
        private void BatchNumTextBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (int.TryParse(BatchNumTextBox.Text, out int num) && num > 0 && num < 1000000)
            {
                batchLength = num;
                GetCurrentDataBatch();
            }

            BatchNumTextBox.Text = batchLength.ToString("N0");
        }

        private void DataNoiseTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (double.TryParse(DataNoiseTxtBox.Text, out double val))
                    NoiseFactor = val;

                DataNoiseTxtBox.Text = NoiseFactor.ToString();
            }
        }
        private void DataNoiseTxtBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (double.TryParse(DataNoiseTxtBox.Text, out double val))
                NoiseFactor = val;

            DataNoiseTxtBox.Text = NoiseFactor.ToString();
        }

        private bool RootShapeInput()
        {
            if (voxelizer != null) return false;

            string[] inputStr = RootShapeTxtBox.Text.Split(',');

            if (inputStr.Length != 4) return false;

            int[] inputVal = new int[4];
            bool isEqual = true;

            for (int i = 0; i < 2; i++)
            {
                if (!int.TryParse(inputStr[i], out int tmp) || tmp <= 0) return false;

                inputVal[i] = tmp;

                if (isEqual && tmp != rootGridShape[i]) isEqual = false;
            }

            for (int i = 2; i < 4; i++)
            {
                bool multiply = inputStr[i][0] == '*';
                int tmp;

                if (multiply)
                {
                    if (!int.TryParse(inputStr[i].Substring(1), out tmp) || tmp <= 0) return false;

                    tmp *= inputVal[i - 2];
                }
                else
                {
                    if (!int.TryParse(inputStr[i], out tmp) || tmp <= 0) return false;
                }

                inputVal[i] = tmp;

                if (isEqual && tmp != rootGridShape[i]) isEqual = false;
            }

            if (isEqual) return false;

            // align the shape
            inputVal[2] = BITMAPWF * ((inputVal[2] + BITMAPWF - 1) / BITMAPWF);
            inputVal[3] = BITMAPHF * ((inputVal[3] + BITMAPHF - 1) / BITMAPHF);

            if (inputVal[0] == rootGridShape[0] && inputVal[1] == rootGridShape[1] &&
                inputVal[2] == rootGridShape[2] && inputVal[3] == rootGridShape[3]) return false;

            if (inputVal[2] / inputVal[0] < TILEMIN || inputVal[3] / inputVal[1] < TILEMIN) return false;

            // --- Reinitialize the grid and model ---
            rootGridShape = inputVal;

            SetModelToCanvasTransform(true);
            IniBitmaps();
            IniGrid();
            //SetGPU();

            //ResetAvg();
            UpdateGI();
            UpdateInfo();

            //if (focusedVoxel != null) DrawFocusRect(focusedVoxel);

            return true;
        }

        private bool TessShapeInput()
        {
            string[] inputStr = TessShapeTxtBox.Text.Split(',');

            if (inputStr.Length != 2) return false;

            int[] inputVal = new int[2];
            bool isEqual = true;

            for (int i = 0; i < 2; i++)
            {
                if (!int.TryParse(inputStr[i], out int tmp) || tmp <= 0) return false;

                inputVal[i] = tmp;

                if (tmp == tessFactors[i]) continue;

                isEqual = false;
            }

            if (isEqual) return false;

            tessFactors = inputVal;

            // Reinitialize the grid
            IniGrid();
            //ResetAvg();
            UpdateGI();
            UpdateInfo();

            return true;
        }

        private void RootShapeTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            RootShapeInput();
            RootShapeTxtBox.Text = ArrayToString(rootGridShape);
        }
        private void RootShapeTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (!RootShapeInput()) RootShapeTxtBox.Text = ArrayToString(rootGridShape);
            }
        }

        private void TessShapeTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            if (!TessShapeInput()) TessShapeTxtBox.Text = ArrayToString(tessFactors);
        }
        private void TessShapeTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (!TessShapeInput()) TessShapeTxtBox.Text = ArrayToString(tessFactors);
            }
        }

        private void TessNumTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            if (voxelizer != null) return;

            string inputStr = TessNumTxtBox.Text;
            bool multiply = string.IsNullOrEmpty(inputStr) ? false : inputStr[0] == '*';

            if (multiply) inputStr = inputStr.Substring(1);

            if (int.TryParse(inputStr, out int n) && n > 0)
            {
                tessNum = n;

                if (multiply)
                {
                    tessNum *= rootGridShape[0] * rootGridShape[1];
                }
            }

            TessNumTxtBox.Text = tessNum.ToString();

            // Reinitialize the grid
            IniGrid();
            //ResetAvg();
            UpdateGI();
            UpdateInfo();
        }
        private void TessNumTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (voxelizer != null) return;

            if (e.Key == Key.Enter)
            {
                string inputStr = TessNumTxtBox.Text;
                bool multiply = inputStr[0] == '*';

                if (multiply) inputStr = inputStr.Substring(1);

                if (int.TryParse(inputStr, out int n) && n > 0)
                {
                    tessNum = n;

                    if (multiply)
                    {
                        tessNum *= rootGridShape[0] * rootGridShape[1];
                    }
                }

                TessNumTxtBox.Text = tessNum.ToString();

                // Reinitialize the grid
                IniGrid();
                //ResetAvg();
                UpdateGI();
                UpdateInfo();
            }
        }

        private void PreviewNextButton_Click(object sender, RoutedEventArgs e)
        {
            GetNextDataBatch();
            isModelRbfFresh = false;
            UpdateGI();
            UpdateInfo();
            
            PreviewButton.IsEnabled = true;
            InitButton.IsEnabled = true;
        }

        private void PreviewButton_Click(object sender, RoutedEventArgs e)
        {
            isModelRbfFresh = false;
            UpdateGI();
        }

        private void ThresholdTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (double.TryParse(ThresholdTxtBox.Text, out double h) && h > 0.0)
                {
                    threshold = h;

                    RBF.CalcRanges(threshold, spaceDiagonal, 1);
                }

                ThresholdTxtBox.Text = threshold.ToString();
            }
        }
        private void ThresholdTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(ThresholdTxtBox.Text, out double h) && h > 0.0)
            {
                threshold = h;

                RBF.CalcRanges(threshold, spaceDiagonal, 1);
            }

            ThresholdTxtBox.Text = threshold.ToString();
        }

        private void DetailsFactorTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (double.TryParse(DetailsFactorTxtBox.Text, out double h) && h > 0.0)
                {
                    batchRatio = h;
                }

                DetailsFactorTxtBox.Text = batchRatio.ToString();
            }
        }
        private void DetailsFactorTxtBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (double.TryParse(DetailsFactorTxtBox.Text, out double h) && h > 0.0)
            {
                batchRatio = h;
            }

            DetailsFactorTxtBox.Text = batchRatio.ToString();
        }

        private void MonotonicityTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                string txt = MonotonicityTxtBox.Text;

                if (double.TryParse(txt, out double h))
                {
                    if (h < 0.0) h = -h;

                    monotonicTol = h;
                }
                else if (txt == "infinity" || txt == "inf")
                {
                    monotonicTol = double.NegativeInfinity;
                }

                if (double.IsInfinity(monotonicTol))
                    MonotonicityTxtBox.Text = "infinity";
                else
                    MonotonicityTxtBox.Text = monotonicTol.ToString();
            }
        }
        private void MonotonicityTxtBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            string txt = MonotonicityTxtBox.Text;

            if (double.TryParse(txt, out double h))
            {
                if (h < 0.0) h = -h;

                monotonicTol = h;
            }
            else if (txt == "infinity" || txt == "inf")
            {
                monotonicTol = double.NegativeInfinity;
            }

            if (double.IsInfinity(monotonicTol))
                MonotonicityTxtBox.Text = "infinity";
            else
                MonotonicityTxtBox.Text = monotonicTol.ToString();
        }

        enum ButtonState { Reseted, InitArmed, Initialized, WaitReset, ResetTerm }
        ButtonState initBttState = ButtonState.Reseted;

        private void InitButton_Click(object sender, RoutedEventArgs e)
        {
            switch (initBttState)
            {
                case ButtonState.Reseted:
                    initBttState = ButtonState.InitArmed;
                    break;

                case ButtonState.InitArmed:
                    // do nothing until button will be released
                    break;

                case ButtonState.Initialized:
                    initBttState = ButtonState.WaitReset;
                    break;

                case ButtonState.WaitReset:
                    InitButton.Content = "Initialize";
                    InitButton.Foreground = txtEditableFore;
                    initBttState = ButtonState.ResetTerm;
                    break;
            }
        }
        private void InitButton_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            switch (initBttState)
            {
                case ButtonState.InitArmed:
                    SetVoxelizer();
                    IniGrid();

                    InitButton.Content = "Reset";
                    InitButton.Foreground = txtRedFore;

                    PointerRect.Visibility = Visibility.Visible;

                    // enable
                    SurveyButton.IsEnabled = true;
                    RefineButton.IsEnabled = true;

                    // disable
                    RbfStack.IsEnabled = false;
                    //BatchNumTextBox.IsEnabled = false;
                    //DataNoiseTxtBox.IsEnabled = false;
                    ModelList.IsEnabled = false;
                    RootShapeTxtBox.IsEnabled = false;
                    TessShapeTxtBox.IsEnabled = false;
                    TessNumTxtBox.IsEnabled = false;

                    UpdateInfo();

                    initBttState = ButtonState.Initialized;
                    break;

                case ButtonState.WaitReset:
                    // early release
                    initBttState = ButtonState.Initialized;

                    if (InitButton.ToolTip == null)
                        InitButton.ToolTip = new ToolTip() { Content = "Hold to reset" };

                    break;

                case ButtonState.ResetTerm:
                    voxelizer.SyncEvent -= UpdateGrid;
                    voxelizer.CompletionEvent -= CompletionHandler;
                    voxelizer = null;

                    trainingNum = 0;

                    PointerRect.Visibility = Visibility.Collapsed;

                    // enable
                    RbfStack.IsEnabled = true;
                    //BatchNumTextBox.IsEnabled = true;
                    //DataNoiseTxtBox.IsEnabled = true;
                    ModelList.IsEnabled = true;
                    RootShapeTxtBox.IsEnabled = true;
                    TessShapeTxtBox.IsEnabled = true;
                    TessNumTxtBox.IsEnabled = true;

                    // disable
                    SurveyButton.IsEnabled = false;
                    RefineButton.IsEnabled = false;
                    EvaluationStack.IsEnabled = false;

                    InitButton.ToolTip = null;

                    UpdateInfo();

                    initBttState = ButtonState.Reseted;

                    break;
            }
        }

        private void SurveyButton_Click(object sender, RoutedEventArgs e)
        {
            voxelizer.Survey(threshold, monotonicTol, batchRatio);
            trainingNum = dataSource.BatchLength;

            EvaluationStack.IsEnabled = true;

            UpdateGI();
            UpdateInfo();
        }

        private bool RefineButtonIsChecked = false;

        private void RefineButton_Click(object sender, RoutedEventArgs e)
        {
            if (RefineButton.IsChecked == true)
            {
                RefineButtonIsChecked = true;

                RefineButton.Content = "Stop";
                InitButton.IsEnabled = false;
                SurveyButton.IsEnabled = false;
                EvaluationStack.IsEnabled = false;
                DataStack.IsEnabled = false;

                // start the refine from the beginning of the data
                if (!voxelizer.IsRefineStarted)
                {
                    dataSource.SeekNext(DataSet.Training, 0);
                    dataSource.GetNextNorm(DataSet.Training, ref rbfPoints, ref rbfLabels);
                    voxelizer.Survey(threshold, monotonicTol, batchRatio);
                    trainingNum = dataSource.BatchLength;
                }

                var voxParams = Tuple.Create(threshold, batchRatio);

                ThreadPool.QueueUserWorkItem(o =>
                {
                    var p = (Tuple<double, double>)o;

                    voxelizer.Voxelize(p.Item1, p.Item2);

                }, voxParams);
            }
            else
            {
                RefineButtonIsChecked = false;
                RefineButton.IsEnabled = false;
            }
        }
        
        private void GainTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (GainTextBox.Text == "*")
                {
                    outputGain = 1.5 * RBF.Amplitude / batchLength;

                    if (outputGain < 0) outputGain = -outputGain;
                }
                else if (double.TryParse(GainTextBox.Text, out double val) && val > 0.0)
                {
                    outputGain = val;
                }

                GainTextBox.Text = outputGain.ToString();
            }
        }
        private void GainTextBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (GainTextBox.Text == "*")
            {
                outputGain = 1.5 * RBF.Amplitude / batchLength;

                if (outputGain < 0) outputGain = -outputGain;
            }
            else if (double.TryParse(GainTextBox.Text, out double val) && val > 0.0)
            {
                outputGain = val;
            }

            GainTextBox.Text = outputGain.ToString();
        }

        private void RenderBttsStack_Switch(FrameworkElement sourceBtt, bool redraw)
        {
            switch (sourceBtt.Name)
            {
                case "ShowGridButton":
                    if (ShowGridButton.IsChecked == true)
                    {
                        if (redraw) UpdateGI(GItems.Grid);

                        GridImage.Visibility = Visibility.Visible;
                    }
                    else GridImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowVoxsButton":
                    if (ShowVoxsButton.IsChecked == true)
                    {
                        ShowModelButton.IsChecked = false;
                        ShowModelRbfButton.IsChecked = false;
                        ShowRbfButton.IsChecked = false;

                        //LDrawGridVoxsTxtBlock.Text = "Draw grid,voxels ms:";

                        if (redraw) UpdateGI(GItems.Voxels);

                        ModelImage.Visibility = Visibility.Visible;
                        ModelRbfImage.Visibility = Visibility.Collapsed;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowModelButton":
                    if (ShowModelButton.IsChecked == true)
                    {
                        ShowVoxsButton.IsChecked = false;
                        ShowModelRbfButton.IsChecked = false;
                        ShowRbfButton.IsChecked = false;

                        //LDrawGridVoxsTxtBlock.Text = "Draw grid,model ms:";

                        if (redraw) UpdateGI(GItems.Model);

                        ModelImage.Visibility = Visibility.Visible;
                        ModelRbfImage.Visibility = Visibility.Collapsed;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowModelRbfButton":
                    if (ShowModelRbfButton.IsChecked == true)
                    {
                        ShowVoxsButton.IsChecked = false;
                        ShowModelButton.IsChecked = false;
                        ShowRbfButton.IsChecked = false;

                        //LDrawGridVoxsTxtBlock.Text = "Draw grid,model ms:";

                        if (redraw) UpdateGI(GItems.ModelRBF);

                        ModelRbfImage.Visibility = Visibility.Visible;
                        ModelImage.Visibility = Visibility.Collapsed;
                    }
                    else ModelRbfImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowRbfButton":
                    if (ShowRbfButton.IsChecked == true)
                    {
                        ShowVoxsButton.IsChecked = false;
                        ShowModelRbfButton.IsChecked = false;
                        ShowModelButton.IsChecked = false;

                        //LDrawGridVoxsTxtBlock.Text = "Draw grid,model ms:";

                        if (redraw) UpdateGI(GItems.RBF);

                        ModelImage.Visibility = Visibility.Visible;
                        ModelRbfImage.Visibility = Visibility.Collapsed;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;
            }
        }
        
        private void RenderBttsStack_Click(object sender, RoutedEventArgs e)
        {
            var sourceBtt = (FrameworkElement)e.OriginalSource;

            RenderBttsStack_Switch(sourceBtt, !RefineButtonIsChecked);
        }

        private void TestNumTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (int.TryParse(TestNumTextBox.Text, out int num))
                {
                    testNum = num < 0 ? int.MaxValue : num;
                }

                TestNumTextBox.Text = testNum.ToString("N0");
            }
        }
        private void TestNumTextBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (int.TryParse(TestNumTextBox.Text, out int num) && num > 0)
            {
                testNum = num < 0 ? int.MaxValue : num;
            }

            TestNumTextBox.Text = testNum.ToString("N0");
        }

        private void ErrorOutButton_Click(object sender, RoutedEventArgs e)
        {
            int errNum = 0;
            int totNum = 0;

            int tposNum = 0;
            int tnegNum = 0;

            dataSource.SeekNext(DataSet.Test, 0);

            while (dataSource.GetNextNorm(DataSet.Test, ref rbfPoints, ref rbfLabels) && totNum < testNum)
            {
                for (int i = 0; i < rbfPoints.Length; i++)
                {
                    double val = voxelizer.Classify(rbfPoints[i]);

                    if ((rbfLabels[i] && val <= 0.0) || (!rbfLabels[i] && val >= 0.0))
                        ++errNum;
                }

                totNum += dataSource.BatchLength;
                tposNum += dataSource.PositiveNum;
                tnegNum += dataSource.NegativeNum;
            }

            if (totNum == 0)
                ErrorOutTxtBlock.Text = "_ %";
            else
            {
                ErrorOutTxtBlock.Text = string.Format("{0:F4} %", 100.0 * errNum / totNum);

                double posP = 100.0 * tposNum / totNum;
                double negP = 100.0 * tnegNum / totNum;

                TestRatioTxtBlock.Text = string.Format("{0,5:F1}, {1,5:F1} %", posP, negP);
            }

            testNum = totNum;
            batchLength = dataSource.BatchLength;

            TestNumTextBox.Text = testNum.ToString("N0");
            BatchNumTextBox.Text = batchLength.ToString("N0");
        }

        private void ErrorInButton_Click(object sender, RoutedEventArgs e)
        {
            int errNum = 0;
            int totNum = 0;

            dataSource.SeekNext(DataSet.Training, 0);

            while (dataSource.GetNextNorm(DataSet.Training, ref rbfPoints, ref rbfLabels) && totNum < trainingNum)
            {
                if (model == null)
                {
                    for (int i = 0; i < rbfPoints.Length; i++)
                    {
                        double val = voxelizer.Classify(rbfPoints[i]);

                        if ((rbfLabels[i] && val <= 0.0) || (!rbfLabels[i] && val >= 0.0))
                            ++errNum;
                    }
                }
                else
                {
                    for (int i = 0; i < rbfPoints.Length; i++)
                    {
                        double val = voxelizer.Classify(rbfPoints[i]);
                        double modelVal = model.Measure(rbfPoints[i]) - 0.5;
                        
                        if ((val >= 0.0 && modelVal <= 0.0) || (val <= 0.0 && modelVal >= 0.0))
                            ++errNum;
                    }
                }

                totNum += dataSource.BatchLength;
            }

            if (totNum == 0)
                ErrorInTxtBlock.Text = "_ %";
            else
                ErrorInTxtBlock.Text = string.Format("{0:F4} %", 100.0 * errNum / totNum);

            batchLength = dataSource.BatchLength;
            BatchNumTextBox.Text = batchLength.ToString("N0");
        }

        private void MainCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point p = e.GetPosition(MainCanvas);
            
            if (voxelizer == null)
            {
                PointToModel(p, rbfCenter);

                AddRbf(rbfCenter, true);
                FillWithAverages();

                UpdateGI();
                UpdateInfo();
            }
            else if (!RefineButtonIsChecked)
            {
                MainCanvas.MouseMove += MainCanvas_MouseMove;
                MainCanvas.MouseLeftButtonUp += MainCanvas_MouseLeftButtonUp;
                MainCanvas.MouseLeave += MainCanvas_MouseLeave;

                double[] modelP = new double[2];
                PointToModel(p, modelP);

                ModelValueTxtBlock.Text = string.Format("{0:F4} %", 100.0 * dataSource.Classify(modelP));
                VoxelValueTxtBlock.Text = string.Format("{0:F4} %", 100.0 * voxelizer.Classify(modelP));

                Canvas.SetTop(PointerRect, p.Y - 0.5 * PointerRect.Height);
                Canvas.SetLeft(PointerRect, p.X - 0.5 * PointerRect.Width);
            }
        }
        
        private void MainCanvas_MouseMove(object sender, MouseEventArgs e)
        {
            Point p = e.GetPosition(MainCanvas);
            double[] modelP = new double[2];

            PointToModel(p, modelP);

            ModelValueTxtBlock.Text = string.Format("{0:F4} %", 100.0 * dataSource.Classify(modelP));
            VoxelValueTxtBlock.Text = string.Format("{0:F4} %", 100.0 * voxelizer.Classify(modelP));

            Canvas.SetTop(PointerRect, p.Y - 0.5 * PointerRect.Height);
            Canvas.SetLeft(PointerRect, p.X - 0.5 * PointerRect.Width);
        }

        private void MainCanvas_MouseLeave(object sender, MouseEventArgs e)
        {
            MainCanvas.MouseMove -= MainCanvas_MouseMove;
            MainCanvas.MouseLeftButtonUp -= MainCanvas_MouseLeftButtonUp;
        }

        private void MainCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            MainCanvas.MouseMove -= MainCanvas_MouseMove;
            MainCanvas.MouseLeftButtonUp -= MainCanvas_MouseLeftButtonUp;
            MainCanvas.MouseLeave -= MainCanvas_MouseLeave;
        }
        
        #endregion

        // --------------------------------------------
        #region Helpers and extensions
        // --------------------------------------------

        public static string ArrayToString<T>(T[] array, char separator = ',')
        {
            int endex = array.Length - 1;
            StringBuilder strBld = new StringBuilder(array.Length + endex);

            for (int i = 0; i < endex; i++)
            {
                strBld.AppendFormat("{0}{1}", array[i].ToString(), separator);
            }

            strBld.Append(array[endex].ToString());

            return strBld.ToString();
        }

        #endregion
    }
}
