/*
 This is the educational project with main purpose is to develop and test the multidimensional 
 adaptive grid which can be used in Reinforcement Learning area. 
 Additional purpose of this project is to learn and try CUDA programming 
 with Alea GPU (http://www.aleagpu.com/release/3_0_4/doc/gpu_programming_csharp.html).

 In typical reinforcement learning scenario an agent takes actions corresponding to current 
 policy and state of an environment. Adaptive grid is needed to represent a manifold of possible
 actions of an agent and manifold of states of an environment with ability to adjust grid resolution
 in regions of interest. The concept of adaptive refinement of the manifolds of possible states and
 actions is close to common learning practices (for example, in learning of car driving
 at a very first stage there is no need for any distinction of severity of bends, instead, 
 the only distinction between left and right turns could be suitable. The more experience and 
 average speeds, the more details about bends and the more precise steering are needed for further
 improvement of the driving skills).
 
 The class AdaptiveGrid is implemented as voxel tree. Each voxel exposes its properties 
 and methods through the IVoxel interface. The maximum number of voxels and shape of the grid
 are set at creation time of the AdaptiveGrid instance and can't be changed during the liftime
 of the instance. The voxels are stored in the internal single-dimensional array, which entirely
 initialized at creation time of the AdaptiveGrid instance. So, during different manipulations
 with the grid (voxel tessellation or merging), voxels only are changed their properties and
 no voxel instances are created or deleted. The grid can be processed in multithreaded manner.
 The methods with names RefineGrid_CGParallel, RefineGrid_FGParallel and others, represent 
 templates of multithreaded processing and also show how the voxel iterator NextAtLevel works.

 This project is intended for testing of two-dimensional AdaptiveGrid instances, for estimation 
 of performance of the grid processing methods and for trying some CUDA programming, which 
 in this case was used for bitmaps calculations. The TestWpf has graphical output for visual 
 presentation of the grid and voxels. Several simple abstract models (located in ModelFunctions.cs) 
 were created for the testing of the grid refinement processes and for fun in some ways.

 2018 zubetto85@gmail.com
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Cognita;
using IVoxel = Cognita.AdaptiveGrid<double>.IVoxel;
using ModelFunctions;
using System.Windows.Threading;
using System.Reflection;
using System.Windows.Shapes;

namespace TestWpf
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private static int _pW, _pH;
        private static double _addW, _addH, _minH;
        private static double _toDevM11, _toDevM22;

        private static IModel Model;
        private static double modeldt = 0;
        private static Func<IVoxel, double> VoxelFunc;
        private static Func<double[], double> ModelFunc;
        private static Func<double, double> Normalizer;

        private int[] rootGridShape = new int[4] { 16, 9, 1280, 720 };
        private int[] tessFactors = new int[2] { 3, 3 };

        private AdaptiveGrid<double> adaptiveGrid;
        private IList<IVoxel> voxelArray;
        private int tessMultp;
        private int tessNum = 40;
        private IVoxel focusedVoxel;
        private int focusedVoxelSI = -1;
        
        private static Matrix ModelToCanvas = Matrix.Identity;
        private static Matrix CanvasToModel = Matrix.Identity;

        private static readonly PixelFormat _pF = PixelFormats.Pbgra32;
        public const int BYTESPP = 4;
        public const int BITMAPWF = 32;
        public const int BITMAPHF = 16;
        public const int TILEMIN = 10;

        private WriteableBitmap gridBitmap;
        private WriteableBitmap signalBitmap;
        private byte[] gridPixelArray;
        private byte[] modelPixelArray;
        private int rowStride;
        private double[] placementTmp = new double[4]; // { pointX, pointY, stepX, stepY }
        private double[] vectorTmp = new double[2]; // { pointX, pointY }

        private byte[] pixelTmp1;
        private byte[] pixelTmp2;
        private byte[] pixelArrTmp1;
        private byte[] pixelArrTmp2;

        private Color colorLine;
        private Color colorLimpid;
        private Color[] orderedGradColors;
        private double[] orderedGradOffsets;

        private byte[] Grad5ColorsBgra;
        private double[] Grad5Offsets;

        private TasksDispenser TPSvoxels;
        private TasksDispenser TPSlines;
        
        public MainWindow()
        {
            InitializeComponent();
            Loaded += SetDPI;

            // store values specified in designer
            _addW = Width - MainCanvas.Width;
            _addH = Height - MainCanvas.Height;
            _minH = Height;

            // set model list
            ModelList.ItemsSource = Enum.GetValues(typeof(Models));
            ModelList.SelectedIndex = 1;
            ModelList.SelectionChanged += SetModel;
            ModelList.MouseDoubleClick += SetModelPropsList;
            SetModel();

            // set refinig algorithms list
            RefiningAlgoList.ItemsSource = Enum.GetValues(typeof(Refiners));
            RefiningAlgoList.SelectedIndex = 3;
            RefiningAlgoList.SelectionChanged += SetRefiner;
            SetRefiner(null, null);

            // ini settings-block text
            tessNum *= rootGridShape[0] * rootGridShape[1];

            RootShapeTxtBox.Text = ArrayToString(rootGridShape);
            TessShapeTxtBox.Text = ArrayToString(tessFactors);
            TessNumTxtBox.Text = tessNum.ToString();
            ThresholdTxtBox.Text = refinerThold.ToString();
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
            SetGPU();

            ResetAvg();
            UpdateGI();
            UpdateInfo();
        }

        private enum Models { Speck, RepulsiveSpecks, SolarSpecks, GravitySpecks }
        
        /// <summary>
        /// Data source for the Model Settings ListBox
        /// </summary>
        public class NameValue
        {
            public static SolidColorBrush ListItemBackground;
            public static SolidColorBrush ForeTextRW;
            public static SolidColorBrush ForeTextReadonly;

            private static bool drawBackground = true;

            public static object Instance { get; set; }
            public PropertyInfo PropInfo { get; set; }

            public bool IsReadOnly { get; set; }
            public string Name { get; set; }
            public string Value { get; set; }
            public SolidColorBrush Background { get; set; }
            public SolidColorBrush Foreground { get; set; }

            public NameValue(PropertyInfo pi)
            {
                PropInfo = pi;
                Name = pi.Name;

                IsReadOnly = !PropInfo.CanWrite;

                if (PropInfo.CanWrite) Foreground = ForeTextRW;
                else Foreground = ForeTextReadonly;

                if (pi.GetIndexParameters().Length == 0)
                {
                    if (pi.CanRead)
                    {
                        Value = pi.GetValue(Instance).ToString();
                    }
                    else Value = "_";
                }
                else Value = "<indexed>";
                
                if (drawBackground)
                {
                    drawBackground = false;
                    Background = ListItemBackground;
                }
                else
                {
                    drawBackground = true;
                    Background = null;
                }
            }

            public void SetValue(TextBox txtBox)
            {
                if (PropInfo.GetIndexParameters().Length != 0)
                {
                    Value = "<indexed>";
                    return;
                }

                Value = txtBox.Text;

                if (PropInfo.CanWrite)
                {
                    if (PropInfo.PropertyType.IsEnum)
                    {
                        if (PropInfo.PropertyType.GetEnumNames().Any(s => s == Value))
                        {
                            PropInfo.SetValue(Instance, Enum.Parse(PropInfo.PropertyType, Value));
                        }
                    }
                    else
                    {
                        try
                        {
                            PropInfo.SetValue(Instance, Convert.ChangeType(Value, PropInfo.PropertyType));
                        }
                        catch
                        {
                            // TODO: err output
                        }
                    }
                }
                
                if (PropInfo.CanRead)
                {
                    Value = PropInfo.GetValue(Instance).ToString();
                }
                else Value = "_";

                txtBox.Text = Value;
            }
        }

        private ModelSettings modelPropsWin;
        public static List<NameValue> modelProps;

        private void SetModelPropsList(object obj)
        {
            if (NameValue.ListItemBackground == null)
            {
                NameValue.ListItemBackground = (TryFindResource("TxtParamsBack") as SolidColorBrush);
                NameValue.ForeTextReadonly = (TryFindResource("TxtParamsFore") as SolidColorBrush);
                NameValue.ForeTextRW = (TryFindResource("TxtEditableFore") as SolidColorBrush);
            }

            NameValue.Instance = obj;
            modelProps = obj.GetType().GetProperties().Select(p => new NameValue(p)).ToList();

            if (modelPropsWin == null || !modelPropsWin.IsVisible)
            {
                modelPropsWin = new ModelSettings();
                modelPropsWin.ModelParamsListBox.ItemsSource = modelProps;
                modelPropsWin.Show();
            }
            else modelPropsWin.ModelParamsListBox.ItemsSource = modelProps;
        }

        private void SetModelPropsList(object sender, MouseEventArgs e)
        {
            if (modelPropsWin == null || !modelPropsWin.IsVisible)
            {
                NameValue.Instance = Model;
                modelProps = Model.GetType().GetProperties().Select(p => new NameValue(p)).ToList();

                modelPropsWin = new ModelSettings();
                modelPropsWin.ModelParamsListBox.ItemsSource = modelProps;
                modelPropsWin.Show();
            }
        }

        private void SetModel()
        {
            Models model = (Models)ModelList.SelectedIndex;
            
            double rX = 1280;
            double rY = 720;

            switch (model)
            {
                case Models.Speck:
                    Model = new Speck()
                    {
                        RangeX = rX,
                        RangeY = rY,
                        CenterX = 0.5 * rX,
                        CenterY = 0.5 * rY,
                        Kx = 7 / rY,
                        Ky = 7 / rY,
                        Exp = 4,
                    };

                    break;

                case Models.RepulsiveSpecks:
                    Model = new RepulsiveSpecks()
                    {
                        RangeX = rX,
                        RangeY = rY,
                        SpecksNumber = 64,
                        MaxIniSpeed = 0,
                        MassFactor = 20,
                        BoundsDamping = 0.0033,
                        MemoryAccess = RepulsiveSpecks.MemoryUsage.GMEM
                    };

                    break;

                case Models.SolarSpecks:
                    Model = new SolarSpecks()
                    {
                        RangeX = rX,
                        RangeY = rY,
                        SpecksNumber = 50,
                        MaxIniSpeed = 0,
                        MassFactor = 33,
                        DragFactor = 0.1,
                        BoundsDamping = 0.0033,
                        AttractionFactor = 1,
                        RepulsionFactor = 100,
                        TemperatureFactor = 0.85,
                        MemoryAccess = SolarSpecks.MemoryUsage.GMEM
                    };

                    break;

                case Models.GravitySpecks:
                    Model = new GravitySpecks()
                    {
                        RangeX = rX,
                        RangeY = rY,
                        MaxStep = 0.001,
                        Amp = 1.1,
                        SpecksNumber = 4,
                        MaxIniSpeed = 0.3,
                        MassFactor = 33,
                        FrictionFactor = 0.1,
                        DragFactor = 0,
                        BoundsDamping = 0.0033,
                        AttractionFactor = 2,
                        RepulsionFactor = 10,
                        TemperatureFactor = 0.04,
                        MemoryAccess = GravitySpecks.MemoryUsage.GMEM
                    };
                    
                    break;
            }

            SetModelPropsList(Model);

            Model.ResetSimulation();
            
            VoxelFunc = Model.Measure;
            ModelFunc = Model.Measure;
            Normalizer = Model.Normalize;
        }

        private void SetModel(object sender, SelectionChangedEventArgs e)
        {
            Model.FreeMemory();

            SetModel();
            SetModelToCanvasTransform(true);

            SetGPU();

            RefineButton_Click(null, null);
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
                ModelToCanvas.M11 = _pW / Model.RangeX;
                ModelToCanvas.M22 = -_pH / Model.RangeY;
            }
            else
            {
                ModelToCanvas.M11 = 1;
                ModelToCanvas.M22 = -1;
            }

            ModelToCanvas.OffsetX = -Model.Xo * ModelToCanvas.M11;
            ModelToCanvas.OffsetY = _pH - Model.Yo * ModelToCanvas.M22;

            // Set inverse transform
            CanvasToModel.M11 = 1 / ModelToCanvas.M11;
            CanvasToModel.M22 = 1 / ModelToCanvas.M22;
            CanvasToModel.OffsetX = -ModelToCanvas.OffsetX / ModelToCanvas.M11;
            CanvasToModel.OffsetY = -ModelToCanvas.OffsetY / ModelToCanvas.M22;
        }

        private void SetGPU()
        {
            Model.FreeMemory();
            Model.SetDrawing(_pW, _pH, modelPixelArray, Grad5ColorsBgra, CanvasToModel);
        }
        
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

        private static void PointToModel(double[] point)
        {
            point[0] -= ModelToCanvas.OffsetX;
            point[0] /= ModelToCanvas.M11;

            point[1] -= ModelToCanvas.OffsetY;
            point[1] /= ModelToCanvas.M22;
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

            // Set threads
            TPSlines = new TasksDispenser(_pH);
        }

        private void IniGrid()
        {
            // Set grid params
            int numX = rootGridShape[0];
            int numY = rootGridShape[1];
            
            double[,] shape = new double[2, 3] 
            { 
                { Model.Xo, Model.Xo + Model.RangeX, numX }, 
                { Model.Yo, Model.Yo + Model.RangeY, numY }
            };

            adaptiveGrid = new AdaptiveGrid<double>(tessNum, shape, tessFactors, initializer: VoxelFunc);
            voxelArray = adaptiveGrid.InternalArray;
            tessMultp = adaptiveGrid.TessellationMultp;
            
            // Set threads
            TPSvoxels = new TasksDispenser(adaptiveGrid.LevelsCounts[0]);
            
            // Ini drawings
            DrawRootGrid();
        }
        
        private void SetColors()
        {
            // grid lines color
            colorLine = (GridBorderRect.Stroke as SolidColorBrush).Color;
            double mult = colorLine.A / 255.0;

            colorLine.R = (byte)(colorLine.R * mult);
            colorLine.G = (byte)(colorLine.G * mult);
            colorLine.B = (byte)(colorLine.B * mult);

            // limpid color
            colorLimpid = Color.FromArgb(0, 0, 0, 0);

            // get gradient stops in ascending order
            var gstops = SignalGradient.GradientStops.OrderBy(s => s.Offset);

            orderedGradColors = gstops.Select(s => s.Color).ToArray();
            orderedGradOffsets = gstops.Select(s => s.Offset).ToArray();

            // hide template controls
            GradientRect.Visibility = Visibility.Collapsed;
            FocusRect.Visibility = Visibility.Collapsed;
            
            // set gradient for gpu
            var gradient = ConvertGradient(orderedGradColors, 5);
            Grad5ColorsBgra = gradient.Item1;
            Grad5Offsets = gradient.Item2;
        }

        private Tuple<byte[], double[]> ConvertGradient(Color[] colors, int stopsNum = -1)
        {
            if (stopsNum < 2) stopsNum = colors.Length;

            byte[] byteArr = new byte[BYTESPP * stopsNum];
            double[] offsetArr = new double[stopsNum];

            double step = 1.0 / (stopsNum - 1);
            byte[] pixel = new byte[BYTESPP];

            for (int i = 0; i < stopsNum; i++)
            {
                double x = i * step;

                offsetArr[i] = x;
                SetPixelBgra(pixel, x);

                Buffer.BlockCopy(pixel, 0, byteArr, BYTESPP * i, BYTESPP);
            }

            return new Tuple<byte[], double[]>(byteArr, offsetArr);
        }

        private void SetPixelBgra(byte[] pixel, Color color)
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

        private void SetPixelBgra(byte[] pixel, double value, byte[] GradColors, double[] GradOffsets)
        {
            double step = GradOffsets[1] - GradOffsets[0];

            double factor = Math.Max(0, 1 - Math.Abs(value - GradOffsets[0]) / step);
            factor = Math.Round(factor, 3);

            pixel[0] = (byte)(factor * GradColors[0]);
            pixel[1] = (byte)(factor * GradColors[1]);
            pixel[2] = (byte)(factor * GradColors[2]);
            pixel[3] = (byte)(factor * GradColors[3]);

            for (int oi = 1, ci = BYTESPP; oi < GradOffsets.Length; oi++, ci += BYTESPP)
            {
                factor = Math.Max(0, 1 - Math.Abs(value - GradOffsets[oi]) / step);
                factor = Math.Round(factor, 3);

                pixel[0] += (byte)(factor * GradColors[ci]);
                pixel[1] += (byte)(factor * GradColors[ci + 1]);
                pixel[2] += (byte)(factor * GradColors[ci + 2]);
                pixel[3] += (byte)(factor * GradColors[ci + 3]);
            }
        }

        private void DrawRootGrid()
        {
            SetPixelBgra(pixelTmp1, colorLine);
            SetPixelBgra(pixelTmp2, colorLimpid);

            double dw = 1.0 * _pW / rootGridShape[0];
            double dh = 1.0 * _pH / rootGridShape[1];

            int cellW = (int)Math.Round(dw);
            int cellH = (int)Math.Round(dh);

            double x = dw;

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

                    Buffer.BlockCopy(pixelTmp1, 0, pixelArrTmp2, i, BYTESPP);
                }   
                else Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP);
            }

            double y = dh;

            // build the root grid layout formed from the solid and dotted lines
            for (int i = rowStride, row = 1; i < gridPixelArray.Length; i += rowStride, row++)
            {
                if (row == cellH)
                {
                    y += dh;
                    cellH = (int)Math.Round(y);

                    Buffer.BlockCopy(pixelArrTmp1, 0, gridPixelArray, i, rowStride); // draw solid line
                }
                else Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, i, rowStride); // draw dotted line
            }
            
            // write pixels to the bitmap
            gridBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), gridPixelArray, rowStride, 0, 0);
        }

        private void DrawCellAfterTess(IVoxel voxel, bool updateBitmap = true)
        {
            voxel.GetPlacement(placementTmp);
            PlacementToCanvas(placementTmp);

            adaptiveGrid.CopyGridStep(voxel.TessLevel + 1, vectorTmp);
            StepToCanvas(vectorTmp);

            int W = (int)Math.Round(placementTmp[2]);
            int H = (int)Math.Round(placementTmp[3]);

            int tessW = (int)Math.Round(vectorTmp[0]);
            int tessH = (int)Math.Round(vectorTmp[1]);

            if (W < 4 || H < 4 || tessW < 2 || tessH < 2) return;

            double Yod = placementTmp[1] - placementTmp[3];

            int xo = (int)Math.Round(placementTmp[0]);
            int yo = (int)Math.Round(Yod);

            xo++;
            yo++;

            int Wclm = (int)Math.Round(placementTmp[0] + placementTmp[2]);
            int Hrow = (int)Math.Round(placementTmp[1]);

            W = Wclm - xo;
            H = Hrow - yo;

            int byteLength = W * BYTESPP;

            double d = placementTmp[0] + vectorTmp[0];
            tessW = (int)Math.Round(d);

            SetPixelBgra(pixelTmp1, colorLine);
            SetPixelBgra(pixelTmp2, colorLimpid);

            // set dotted line buffer
            for (int i = 0, clm = xo; i < byteLength; i += BYTESPP, clm++)
            {
                if (clm == tessW)
                {
                    d += vectorTmp[0];
                    tessW = (int)Math.Round(d);

                    Buffer.BlockCopy(pixelTmp1, 0, pixelArrTmp2, i, BYTESPP); // dot
                }
                else Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP); // clear
            }
            
            int index0 = xo * BYTESPP + yo * rowStride;
            
            d = Yod + vectorTmp[1];
            tessH = (int)Math.Round(d);

            // build the voxel grid layout formed from the solid and dotted lines
            for (int i = index0, row = yo; row < Hrow; i += rowStride, row++)
            {
                if (row == tessH)
                {
                    d += vectorTmp[1];
                    tessH = (int)Math.Round(d);

                    Buffer.BlockCopy(pixelArrTmp1, 0, gridPixelArray, i, byteLength); // draw solid line
                }
                else Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, i, byteLength); // draw dotted line
            }
            
            // write pixels to the bitmap
            if (updateBitmap)
                gridBitmap.WritePixels(new Int32Rect(xo, yo, W, H), gridPixelArray, rowStride, xo, yo);
        }

        private void DrawCellAfterMerge(IVoxel voxel, bool updateBitmap = true)
        {
            voxel.GetPlacement(placementTmp);
            PlacementToCanvas(placementTmp);

            int W = (int)Math.Round(placementTmp[2]);
            int H = (int)Math.Round(placementTmp[3]);

            if (W < 4 || H < 4) return;

            double Yod = placementTmp[1] - placementTmp[3];

            int xo = (int)Math.Round(placementTmp[0]);
            int yo = (int)Math.Round(Yod);

            xo++;
            yo++;

            int Wclm = (int)Math.Round(placementTmp[0] + placementTmp[2]);
            int Hrow = (int)Math.Round(placementTmp[1]);

            W = Wclm - xo;
            H = Hrow - yo;

            int byteLength = W * BYTESPP;
            
            SetPixelBgra(pixelTmp2, colorLimpid);

            // set limpid line buffer
            for (int i = 0; i < byteLength; i += BYTESPP)
            {
                Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP);
            }
            
            int index0 = xo * BYTESPP + yo * rowStride;

            // clear the voxel grid layout by the limpid lines
            for (int i = index0, row = yo; row < Hrow; i += rowStride, row++)
            {
                Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, i, byteLength);
            }

            // write pixels to the bitmap
            if (updateBitmap)
                gridBitmap.WritePixels(new Int32Rect(xo, yo, W, H), gridPixelArray, rowStride, xo, yo);
        }

        private void DrawVoxelAfterTess(IVoxel voxel, bool updateBitmap = true)
        {
            voxel.GetPlacement(placementTmp);
            PlacementToCanvas(placementTmp);

            int level = voxel.TessLevel;

            adaptiveGrid.CopyGridStep(++level, vectorTmp);
            StepToCanvas(vectorTmp);

            int W = (int)Math.Round(placementTmp[2]);
            int H = (int)Math.Round(placementTmp[3]);

            int tessW = (int)Math.Round(vectorTmp[0]);
            int tessH = (int)Math.Round(vectorTmp[1]);

            if (W < 1 || H < 1 || tessW < 1 || tessH < 1) return;

            double d = placementTmp[1] - placementTmp[3];

            int Xo = (int)Math.Round(placementTmp[0]);
            int Yo = (int)Math.Round(d);

            d = vectorTmp[1];

            int byteLength = tessW * BYTESPP;

            if (!adaptiveGrid.GetAtLevel(level, ref voxel)) return;

            do
            {
                voxel.CopyOrigin(vectorTmp);
                PointToCanvas(vectorTmp);

                int xi = (int)Math.Round(vectorTmp[0]);
                int yi = (int)Math.Round(vectorTmp[1] - d);

                SetPixelBgra(pixelTmp2, Normalizer(voxel.Data));

                // set color line buffer
                for (int i = 0; i < byteLength; i += BYTESPP)
                {
                    Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP);
                }

                int index0 = xi * BYTESPP + yi * rowStride;

                // shade voxel by the color line
                for (int i = index0, row = 0; row < tessH; i += rowStride, row++)
                {
                    Buffer.BlockCopy(pixelArrTmp2, 0, modelPixelArray, i, byteLength);
                }
            }
            while (adaptiveGrid.NextAtLevel(ref voxel));

            // write pixels to the bitmap
            if (updateBitmap)
                signalBitmap.WritePixels(new Int32Rect(Xo, Yo, W, H), modelPixelArray, rowStride, Xo, Yo);
        }

        private void DrawVoxelAfterMerge(IVoxel voxel, bool updateBitmap = true)
        {
            voxel.GetPlacement(placementTmp);
            PlacementToCanvas(placementTmp);

            int W = (int)Math.Round(placementTmp[2]);
            int H = (int)Math.Round(placementTmp[3]);

            if (W < 1 || H < 1) return;

            double Yod = placementTmp[1] - placementTmp[3];

            int xo = (int)Math.Round(placementTmp[0]);
            int yo = (int)Math.Round(Yod);

            int byteLength = W * BYTESPP;

            SetPixelBgra(pixelTmp2, Normalizer(voxel.Data));

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

            // write pixels to the bitmap
            if (updateBitmap)
                signalBitmap.WritePixels(new Int32Rect(xo, yo, W, H), modelPixelArray, rowStride, xo, yo);
        }

        private void DrawFocusRect(IVoxel voxel)
        {
            // stroke thickness is equal 3
            double[] place = new double[4];

            voxel.GetPlacement(place);
            PlacementToCanvas(place);

            double xo = Math.Round(place[0]) - 1;
            double yo = Math.Round(place[1] - place[3]) - 1;
            
            FocusRect.Width = Math.Round(place[2]) + 3;
            FocusRect.Height = Math.Round(place[3]) + 3;

            Canvas.SetLeft(FocusRect, xo);
            Canvas.SetTop(FocusRect, yo);

            if (!FocusRect.IsVisible) FocusRect.Visibility = Visibility.Visible;
        }
        
        private void DrawModelParallelFor()
        {
            Parallel.For(0, _pH, row =>
            {
                byte[] pixel = new byte[4];
                double[] point = new double[2];
                int start = row * rowStride;
                int stop = start + rowStride;

                for (int i = start, clm = 0; i < stop; i += BYTESPP, clm++)
                {
                    point[0] = clm;
                    point[1] = row;
                    PointToModel(point);
                    
                    SetPixelBgra(pixel, Normalizer(ModelFunc(point)));

                    Buffer.BlockCopy(pixel, 0, modelPixelArray, i, BYTESPP);
                }
            });

            // write pixels to the bitmap
            signalBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), modelPixelArray, rowStride, 0, 0);
        }

        private void DrawModelParallel()
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

                            SetPixelBgra(pixel, Normalizer(ModelFunc(point)));

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
            IVoxel ivoxel;
            int level = 0;
            bool isResolvable = true;
            bool isLevelCached = false;

            SetPixelBgra(pixelTmp1, colorLine);
            SetPixelBgra(pixelTmp2, colorLimpid);

            while (isResolvable && adaptiveGrid.FirstAtLevel(level++, out ivoxel))
            {
                isLevelCached = false;

                ivoxel.GetPlacement(placementTmp);
                PlacementToCanvas(placementTmp);

                int W = (int)Math.Round(placementTmp[2]);
                int H = (int)Math.Round(placementTmp[3]);

                if (W < 4 || H < 4) break;
                
                do
                {
                    ivoxel.CopyOrigin(placementTmp);
                    PointToCanvas(placementTmp);

                    double Yod = placementTmp[1] - placementTmp[3];

                    int xo = (int)Math.Round(placementTmp[0]);
                    int yo = (int)Math.Round(Yod);

                    xo++;
                    yo++;

                    int Wclm = (int)Math.Round(placementTmp[0] + placementTmp[2]);
                    int Hrow = (int)Math.Round(placementTmp[1]);

                    Wclm -= xo;

                    int byteLength = Wclm * BYTESPP;

                    if (ivoxel.ContentSI > 0) // then draw tessellation lines
                    {
                        if (!isLevelCached)
                        {
                            isLevelCached = true;

                            adaptiveGrid.CopyGridStep(level, vectorTmp);
                            StepToCanvas(vectorTmp);

                            int tessW = (int)Math.Round(vectorTmp[0]);
                            int tessH = (int)Math.Round(vectorTmp[1]);

                            if (tessW < 2 || tessH < 2) { isResolvable = false; break; }
                        }
                        
                        double d = placementTmp[0] + vectorTmp[0];
                        int notch = (int)Math.Round(d);

                        // set dotted line buffer
                        for (int i = 0, clm = xo; i < byteLength; i += BYTESPP, clm++)
                        {
                            if (clm == notch)
                            {
                                d += vectorTmp[0];
                                notch = (int)Math.Round(d);

                                Buffer.BlockCopy(pixelTmp1, 0, pixelArrTmp2, i, BYTESPP); // dot
                            }
                            else Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP); // clear
                        }

                        int index0 = xo * BYTESPP + yo * rowStride;

                        d = Yod + vectorTmp[1];
                        notch = (int)Math.Round(d);

                        // build the voxel grid layout formed from the solid and dotted lines
                        for (int i = index0, row = yo; row < Hrow; i += rowStride, row++)
                        {
                            if (row == notch)
                            {
                                d += vectorTmp[1];
                                notch = (int)Math.Round(d);

                                Buffer.BlockCopy(pixelArrTmp1, 0, gridPixelArray, i, byteLength); // draw solid line
                            }
                            else Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, i, byteLength); // draw dotted line
                        }
                    }
                    else if (level == 1) // --- erase tessellation lines ------------------------------------------------
                    {
                        // set limpid line buffer
                        for (int i = 0; i < byteLength; i += BYTESPP)
                        {
                            Buffer.BlockCopy(pixelTmp2, 0, pixelArrTmp2, i, BYTESPP);
                        }

                        int index0 = xo * BYTESPP + yo * rowStride;

                        // clear the voxel grid layout by the limpid lines
                        for (int i = index0, row = yo; row < Hrow; i += rowStride, row++)
                        {
                            Buffer.BlockCopy(pixelArrTmp2, 0, gridPixelArray, i, byteLength);
                        }
                    }
                }
                while (adaptiveGrid.NextAtLevel(ref ivoxel));
            }

            // write pixels to the bitmap
            gridBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), gridPixelArray, rowStride, 0, 0);
        }

        private void DrawVoxels()
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

                if (W == 1 || H == 1) isResolvable = false;

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

                    SetPixelBgra(pixelTmp2, Normalizer(ivoxel.Data));

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

        private bool ThresholdExceeded(IVoxel voxel, double threshold)
        {
            double min = voxelArray[voxel.ContentSI].Data;
            double max = min;
            double mean = min;

            for (int serial = voxel.ContentSI + 1, stop = voxel.ContentSI + tessMultp; serial < stop; serial++)
            {
                double val = voxelArray[serial].Data;
                
                if (val < min) min = val;
                else if (val > max) max = val;

                mean += val;
            }
            
            if (max - min < threshold) 
            {
                mean /= tessMultp;
                voxel.Data = mean;

                return false; // within threshold
            }

            return true;
        }
        
        private void RefineGridMerge(double threshold)
        {
            int rootNum = adaptiveGrid.LevelsCounts[0];
            int level = 0;

            // traversal root voxels
            for (int i = 0; i < rootNum; i++)
            {
                IVoxel voxel = voxelArray[i];

                if (voxel.ContentSI > 0)
                {
                    voxel.SetContent(true, VoxelFunc);

                    if (!ThresholdExceeded(voxel, threshold))
                        voxel.Merge(true, iniValue: voxel.Data);
                }
            }

            // level-loop
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                // traversal root voxels
                for (int i = 0; i < rootNum; i++)
                {
                    IVoxel voxel;
                    if (!adaptiveGrid.FirstAtLevel(level, i, out voxel)) continue;

                    // traversal all voxels at the current level
                    do
                    {
                        if (voxel.ContentSI > 0)
                        {
                            voxel.SetContent(true, VoxelFunc);

                            if (!ThresholdExceeded(voxel, threshold))
                                voxel.Merge(true, iniValue: voxel.Data);
                        }
                    }
                    while (adaptiveGrid.NextAtLevel(i, ref voxel));
                }
            }
        }

        private void RefineGridTess(double threshold)
        {
            int rootNum = adaptiveGrid.LevelsCounts[0];
            int level = 0;

            // traversal root voxels
            for (int i = 0; i < rootNum; i++)
            {
                IVoxel voxel = voxelArray[i];

                if (!voxel.Tessellate(true, VoxelFunc)) continue;

                if (!ThresholdExceeded(voxel, threshold))
                    voxel.Merge(true, iniValue: voxel.Data);
            }

            // level-loop
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                // traversal root voxels
                for (int i = 0; i < rootNum; i++)
                {
                    if (adaptiveGrid.IsFull) return;

                    IVoxel voxel;
                    if (!adaptiveGrid.FirstAtLevel(level, i, out voxel)) continue;

                    // traversal all voxels at the current level
                    do
                    {
                        if (!voxel.Tessellate(true, VoxelFunc)) continue;

                        if (!ThresholdExceeded(voxel, threshold))
                            voxel.Merge(true, iniValue: voxel.Data);
                    }
                    while (adaptiveGrid.NextAtLevel(i, ref voxel));
                }
            }
        }

        private void RefineGrid(double threshold, Updater callback = null, bool block = false)
        {
            // During the merge pass all merged voxel are set the state to false and
            // therefore will be ignored during the tessellation pass;
            // The new voxels caused by tessellation should have state equal to true
            adaptiveGrid.ProcessedState = false;

            int rootNum = adaptiveGrid.LevelsCounts[0];
            int level = -1;

            // --- Merger-pass-----------------------------------------------------------
            // level-loop
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                IVoxel voxel;
                if (!adaptiveGrid.FirstAtLevel(level, out voxel)) continue;

                // traversal all voxels at the current level
                do
                {
                    if (voxel.SetContent(true, VoxelFunc))
                    {
                        if (!ThresholdExceeded(voxel, threshold))
                        {
                            voxel.Merge(true, iniValue: voxel.Data);
                            voxel.State = false;
                        }
                        else voxel.State = true;
                    }
                    else voxel.State = true;
                }
                while (adaptiveGrid.NextAtLevel(ref voxel));
            }

            // --- Tessellation-pass -----------------------------------------------------
            level = -1;

            // level-loop
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                if (adaptiveGrid.IsFull) break;

                IVoxel voxel;
                if (!adaptiveGrid.FirstAtLevel(level, out voxel)) continue;

                // traversal all voxels at the current level
                do
                {
                    if (voxel.State && voxel.Tessellate(true, VoxelFunc))
                    {
                        if (!ThresholdExceeded(voxel, threshold))
                            voxel.Merge(true, iniValue: voxel.Data);
                    }
                }
                while (adaptiveGrid.NextAtLevel(ref voxel));
            }

            callback?.Invoke();
        }
        
        private bool IsParallelComplete = true;
        
        /// <summary>
        /// Coarse-grained parallelism; 
        /// The root grid is divided into groups of voxels with consecutive indexes, 
        /// each of this group consists of adjacent root voxels and is processed by one thread
        /// </summary>
        /// <param name="threshold"></param>
        /// <param name="callback"></param>
        /// <param name="block"></param>
        private void RefineGrid_CGParallel(double threshold, Updater callback = null, bool block = false)
        {
            if (!IsParallelComplete) return;

            IsParallelComplete = false;
            int IsRefiningComplete = 0;

            // During the merge pass all merged voxel are set the state to false and
            // therefore will be ignored during the tessellation pass;
            // The new voxels caused by tessellation should have state equal to true
            adaptiveGrid.ProcessedState = false;

            int numIn = TPSvoxels.ThreadsNum;
            int numOut = TPSvoxels.ThreadsNum;

            var RelayIN = new ManualResetEvent(true);
            var RelayOUT = new ManualResetEvent(false);
            var RelayComplete = new AutoResetEvent(false);

            int voxelInd = 0;
            int voxelNum;

            TPSvoxels.ResetDispenser();

            while (TPSvoxels.DispenseNext(out voxelNum))
            {
                voxelNum += voxelInd;

                var interval = new Tuple<int, int>(voxelInd, voxelNum);

                ThreadPool.QueueUserWorkItem(new WaitCallback((o) => 
                {
                    var segment = (Tuple<int, int>)o;
                    int index = segment.Item1;
                    int length = segment.Item2;

                    // --- Merger-pass ------------------------------
                    int level = -1;

                    // level-loop
                    while (++level < adaptiveGrid.LevelsCounts.Count)
                    {
                        // traversal root voxels
                        for (int i = index; i < length; i++)
                        {
                            IVoxel voxel;
                            if (!adaptiveGrid.FirstAtLevel(level, i, out voxel)) continue;

                            // traversal all voxels at the current level
                            do
                            {
                                if (voxel.ContentSI > 0)
                                {
                                    voxel.SetContent(true, VoxelFunc);

                                    if (!ThresholdExceeded(voxel, threshold))
                                    {
                                        voxel.Merge(true, iniValue: voxel.Data);
                                        voxel.State = false;
                                    }
                                    else voxel.State = true;
                                }
                                else voxel.State = true;
                            }
                            while (adaptiveGrid.NextAtLevel(i, ref voxel));
                        }
                    }

                    // --- sync-point: next pass ---
                    RelayIN.WaitOne();

                    if (Interlocked.Decrement(ref numIn) == 0)
                    {
                        numIn = TPSvoxels.ThreadsNum;
                        RelayIN.Reset();
                        RelayOUT.Set();
                    }
                    else RelayOUT.WaitOne();

                    if (Interlocked.Decrement(ref numOut) == 0)
                    {
                        numOut = TPSvoxels.ThreadsNum;
                        RelayOUT.Reset();
                        RelayIN.Set();
                    }

                    // --- Tessellation-pass-----------------------------------------------------
                    level = -1;

                    // level-loop
                    while (++level < adaptiveGrid.LevelsCounts.Count)
                    {
                        // traversal root voxels
                        for (int i = index; i < length; i++)
                        {
                            if (adaptiveGrid.IsFull) break;

                            IVoxel voxel;
                            if (!adaptiveGrid.FirstAtLevel(level, i, out voxel)) continue;

                            // traversal all voxels at the current level
                            do
                            {
                                if (!voxel.State || !voxel.Tessellate(true, VoxelFunc)) continue;

                                if (!ThresholdExceeded(voxel, threshold))
                                    voxel.Merge(true, iniValue: voxel.Data);
                            }
                            while (adaptiveGrid.NextAtLevel(i, ref voxel));
                        }

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: update bitmaps ---
                    if (0 == Interlocked.Exchange(ref IsRefiningComplete, 1))
                    {
                        if (block)
                        {
                            RelayComplete.Set();
                        }
                        else
                        {
                            if (callback != null) Dispatcher.Invoke(callback, null);
                            IsParallelComplete = true;
                        }
                    }
                }), interval);

                voxelInd = voxelNum;
            }
            
            if (block)
            {
                RelayComplete.WaitOne();

                callback?.Invoke();

                IsParallelComplete = true;
            }
        }

        /// <summary>
        /// Interleaved coarse-grained parallelism;
        /// The root grid is divided into groups of voxels in such a way, that
        /// any pair of root voxels with consecutive indexes is processed by different threads
        /// </summary>
        /// <param name="threshold"></param>
        /// <param name="callback"></param>
        /// <param name="block"></param>
        private void RefineGrid_iCGParallel(double threshold, Updater callback = null, bool block = false)
        {
            if (!IsParallelComplete) return;

            IsParallelComplete = false;
            int IsRefiningComplete = 0;

            // During the merge pass all merged voxel are set the state to false and
            // therefore will be ignored during the tessellation pass;
            // The new voxels caused by tessellation should have state equal to true
            adaptiveGrid.ProcessedState = false;

            int numIn = TPSvoxels.ThreadsNum;
            int numOut = TPSvoxels.ThreadsNum;

            var RelayIN = new ManualResetEvent(true);
            var RelayOUT = new ManualResetEvent(false);
            var RelayComplete = new AutoResetEvent(false);

            int[] tmpIndexes;

            TPSvoxels.ResetDispenser();

            while (TPSvoxels.DispenseNext(out tmpIndexes))
            {
                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    int[] rootInd = (int[])o;

                    // --- Merger-pass ------------------------------
                    int level = -1;

                    // level-loop
                    while (++level < adaptiveGrid.LevelsCounts.Count)
                    {
                        // traversal root voxels
                        for (int i = 0; i < rootInd.Length; i++)
                        {
                            IVoxel voxel;
                            if (!adaptiveGrid.FirstAtLevel(level, rootInd[i], out voxel)) continue;

                            // traversal all voxels at the current level
                            do
                            {
                                if (voxel.ContentSI > 0)
                                {
                                    voxel.SetContent(true, VoxelFunc);

                                    if (!ThresholdExceeded(voxel, threshold))
                                    {
                                        voxel.Merge(true, iniValue: voxel.Data);
                                        voxel.State = false;
                                    }
                                    else voxel.State = true;
                                }
                                else voxel.State = true;
                            }
                            while (adaptiveGrid.NextAtLevel(rootInd[i], ref voxel));
                        }
                    }

                    // --- sync-point: next pass ---
                    RelayIN.WaitOne();

                    if (Interlocked.Decrement(ref numIn) == 0)
                    {
                        numIn = TPSvoxels.ThreadsNum;
                        RelayIN.Reset();
                        RelayOUT.Set();
                    }
                    else RelayOUT.WaitOne();

                    if (Interlocked.Decrement(ref numOut) == 0)
                    {
                        numOut = TPSvoxels.ThreadsNum;
                        RelayOUT.Reset();
                        RelayIN.Set();
                    }

                    // --- Tessellation-pass-----------------------------------------------------
                    level = -1;

                    // level-loop
                    while (++level < adaptiveGrid.LevelsCounts.Count)
                    {
                        // traversal root voxels
                        for (int i = 0; i < rootInd.Length; i++)
                        {
                            if (adaptiveGrid.IsFull) break;

                            IVoxel voxel;
                            if (!adaptiveGrid.FirstAtLevel(level, rootInd[i], out voxel)) continue;

                            // traversal all voxels at the current level
                            do
                            {
                                if (!voxel.State || !voxel.Tessellate(true, VoxelFunc)) continue;

                                if (!ThresholdExceeded(voxel, threshold))
                                    voxel.Merge(true, iniValue: voxel.Data);
                            }
                            while (adaptiveGrid.NextAtLevel(rootInd[i], ref voxel));
                        }

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: update bitmaps ---
                    if (0 == Interlocked.Exchange(ref IsRefiningComplete, 1))
                    {
                        if (block)
                        {
                            RelayComplete.Set();
                        }
                        else
                        {
                            if (callback != null) Dispatcher.Invoke(callback);

                            IsParallelComplete = true;
                        }
                    }
                }), tmpIndexes);
            }

            if (block)
            {
                RelayComplete.WaitOne();

                callback?.Invoke();

                IsParallelComplete = true;
            }
        }

        /// <summary>
        /// Fine-grained parallelism;
        /// The next voxel to process, is stored in the variable sharedVoxel, 
        /// which are sequentially accessed by all threads;
        /// During the such sequential access each thread stores the current value of the sharedVoxel 
        /// in the "local" variable and assigns to the sharedVoxel the next voxel to process;
        /// As a result, the sharedVoxel traversals all voxels of the grid
        /// </summary>
        /// <param name="threshold"></param>
        /// <param name="callback"></param>
        /// <param name="block"></param>
        private void RefineGrid_FGParallel(double threshold, Updater callback = null, bool block = false)
        {
            if (!IsParallelComplete) return;

            IsParallelComplete = false;
            int IsRefiningComplete = 0;

            // During the merge pass all merged voxel are set the state to false and
            // therefore will be ignored during the tessellation pass;
            // The new voxels caused by tessellation should have state equal to true
            adaptiveGrid.ProcessedState = false;

            int numIn = TPSvoxels.ThreadsNum;
            int numOut = TPSvoxels.ThreadsNum;

            var RelayIN = new ManualResetEvent(true);
            var RelayOUT = new ManualResetEvent(false);
            var RelayComplete = new AutoResetEvent(false);

            var locker = new Object();
            bool notFull = true;
            IVoxel sharedVoxel;

            adaptiveGrid.FirstAtLevel(0, out sharedVoxel);

            for (int index = 0; index < TPSvoxels.ThreadsNum; index++)
            {
                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    // --- Merger-pass ------------------------------
                    int level = 0;

                    // level-loop
                    while (level++ < adaptiveGrid.LevelsCounts.Count)
                    {
                        // traversal all voxels at the current level
                        while (true)
                        {
                            IVoxel voxel;

                            lock (locker)
                            {
                                if (sharedVoxel == null) break;

                                voxel = sharedVoxel;

                                if (!adaptiveGrid.NextAtLevel(ref sharedVoxel))
                                    sharedVoxel = null;
                            }

                            if (voxel.ContentSI > 0)
                            {
                                voxel.SetContent(true, VoxelFunc);

                                if (!ThresholdExceeded(voxel, threshold))
                                {
                                    voxel.Merge(true, iniValue: voxel.Data);
                                    voxel.State = false;
                                }
                                else voxel.State = true;
                            }
                            else voxel.State = true;
                        }

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            adaptiveGrid.FirstAtLevel(level, out sharedVoxel);

                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: next pass ---
                    RelayIN.WaitOne();

                    if (Interlocked.Decrement(ref numIn) == 0)
                    {
                        adaptiveGrid.FirstAtLevel(0, out sharedVoxel);

                        numIn = TPSvoxels.ThreadsNum;
                        RelayIN.Reset();
                        RelayOUT.Set();
                    }
                    else RelayOUT.WaitOne();

                    if (Interlocked.Decrement(ref numOut) == 0)
                    {
                        numOut = TPSvoxels.ThreadsNum;
                        RelayOUT.Reset();
                        RelayIN.Set();
                    }

                    // --- Tessellation-pass-----------------------------------------------------
                    level = 0;

                    // level-loop
                    while (notFull && level++ < adaptiveGrid.LevelsCounts.Count)
                    {
                        // traversal all voxels at the current level
                        while (true)
                        {
                            IVoxel voxel;

                            lock (locker)
                            {
                                if (sharedVoxel == null) break;

                                voxel = sharedVoxel;

                                if (!adaptiveGrid.NextAtLevel(ref sharedVoxel))
                                    sharedVoxel = null;
                            }

                            if (!voxel.State || !voxel.Tessellate(true, VoxelFunc)) continue;

                            if (!ThresholdExceeded(voxel, threshold))
                                voxel.Merge(true, iniValue: voxel.Data);
                        }

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            adaptiveGrid.FirstAtLevel(level, out sharedVoxel);

                            if (adaptiveGrid.IsFull) notFull = false;

                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: update bitmaps ---
                    if (0 == Interlocked.Exchange(ref IsRefiningComplete, 1))
                    {
                        if (block)
                        {
                            RelayComplete.Set();
                        }
                        else
                        {
                            if (callback != null) Dispatcher.Invoke(callback, null);

                            IsParallelComplete = true;
                        }
                    }
                }));
            }

            if (block)
            {
                RelayComplete.WaitOne();

                callback?.Invoke();

                IsParallelComplete = true;
            }
        }

        private bool SFGParallelUniformity = false;

        /// <summary>
        /// Stochastic fine-grained parallelism;
        /// Threads utilise grid internal synchronisation by treating the State property of the IVoxel;
        /// Voxels with state equal to the grid property ProcessedState are skipped by all threads
        /// </summary>
        /// <param name="threshold"></param>
        /// <param name="callback"></param>
        /// <param name="block"></param>
        private void RefineGrid_SFGParallel(double threshold, Updater callback = null, bool block = false)
        {
            if (!IsParallelComplete) return;

            IsParallelComplete = false;
            int IsRefiningComplete = 0;

            if (!SFGParallelUniformity)
            {
                adaptiveGrid.UnifyStates(true);
                SFGParallelUniformity = true;
            }

            // During the merger-pass, voxels states are switched from true to false;
            // During the tessellation-pass, the states are switched on the contrary, from false to true
            adaptiveGrid.ProcessedState = false;

            //// ### Debug
            //void CheckState_Debug(bool state, bool? andAct = null)
            //{
            //    foreach (IVoxel v in voxelArray)
            //    {
            //        if ((v.State != state) && ((andAct == null) || (v.IsActive == andAct)))
            //        {

            //        }
            //    }
            //}

            //// ### Debug
            //CheckState_Debug(true);

            int numIn = TPSvoxels.ThreadsNum;
            int numOut = TPSvoxels.ThreadsNum;

            var RelayIN = new ManualResetEvent(true);
            var RelayOUT = new ManualResetEvent(false);
            var RelayComplete = new AutoResetEvent(false);
            
            IVoxel startVoxel;

            adaptiveGrid.FirstAtLevel(0, out startVoxel);

            for (int index = 0; index < TPSvoxels.ThreadsNum; index++)
            {
                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    // --- Merger-pass ------------------------------
                    int level = 0;

                    // level-loop
                    while (level++ < adaptiveGrid.LevelsCounts.Count)
                    {
                        IVoxel voxel = startVoxel;

                        // traversal all voxels at the current level
                        do
                        {
                            if (!voxel.SetContent(false, VoxelFunc)) continue;

                            if (!ThresholdExceeded(voxel, threshold))
                            {
                                voxel.Merge(true, iniValue: voxel.Data);
                                voxel.State = true; // to ignore merged voxels during the tess-pass
                            }
                        }
                        while (adaptiveGrid.NextAtLevel(ref voxel));

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            adaptiveGrid.FirstAtLevel(level, out startVoxel);

                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: next pass ---
                    RelayIN.WaitOne();

                    if (Interlocked.Decrement(ref numIn) == 0)
                    {
                        adaptiveGrid.FirstAtLevel(0, out startVoxel);
                        adaptiveGrid.ProcessedState = true;

                        //// ### Debug
                        //CheckState_Debug(false, andAct: true);

                        numIn = TPSvoxels.ThreadsNum;
                        RelayIN.Reset();
                        RelayOUT.Set();
                    }
                    else RelayOUT.WaitOne();

                    if (Interlocked.Decrement(ref numOut) == 0)
                    {
                        numOut = TPSvoxels.ThreadsNum;
                        RelayOUT.Reset();
                        RelayIN.Set();
                    }

                    // --- Tessellation-pass-----------------------------------------------------
                    level = 0;

                    // level-loop
                    // during tessellation, all active voxels must be handled 
                    // to change their state to adaptiveGrid.ProcessedState
                    while (level++ < adaptiveGrid.LevelsCounts.Count)
                    {
                        IVoxel voxel = startVoxel;

                        // traversal all voxels at the current level
                        do
                        {
                            if (!voxel.Tessellate(false, VoxelFunc)) continue;

                            if (!ThresholdExceeded(voxel, threshold))
                                voxel.Merge(true, iniValue: voxel.Data);
                        }
                        while (adaptiveGrid.NextAtLevel(ref voxel));

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            adaptiveGrid.FirstAtLevel(level, out startVoxel);

                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: update bitmaps ---
                    if (0 == Interlocked.Exchange(ref IsRefiningComplete, 1))
                    {
                        //// ### Debug
                        //CheckState_Debug(true);

                        if (block)
                        {
                            RelayComplete.Set();
                        }
                        else
                        {
                            if (callback != null) Dispatcher.Invoke(callback, null);

                            IsParallelComplete = true;
                        }
                    }
                }));
            }

            if (block)
            {
                RelayComplete.WaitOne();

                callback?.Invoke();

                IsParallelComplete = true;
            }
        }

        /// <summary>
        /// Interleaved fine-grained parallelism;
        /// Each thread processes the certain number of voxels at current tessellation level 
        /// (a block of voxels), then skips blocks processed by other threads and proceeds to
        /// its next block at the same level, and so on while there are any unprocessed voxels
        /// at the current level;
        /// Voxels are grouped into such blocks at all tessellation levels 
        /// which makes this algorithm fine-grained parallel
        /// </summary>
        /// <param name="threshold"></param>
        /// <param name="callback"></param>
        /// <param name="block"></param>
        private void RefineGrid_iFGParallel(double threshold, Updater callback = null, bool block = false)
        {
            if (!IsParallelComplete) return;

            IsParallelComplete = false;
            int IsRefiningComplete = 0;

            // During the merge pass all merged voxel are set the state to false and
            // therefore will be ignored during the tessellation pass;
            // The new voxels caused by tessellation should have state equal to true
            adaptiveGrid.ProcessedState = false; 

            int numIn = TPSvoxels.ThreadsNum;
            int numOut = TPSvoxels.ThreadsNum;

            var RelayIN = new ManualResetEvent(true);
            var RelayOUT = new ManualResetEvent(false);
            var RelayComplete = new AutoResetEvent(false);

            // The number of voxels constituting the block at the root level
            // is choosed empirically
            int blockNum = 9;

            // The number of voxels which should be skipped to define the next
            // block to process
            int stride = blockNum * (TPSvoxels.ThreadsNum - 1);

            // The number of voxels constituting the block at all other levels
            // should be multiple of the grid tessMultp parameter
            int blockDim = blockNum * tessMultp;

            bool notFull = true;
            IVoxel startVoxel;

            adaptiveGrid.FirstAtLevel(0, out startVoxel);

            for (int index = 0; index < TPSvoxels.ThreadsNum; index++)
            {
                ThreadPool.QueueUserWorkItem(new WaitCallback((o) =>
                {
                    int threadInd = (int)o;
                    int startSkipRoot = blockNum * threadInd;
                    int startSkipLevel = startSkipRoot - 1; // -1 because count is started from the ancestor of startVoxel
                    int startSkip = startSkipRoot;

                    // --- Merger-pass ------------------------------
                    int level = 0;
                    int stop = blockNum;

                    // level-loop
                    while (level++ < adaptiveGrid.LevelsCounts.Count)
                    {
                        IVoxel voxel = startVoxel;
                        bool go = true;

                        if (threadInd > 0)
                        {
                            if (!adaptiveGrid.SkipNextAtLevel(startSkip, ref voxel))
                                go = false;
                        }

                        // traversal all voxels at the current level
                        int i = stop;

                        while (go)
                        {
                            if (voxel.SetContent(true, VoxelFunc))
                            {
                                if (!ThresholdExceeded(voxel, threshold))
                                {
                                    voxel.Merge(true, iniValue: voxel.Data);
                                    voxel.State = false;
                                }
                                else voxel.State = true;
                            }
                            else voxel.State = true;

                            if (--i > 0)
                            {
                                if (!adaptiveGrid.NextAtLevel(ref voxel)) break;
                            }
                            else
                            {
                                if (!adaptiveGrid.SkipNextAtLevel(stride, ref voxel)) break;

                                i = stop;
                            }
                        }

                        startSkip = startSkipLevel;
                        stop = blockDim;

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            adaptiveGrid.FirstAtLevel(level, out startVoxel);

                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: next pass ---
                    RelayIN.WaitOne();

                    if (Interlocked.Decrement(ref numIn) == 0)
                    {
                        adaptiveGrid.FirstAtLevel(0, out startVoxel);

                        numIn = TPSvoxels.ThreadsNum;
                        RelayIN.Reset();
                        RelayOUT.Set();
                    }
                    else RelayOUT.WaitOne();

                    if (Interlocked.Decrement(ref numOut) == 0)
                    {
                        numOut = TPSvoxels.ThreadsNum;
                        RelayOUT.Reset();
                        RelayIN.Set();
                    }

                    // --- Tessellation-pass-----------------------------------------------------
                    level = 0;
                    startSkip = startSkipRoot;
                    stop = blockNum;

                    // level-loop
                    while (notFull && level++ < adaptiveGrid.LevelsCounts.Count)
                    {
                        IVoxel voxel = startVoxel;
                        bool go = true;

                        if (threadInd > 0)
                        {
                            if (!adaptiveGrid.SkipNextAtLevel(startSkip, ref voxel))
                                go = false;
                        }

                        // traversal all voxels at the current level
                        int i = stop;

                        while (go)
                        {
                            if (voxel.State && voxel.Tessellate(true, VoxelFunc))
                            {
                                if (!ThresholdExceeded(voxel, threshold))
                                    voxel.Merge(true, iniValue: voxel.Data);
                            }

                            if (--i > 0)
                            {
                                if (!adaptiveGrid.NextAtLevel(ref voxel)) break;
                            }
                            else
                            {
                                if (!adaptiveGrid.SkipNextAtLevel(stride, ref voxel)) break;

                                i = stop;
                            }
                        }

                        startSkip = startSkipLevel;
                        stop = blockDim;

                        // --- sync-point: next level ---
                        RelayIN.WaitOne();

                        if (Interlocked.Decrement(ref numIn) == 0)
                        {
                            adaptiveGrid.FirstAtLevel(level, out startVoxel);

                            if (adaptiveGrid.IsFull) notFull = false;

                            numIn = TPSvoxels.ThreadsNum;
                            RelayIN.Reset();
                            RelayOUT.Set();
                        }
                        else RelayOUT.WaitOne();

                        if (Interlocked.Decrement(ref numOut) == 0)
                        {
                            numOut = TPSvoxels.ThreadsNum;
                            RelayOUT.Reset();
                            RelayIN.Set();
                        }
                    }

                    // --- sync-point: update bitmaps ---
                    if (0 == Interlocked.Exchange(ref IsRefiningComplete, 1))
                    {
                        if (block)
                        {
                            RelayComplete.Set();
                        }
                        else
                        {
                            if (callback != null) Dispatcher.Invoke(callback, null);

                            IsParallelComplete = true;
                        }
                    }
                }), index);
            }

            if (block)
            {
                RelayComplete.WaitOne();

                callback?.Invoke();

                IsParallelComplete = true;
            }
        }

        private Tuple<int, int> GetVoxelCount(IVoxel voxel)
        {
            int total = 0;
            int leafs = 0;

            int level = voxel.TessLevel;
            int index = voxel.SerialIndex;
            
            while (++level < adaptiveGrid.LevelsCounts.Count)
            {
                IVoxel iV;

                if (!adaptiveGrid.FirstAtLevel(level, index, out iV)) break;

                do
                {
                    total++;

                    if (iV.ContentSI < 0) leafs++;
                }
                while (adaptiveGrid.NextAtLevel(index, ref iV));
            }

            return new Tuple<int, int>(total, leafs);
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

            // voxel count
            if (focusedVoxel != null)
            {
                if (focusedVoxel.SerialIndex != focusedVoxelSI)
                {
                    focusedVoxelSI = focusedVoxel.SerialIndex;

                    var Vc = GetVoxelCount(focusedVoxel);

                    VoxelSerialTxtBlock.Text = focusedVoxelSI.ToString();
                    VoxelLevelTxtBlock.Text = focusedVoxel.TessLevel.ToString();
                    InnerVoxelsTxtBlock.Text = string.Format("{0,6}, {1,6}", Vc.Item1, Vc.Item2);
                }
            }
            else
            {
                VoxelSerialTxtBlock.Text = "_";
                VoxelLevelTxtBlock.Text = "_";
                InnerVoxelsTxtBlock.Text = "_";
            }

            // output performance 
            FPSTxtBlock.Text = string.Format("{0,4:N0}, {1,4:N0} avg", fps, fpsAvg);
            SimTxtBlock.Text = timeAdvModel == 0 ? "<1" : timeAdvModel.ToString();
            RefineTxtBlock.Text = string.Format("{0,4:N0}, {1,4:N0} avg", timeRefine, timeRefineAvg);
            DrawGridVoxsTxtBlock.Text = string.Format("{0,4:N0}, {1,4:N0}", timeDrawGrid, timeDrawVoxsModel);

            // output simulation time
            ModelTimeTxtBlock.Text = TimeSpan.FromMilliseconds(Model.Time).ToString("c");
            
            if (modeldt == 0)
            {
                if (Model.TimeStep == 0) ModelStepTxtBox.Text = "variable";
                else ModelStepTxtBox.Text = Model.TimeStep.ToString("G8");
            }
            else modeldt.ToString("G8");
        }
        
        [Flags]
        private enum GItems { None = 0, Grid = 1, Voxels = 2, Model = 4, ModelGPU = 8, All = 15 }

        private enum Refiners { None, Serial, CGParallel, iCGParallel, FGParallel, SFGParallel, iFGParallel }

        private delegate void Updater();
        private delegate void Refiner(double threshold, Updater callback = null, bool block = false);

        private Refiner activeRefiner;
        private double refinerThold = 0.005;

        private void SetRefiner(object sender, SelectionChangedEventArgs e)
        {
            switch (RefiningAlgoList.SelectedIndex)
            {
                case 1: activeRefiner = RefineGrid;
                    break;
                case 2: activeRefiner = RefineGrid_CGParallel;
                    break;
                case 3: activeRefiner = RefineGrid_iCGParallel;
                    break;
                case 4: activeRefiner = RefineGrid_FGParallel;
                    break;
                case 5:
                    activeRefiner = RefineGrid_SFGParallel;
                    SFGParallelUniformity = false;
                    break;
                case 6: activeRefiner = RefineGrid_iFGParallel;
                    break;
                default: activeRefiner = null;
                    break;
            }

            if (activeRefiner == null) RefineButton.IsEnabled = false;
            else RefineButton.IsEnabled = true;
        }

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
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;
            }
            else if (ShowModelButton.IsChecked == true && (gi & GItems.Model) == GItems.Model)
            {
                // ### Diagnostic
                funcTimer.Restart();

                DrawModelParallel();

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;
            }
            else if (ShowModelGPUButton.IsChecked == true && (gi & GItems.ModelGPU) == GItems.ModelGPU)
            {
                // ### Diagnostic
                funcTimer.Restart();

                // draw with GPU
                Model.DrawFrame();

                // and write pixels to the bitmap
                signalBitmap.WritePixels(new Int32Rect(0, 0, _pW, _pH), modelPixelArray, rowStride, 0, 0);

                // ### Diagnostic
                funcTimer.Stop();
                timeDrawVoxsModel = funcTimer.ElapsedMilliseconds;
            }
            else timeDrawVoxsModel = 0;
        }
        
        private Stopwatch frameTimer = new Stopwatch();
        private Stopwatch funcTimer = new Stopwatch();

        private const long InfoFreshPeriod = 250;
        private long timeRefine, timeDrawGrid, timeDrawVoxsModel, timeAdvModel, fps;
        private long avgCounter, updateCounter;
        private double timeRefineAvg, fpsAvg;

        private void ResetAvg()
        {
            timeRefineAvg = 0;
            fpsAvg = 0;

            avgCounter = 1;
            updateCounter = 0;
        }

        private void UpdateAvg()
        {
            fpsAvg += (fps - fpsAvg) / avgCounter;
            timeRefineAvg += (timeRefine - timeRefineAvg) / avgCounter;

            avgCounter++;
        }

        private void UpdateFrame(object o, EventArgs e)
        {
            // ### Diagnostic
            funcTimer.Restart();

            Model.IncrementTime(modeldt);

            // ### Diagnostic
            funcTimer.Stop();
            timeAdvModel = funcTimer.ElapsedMilliseconds;

            // ### Diagnostic
            funcTimer.Restart();

            activeRefiner?.Invoke(refinerThold, block: true);
            UpdateGI();
            
            //int dt = ((RenderingEventArgs)e).RenderingTime.Milliseconds;
            long dt = frameTimer.ElapsedMilliseconds;
            frameTimer.Restart();

            if (dt != 0) fps = 1000 / dt;

            updateCounter += dt;
            
            UpdateAvg();

            if (updateCounter > InfoFreshPeriod)
            {
                updateCounter = 0;
                UpdateInfo();
            }
        }

        private void ModelSimulationToggle(bool start)
        {
            if (start)
            {
                ResetAvg();
                SFGParallelUniformity = false;

                frameTimer.Restart();
                CompositionTarget.Rendering += UpdateFrame;
            }
            else
            {
                frameTimer.Stop();
                CompositionTarget.Rendering -= UpdateFrame;
            }
        }
        
        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Back)
            {
                foreach (var v in voxelArray) v.Merge(true, initializer: VoxelFunc);

                UpdateGI();
                UpdateInfo();
            }
        }
        
        private bool RootShapeInput()
        {
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
            SetGPU();

            ResetAvg();
            UpdateGI();
            UpdateInfo();

            if (focusedVoxel != null) DrawFocusRect(focusedVoxel);

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
            ResetAvg();
            UpdateGI();
            UpdateInfo();

            return true;
        }

        private void SettingsBlock_MLBdown(object sender, MouseButtonEventArgs e)
        {
            // stop model simulation
            if (StartStopButton.IsChecked == true)
            {
                StartStopButton.IsChecked = false;
                StartStopButton_Click(null, null);
            }
        }

        private void RenderBttsStack_Click(object sender, RoutedEventArgs e)
        {
            var sourceBtt = (FrameworkElement)e.OriginalSource;
            bool isSimActive = StartStopButton.IsChecked == true;

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

                        LDrawGridVoxsTxtBlock.Text = "Draw grid,voxels ms:";

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

                        LDrawGridVoxsTxtBlock.Text = "Draw grid,model ms:";

                        if (!isSimActive) UpdateGI(GItems.Model);

                        ModelImage.Visibility = Visibility.Visible;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;

                case "ShowModelGPUButton":
                    if (ShowModelGPUButton.IsChecked == true)
                    {
                        ShowVoxsButton.IsChecked = false;
                        ShowModelButton.IsChecked = false;

                        LDrawGridVoxsTxtBlock.Text = "Draw grid,model ms:";

                        if (!isSimActive) UpdateGI(GItems.ModelGPU);

                        ModelImage.Visibility = Visibility.Visible;
                    }
                    else ModelImage.Visibility = Visibility.Collapsed;

                    break;
            }
        }

        private void RootShapeTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            RootShapeInput();
            RootShapeTxtBox.Text = ArrayToString(rootGridShape);
        }

        private void TessShapeTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            if (!TessShapeInput()) TessShapeTxtBox.Text = ArrayToString(tessFactors);
        }

        private void RootShapeTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (!RootShapeInput()) RootShapeTxtBox.Text = ArrayToString(rootGridShape);
            }
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
            ResetAvg();
            UpdateGI();
            UpdateInfo();
        }

        private void TessNumTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
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
                ResetAvg();
                UpdateGI();
                UpdateInfo();
            }
        }
        
        private void ThresholdTxtBox_LostKeyFocus(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(ThresholdTxtBox.Text, out double h) && h > 0)
            {
                refinerThold = h;
            }

            ThresholdTxtBox.Text = refinerThold.ToString();
        }

        private void ThresholdTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (double.TryParse(ThresholdTxtBox.Text, out double h) && h > 0)
                {
                    refinerThold = h;
                }

                ThresholdTxtBox.Text = refinerThold.ToString();
            }
        }
        
        private void MainCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point point = e.GetPosition(MainCanvas);
            PointToModel(ref point);
            IVoxel voxel;

            if (e.ClickCount == 1)
            {
                if (adaptiveGrid.GetInnermost(new double[2] { point.X, point.Y }, out voxel))
                {
                    focusedVoxel = voxel;
                    DrawFocusRect(voxel);
                    UpdateInfo();
                }
            }
            else
            {
                if (adaptiveGrid.GetInnermost(new double[2] { point.X, point.Y }, out voxel))
                {
                    if (voxel.Tessellate(true, VoxelFunc))
                    {
                        DrawCellAfterTess(voxel);
                        DrawVoxelAfterTess(voxel);
                        focusedVoxelSI = -1;
                        UpdateInfo();
                    }
                }
            }
        }
        
        private void MainCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (focusedVoxel == null) return;

            if (e.Delta > 0) // dive to inner voxels
            {
                if (focusedVoxel.ContentSI > 0)
                {
                    focusedVoxel = adaptiveGrid.InternalArray[focusedVoxel.ContentSI];
                    DrawFocusRect(focusedVoxel);
                    UpdateInfo();
                }
            }
            else // emerge to upper level
            {
                focusedVoxel = focusedVoxel.Ancestor;

                if (focusedVoxel == null)
                {
                    FocusRect.Visibility = Visibility.Collapsed;
                }
                else
                {
                    DrawFocusRect(focusedVoxel);
                }

                UpdateInfo();
            }
        }

        private void MainCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (focusedVoxel != null && e.ClickCount == 2)
            {
                Point point = e.GetPosition(MainCanvas);
                PointToModel(ref point);

                if (focusedVoxel.Merge(true, VoxelFunc))
                {
                    DrawCellAfterMerge(focusedVoxel);
                    DrawVoxelAfterMerge(focusedVoxel);
                    focusedVoxelSI = -1;
                    UpdateInfo();
                }
            }
        }
        
        private void RefineButton_Click(object sender, RoutedEventArgs e)
        {
            // ### Diagnostic
            funcTimer.Restart();

            activeRefiner?.Invoke(refinerThold, block: true);
            UpdateGI();
            UpdateInfo();
        }

        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            foreach (var v in voxelArray) v.Merge(true, initializer: VoxelFunc);

            UpdateGI();
            UpdateInfo();
        }

        private void ModelStepTxtBox_MLBdown(object sender, MouseButtonEventArgs e)
        {
            // stop model simulation
            if (StartStopButton.IsChecked == true)
            {
                StartStopButton.IsChecked = false;
                StartStopButton_Click(null, null);
            }
        }

        private void ModelStepTxtBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                string inputStr = ModelStepTxtBox.Text;
                bool multiply = inputStr[0] == '*';

                if (multiply) inputStr = inputStr.Substring(1);

                if (double.TryParse(inputStr, out double val))
                {
                    if (multiply)
                    {
                        modeldt = val * Model.TimeStep;
                    }
                    else
                    {
                        modeldt = val;
                    }
                }

                ModelStepTxtBox.Text = modeldt == 0 ? "variable" : modeldt.ToString("G8");
            }
        }

        private void ModelStepTxtBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            string inputStr = ModelStepTxtBox.Text;
            bool multiply = inputStr[0] == '*';

            if (multiply) inputStr = inputStr.Substring(1);

            if (double.TryParse(inputStr, out double val))
            {
                if (multiply)
                {
                    modeldt = val * Model.TimeStep;
                }
                else
                {
                    modeldt = val;
                }
            }

            ModelStepTxtBox.Text = modeldt == 0 ? "variable" : modeldt.ToString("G8");
        }

        private void StartStopButton_Click(object sender, RoutedEventArgs e)
        {
            if (StartStopButton.IsChecked == true)
            {
                ModelSimulationToggle(true);
            }
            else
            {
                ModelSimulationToggle(false);
            }
        }

        private void NextButton_Click(object sender, RoutedEventArgs e)
        {
            UpdateFrame(null, null);
        }

        private void PreviousButton_Click(object sender, RoutedEventArgs e)
        {
            if (modeldt != 0)
            {
                modeldt = -modeldt;
                UpdateFrame(null, null);
                modeldt = -modeldt;
            }
            else
            {
                modeldt = -Model.TimeStep;
                UpdateFrame(null, null);
                modeldt = 0;
            }
        }

        private void ResetModelButton_Click(object sender, RoutedEventArgs e)
        {
            // stop model simulation
            if (StartStopButton.IsChecked == true)
            {
                StartStopButton.IsChecked = false;
                StartStopButton_Click(null, null);
            }

            Model.ResetSimulation();
            RefineButton_Click(null, null);
        }

        #region Helpers and extensions

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
