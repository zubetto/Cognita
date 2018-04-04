using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace TestWpf
{
    /// <summary>
    /// Interaction logic for ModelSettings.xaml
    /// </summary>
    public partial class ModelSettings : Window
    {
        int itemIndex = -1;

        public ModelSettings()
        {
            InitializeComponent();
            Loaded += Window_Loaded;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            var desktopWorkingArea = SystemParameters.WorkArea;
            Left = desktopWorkingArea.Right - ActualWidth - 20;
        }

        private void ModelParamsListBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (e.OriginalSource is TextBox txb)
                {
                    int index = ModelParamsListBox.SelectedIndex;

                    if (itemIndex >= 0)
                    {
                        MainWindow.modelProps[itemIndex].SetValue(txb);
                    }
                }
            }
        }

        private void ModelParamsListBox_LostKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            if (e.OriginalSource is TextBox txb)
            {
                if (itemIndex >= 0)
                {
                    MainWindow.modelProps[itemIndex].SetValue(txb);
                }
            }
        }

        private void ModelParamsListBox_GotKeyboardFocus(object sender, KeyboardFocusChangedEventArgs e)
        {
            itemIndex = ModelParamsListBox.SelectedIndex;
        }
    }
}
