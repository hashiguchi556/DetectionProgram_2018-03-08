using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Drawing;
using System.Drawing.Imaging;
using DisplayDetector;
using PixelFormat = System.Drawing.Imaging.PixelFormat;
using System.Diagnostics;


namespace TestWpf
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        private ObservableCollection<HWndData> _itemsList;
        
        //コンストラクタ
        public MainWindow()
        {
            
            InitializeComponent();
            _itemsList=new ObservableCollection<HWndData>();
            hWndData.ItemsSource = _itemsList;
        }

        //キャプチャーボタン
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            if (hWndData.SelectedItem == null)
                return;
            PictureWindow.Create(((HWndData)hWndData.SelectedItem).Get_hWnd);


        }

        //再読み込みボタン
        private void Road_Click(object sender, RoutedEventArgs e)
        {
            
            _itemsList.Clear();

            (string, string, IntPtr)[] hwnds = DisplayCapt.GetEnumWindowsHandle();

            foreach (var h in hwnds)
            {
                _itemsList.Add(new HWndData(h));
            }

        }

    }

    //GridData用のクラス
    class HWndData
    {
        public HWndData((string, string, IntPtr) item)
        {
            WindowName = item.Item1;
            ClassName = item.Item2;
            Get_hWnd = item.Item3;
        }
        public string WindowName { get; }
        public string ClassName { get; }
        public IntPtr Get_hWnd { get; }
    }
}
