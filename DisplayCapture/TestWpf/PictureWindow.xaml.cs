using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
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
using System.Windows.Threading;
using DisplayDetector;

namespace TestWpf
{
    /// <summary>
    /// PictureWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class PictureWindow : Window
    {
        Detector _detector; //検出器

        

        private object _BSourceLock = new object(); //
        private WriteableBitmap _wb; //キャプチャ画面用

        private BindingPWindow _bindingP;
        private string _defWinTitle;

        private bool _isRunDetector;//キャプチャ中かどうか
        private Exception _exception;//非同期処理の例外

        //コンストラクタ
        public PictureWindow(IntPtr hWnd)
        {
            InitializeComponent();

            _detector = new Detector(hWnd, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
            _wb = new WriteableBitmap(_detector.Width, _detector.Height, 96, 96, PixelFormats.Bgr32, null);
            pic.Source = _wb;


            _defWinTitle = Title;

            _bindingP=new BindingPWindow();
            DataContext = _bindingP;

            Binding binding2 = new Binding("WinTitle");
            binding2.Mode = BindingMode.TwoWay;
            binding2.UpdateSourceTrigger = UpdateSourceTrigger.PropertyChanged;
            binding2.IsAsync = true;
            this.SetBinding(Window.TitleProperty, binding2);

            
            _detector.UpdatePictureCall += UpdateCapture;

            _isRunDetector = true;
            _detector.Start();

        }


        //キャプチャ画面作成用関数
        public static void Create(IntPtr hWnd)
        {
            try
            {
                PictureWindow pctWin = new PictureWindow(hWnd);
                pctWin.Show();
            }
            catch (InvalidHandleException)
            {
                MessageBox.Show("そのウィンドウは存在しません。", "エラー", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        //定期的にキャプチャ画面を更新
        private bool _doUpdateCapture=false;
        void UpdateCapture(object sender,UpdatePictureArgs e)
        {
            if (_doUpdateCapture != false)
                return;
            _doUpdateCapture = true;
            if (e.Exception == null) { 
                byte[] buf = e.Bitmap;
                if (buf != null && !_wb.CheckAccess())
                    _wb.Dispatcher.InvokeAsync(() =>
                        {


                            _wb.WritePixels(new Int32Rect(0, 0, _wb.PixelWidth, _wb.PixelHeight), buf,
                                _wb.BackBufferStride, 0);
                        }
                    );
                        
                _bindingP.WinTitle = _defWinTitle + "[" + e.FrameRate + "]";
                //pic.Source = _wb;
                
            }
            else
            {
                _exception =e.Exception;
                _isRunDetector = false;
                if (!CheckAccess())
                {
                    Dispatcher.InvokeAsync(() =>
                        {
                            Close();
                        }
                    );
                }
            }
            _doUpdateCapture = false;
        }

        //閉じるボタン
        private void Close_Click(object sender, RoutedEventArgs e)
        {
            
            _isRunDetector = true;
            Close();
        }

        //ウィンドウを閉じるときの動作（例外処理）
        private void Closing_Window(object sender, System.ComponentModel.CancelEventArgs e)
        {
            _detector.Stop();
            if (_exception != null)
            {
                Type ex = _exception.GetType();
                if(ex==typeof(ChangeSizeException))
                    MessageBox.Show("キャプチャしているウィンドウのサイズを変えてはいけません。", "エラー", MessageBoxButton.OK, MessageBoxImage.Error);
                else if (ex == typeof(NullReferenceException))
                    MessageBox.Show("キャプチャウィンドウが閉じられました。", "エラー");
                else
                    MessageBox.Show("不明なエラーが発生しました。", "エラー");
            }

        }
    }
    public class BindingPWindow : INotifyPropertyChanged
    {

        private string _winTitle;
        public string WinTitle
        {
            get { return _winTitle; }
            set { OnPropertyChanged(ref _winTitle,value);}
        }

        #region 明示的インターフェース
        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged<T>(ref T field, T value, [CallerMemberName]string propertyName = "")
        {
            if (Equals(field, value)) return;
            field = value;

            if (PropertyChanged == null) return;
            PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
        }
        #endregion
    }

}
