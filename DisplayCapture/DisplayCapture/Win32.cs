using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Mime;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Interop;
using System.Runtime.InteropServices;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows;
using System.Windows.Controls;
using System.Drawing;


namespace DisplayDetector
{
    //Win32を操作する部分
    //参考：https://dobon.net/vb/dotnet/graphics/screencapture.html
    //      https://dobon.net/vb/dotnet/process/enumwindows.html
    //
    public class DisplayCapt
    {
        #region ウィンドウハンドルの一覧を取得する部分

        private static Queue<(string WindowName, string ClassName, IntPtr hWnd)> _hWnds;
        public static (string, string, IntPtr)[] GetEnumWindowsHandle()
        {
            _hWnds = new Queue<(string WindowName, string ClassName, IntPtr hWnd)>();
            //ウィンドウを列挙する
            EnumWindows(new EnumWindowsDelegate(EnumWindowCallBack), IntPtr.Zero);
            return _hWnds.ToArray();
        }

        public delegate bool EnumWindowsDelegate(IntPtr hWnd, IntPtr lparam);

        //表示されているウィンドウハンドルを取得する。
        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        public extern static bool EnumWindows(EnumWindowsDelegate lpEnumFunc,
            IntPtr lparam);

        //ウィンドウ名を取得する関数
        [DllImport("user32.dll", CharSet = CharSet.Auto, SetLastError = true)]
        private static extern int GetWindowText(IntPtr hWnd,
            StringBuilder lpString, int nMaxCount);

        //テキストの長さを取得する関数
        [DllImport("user32.dll", CharSet = CharSet.Auto, SetLastError = true)]
        private static extern int GetWindowTextLength(IntPtr hWnd);

        //クラス名を取得する関数
        [DllImport("user32.dll", CharSet = CharSet.Auto, SetLastError = true)]
        private static extern int GetClassName(IntPtr hWnd,
            StringBuilder lpClassName, int nMaxCount);

        //そのウィンドウが表示状態かどうかを確認するメソッド
        [DllImport("user32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool IsWindowVisible(IntPtr hWnd);

        //コールバック用メソッド
        private static bool EnumWindowCallBack(IntPtr hWnd, IntPtr lparam)
        {


            //ウィンドウのタイトルの長さを取得する
            int textLen = GetWindowTextLength(hWnd);
            if (0 < textLen && IsWindowVisible(hWnd))
            {
                //ウィンドウのタイトルを取得する
                StringBuilder tsb = new StringBuilder(textLen + 1);
                GetWindowText(hWnd, tsb, tsb.Capacity);



                //ウィンドウのクラス名を取得する
                StringBuilder csb = new StringBuilder(256);
                GetClassName(hWnd, csb, csb.Capacity);

                _hWnds.Enqueue((tsb.ToString(), csb.ToString(), hWnd));
            }



            //すべてのウィンドウを列挙する
            return true;
        }


        #endregion


        #region ウィンドウをキャプチャする関数
        private const int SRCCOPY = 13369376;


        //クライアントDCの取得する関数
        [DllImport("user32.dll")]
        private static extern IntPtr GetDC(IntPtr hwnd);

        // 例 の あ れ
        [DllImport("gdi32.dll")]
        private static extern int BitBlt(IntPtr hDestDC,
            int x,
            int y,
            int nWidth,
            int nHeight,
            IntPtr hSrcDC,
            int xSrc,
            int ySrc,
            int dwRop);

        //hDCを解放する関数
        [DllImport("user32.dll")]
        private static extern IntPtr ReleaseDC(IntPtr hwnd, IntPtr hdc);



        //Win32 RECT構造体
        [StructLayout(LayoutKind.Sequential)]
        private struct RECT
        {
            public int left;
            public int top;
            public int right;
            public int bottom;
        }

        //クライアント領域のサイズを取得する関数
        [DllImport("user32.dll")]
        private static extern int GetClientRect(IntPtr hwnd,
            ref RECT lpRect);


        internal static BITMAP CaptureWindow(IntPtr hWnd)
        {
            //アクティブなウィンドウのデバイスコンテキストを取得
            IntPtr winDC = GetDC(hWnd);

            //クライアント領域の大きさを取得
            RECT winRect = new RECT();

            GetClientRect(hWnd, ref winRect);


            if ((winRect.right - winRect.left) * (winRect.bottom - winRect.top) == 0)
                return null;
            //Bitmapの作成
            Bitmap bmp = new Bitmap(winRect.right - winRect.left,
                winRect.bottom - winRect.top);

            //BITMAPの設定
            BITMAP bitmap = new BITMAP();
            bitmap.Width = bmp.Width;
            bitmap.Height = bmp.Height;
            bitmap.Bitmap = bmp;

            //Graphicsの作成
            Graphics g = Graphics.FromImage(bitmap.Bitmap);

            //Graphicsのデバイスコンテキストを取得
            IntPtr hDC = g.GetHdc();

            //Bitmapに画像をコピーする
            BitBlt(hDC, 0, 0, bmp.Width, bmp.Height,
                winDC, 0, 0, SRCCOPY);

            //解放
            g.ReleaseHdc(hDC);
            g.Dispose();
            ReleaseDC(hWnd, winDC);

            return bitmap;
        }
        

        #endregion

    }

    //ビットマップのデータを管理するためのクラス
    internal class BITMAP
    {
        public int Width;
        public int Height;
        public int Stride = 0;
        public int Size = 0;
        public System.Drawing.Imaging.PixelFormat PixelFormat;
        public byte[] BitData;
        public Bitmap Bitmap;
    }
}
