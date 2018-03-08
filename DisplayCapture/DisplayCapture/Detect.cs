using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Data;
using System.Windows.Input;
using System.Diagnostics;



namespace DisplayDetector
{

    public delegate void UpdatePictureHandler(object sender, UpdatePictureArgs e);

    public class Detector
    {
        #region リングバッファ

        class RingBuf<T>
        {
            private readonly object _asyncLock = new object();
            private readonly int _bufSize;

            private T[] _items;
            private int _bufPos = 0;
            public int Count = 0;

            [Description("インデックスは0が最新で大きくなるほど古くなる")]
            public T this[int index]
            {
                get
                {
                    if (index >= _bufSize || index >= Count)
                        throw new IndexOutOfRangeException();

                    return _items[(_bufPos + _bufSize - index - 1) % _bufSize];
                }
            }

            public RingBuf(int bufSize)
            {
                _bufSize = bufSize;
                _items = new T[_bufSize];
            }

            public void Add(T item)
            {
                lock (_asyncLock)
                {

                    _items[_bufPos] = item;
                    _bufPos++;
                    Count++;
                    if (_bufPos == _bufSize)
                        _bufPos = 0;
                }
            }

            public T Latest()
            {
                lock (_asyncLock)
                {
                    return _items[_bufPos];
                }
            }

            [Description("インデックスは0が最新で大きくなるほど古くなる")]
            public T[] ToArray()
            {
                T[] items;
                lock (_asyncLock)
                {

                    int num = _bufSize < Count ? _bufSize : Count;
                    items = new T[num];
                    for (int i = 0; i < num; i++)
                    {
                        items[i] = _items[(_bufPos + _bufSize - i) / _bufSize];
                    }
                }
                return items;
            }

            public void Clear()
            {
                lock (_asyncLock)
                {
                    Count = 0;
                    _bufPos = 0;
                    _items = new T[_bufSize];
                }
            }

        }

        #endregion

        #region private変数



        //キャプチャ画面の情報
        private IntPtr _hwnd; //ウィンドウハンドル

        private BITMAP _defBitmap; //デフォルトのビットマップ情報

        //検出器の情報
        private bool _isRun = false; //検出器が起動しているかどうか


        internal Exception _exception { get; private set; } //例外が発生しているかどうか

        private Stopwatch _sw = new Stopwatch();
        private long _divTime;
        private object _lapTimeLock = new object();

        #endregion

        #region privateメソッド

        //初期化
        void Initialize(PixelFormat pixelFormat)
        {

            _defBitmap = DisplayCapt.CaptureWindow(_hwnd);

            if (_defBitmap == null)
                throw new InvalidHandleException();

            _defBitmap.PixelFormat = pixelFormat;
            BitmapData bitmapData = _defBitmap.Bitmap.LockBits(new Rectangle(0, 0, _defBitmap.Width, _defBitmap.Height),
                ImageLockMode.ReadOnly, _defBitmap.PixelFormat);
            _defBitmap.Stride = bitmapData.Stride;
            _defBitmap.Size = _defBitmap.Stride * _defBitmap.Height;
            _defBitmap.Bitmap.UnlockBits(bitmapData);

        }

        //一定時間ごとにスクショを保存する。
        void GetPict()
        {
            BITMAP bitmap = DisplayCapt.CaptureWindow(_hwnd);
            if (bitmap == null)
            {
                _exception = new InvalidHandleException();
                _isRun = false;
                return;
            }

            if (bitmap.Width != _defBitmap.Width || bitmap.Height != _defBitmap.Height)
            {
                _exception = new ChangeSizeException();
                _isRun = false;
                return;
            }


            BitmapData bdata = bitmap.Bitmap.LockBits(new Rectangle(0, 0, _defBitmap.Width, _defBitmap.Height),
                ImageLockMode.ReadOnly, _defBitmap.PixelFormat);

            byte[] buf = new byte[_defBitmap.Size];

            Marshal.Copy(bdata.Scan0, buf, 0, _defBitmap.Size);

            bitmap.Bitmap.UnlockBits(bdata);

            _pictureBuf.Add(buf);

        }

        //検出器の起動中のメソッド
        async void Running()
        {
            await Task.Run(() =>
            {

                _sw.Start();
                long formerTime = 0;
                while (_isRun)
                {
                    try
                    {


                        GetPict();
                        Detection();

                        long time;
                        do
                        {
                            time = _sw.ElapsedMilliseconds;
                        } while (time - formerTime < 200);

                        lock (_lapTimeLock)
                        {
                            _divTime = time - formerTime;
                        }
                        formerTime = time;

                        UpdatePictureCall(this, new UpdatePictureArgs(this));
                    }
                    catch (Exception e)
                    {
                        _isRun = false;
                        _exception = e;
                    }
                }
                _sw.Stop();
            });
        }

        #endregion

        #region 検出器プログラム

        //HSV構造体
        struct HSV
        {
            public readonly static uint uintMax = uint.MaxValue;

            public float H; //0~360
            public float S; //0~uintMax
            public float V; //0~uintMax

            public HSV(float h, float s, float v)
            {
                H = h;
                S = s;
                V = v;
            }

            public static HSV BGRtoHSV(byte b, byte g, byte r)
            {
                float[] bgr = new float[] {b, g, r};
                int maxI; //rgb最大
                int minI; //rgb最小
                float Hh, Ss, Vv;

                #region 最大最小の割り当て

                if (bgr[0] >= bgr[1])
                    if (bgr[0] >= bgr[2])
                    {
                        maxI = 0;
                        if (bgr[1] >= bgr[2])
                        {
                            if (bgr[0] == bgr[2])
                                minI = 0;
                            else
                                minI = 2;
                        }
                        else
                            minI = 1;
                    }
                    else
                    {
                        minI = 1;
                        maxI = 2;
                    }
                else if (bgr[1] >= bgr[2])
                {
                    maxI = 1;
                    if (bgr[0] >= bgr[2])
                        minI = 2;
                    else
                        minI = 0;
                }
                else
                {
                    minI = 0;
                    maxI = 2;
                }

                #endregion

                #region SとV

                if (bgr[maxI] != 0)
                    Ss = (bgr[maxI] - bgr[minI]) / bgr[maxI];
                else
                    Ss = 0;
                Vv = bgr[maxI] / 255;

                #endregion

                #region H

                if (maxI != minI)
                {
                    float h = 0;
                    switch (maxI)
                    {
                        case 0:
                            h = 60 * (bgr[1] - bgr[2]) / (bgr[maxI] - bgr[minI]);
                            break;
                        case 1:
                            h = 60 * (bgr[2] - bgr[0]) / (bgr[maxI] - bgr[minI]) + 120;
                            break;
                        case 2:
                            h = 60 * (bgr[0] - bgr[1]) / (bgr[maxI] - bgr[minI]) + 240;
                            break;
                    }
                    Hh = (h + 360) % 360;
                }
                else
                {

                    Hh = -1;
                }

                #endregion

                return new HSV(Hh, Ss, Vv);
            }

            public byte[] ToBGR()
            {
                float max = V * 255;
                float min = (V - S * V) * 255;

                if (S == 0)
                {
                    return new[] {(byte) min, (byte) min, (byte) min};
                }
                else if (H < 60)
                {

                    return new byte[] {(byte) max, (byte) ((H / 60.0f) * (max - min) + min), (byte) min};
                }
                else if (H < 120)
                {

                    return new byte[] {(byte) (((120 - H) / 60.0f) * (max - min) + min), (byte) max, (byte) min};
                }
                else if (H < 180)
                {

                    return new byte[] {(byte) min, (byte) max, (byte) (((H - 120) / 60.0f) * (max - min) + min)};
                }
                else if (H < 240)
                {

                    return new byte[] {(byte) min, (byte) (((240 - H) / 60.0f) * (max - min) + min), (byte) max};
                }
                else if (H < 300)
                {

                    return new byte[] {(byte) (((H - 240) / 60.0f) * (max - min) + min), (byte) min, (byte) max};
                }

                return new byte[] {(byte) max, (byte) min, (byte) (((360 - H) / 60.0f) * (max - min) + min)};

            }
        }

        RingBuf<byte[]> _pictureBuf = new RingBuf<byte[]>(5); //キャプチャ画面のリングバッファ
        //private RingBuf<byte[,]> _divs = new RingBuf<byte[,]>(2); //
        private RingBuf<HSV[,]> _hsvs = new RingBuf<HSV[,]>(10);
        //private byte[][,] _marks = new byte[5][,];

        private RingBuf<float[,]> _grayScales = new RingBuf<float[,]>(5); //キャプチャ画面のグレースケール


        private byte _lowThresholdTime;
        private byte _highThresholdTime;

        private byte[] OutputPicture; //出力する画像

        //一定時間ごとに解析
        void Detection()
        {
            if (_pictureBuf.Count < 2)
                return;

            int height = _defBitmap.Height;
            int width = _defBitmap.Width;
            int stride = _defBitmap.Stride;

            byte[] pic1 = _pictureBuf[0];


            HSV[,] picHsv = new HSV[width, height];
            float[,] picGray = new float[width, height];

            //hsv作成
            for (int i = 0; i < height; i++)
            {
                int h = i * stride;
                for (int j = 0; j < width; j++)
                {
                    byte b = pic1[4 * j + h];
                    byte g = pic1[4 * j + h + 1];
                    byte r = pic1[4 * j + h + 2];

                    picHsv[j, i] = HSV.BGRtoHSV(b, g, r);
                    picGray[j, i] = (b * 0.114f + g * 0.587f + r * 0.299f);
                }
            }
            _hsvs.Add(picHsv);
            _grayScales.Add(picGray);

            //時間軸検出器の実行
            //Detector1(width, height, stride, ref pic1);
            //Detector2(width, height, stride, ref pic1);
            //Detector3(width,height,stride,ref pic1);

            //エッジ抽出
            if (_grayScales.Count > 5)
            {
                pic1=new byte[pic1.Length];
                //EdgeDetector(width, height, stride, ref pic1, _grayScales[1]);
                EdgeDetector(width, height, stride, ref pic1, _grayScales[4]);
            }
            OutputPicture = pic1;
        }

        #region 時間軸検出器1～3

        void Detector1(int width, int height, int stride, ref byte[] pic)
        {
            //一定数の画像がなければ実行しない
            if (_hsvs.Count < 3)
                return;

            //表示する画像を設定
            pic = _pictureBuf[1];

            //検出処理
            for (int i = 0; i < height; i++)
            {
                int h = i * stride;
                for (int j = 0; j < width; j++)
                {
                    HSV hsv0 = _hsvs[1][j, i];
                    HSV hsv1 = _hsvs[0][j, i];
                    float hdiv = hsv0.H - hsv1.H;

                    if (((hdiv > 2 || hdiv < -2) && hsv0.V >= 0.2 && hsv1.V >= 0.2 && hsv0.S >= 0.1 && hsv1.S >= 0.1)
                        || (hsv0.V - 0.2) * (hsv1.V - 0.2) < 0
                        || (hsv0.S - 0.1) * (hsv1.S - 0.1) < 0)
                    {

                        hsv1 = _hsvs[2][j, i];
                        hdiv = hsv0.H - hsv1.H;

                        if (((hdiv > 2 || hdiv < -2) && hsv0.V >= 0.2 && hsv1.V >= 0.2 && hsv0.S >= 0.1 &&
                             hsv1.S >= 0.1)
                            || (hsv0.V - 0.2) * (hsv1.V - 0.2) < 0
                            || (hsv0.S - 0.1) * (hsv1.S - 0.1) < 0)
                        {
                            //検出している部分ならそのままにする
                            continue;
                        }
                    }

                    //検出していない部分を黒く塗りつぶす
                    pic[4 * j + h] = 0;
                    pic[1 + 4 * j + h] = 0;
                    pic[2 + 4 * j + h] = 0;

                }
            }


        }

        void Detector2(int width, int height, int stride, ref byte[] pic)
        {
            //一定数の画像がなければ実行しない
            if (_hsvs.Count < 9)
                return;

            //rgb変換
            pic = _pictureBuf[4];

            for (int i = 0; i < height; i++)
            {
                int h = i * stride;

                for (int j = 0; j < width; j++)
                {
                    HSV hsv0 = _hsvs[4][j, i];
                    bool flag = false;

                    for (int k = 0; k < 4; k++)
                    {

                        HSV hsv1 = _hsvs[k][j, i];
                        float hdiv = hsv0.H - hsv1.H;
                        if (((hdiv > 2 || hdiv < -2) && hsv0.V >= 0.2 && hsv1.V >= 0.2 && hsv0.S >= 0.1 &&
                             hsv1.S >= 0.1)
                            || (hsv0.V - 0.2) * (hsv1.V - 0.2) < 0
                            || (hsv0.S - 0.1) * (hsv1.S - 0.1) < 0)
                        {
                            hsv1 = _hsvs[8 - k][j, i];
                            hdiv = hsv0.H - hsv1.H;
                            if (((hdiv > 2 || hdiv < -2) && hsv0.V >= 0.2 && hsv1.V >= 0.2 && hsv0.S >= 0.1 &&
                                 hsv1.S >= 0.1)
                                || (hsv0.V - 0.2) * (hsv1.V - 0.2) < 0
                                || (hsv0.S - 0.1) * (hsv1.S - 0.1) < 0)
                            {
                                //検出しているならフラグを立てる。
                                flag = true;
                                break;
                            }
                        }

                    }
                    //フラグが立ってなかったら（検出してなかったら）黒く塗りつぶす
                    if (!flag)
                    {
                        pic[4 * j + h] = 0;
                        pic[1 + 4 * j + h] = 0;
                        pic[2 + 4 * j + h] = 0;
                    }
                }
            }




        }
        void Detector3(int width, int height, int stride, ref byte[] pic)
        {
            //一定数の画像がなければ実行しない
            if (_hsvs.Count < 9)
                return;

            //rgb変換
            pic = _pictureBuf[4];

            for (int i = 0; i < height; i++)
            {
                int h = i * stride;

                for (int j = 0; j < width; j++)
                {
                    HSV hsv0 = _hsvs[4][j, i];
                    bool flag = true;
                    bool[] prevFlags=new bool[4];
                    bool[] nextFlags = new bool[4];

                    for (int k = 0; k < 4; k++)
                    {

                        HSV hsv1 = _hsvs[k][j, i];
                        float hdiv = hsv0.H - hsv1.H;
                        if (!((hdiv > 2 || hdiv < -2) && hsv0.V >= 0.2 && hsv1.V >= 0.2 && hsv0.S >= 0.1 &&
                             hsv1.S >= 0.1
                            || (hsv0.V - 0.2) * (hsv1.V - 0.2) < 0
                            || (hsv0.S - 0.1) * (hsv1.S - 0.1) < 0))
                        {
                            flag = false;
                        }

                    }
                    if (!flag)
                    {
                        flag = true;
                        for (int k = 0; k < 4; k++)
                        {
                            HSV hsv1 = _hsvs[8 - k][j, i];
                            float hdiv = hsv0.H - hsv1.H;
                            if (!((hdiv > 2 || hdiv < -2) && hsv0.V >= 0.2 && hsv1.V >= 0.2 && hsv0.S >= 0.1 &&
                                  hsv1.S >= 0.1
                                  || (hsv0.V - 0.2) * (hsv1.V - 0.2) < 0
                                  || (hsv0.S - 0.1) * (hsv1.S - 0.1) < 0))
                            {
                                //検出しているならフラグを立てる。
                                flag = false;
                                break;
                            }
                        }
                    }
                    //フラグが立ってなかったら（検出してなかったら）黒く塗りつぶす
                    if (!flag)
                    {
                        pic[4 * j + h] = 0;
                        pic[1 + 4 * j + h] = 0;
                        pic[2 + 4 * j + h] = 0;
                    }
                }
            }




        }
        #endregion

        #region エッジを用いた検出器

        private static float[,] _GK = new float[,]
        {
            {2.0f / 159, 4.0f / 159, 5.0f / 159, 4.0f / 159, 2.0f / 159},
            {4.0f / 159, 9.0f / 159, 12.0f / 159, 9.0f / 159, 4.0f / 159},
            {5.0f / 159, 12.0f / 159, 15.0f / 159, 12.0f / 159, 5.0f / 159},
            {4.0f / 159, 9.0f / 159, 12.0f / 159, 9.0f / 159, 4.0f / 159},
            {2.0f / 159, 4.0f / 159, 5.0f / 159, 4.0f / 159, 2.0f / 159}
        }; //ガウシアンフィルター（平衡化フィルタ）

        private static int[,] _Fx = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}}; //y方向のフィルタ
        private static int[,] _Fy = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}}; //x方向のフィルタ

        private static (int x, int y)[,] _angleCheck = new (int, int)[,]//non-maximumsppression
            {{(-1, 0), (1, 0)}, {(-1, -1), (1, 1)}, {(0, -1), (0, 1)}, {(-1, 1), (1, -1)}}; 

        private static float _lowThresholdPic=5.0f;//低いほうの閾値
        private static float _highThresholdPic=15.0f;//高いほうの閾値

        //エッジのバッファ
        private RingBuf<bool[,]> _edges=new RingBuf<bool[,]>(5);


        void EdgeDetector(int width, int height, int stride, ref byte[] pic,float[,] originGray)
        {
            

            //表示する画像を設定
            //pic = _pictureBuf[1];

            float[,] eGradient=new float[width,height];//勾配の大きさ
            int[,] eAngle=new int[width,height];//角度(0,45,90,135)

            //エッジにかかってるピクセル（0:エッジではない,1:エッジ,2:未確認）
            byte[,] pixEdge=new byte[width,height];

            //エッジに結合してるチェックする必要があるピクセル
            Stack<(int x,int y)> pixCheck=new Stack<(int,int)>();


            //勾配の計算
            for (int i = 1; i < height - 1; i++)
            {
                int h = i * stride;
                for (int j = 1; j < width - 1; j++)
                {
                    //エッジの強度を検出
                    double Gx = 0; //x方向
                    double Gy = 0; //y方向

                    for (int kx = 0; kx < 3; kx++)
                    {
                        for (int ky = 0; ky < 3; ky++)
                        {
                            Gx += _Fx[kx, ky] * _GK[kx + 1, ky + 1] * originGray[j + kx - 1, i + ky - 1];
                            Gy += _Fy[kx, ky] * _GK[kx + 1, ky + 1] * originGray[j + kx - 1, i + ky - 1];
                        }
                    }

                    eGradient[j,i] = (float) Math.Sqrt(Gx*Gx+Gy*Gy);
                    if (Gx != 0)
                    {
                        float tanXY = (float) (Gy / Gx);
                        if (tanXY < -2.4142135)
                            eAngle[j, i] = 2;
                        else if (tanXY < -0.41421356)
                            eAngle[j, i] = 3;
                        else if (tanXY < 0.41421356)
                            eAngle[j,i] = 0;
                        else if (tanXY < -2.4142135)
                            eAngle[j, i] = 1;
                        else
                            eAngle[j, i] = 2;
                    }
                    else
                    {
                        eAngle[j, i] = 2;
                    }
                }
            }
            
            //最大値の確認と閾値チェック
            for (int i = 1; i < height - 1; i++)
            {
                int h = i * stride;
                for (int j = 1; j < width - 1; j++)
                {
                    float eg = eGradient[j, i];
                    if (eg >= eGradient[j + _angleCheck[eAngle[j, i], 0].x,
                            i + _angleCheck[eAngle[j, i], 0].y] &&
                        eg >= eGradient[j + _angleCheck[eAngle[j, i], 1].x,
                            i + _angleCheck[eAngle[j, i], 1].y])
                    {
                        if (eg < _lowThresholdPic)
                        {
                            //低いほう閾値よりも低い
                            pixEdge[j, i] = 0;
                        }
                        else if (_highThresholdPic <= eg)
                        {
                            //高いほうの閾値よりも高い
                            pixEdge[j, i] = 1;
                        }
                        else
                        {
                            for (int kx = 0; kx < 3; kx++)
                            {
                                for (int ky = 0; ky < 3; ky++)
                                {
                                    if (pixEdge[j + kx - 1, i + ky - 1] == 1)
                                    {
                                        pixEdge[j, i] = 1;
                                    }
                                }
                            }
                            if (pixEdge[j, i] == 0)
                            {
                                pixCheck.Push((j, i));
                                pixEdge[j, i] = 2;
                            }
                        }
                    }
                    else
                    {
                        eGradient[j, i] = 0;
                    }

                }
            }

            //未確認部分のチェック
            bool flag;
            Stack<(int, int)> pixCheck2=new Stack<(int, int)>();
            do
            {
                flag = false;
                if (pixCheck.Count == 0)
                    Checking(ref pixCheck2,ref pixCheck,pixCheck2.Count);
                else 
                    Checking(ref pixCheck,ref pixCheck2,pixCheck.Count);
                
                void Checking(ref Stack<(int, int)> prevS, ref Stack<(int, int)> nextS,int count)
                {
                    for (int i = 0; i < count; i++)
                    {
                        (int x, int y) point = prevS.Pop();
                        bool flag2 = false;

                        for (int kx = 0; kx < 3; kx++)
                        {
                            for (int ky = 0; ky < 3; ky++)
                            {
                                switch (pixEdge[point.x + kx - 1, point.y + ky - 1])
                                {
                                    case 1:
                                        pixEdge[point.x, point.y] = 1;
                                        flag = true;
                                        break;
                                    case 2:
                                        flag2 = true;
                                        break;
                                }
                                
                                    
                            }
                        }
                        if (pixEdge[point.x, point.y] == 1)
                            continue;

                        if (flag2 )
                            nextS.Push(point);
                        else
                        {
                            pixEdge[point.x, point.y] = 0;
                            flag = true;
                        }
                    }
                }
            } while (flag);

            //一定数の画像がなければ実行しない
            
            if (_edges.Count == 0)
                _edges.Add(new bool[width, height]);

            bool[,] prevEdge=_edges[0];

            bool[,] edge=new bool[width,height];
            

            //反映処理
            for (int i = 0; i < height; i++)
            {
                int h = stride * i;
                for (int j = 0; j < width; j++)
                {
                    if (pixEdge[j, i] == 1)
                    {
                        if (!prevEdge[j, i])
                        {
                            pic[j * 4 + h] = 0;
                            pic[j * 4 + h + 1] = 0;
                            pic[j * 4 + h + 2] = 255;
                        }
                        edge[j, i] = true;
                    }
                }
            }
            _edges.Add(edge);
            
        }

        
        #endregion

        #endregion

        #region publicメンバー



        //画像情報のプロパティ
        public int Height => _defBitmap.Height;
        public int Width => _defBitmap.Width;
        public int Stride => _defBitmap.Stride;

        //コンストラクタ
        public Detector(IntPtr hWnd, PixelFormat pixelFormat)
        {
            _hwnd = hWnd;//ハンドルを取得

            Initialize(pixelFormat);//初期化
        }

        //探索の開始
        public void Start()
        {
            if (_exception != null)
                throw _exception;

            if (_isRun)
                return;

            _isRun = true;
            Running();
        }

        //探索の終了
        public void Stop()
        {
            //if (_exception != null)
            //  throw _exception;

            _isRun = false;
        }

        //画像の取得
        public byte[] GetPicture()
        {
            if (_exception != null)
                throw _exception;

            if (!_isRun)
                return null;
            return OutputPicture;
        }

        //フレームレートの取得
        public double GetFramerate()
        {
            lock (_lapTimeLock)
            {
                return 1000.0f / _divTime;
            }
        }


        public event UpdatePictureHandler UpdatePictureCall = delegate { };

        #endregion
    }

    public class UpdatePictureArgs
    {
        internal UpdatePictureArgs(Detector d)
        {
            try
            {
                Bitmap = d.GetPicture();
                FrameRate = d.GetFramerate();
                Width = d.Width;
                Height = d.Height;
                Stride = d.Stride;
                Exception = d._exception;
            }
            catch (Exception e)
            {
                Exception = e;
            }
        }

        public byte[] Bitmap { get; }
        public double FrameRate { get; }
        public int Width { get; }
        public int Height { get; }
        public int Stride { get; }
        public Exception Exception;
    }



    //キャプチャ画面のサイズが変わったときに起こる例外
    public class ChangeSizeException : Exception
    {

    }

    //キャプチャ画面のハンドルが不正であるときに起こる例外
    public class InvalidHandleException : Exception
    {

    }

}
