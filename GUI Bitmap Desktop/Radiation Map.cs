using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using IronPython.Hosting;
using System.IO;
using System.Runtime.InteropServices;
using System.Drawing.Imaging;
using System.Web;
using System.Drawing.Design;

namespace GUI_Bitmap_Desktop
{
    public partial class Radiashown : Form
    {
        Bitmap myBitmap;
        public Radiashown()
        {
            InitializeComponent();
        }
        private Bitmap DrawFilledRectangle(int x, int y)
        {
            Bitmap myBitmap = new Bitmap(x, y);
            using (Graphics graph = Graphics.FromImage(myBitmap))
            {
                Rectangle ImageSize = new Rectangle(0, 0, x, y);
                graph.FillRectangle(Brushes.White, ImageSize);
            }

            return myBitmap;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            //AllocConsole();
            //panel1.Controls.Add(pictureBox1);
        }

        private void startButton_Click(object sender, EventArgs e)
        {
            startButton.BackColor = Color.FromName("Green");
            startButton.Text = "Sweeping";
            startButton.Enabled = false;
            emergencyButton.Visible = true;
            setupButton.Visible = false;

            startScan();
        }

        private void emergencyButton_Click(object sender, EventArgs e)
        {
            startButton.BackColor = Color.FromName("FireBrick");
            startButton.Text = "Returning Home";
            startButton.Enabled = true;

            emergencyButton.Visible = false;
        }

        //This is a placeholder
        private void home_Click(object sender, EventArgs e)
        {
            startButton.BackColor = Color.FromName("Buttonshadow");
            startButton.Text = "Start Sweep";
            startButton.Enabled = true;

            emergencyButton.Visible = false;
            setupButton.Visible = true;
        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {
            startScan();
        }
        private void startScan()
        {
            //Graphics gr = CreateGraphics();
            myBitmap = DrawFilledRectangle(1000, 1000);
            pictureBox1.Image = myBitmap;
            //Bitmap robotIcon = new Bitmap("C:/Users/Marilyn/Desktop/Engineering 5/4TB6/Icon/moon-rover50.png");
            //pictureBox2.Image = robotIcon;
            pictureBox2.Image = pictureBox2.InitialImage;
            //Bitmap radIcon = new Bitmap("C:/Users/Marilyn/Desktop/Engineering 5/4TB6/Icon/radioactive-48.png");
            //pictureBox3.Image = radIcon;
            pictureBox3.Image = pictureBox3.InitialImage;
            //myBitmap.SetResolution(500, 500);
            //pictureBox1.SizeMode = PictureBoxSizeMode.AutoSize;

            simulationPerimeter();
            
        }

        private void DrawRadInfo(int xPos, int yPos, int radLevel)
        {
            var radColour = Brushes.White;
            using (Graphics graph = Graphics.FromImage(myBitmap))
            {
                Rectangle ImageSize = new Rectangle(xPos, yPos, 25, 25); 
                if (radLevel < 2)
                {
                    radColour = Brushes.Yellow;
                }
                if (radLevel > 1 & radLevel < 5)
                {
                    radColour = Brushes.Orange;
                }
                if (radLevel > 4 & radLevel < 10)
                {
                    radColour = Brushes.Red;
                }
                if (radLevel > 9)
                {
                    radColour = Brushes.DarkRed;
                }
                graph.FillRectangle(radColour, ImageSize);
            }

        }

        private void simulationSweep()
        {

            for (int Xcount = 50; Xcount < 950; Xcount = Xcount + 25)
            {
                for (int Ycount = 50; Ycount < 950; Ycount = Ycount + 25)
                {
                    //robotLocation(Xcount, Ycount);
                    if (Xcount % 25 == 0 & Ycount % 25 == 0)
                    {
                        DrawRadInfo(Xcount, Ycount, RandomNumber(0, 11));
                    }
                    pictureBox1.Refresh();

                }
            }
        }
        public int RandomNumber(int min, int max)
        {
            Random random = new Random();
            return random.Next(min, max);
        }

        private void simulationPerimeter()
        {
            radLocation(500, 500);
            for (int Ycount = 50; Ycount < 950; Ycount++)
            {
                myBitmap.SetPixel(50, Ycount, Color.Black);
                robotLocation(50, Ycount);
                pictureBox1.Refresh();
            }

            for (int Xcount = 50; Xcount < 950; Xcount++)
            {
                myBitmap.SetPixel(Xcount, 950, Color.Black);
                robotLocation(Xcount, 950);
                pictureBox1.Refresh();
            }

            for (int Ycount = 950; Ycount > 50; Ycount--)
            {
                myBitmap.SetPixel(950, Ycount, Color.Black);
                robotLocation(950, Ycount);
                pictureBox1.Refresh();
            }

            for (int Xcount = 950; Xcount > 50; Xcount--)
            {
                myBitmap.SetPixel(Xcount, 50, Color.Black);
                robotLocation(Xcount, 50);
                pictureBox1.Refresh();
            }
            simulationSweep();
        }

        private void radLocation(int x, int y)
        {
            pictureBox3.Location = new Point(x, y);
            pictureBox3.Refresh();
        }
        private void robotLocation(int x, int y)
        {
            pictureBox2.Location = new Point(x, y);
            pictureBox2.Refresh();
        }
        private void setupButton_Click(object sender, EventArgs e)
        {
            //this.Hide();
            radSetup fsetup = new radSetup();
            fsetup.ShowDialog();
            //this.Close();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            /*textBox1.Text = "test";
            // 1) Create engine
            var engine = Python.CreateEngine();

            // 2) Provide script and arguments
            var script = @"getRadAssistData.py";
            var source = engine.CreateScriptSourceFromFile(script);

            // 3) Output redirect
            var eIO = engine.Runtime.IO;

            var results = new MemoryStream();
            eIO.SetOutput(results, Encoding.Default);

            // 4) Execute script
            var scope = engine.CreateScope();
            source.Execute(scope);

            // 5) Display output
            string str(byte[] x) => Encoding.Default.GetString(x);

            Console.WriteLine();
            Console.WriteLine("Results:");
            Console.WriteLine(str(results.ToArray()));


            // Display the pixel format in Label1.
            textBox1.Text = str(results.ToArray());*/
        }
    }
}
/*
        private void MainScreenThread()
        {
            ReadData();//reading data from socket.
            initial = bufferToJpeg();//first intial full screen image.
            pictureBox1.Paint += pictureBox1_Paint;//activating the paint event.
            while (true)
            {
                int pos = ReadData();
                x = BlockX();//where to draw :X
                y = BlockY();//where to draw :Y
                Bitmap block = bufferToJpeg();//constantly reciving blocks.
                Draw(block, new Point(x, y));//applying the changes-drawing the block on the big initial image.using native memcpy.

                this.Invoke(new Action(() =>
                {
                    pictureBox1.Refresh();//updaing the picturebox for seeing results.
                                          // this.Text = ((pos / 1000).ToString() + "KB");
                }));
            }
        }

        private void pictureBox1_Paint(object sender, PaintEventArgs e)
        {
            lock (initial)
            {
                e.Graphics.DrawImage(initial, pictureBox1.ClientRectangle); //draws at picturebox's bounds
            }
        }

        private unsafe void Draw(Bitmap bmp2, Point point)
        {
            lock (initial)
            {
                BitmapData bmData = initial.LockBits(new Rectangle(0, 0, initial.Width, initial.Height), System.Drawing.Imaging.ImageLockMode.WriteOnly, initial.PixelFormat);
                BitmapData bmData2 = bmp2.LockBits(new Rectangle(0, 0, bmp2.Width, bmp2.Height), System.Drawing.Imaging.ImageLockMode.ReadOnly, bmp2.PixelFormat);
                IntPtr scan0 = bmData.Scan0;
                IntPtr scan02 = bmData2.Scan0;
                int stride = bmData.Stride;
                int stride2 = bmData2.Stride;
                int Width = bmp2.Width;
                int Height = bmp2.Height;
                int X = point.X;
                int Y = point.Y;

                scan0 = IntPtr.Add(scan0, stride * Y + X * 3);//setting the pointer to the requested line
                for (int y = 0; y < Height; y++)
                {
                    memcpy(scan0, scan02, (UIntPtr)(Width * 3));//copy one line

                    scan02 = IntPtr.Add(scan02, stride2);//advance pointers
                    scan0 = IntPtr.Add(scan0, stride);//advance pointers//
                }


                initial.UnlockBits(bmData);
                bmp2.UnlockBits(bmData2);
            }
        }*/

/*if (Xcount > scaleX ^ Ycount > scaleY)
{                       
//myBitmap = new Bitmap(myBitmap, new Size(myBitmap.Width / 4, myBitmap.Height / 4));
//myBitmap.SetResolution(2500, 2500);
pictureBox1.Image = myBitmap;
using (Graphics graph = Graphics.FromImage(myBitmap))
{
    DrawFilledRectangle(500, 500);
    var x = (500 - myBitmap.Width) / 2;  
    var y = (500 - myBitmap.Height) / 2;  
    graph.DrawImageUnscaled(myBitmap, x, y, myBitmap.Width, myBitmap.Height);
}
//Xcount = Xcount / 4;
//Ycount = Ycount / 4;
System.Threading.Thread.Sleep(5000);
}*/

