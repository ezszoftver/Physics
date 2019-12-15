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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

using OpenTK.Graphics.OpenGL;
using OpenTK;

using Physics;

namespace Physics.Wpf._001
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        DateTime currentTime = DateTime.Now;
        DateTime elapsedTime = DateTime.Now;

        Physics.Plane plane;
        Physics.RigidBody rigidBody;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void glControl_Load(object sender, EventArgs e)
        {
            glControl.MakeCurrent();
            GL.Disable(EnableCap.Lighting);
            GL.Disable(EnableCap.Texture2D);
            GL.ClearColor(0.5f, 0.5f, 1.0f, 1f);

            plane = new Physics.Plane(new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            rigidBody = new RigidBody();

            rigidBody.m_v3Position = new Vector3(0, 10.0f, 0);
            rigidBody.m_v3Force = new Vector3(0, -1.0f, 0);

            rigidBody.m_listPoints.Add(new Vector3(-1, -1, -1));
            rigidBody.m_listPoints.Add(new Vector3(+1, -1, -1));
            rigidBody.m_listPoints.Add(new Vector3(-1, +1, -1));
            rigidBody.m_listPoints.Add(new Vector3(+1, +1, -1));
            rigidBody.m_listPoints.Add(new Vector3(-1, +1, +1));
            rigidBody.m_listPoints.Add(new Vector3(+1, +1, +1));
            rigidBody.m_listPoints.Add(new Vector3(-1, -1, +1));
            rigidBody.m_listPoints.Add(new Vector3(+1, -1, +1));



            DispatcherTimer timer = new DispatcherTimer();
            timer.Tick += Timer_Tick;
            timer.Interval = TimeSpan.FromMilliseconds(0);
            timer.Start();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            glControl.MakeCurrent();

            // update
            elapsedTime = currentTime;
            currentTime = DateTime.Now;
            float dt = (float)(currentTime - elapsedTime).TotalSeconds;

            // draw
            GL.Viewport(0, 0, glControl.Width, glControl.Height);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            Matrix4 m_world = Matrix4.Identity;
            Matrix4 m_view = Matrix4.LookAt(new Vector3(0, 10, 10), new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            Matrix4 m_modelview = Matrix4.Mult(m_world, m_view);
            Matrix4 m_proj = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 2.0f, (float)glControl.Width / (float)glControl.Height, 0.05f, 1000.0f);

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref m_proj);

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref m_modelview);

            rigidBody.Update(dt);
            List<Hit> listHits = new List<Hit>();
            if (true == rigidBody.CollisionDetection(plane, listHits)) 
            {
                rigidBody.CollisionResponse(listHits);
            }

            plane.Draw();
            rigidBody.Draw();

            glControl.SwapBuffers();
        }

    }
}
