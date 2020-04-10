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

        float ToRadian(float fDegree) 
        {
            return (fDegree / 180.0f * (float)Math.PI);
        }

        private void glControl_Load(object sender, EventArgs e)
        {
            glControl.MakeCurrent();
            GL.Disable(EnableCap.Lighting);
            GL.Disable(EnableCap.Texture2D);
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
            GL.ClearColor(0.5f, 0.5f, 1.0f, 1.0f);

            plane = new Physics.Plane(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0, 1, 0));
            rigidBody = new RigidBody();

            rigidBody.m_fMass = 1.0f;
            rigidBody.m_v3Position = new Vector3(0, 10.0f, 0);
            rigidBody.m_fGravity = new Vector3(0, -9.81f, 0);
            rigidBody.m_fRestitution = 0.5f;
            rigidBody.m_v3Rotate = new Vector3(ToRadian(30.0f), 0, ToRadian(20.0f));

            rigidBody.m_listPoints.Add(new Vector3(-2.0f, -1.0f, -1.5f));
            rigidBody.m_listPoints.Add(new Vector3(+2.0f, -1.0f, -1.5f));
            rigidBody.m_listPoints.Add(new Vector3(-2.0f, +1.0f, -1.5f));
            rigidBody.m_listPoints.Add(new Vector3(+2.0f, +1.0f, -1.5f));
            rigidBody.m_listPoints.Add(new Vector3(-2.0f, -1.0f, +1.5f));
            rigidBody.m_listPoints.Add(new Vector3(+2.0f, -1.0f, +1.5f));
            rigidBody.m_listPoints.Add(new Vector3(-2.0f, +1.0f, +1.5f));
            rigidBody.m_listPoints.Add(new Vector3(+2.0f, +1.0f, +1.5f));

            // back
            rigidBody.m_listIndices.AddRange(new int[] { 0, 1, 3 });
            rigidBody.m_listIndices.AddRange(new int[] { 0, 2, 3 });
            // front
            rigidBody.m_listIndices.AddRange(new int[] { 4, 5, 7 });
            rigidBody.m_listIndices.AddRange(new int[] { 4, 6, 7 });
            // left
            rigidBody.m_listIndices.AddRange(new int[] { 0, 2, 6 });
            rigidBody.m_listIndices.AddRange(new int[] { 0, 4, 6 });
            // right
            rigidBody.m_listIndices.AddRange(new int[] { 1, 3, 7 });
            rigidBody.m_listIndices.AddRange(new int[] { 1, 5, 7 });
            // bottom
            rigidBody.m_listIndices.AddRange(new int[] { 0, 1, 5 });
            rigidBody.m_listIndices.AddRange(new int[] { 0, 4, 5 });
            // top
            rigidBody.m_listIndices.AddRange(new int[] { 2, 3, 7 });
            rigidBody.m_listIndices.AddRange(new int[] { 2, 6, 7 });

            DispatcherTimer timer = new DispatcherTimer();
            timer.Tick += Timer_Tick;
            timer.Interval = TimeSpan.FromSeconds(0);
            timer.Start();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            // update
            elapsedTime = currentTime;
            currentTime = DateTime.Now;
            float dt = (float)(currentTime - elapsedTime).TotalSeconds;
            if (dt < 0.0f) { dt = 0.0f; }
            if (dt > (1.0f / 10.0f)) { dt = (1.0f / 10.0f); } // min. 10fps

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            Matrix4 m_world = Matrix4.Identity;
            Matrix4 m_view = Matrix4.LookAt(new Vector3(0, 2, 10), new Vector3(0, 2, 0), new Vector3(0, 1, 0));
            Matrix4 m_modelview = Matrix4.Mult(m_world, m_view);
            Matrix4 m_proj = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 2.0f, (float)glControl.Width / (float)glControl.Height, 0.05f, 1000.0f);

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref m_proj);

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref m_modelview);

            int steps = 10;
            float step = dt / (float)steps;

            for (int i = 0; i < steps; i++) 
            {
                rigidBody.Update(step);
                List<Hit> listHits = new List<Hit>();
                if (true == Physics.CollisionDetection.RigidBodyAndPlane(rigidBody, plane, listHits))
                {
                    Physics.CollisionResponse.Apply(rigidBody, listHits);
                }
            }

            plane.Draw();
            rigidBody.Draw();

            glControl.SwapBuffers();
        }

        private void glControl_Resize(object sender, EventArgs e)
        {
            int iWidth = ((GLControl)sender).Width;
            int iHeight = ((GLControl)sender).Height;

            GL.Viewport(0, 0, iWidth, iHeight);
        }
    }
}
