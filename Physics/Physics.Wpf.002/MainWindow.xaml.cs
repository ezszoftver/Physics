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

using OpenTK.Graphics.OpenGL;
using OpenTK;

using Physics;
using System.Windows.Threading;

namespace Physics.Wpf._002
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        DateTime currentTime = DateTime.Now;
        DateTime elapsedTime = DateTime.Now;

        Physics.Plane plane;
        Physics.RigidBody rigidBody1;
        Physics.RigidBody rigidBody2;

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
            rigidBody1 = new RigidBody();

            rigidBody1.m_fMass = 1.0f;
            rigidBody1.m_v3Position = new Vector3(0, 1.0f, 0);
            rigidBody1.m_fGravity = new Vector3(0, -1.0f, 0);
            rigidBody1.m_fRestitution = 0.5f;
            rigidBody1.m_v3Rotate = new Vector3(ToRadian(0.0f), 0, ToRadian(0.0f));

            rigidBody1.m_listPoints.Add(new Vector3(-2.0f, -1.0f, -1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(+2.0f, -1.0f, -1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(-2.0f, +1.0f, -1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(+2.0f, +1.0f, -1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(-2.0f, -1.0f, +1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(+2.0f, -1.0f, +1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(-2.0f, +1.0f, +1.5f));
            rigidBody1.m_listPoints.Add(new Vector3(+2.0f, +1.0f, +1.5f));

            // back
            rigidBody1.m_listIndices.AddRange(new int[] { 3, 1, 0 });
            rigidBody1.m_listIndices.AddRange(new int[] { 0, 2, 3 });
            // front
            rigidBody1.m_listIndices.AddRange(new int[] { 4, 5, 7 });
            rigidBody1.m_listIndices.AddRange(new int[] { 7, 6, 4 });
            // left
            rigidBody1.m_listIndices.AddRange(new int[] { 6, 2, 0 });
            rigidBody1.m_listIndices.AddRange(new int[] { 0, 4, 6 });
            // right
            rigidBody1.m_listIndices.AddRange(new int[] { 1, 3, 7 });
            rigidBody1.m_listIndices.AddRange(new int[] { 7, 5, 1 });
            // bottom
            rigidBody1.m_listIndices.AddRange(new int[] { 0, 1, 5 });
            rigidBody1.m_listIndices.AddRange(new int[] { 5, 4, 0 });
            // top
            rigidBody1.m_listIndices.AddRange(new int[] { 7, 3, 2 });
            rigidBody1.m_listIndices.AddRange(new int[] { 2, 6, 7 });

            rigidBody1.CalculateNormals();

            ;

            rigidBody2 = new RigidBody();

            rigidBody2.m_fMass = 1.0f;
            rigidBody2.m_v3Position = new Vector3(0, 6.0f, 0);
            rigidBody2.m_fGravity = new Vector3(0, -1.0f, 0);
            rigidBody2.m_fRestitution = 0.5f;
            rigidBody2.m_v3Rotate = new Vector3(ToRadian(30.0f), 0, ToRadian(20.0f));

            rigidBody2.m_listPoints.Add(new Vector3(-2.0f, -1.0f, -1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(+2.0f, -1.0f, -1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(-2.0f, +1.0f, -1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(+2.0f, +1.0f, -1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(-2.0f, -1.0f, +1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(+2.0f, -1.0f, +1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(-2.0f, +1.0f, +1.5f));
            rigidBody2.m_listPoints.Add(new Vector3(+2.0f, +1.0f, +1.5f));

            // back
            rigidBody2.m_listIndices.AddRange(new int[] { 3, 1, 0 });
            rigidBody2.m_listIndices.AddRange(new int[] { 0, 2, 3 });
            // front
            rigidBody2.m_listIndices.AddRange(new int[] { 4, 5, 7 });
            rigidBody2.m_listIndices.AddRange(new int[] { 7, 6, 4 });
            // left
            rigidBody2.m_listIndices.AddRange(new int[] { 6, 2, 0 });
            rigidBody2.m_listIndices.AddRange(new int[] { 0, 4, 6 });
            // right
            rigidBody2.m_listIndices.AddRange(new int[] { 1, 3, 7 });
            rigidBody2.m_listIndices.AddRange(new int[] { 7, 5, 1 });
            // bottom
            rigidBody2.m_listIndices.AddRange(new int[] { 0, 1, 5 });
            rigidBody2.m_listIndices.AddRange(new int[] { 5, 4, 0 });
            // top
            rigidBody2.m_listIndices.AddRange(new int[] { 7, 3, 2 });
            rigidBody2.m_listIndices.AddRange(new int[] { 2, 6, 7 });

            rigidBody2.CalculateNormals();

            DispatcherTimer timer = new DispatcherTimer();
            timer.Tick += Timer_Tick; ;
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
                rigidBody1.Update(step);
                rigidBody2.Update(step);

                Vector3 v3Separate = new Vector3();
                List<Hit> listHits = new List<Hit>();
                if (true == Physics.CollisionDetection.RigidBodyAndPlane(rigidBody1, plane, ref listHits, ref v3Separate))
                {
                    Physics.CollisionResponse.Apply(rigidBody1, listHits, v3Separate);
                }

                listHits.Clear();
                if (true == Physics.CollisionDetection.RigidBodyAndPlane(rigidBody2, plane, ref listHits, ref v3Separate))
                {
                    Physics.CollisionResponse.Apply(rigidBody2, listHits, v3Separate);
                }

                listHits.Clear();
                if (true == Physics.CollisionDetection.RigidBodyAndRigidBody(rigidBody1, rigidBody2, ref listHits, ref v3Separate)) 
                {
                    ;
                }
            }

            plane.Draw();
            rigidBody1.Draw();
            rigidBody2.Draw();

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
