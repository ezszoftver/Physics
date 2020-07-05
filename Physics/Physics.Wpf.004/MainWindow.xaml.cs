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

namespace Physics.Wpf._004
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

        OBJLoader loader = new OBJLoader();

        float ToRadian(float fDegree)
        {
            return (fDegree / 180.0f * (float)Math.PI);
        }

        public MainWindow()
        {
            InitializeComponent();
        }

        private void glControl_Load(object sender, EventArgs e)
        {
            glControl.MakeCurrent();
            GL.Enable(EnableCap.DepthTest);
            GL.Disable(EnableCap.Lighting);
            GL.Disable(EnableCap.Texture2D);
            GL.Disable(EnableCap.CullFace);
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            GL.ClearColor(0.5f, 0.5f, 1.0f, 1.0f);

            loader.LoadFromFile("", "henger.obj");

            plane = new Physics.Plane(new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0, 1, 0));
            rigidBody1 = new RigidBody();

            Vector3 v3Gravity = new Vector3(0, -2.0f, 0);

            rigidBody1.m_fMass = 1.0f;
            rigidBody1.m_v3Position = new Vector3(0f, 5.0f, 0);
            rigidBody1.m_fGravity = v3Gravity;
            rigidBody1.m_fRestitution = 0.1f;
            rigidBody1.m_v3Rotate = new Vector3(ToRadian(0.0f), 0, ToRadian(60.0f));
            rigidBody1.m_fLinearDamping = 0.1f;
            rigidBody1.m_fAngularDamping = 0.1f;

            float scale = 0.1f;
            foreach (Vector3 v in loader.vertices) 
            {
                rigidBody1.m_listPoints.Add(new Vector3(v.X * scale, v.Y * scale, v.Z * scale));
            }

            foreach (OBJLoader.Material mat in loader.materials) 
            {
                for (int i = 0; i < mat.indices.Count; i += 3) 
                {
                    int i0 = mat.indices[i + 0].id_vertex;
                    int i1 = mat.indices[i + 1].id_vertex;
                    int i2 = mat.indices[i + 2].id_vertex;

                    rigidBody1.m_listIndices.AddRange(new int[] { i0, i1, i2 });
                }
            }
            
            rigidBody1.Create();

            rigidBody2 = new RigidBody();

            rigidBody2.m_fMass = 1.0f;
            rigidBody2.m_v3Position = new Vector3(2f, 10.0f, 2);
            rigidBody2.m_fGravity = v3Gravity;
            rigidBody2.m_fRestitution = 0.1f;
            rigidBody2.m_v3Rotate = new Vector3(ToRadian(0.0f), 0, ToRadian(-15.0f));
            rigidBody2.m_fLinearDamping = 0.1f;
            rigidBody2.m_fAngularDamping = 0.1f;

            foreach (Vector3 v in loader.vertices)
            {
                rigidBody2.m_listPoints.Add(new Vector3(v.X * scale, v.Y * scale, v.Z * scale));
            }

            foreach (OBJLoader.Material mat in loader.materials)
            {
                for (int i = 0; i < mat.indices.Count; i += 3)
                {
                    int i0 = mat.indices[i + 0].id_vertex;
                    int i1 = mat.indices[i + 1].id_vertex;
                    int i2 = mat.indices[i + 2].id_vertex;

                    rigidBody2.m_listIndices.AddRange(new int[] { i0, i1, i2 });
                }
            }

            rigidBody2.Create();

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
            Matrix4 m_view = Matrix4.LookAt(new Vector3(2, 6, 15), new Vector3(0, 2, 0), new Vector3(0, 1, 0));
            Matrix4 m_modelview = Matrix4.Mult(m_world, m_view);
            Matrix4 m_proj = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 2.0f, (float)glControl.Width / (float)glControl.Height, 0.05f, 1000.0f);

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref m_proj);

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref m_modelview);

            int steps = 50;
            float step = dt / (float)steps;

            for (int i = 0; i < steps; i++)
            {
                rigidBody1.Update(step);
                rigidBody2.Update(step);

                // collision detection and response
                List<Hit> listHits1 = new List<Hit>();
                Physics.CollisionDetection.RigidBodyAndPlane(rigidBody1, plane, ref listHits1);
                Physics.CollisionResponse.Apply(rigidBody1, listHits1, step);

                List<Hit> listHits2 = new List<Hit>();
                Physics.CollisionDetection.RigidBodyAndPlane(rigidBody2, plane, ref listHits2);
                Physics.CollisionResponse.Apply(rigidBody2, listHits2, step);

                List<Hit> listHits3 = new List<Hit>();
                Physics.CollisionDetection.RigidBodyAndRigidBody(rigidBody1, rigidBody2, ref listHits3);
                Physics.CollisionResponse.Apply(rigidBody2, rigidBody1, listHits3, step);

                // draw hits
                //Physics.CollisionDetection.DrawHits(listHits1);
                //Physics.CollisionDetection.DrawHits(listHits2);
                //Physics.CollisionDetection.DrawHits(listHits3);
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
