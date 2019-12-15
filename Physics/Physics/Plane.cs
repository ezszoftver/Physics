using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL;
using OpenTK;

namespace Physics
{
    public class Plane
    {
        public Vector3 m_v3Position = new Vector3();
        public Vector3 m_v3Normal   = new Vector3();

        public Plane(Vector3 v3Position, Vector3 v3Normal) 
        {
            m_v3Position = new Vector3(v3Position);
            m_v3Normal = new Vector3(v3Normal);
        }

        public float GetDistance(Vector3 v3Point) 
        {
            float t = Vector3.Dot(m_v3Normal, v3Point - m_v3Position);
            return t;
        }

        public void Draw() 
        {
            GL.PointSize(5.0f);
            GL.Begin(PrimitiveType.Points);
            {
                GL.Color3(0.0f, 1.0f, 0.0f);
                GL.Vertex3(m_v3Position);
            }
            GL.End();
            GL.PointSize(1.0f);

            GL.Begin(PrimitiveType.Lines);
            {
                GL.Color3(0.0f, 1.0f, 0.0f);
                GL.Vertex3(m_v3Position);
                GL.Vertex3(m_v3Position + m_v3Normal);
            }
            GL.End();           
        }
    }
}
