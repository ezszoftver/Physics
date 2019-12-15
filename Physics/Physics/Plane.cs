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
            ;
        }
    }
}
