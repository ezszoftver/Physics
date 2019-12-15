using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK;

namespace Physics
{
    public class Hit
    {
        public Vector3 m_v3Position = new Vector3();
        public Vector3 m_v3Normal   = new Vector3();
        public float m_fRestitution = 0.0f;
    }
}
