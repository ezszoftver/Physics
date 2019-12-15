using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL;
using OpenTK;

namespace Physics
{
    public class RigidBody
    {
        public List<Vector3> m_listPoints = new List<Vector3>();
        
        public float m_fMass = 1.0f;
        
        public Vector3 m_v3Force = new Vector3();
        public Vector3 m_v3LinearAcceleration = new Vector3();
        public Vector3 m_v3LinearVelocity = new Vector3();
        public Vector3 m_v3Position = new Vector3();
        
        public Vector3 m_v3Torque = new Vector3();
        public Vector3 m_v3AngularAcceleration = new Vector3();
        public Vector3 m_v3AngularVelocity = new Vector3();
        public Quaternion m_qOrientation = new Quaternion(0,1,0, 0);

        public Matrix4 m_m4World = Matrix4.Identity;

        public float m_fDeltaTime = 0.0f;

        Matrix4 InvInertia() 
        {
            return Matrix4.Identity;
        }

        public void Update(float dt)
        {
            m_fDeltaTime = dt;

            m_v3LinearAcceleration += (m_v3Force / m_fMass) * m_fDeltaTime;
            m_v3LinearVelocity += m_v3LinearAcceleration * m_fDeltaTime;
            m_v3Position += m_v3LinearVelocity * m_fDeltaTime;

            m_v3AngularAcceleration += (m_v3Torque / m_fMass) * m_fDeltaTime;
            m_v3AngularVelocity += m_v3AngularAcceleration * m_fDeltaTime;
            m_qOrientation += Quaternion.Multiply(m_qOrientation, new Quaternion(m_v3AngularVelocity * (m_fDeltaTime / 2), 0));
            m_qOrientation.Normalize();

            m_m4World = Matrix4.Mult(Matrix4.CreateFromQuaternion(m_qOrientation), Matrix4.CreateTranslation(m_v3Position));
        }

        public bool CollisionDetection(Plane plane, List<Hit> listHits)
        {
            listHits.Clear();

            foreach (Vector3 v3Point in m_listPoints) 
            {
                Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), m_m4World).Xyz;
                if (plane.GetDistance(v3PointInWorld) <= 0.0f) 
                {
                    Hit hit = new Hit();

                    hit.m_v3PositionInLocal = v3Point;
                    hit.m_v3PositionInWorld = v3PointInWorld;
                    hit.m_v3Normal = plane.m_v3Normal;
                    hit.m_fRestitution = 0.5f;

                    listHits.Add(hit);
                }
            }

            return (listHits.Count() > 0);
        }

        public void CollisionResponse(List<Hit> listHits)
        {
            foreach (Hit hit in listHits) 
            {
                Vector3 v3Velocity = GetPointVelocity(hit.m_v3PositionInLocal);
                float speed = -Vector3.Dot(hit.m_v3Normal, v3Velocity);

                Vector3 rA = hit.m_v3PositionInWorld - m_v3Position;

                ;
            }

            ;
        }

        public Vector3 GetPointVelocity(Vector3 v3Point)
        {
            return (m_v3LinearVelocity + Vector3.Cross(m_v3AngularVelocity, v3Point));
        }

        public void Draw() 
        {
            GL.PointSize(5.0f);
            GL.Begin(PrimitiveType.Points);
            {
                GL.Color3(1.0f, 0.0f, 0.0f);
                foreach (Vector3 v3Point in m_listPoints) 
                {
                    Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), m_m4World).Xyz;
                    GL.Vertex3(v3PointInWorld);
                }
            }
            GL.End();
            GL.PointSize(1.0f);
        }
    }
}
