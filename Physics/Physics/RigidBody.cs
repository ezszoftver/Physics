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
        public List<int> m_listIndices = new List<int>();

        public Vector3 m_fGravity = new Vector3();
        public float m_fMass = 1.0f;
        public float m_fRestitution = 0.0f;

        public Vector3 m_v3Force = new Vector3();
        public Vector3 m_v3LinearAcceleration = new Vector3();
        public Vector3 m_v3LinearVelocity = new Vector3();
        public Vector3 m_v3Position = new Vector3();
        
        public Vector3 m_v3Torque = new Vector3();
        public Vector3 m_v3AngularAcceleration = new Vector3();
        public Vector3 m_v3AngularVelocity = new Vector3();
        public Quaternion m_qOrientation = new Quaternion(new Vector3(0,0,0));

        public Matrix4 m_m4World = Matrix4.Identity;

        public float m_fDeltaTime = 0.0f;

        public static float ToRadian(float fDegree)
        {
            float fRadian = (fDegree / 180.0f) * (float)Math.PI;
            return fRadian;
        }

        public void Update(float dt)
        {
            m_fDeltaTime = dt;

            m_v3LinearAcceleration = m_fGravity + (m_v3Force / m_fMass);
            m_v3LinearVelocity += m_v3LinearAcceleration * m_fDeltaTime;
            m_v3Position += m_v3LinearVelocity * m_fDeltaTime;

            m_v3AngularAcceleration = m_v3Torque / m_fMass;
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
                float t = plane.GetDistance(v3PointInWorld);
                if (t < 0.0f) 
                {
                    Hit hit = new Hit();

                    hit.m_v3Normal = plane.m_v3Normal;
                    hit.m_fRestitution = m_fRestitution;
                    hit.t = Math.Abs(t);
                    hit.m_v3PositionInWorld = v3PointInWorld + (plane.m_v3Normal * Math.Abs(t));

                    listHits.Add(hit);
                }
            }

            return (listHits.Count() > 0);
        }

        public void CollisionResponse(Plane plane, List<Hit> listHits)
        {
            Hit hit = new Hit();
            hit.t = float.MinValue;
            foreach (Hit hit2 in listHits) 
            {
                hit.m_fRestitution = hit2.m_fRestitution;
                hit.m_v3Normal = hit2.m_v3Normal;
                hit.m_v3PositionInWorld += hit2.m_v3PositionInWorld;

                if (hit2.t > hit.t) { hit.t = hit2.t; }
            }

            hit.m_v3PositionInWorld /= (float)listHits.Count();

            {
                Vector3 rA = hit.m_v3PositionInWorld - m_v3Position;
                Vector3 v3Velocity = GetPointVelocity(rA);

                float fRelVelocity = Vector3.Dot(v3Velocity, hit.m_v3Normal);

                float nominator = -(1.0f + hit.m_fRestitution) * fRelVelocity;
                float term1 = 1.0f / m_fMass;
                float term2 = 0.001f;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = 0.0f;
                float J = nominator / (term1 + term2 + term3 + term4);

                m_v3LinearVelocity += (J / m_fMass) * hit.m_v3Normal;
                m_v3AngularVelocity = J * Vector3.Cross(rA, hit.m_v3Normal);
            }

            // separate
            m_v3Position += hit.m_v3Normal * hit.t;
        }

        public Vector3 GetPointVelocity(Vector3 v3Point)
        {
            return (m_v3LinearVelocity + Vector3.Cross(m_v3AngularVelocity, v3Point));
        }

        public void Draw() 
        {
            GL.Begin(PrimitiveType.Triangles);
            {
                GL.Color3(1.0f, 0.0f, 0.0f);
                foreach (int id in m_listIndices) 
                {
                    Vector3 v3Point = m_listPoints[id];
                    Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), m_m4World).Xyz;
                    GL.Vertex3(v3PointInWorld);
                }
            }
            GL.End();
        }
    }
}
