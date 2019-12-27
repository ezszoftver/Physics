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
        public Vector3 m_v3Rotate = new Vector3();

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
            Matrix4 m4Translate = Matrix4.CreateTranslation(m_v3Position);

            m_v3AngularAcceleration = m_v3Torque / m_fMass;
            m_v3AngularVelocity += m_v3AngularAcceleration * m_fDeltaTime;
            m_v3Rotate += m_v3AngularVelocity * m_fDeltaTime;
            Matrix4 m4RotX = Matrix4.CreateRotationX(m_v3Rotate.X);
            Matrix4 m4RotY = Matrix4.CreateRotationY(m_v3Rotate.Y);
            Matrix4 m4RotZ = Matrix4.CreateRotationZ(m_v3Rotate.Z);
            Matrix4 m4RotXYZ = Matrix4.Mult(Matrix4.Mult(m4RotX, m4RotY), m4RotZ);

            m_m4World = Matrix4.Mult(m4RotXYZ, m4Translate);
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
                    hit.t = Math.Abs(t);
                    hit.m_v3PositionInWorld = v3PointInWorld + (plane.m_v3Normal * Math.Abs(t));

                    listHits.Add(hit);
                }
            }

            return (listHits.Count() > 0);
        }

        public void CollisionResponse(Plane plane, List<Hit> listHits)
        {
            foreach (Hit hit in listHits)
            {
                Vector3 rA = hit.m_v3PositionInWorld - m_v3Position;
                Vector3 v3Velocity = GetPointVelocity(rA);

                float fRelVelocity = Vector3.Dot(v3Velocity, hit.m_v3Normal);

                float nominator = -(1.0f + m_fRestitution) * fRelVelocity;
                float term1 = 1.0f / m_fMass;
                float term2 = 0.0f;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = 0.0f;
                float J = nominator / (term1 + term2 + term3 + term4);

                m_v3LinearVelocity += (J / m_fMass) * hit.m_v3Normal;
                m_v3AngularVelocity += (J * Vector3.Cross(rA, hit.m_v3Normal)) / m_fMass;
            }

            // separate
            Hit hitMaxT = new Hit();
            foreach (Hit currentHit in listHits) 
            {
                if (currentHit.t > hitMaxT.t) { hitMaxT = currentHit; }
            }
            m_v3Position += hitMaxT.m_v3Normal * hitMaxT.t;
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
