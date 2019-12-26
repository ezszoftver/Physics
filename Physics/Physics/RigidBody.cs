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
        public float m_fLinearDamping = 0.0f;
        public Vector3 m_v3Position = new Vector3();
        
        public Vector3 m_v3Torque = new Vector3();
        public Vector3 m_v3AngularAcceleration = new Vector3();
        public Vector3 m_v3AngularVelocity = new Vector3();
        public float m_fAngularDamping = 0.0f;
        public Quaternion m_qOrientation = new Quaternion(new Vector3(0,0,0));

        public Matrix4 m_m4World = Matrix4.Identity;

        public float m_fDeltaTime = 0.0f;

        float ToRadian(float fDegree) 
        {
            float fRadian = (fDegree / 180.0f) * (float)Math.PI;
            return fRadian;
        }

        public void Update(float dt)
        {
            m_fDeltaTime = dt;

            m_v3LinearAcceleration = m_fGravity + (m_v3Force / m_fMass);
            m_v3LinearVelocity += m_v3LinearAcceleration * m_fDeltaTime;
            // damping
            if (m_v3LinearVelocity.Length > 0.001f)
            {
                m_v3LinearVelocity -= m_v3LinearVelocity.Normalized() * (1.0f - m_fLinearDamping) * m_fDeltaTime;
            }
            m_v3Position += m_v3LinearVelocity * m_fDeltaTime;

            m_v3AngularAcceleration = m_v3Torque / m_fMass;
            m_v3AngularVelocity += m_v3AngularAcceleration * m_fDeltaTime;
            // damping
            if (m_v3AngularVelocity.Length > ToRadian(1.0f)) 
            {
                m_v3AngularVelocity -= m_v3AngularVelocity.Normalized() * (1.0f - m_fAngularDamping) * m_fDeltaTime;
            }

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
                if (t <= 0.0f) 
                {
                    Hit hit = new Hit();

                    hit.m_v3PositionInWorld = v3PointInWorld;
                    hit.m_v3Normal = plane.m_v3Normal;
                    hit.m_fRestitution = m_fRestitution;
                    hit.t = Math.Abs(t);

                    listHits.Add(hit);
                }
            }

            return (listHits.Count() > 0);
        }

        Matrix3 InvInertia() 
        {
            Matrix3 mat4InvInertia = Matrix3.CreateFromQuaternion(m_qOrientation);
            mat4InvInertia.Invert();

            return mat4InvInertia;
        }

        public void CollisionResponse(Plane plane, List<Hit> listHits)
        {
            foreach (Hit hit in listHits) 
            {
                Vector3 rA = hit.m_v3PositionInWorld - m_v3Position;
                rA = Vector3.Transform(rA, InvInertia());

                Vector3 v3Velocity = GetPointVelocity(rA);

                float nominator = -(1.0f + hit.m_fRestitution) * Vector3.Dot(v3Velocity, hit.m_v3Normal);
                float denominator = (1.0f / m_fMass) + Vector3.Dot(Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA), hit.m_v3Normal);
                float J = nominator / denominator;

                m_v3LinearVelocity += (J / m_fMass) * hit.m_v3Normal;
                m_v3AngularVelocity += Vector3.Cross(rA, hit.m_v3Normal * J);
            }

            // separate
            float t = float.MinValue;
            foreach (Hit hit in listHits) { if (t < hit.t) { t = hit.t; } }
            m_v3Position += plane.m_v3Normal * t;
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
