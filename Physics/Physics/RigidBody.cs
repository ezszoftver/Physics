using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL;
using OpenTK;

namespace Physics
{
    public struct MinMax 
    {
        public float m_fMin;
        public float m_fMax;
    }

    public class RigidBody
    {
        public static float step = 0.1f; // 0.5 meter

        public List<Vector3> m_listPoints = new List<Vector3>();
        public List<int> m_listIndices = new List<int>();
        public List<Vector3> m_listTriangleNormals = new List<Vector3>();

        public List<Vector3> m_listPoints2        = new List<Vector3>();
        public List<Vector3> m_listPointsNormals2 = new List<Vector3>();

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

        public float m_fFriction = 1.0f;

        public float m_fLinearDamping = 1.0f;
        public float m_fAngularDamping = 1.0f;

        public float m_fDeltaTime = 0.0f;

        public Dictionary<int, MinMax> m_dictMinMaxs = new Dictionary<int, MinMax>();
        

        public static float ToRadian(float fDegree)
        {
            float fRadian = (fDegree / 180.0f) * (float)Math.PI;
            return fRadian;
        }

        OpenTK.Quaternion ToQuaternion(float roll, float pitch, float yaw) // yaw (Z), pitch (Y), roll (X)
        {
            // Abbreviations for the various angular functions
            float cy = (float)Math.Cos(yaw * 0.5f);
            float sy = (float)Math.Sin(yaw * 0.5f);
            float cp = (float)Math.Cos(pitch * 0.5f);
            float sp = (float)Math.Sin(pitch * 0.5f);
            float cr = (float)Math.Cos(roll * 0.5f);
            float sr = (float)Math.Sin(roll * 0.5f);

            OpenTK.Quaternion q = new Quaternion();
            q.W = cr * cp * cy + sr * sp * sy;
            q.X = sr * cp * cy - cr * sp * sy;
            q.Y = cr * sp * cy + sr * cp * sy;
            q.Z = cr * cp * sy - sr * sp * cy;

            return q;
        }

        public void Update(float dt)
        {
            m_fDeltaTime = dt;

            m_v3LinearAcceleration = m_fGravity + (m_v3Force / m_fMass);
            m_v3LinearVelocity += m_v3LinearAcceleration * m_fDeltaTime;
            m_v3Position += m_v3LinearVelocity * m_fDeltaTime;
            Matrix4 m4Translate = Matrix4.CreateTranslation(m_v3Position);

            m_v3AngularAcceleration = (m_v3Torque / m_fMass);
            m_v3AngularVelocity += m_v3AngularAcceleration * m_fDeltaTime;
            m_v3Rotate += m_v3AngularVelocity * m_fDeltaTime;

            // damping
            m_v3LinearVelocity *= (float)Math.Pow(m_fLinearDamping, m_fDeltaTime);
            m_v3AngularVelocity *= (float)Math.Pow(m_fAngularDamping, m_fDeltaTime);

            //Matrix4 m4RotX = Matrix4.CreateRotationX(m_v3Rotate.X);
            //Matrix4 m4RotY = Matrix4.CreateRotationY(m_v3Rotate.Y);
            //Matrix4 m4RotZ = Matrix4.CreateRotationZ(m_v3Rotate.Z);
            //Matrix4 m4RotXYZ = Matrix4.Mult(Matrix4.Mult(m4RotX, m4RotY), m4RotZ);
            OpenTK.Quaternion quat = ToQuaternion(m_v3Rotate.X, m_v3Rotate.Y, m_v3Rotate.Z);
            Matrix4 m4Rotate = Matrix4.CreateFromQuaternion(quat);

            m_m4World = Matrix4.Mult(m4Rotate, m4Translate);
        }

        public Vector3 GetPointVelocity(Vector3 v3Point)
        {
            return (m_v3LinearVelocity + Vector3.Cross(m_v3AngularVelocity, v3Point));
        }

        public void Create() 
        {
            if (0 == CollisionDetection.s_v3SATPlanes.Count) { CollisionDetection.GenerateSATPlanes(); }

            CalculateNormals();

            m_listPoints2.Clear();
            m_listPointsNormals2.Clear();

            int nIdN = 0;
            for (int id = 0; id < m_listIndices.Count(); id += 3, nIdN++)
            {
                Vector3 v3A = m_listPoints[m_listIndices[id + 0]];
                Vector3 v3B = m_listPoints[m_listIndices[id + 1]];
                Vector3 v3C = m_listPoints[m_listIndices[id + 2]];

                Vector3 v3Normal = m_listTriangleNormals[nIdN];

                // csucsok
                m_listPoints2.Add(v3A);
                m_listPointsNormals2.Add(v3Normal);

                m_listPoints2.Add(v3B);
                m_listPointsNormals2.Add(v3Normal);

                m_listPoints2.Add(v3C);
                m_listPointsNormals2.Add(v3Normal);
            }
        }

        private void CalculateNormals() 
        {
            m_listTriangleNormals.Clear();

            for (int id = 0; id < m_listIndices.Count; id += 3)
            {
                Vector3 v3A = m_listPoints[m_listIndices[id + 0]]; // A
                Vector3 v3B = m_listPoints[m_listIndices[id + 1]]; // B
                Vector3 v3C = m_listPoints[m_listIndices[id + 2]]; // C

                Vector3 v3N = Vector3.Cross(v3B - v3A, v3C - v3A).Normalized(); // N
                m_listTriangleNormals.Add(v3N);
            }
        }

        

        public void Draw() 
        {
            Vector3 v3LightDir = new Vector3(-0.5f, -1.0f, -0.5f).Normalized();

            // vertices
            GL.Begin(PrimitiveType.Triangles);
            {
                int nId = 0;
                for (int id = 0; id < m_listIndices.Count; id += 3, nId++) 
                {
                    // N
                    Vector3 v3N = m_listTriangleNormals[nId];
                    Vector3 v3NInWorld = Vector4.Transform(new Vector4(v3N, 0), m_m4World).Xyz;
                    v3NInWorld.Normalize();

                    float fIntensity = Math.Max(0.0f, Vector3.Dot(-v3LightDir, v3NInWorld));
                    GL.Color4(1.0f * fIntensity, 0.0f, 0.0f, 1.0f);

                    // A
                    Vector3 v3PA = m_listPoints[ m_listIndices[id + 0] ];
                    Vector3 v3PAInWorld = Vector4.Transform(new Vector4(v3PA, 1), m_m4World).Xyz;
                    GL.Vertex3(v3PAInWorld);

                    // B
                    Vector3 v3PB = m_listPoints[ m_listIndices[id + 1] ];
                    Vector3 v3PBInWorld = Vector4.Transform(new Vector4(v3PB, 1), m_m4World).Xyz;
                    GL.Vertex3(v3PBInWorld);

                    // C
                    Vector3 v3PC = m_listPoints[ m_listIndices[id + 2] ];
                    Vector3 v3PCInWorld = Vector4.Transform(new Vector4(v3PC, 1), m_m4World).Xyz;
                    GL.Vertex3(v3PCInWorld);
                }
            }
            GL.End();

            // normals
            //GL.Begin(PrimitiveType.Lines);
            //{
            //    GL.Color3(0.0f, 1.0f, 0.0f);
            //
            //    int nId = 0;
            //    for (int id = 0; id < m_listIndices.Count; id += 3, nId++)
            //    {
            //        // A
            //        Vector3 v3PA = m_listPoints[m_listIndices[id + 0]];
            //        Vector3 v3PAInWorld = Vector4.Transform(new Vector4(v3PA, 1), m_m4World).Xyz;
            //
            //        // B
            //        Vector3 v3PB = m_listPoints[m_listIndices[id + 1]];
            //        Vector3 v3PBInWorld = Vector4.Transform(new Vector4(v3PB, 1), m_m4World).Xyz;
            //        
            //        // C
            //        Vector3 v3PC = m_listPoints[m_listIndices[id + 2]];
            //        Vector3 v3PCInWorld = Vector4.Transform(new Vector4(v3PC, 1), m_m4World).Xyz;
            //
            //        // N
            //        Vector3 v3N = m_listTriangleNormals[nId];
            //        Vector3 v3NInWorld = Vector4.Transform(new Vector4(v3N, 0), m_m4World).Xyz;
            //
            //        Vector3 v3NStart = (v3PAInWorld + v3PBInWorld + v3PCInWorld) / 3.0f;
            //        Vector3 v3NEnd = v3NStart + (v3NInWorld * 1.0f);
            //
            //        GL.Vertex3(v3NStart);
            //        GL.Vertex3(v3NEnd);
            //    }
            //
            //}
            //GL.End();
        }
    }
}
