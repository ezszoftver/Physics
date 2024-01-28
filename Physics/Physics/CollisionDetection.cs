using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL;
using OpenTK;
using System.Threading;

namespace Physics
{
    public class CollisionDetection
    {
        public static float step = ((float)Math.PI * 2.0f) / 32.0f;
        public static List<Plane> s_v3SATPlanes = new List<Plane>();

        public static void GenerateSATPlanes()
        {
            s_v3SATPlanes.Clear();

            for (float yaw = 0.0f; yaw < (float)(Math.PI * 2.0); yaw += step)
            {
                for (float pitch = (float)(-Math.PI / 2.0); pitch < (float)(Math.PI / 2.0); pitch += step)
                {
                    Matrix4 mat4Rot = Matrix4.CreateRotationY(yaw) * Matrix4.CreateRotationX(pitch);
                    Vector3 v3Normal = Vector4.Transform(new Vector4(new Vector3(1.0f, 0.0f, 0.0f), 0), mat4Rot).Xyz;
                    v3Normal.Normalize();

                    Plane plane = new Plane(/*v3PointInWorld*/new Vector3(0.0f, 0.0f, 0.0f), v3Normal);

                    s_v3SATPlanes.Add(plane);
                }
            }
        }

        public static bool RigidBodyAndPlane(RigidBody rigidBody, Plane plane, ref List<Hit> listHits2)
        {
            List<Hit> listHits = new List<Hit>();

            Parallel.For(0, rigidBody.m_listPoints2.Count, i => 
            {
                Vector3 v3Point = rigidBody.m_listPoints2[i];
                Vector3 v3Normal = rigidBody.m_listPointsNormals2[i];

                Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), rigidBody.m_m4World).Xyz;
                float dist = plane.GetDistance(v3PointInWorld);
                if (dist < 0.0f)
                {
                    Hit hit = new Hit();
                    hit.m_v3Normal = plane.m_v3Normal;
                    hit.m_v3PositionInWorld = v3PointInWorld;
                    hit.m_fPenetration = Math.Abs(dist);

                    lock (listHits) 
                    {
                        listHits.Add(hit);
                    }
                }
            });

            listHits2.Clear();
            listHits2.AddRange(listHits);

            return (listHits2.Count() > 0);
        }

        public static bool RigidBodyAndRigidBody(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Hit> listHits) 
        {
            // points in other body
            listHits = new List<Hit>();

            Plane separatePlane = null;
            if (false == SAT(rigidBody1, rigidBody2, ref separatePlane))
            {
                return false;
            }

            ;

            return (listHits.Count() > 0);
        }

        private static void GetLocalMinMax(RigidBody rigidBody, Plane plane, out float fMin, out float fMax)
        {
            fMin = float.MaxValue;
            fMax = float.MinValue;

            for (int i = 0; i < rigidBody.m_listIndices.Count; i++)
            {
                Vector3 v3Point = rigidBody.m_listPoints[rigidBody.m_listIndices[i]];

                float t = plane.GetDistance(v3Point);

                if (t < fMin) { fMin = t; }
                if (fMax < t) { fMax = t; }
            }
        }

        private static bool SAT(RigidBody rigidBody1, RigidBody rigidBody2, ref Plane bestPlane) 
        {
            float min_t = float.MaxValue;
            bestPlane = null;

            for (int i = 0; i < s_v3SATPlanes.Count; i++)
            {
                Plane planeWorld = s_v3SATPlanes[i];

                // RigidBody1
                float fMin1 = 0.0f;
                float fMax1 = 0.0f;
                {
                    Vector3 v3NormalLocal = Vector4.Transform(new Vector4(planeWorld.m_v3Normal, 0), rigidBody1.m_m4World.Inverted()).Xyz;
                    Plane planeLocal = new Plane(new Vector3(0.0f, 0.0f, 0.0f), v3NormalLocal);

                    MinMax localMinMax;
                    GetLocalMinMax(rigidBody1, planeLocal, out localMinMax.m_fMin, out localMinMax.m_fMax);

                    float fCenter = planeWorld.GetDistance(rigidBody1.m_v3Position);
                    fMin1 = fCenter + localMinMax.m_fMin;
                    fMax1 = fCenter + localMinMax.m_fMax;
                }

                // RigidBody2
                float fMin2 = 0.0f;
                float fMax2 = 0.0f;
                {
                    Vector3 v3NormalLocal = Vector4.Transform(new Vector4(planeWorld.m_v3Normal, 0), rigidBody2.m_m4World.Inverted()).Xyz;
                    Plane planeLocal = new Plane(new Vector3(0.0f, 0.0f, 0.0f), v3NormalLocal);

                    MinMax localMinMax;
                    GetLocalMinMax(rigidBody2, planeLocal, out localMinMax.m_fMin, out localMinMax.m_fMax);

                    float fCenter = planeWorld.GetDistance(rigidBody2.m_v3Position);
                    fMin2 = fCenter + localMinMax.m_fMin;
                    fMax2 = fCenter + localMinMax.m_fMax;
                }

                float fFullMin = Math.Min(fMin1, fMin2);
                float fFullMax = Math.Max(fMax1, fMax2);
                float fFullDistance = fFullMax - fFullMin;

                float fDistance1 = fMax1 - fMin1;
                float fDistance2 = fMax2 - fMin2;

                if (fFullDistance > (fDistance1 + fDistance2))
                {
                    return false;
                }

                float fPenetration = (fDistance1 + fDistance2) - fFullDistance;

                if (fPenetration < min_t)
                {
                    bestPlane = planeWorld;
                }
            }

            return true;
        }

        public static void DrawHits(List<Hit> listHits) 
        {
            GL.PointSize(4);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(1.0f, 1.0f, 1.0f);
            foreach (Hit hit in listHits) 
            {
                GL.Vertex3(hit.m_v3PositionInWorld);
            }
            GL.End();
            GL.PointSize(1);

            GL.LineWidth(2);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(1.0f, 1.0f, 1.0f);
            foreach (Hit hit in listHits)
            {
                GL.Vertex3(hit.m_v3PositionInWorld);
                GL.Vertex3(hit.m_v3PositionInWorld + (hit.m_v3Normal * 1.0f));
            }
            GL.End();
            GL.LineWidth(1);
        }

        private static float ToRadian(float fDegree)
        {
            return (fDegree / 180.0f * (float)Math.PI);
        }
    }
}
