using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL;
using OpenTK;

namespace Physics
{
    public class CollisionDetection
    {
        public static int step = 4;
        public static float margin = 0.01f;

        public static bool RigidBodyAndPlane(RigidBody rigidBody, Plane plane, ref List<Hit> listHits, ref Vector3 v3Separate)
        {
            listHits.Clear();

            for (int id = 0; id < (rigidBody.m_listIndices.Count() - 1); id += 2)
            {
                Vector3 v3Start = rigidBody.m_listPoints[rigidBody.m_listIndices[id + 0]];
                Vector3 v3End = rigidBody.m_listPoints[rigidBody.m_listIndices[id + 1]];

                for (int i = 0; i <= step; i++)
                {
                    float t = (float)i / (float)step;
                    Vector3 v3Point = ((1.0f - t) * v3Start) + (t * v3End);
                    {
                        Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), rigidBody.m_m4World).Xyz;
                        float dist = plane.GetDistance(v3PointInWorld);
                        if (dist <= 0.0f)
                        {
                            Hit hit = new Hit();

                            hit.m_v3Normal = plane.m_v3Normal;
							hit.t = Math.Abs(dist);
                            hit.m_v3PositionInWorld = v3PointInWorld - (hit.m_v3Normal * hit.t);

                            listHits.Add(hit);
                        }
                    }
                }
            }

            // separate
            if (listHits.Count() > 0)
            {
				Hit hitMaxT = new Hit();
                foreach (Hit currentHit in listHits)
                {
                    if (currentHit.t > hitMaxT.t) { hitMaxT = currentHit; }
                }
                v3Separate = hitMaxT.m_v3Normal * hitMaxT.t;                
                return true;
            }

            return false;
        }

        public static bool RigidBodyAndRigidBody(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Hit> listHits2, ref Vector3 v3Separate2) 
        {
            // aabb swep-test
            if
            (
                false == IsOverlapAABB(rigidBody1, rigidBody2)
             || false == IsOverlapAABB(rigidBody2, rigidBody1)
            )
            {
                return false;
            }

            // points in other body
            List<Vector3> listPoints = new List<Vector3>();
            GetPointsInConvexMesh(rigidBody1, rigidBody2, ref listPoints);
            GetPointsInConvexMesh(rigidBody2, rigidBody1, ref listPoints);

            // separate
            Vector3 v3Normal1 = new Vector3();
            float t1 = 0.0f;
            SearchMinSeparate(rigidBody1, listPoints, ref v3Normal1, ref t1);

            // create hits
            listHits2.Clear();
            foreach (Vector3 v3PosInWorld in listPoints) 
            {
                Hit hit = new Hit();
                hit.m_v3Normal = v3Normal1;
				hit.t = t1;
                hit.m_v3PositionInWorld = v3PosInWorld - (hit.m_v3Normal * hit.t);

                listHits2.Add(hit);
            }

            v3Separate2 = v3Normal1 * t1;

            return (listHits2.Count() > 0);
        }

        private static bool IsOverlapAABB(RigidBody rigidBody1, RigidBody rigidBody2) 
        {

            Matrix4 m4FinalTransform = Matrix4.Mult(rigidBody2.m_m4World, rigidBody1.m_m4World.Inverted());
            
            Vector3 v3RigidBody1Min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 v3RigidBody1Max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

            foreach (Vector3 v3Point in rigidBody1.m_listPoints) 
            {
                v3RigidBody1Min.X = (v3Point.X < v3RigidBody1Min.X) ? (v3Point.X) : (v3RigidBody1Min.X);
                v3RigidBody1Min.Y = (v3Point.Y < v3RigidBody1Min.Y) ? (v3Point.Y) : (v3RigidBody1Min.Y);
                v3RigidBody1Min.Z = (v3Point.Z < v3RigidBody1Min.Z) ? (v3Point.Z) : (v3RigidBody1Min.Z);

                v3RigidBody1Max.X = (v3Point.X > v3RigidBody1Max.X) ? (v3Point.X) : (v3RigidBody1Max.X);
                v3RigidBody1Max.Y = (v3Point.Y > v3RigidBody1Max.Y) ? (v3Point.Y) : (v3RigidBody1Max.Y);
                v3RigidBody1Max.Z = (v3Point.Z > v3RigidBody1Max.Z) ? (v3Point.Z) : (v3RigidBody1Max.Z);
            }

            Vector3 v3RigidBody2Min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 v3RigidBody2Max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

            foreach (Vector3 v3Point in rigidBody2.m_listPoints) 
            {
                Vector3 v3Point2 = Vector4.Transform(new Vector4(v3Point, 1), m4FinalTransform).Xyz;

                v3RigidBody2Min.X = (v3Point2.X < v3RigidBody2Min.X) ? (v3Point2.X) : (v3RigidBody2Min.X);
                v3RigidBody2Min.Y = (v3Point2.Y < v3RigidBody2Min.Y) ? (v3Point2.Y) : (v3RigidBody2Min.Y);
                v3RigidBody2Min.Z = (v3Point2.Z < v3RigidBody2Min.Z) ? (v3Point2.Z) : (v3RigidBody2Min.Z);
                
                v3RigidBody2Max.X = (v3Point2.X > v3RigidBody2Max.X) ? (v3Point2.X) : (v3RigidBody2Max.X);
                v3RigidBody2Max.Y = (v3Point2.Y > v3RigidBody2Max.Y) ? (v3Point2.Y) : (v3RigidBody2Max.Y);
                v3RigidBody2Max.Z = (v3Point2.Z > v3RigidBody2Max.Z) ? (v3Point2.Z) : (v3RigidBody2Max.Z);
            }

            

            // X
            bool bCollX = false;
            float fXMin = Math.Min(v3RigidBody1Min.X, v3RigidBody2Min.X);
            float fXMax = Math.Max(v3RigidBody1Max.X, v3RigidBody2Max.X);
            float fFull = fXMax - fXMin;
            float fXPart1 = v3RigidBody1Max.X - v3RigidBody1Min.X;
            float fXPart2 = v3RigidBody2Max.X - v3RigidBody2Min.X;
            if ( (fXPart1 + fXPart2 + margin) > fFull ) { bCollX = true; }

            // Y
            bool bCollY = false;
            float fYMin = Math.Min(v3RigidBody1Min.Y, v3RigidBody2Min.Y);
            float fYMax = Math.Max(v3RigidBody1Max.Y, v3RigidBody2Max.Y);
            fFull = fYMax - fYMin;
            float fYPart1 = v3RigidBody1Max.Y - v3RigidBody1Min.Y;
            float fYPart2 = v3RigidBody2Max.Y - v3RigidBody2Min.Y;
            if ((fYPart1 + fYPart2 + margin) > fFull) { bCollY = true; }

            // Z
            bool bCollZ = false;
            float fZMin = Math.Min(v3RigidBody1Min.Z, v3RigidBody2Min.Z);
            float fZMax = Math.Max(v3RigidBody1Max.Z, v3RigidBody2Max.Z);
            fFull = fZMax - fZMin;
            float fZPart1 = v3RigidBody1Max.Z - v3RigidBody1Min.Z;
            float fZPart2 = v3RigidBody2Max.Z - v3RigidBody2Min.Z;
            if ((fZPart1 + fZPart2 + margin) > fFull) { bCollZ = true; }

            return (bCollX && bCollY && bCollZ);
        }

        private static void GetPointsInConvexMesh(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Vector3> listPoints)
        {
            Matrix4 m4FinalTransform = Matrix4.Mult(rigidBody2.m_m4World, rigidBody1.m_m4World.Inverted());

            for (int id = 0; id < ( rigidBody2.m_listIndices.Count() - 1); id += 2)
            {
                Vector3 v3Start = rigidBody2.m_listPoints[rigidBody2.m_listIndices[id + 0]];
                Vector3 v3End = rigidBody2.m_listPoints[rigidBody2.m_listIndices[id + 1]];
                
                for (int i = 0; i <= step; i++)
                {
                    float t = (float)i / (float)step;
                    Vector3 v3Point = ((1.0f - t) * v3Start) + (t * v3End);

                    Vector3 v3PointInLocal = Vector4.Transform(new Vector4(v3Point, 1), m4FinalTransform).Xyz;

                    bool bIsIn = true;
                    int nId = 0;
                    for (int id2 = 0; id2 < rigidBody1.m_listIndices.Count; id2 += 3, nId++)
                    {
                        Vector3 v3AInLocal = rigidBody1.m_listPoints[rigidBody1.m_listIndices[id2 + 0]];
                        Vector3 v3NLocal = rigidBody1.m_listTriangleNormals[nId];

                        Plane plane = new Plane(v3AInLocal + (v3NLocal * margin), v3NLocal);
                        float dist = plane.GetDistance(v3PointInLocal);

                        if (dist > margin)
                        {
                            bIsIn = false;
                        }
                    }

                    if (true == bIsIn)
                    {
                        Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), rigidBody2.m_m4World).Xyz;
                        listPoints.Add(v3PointInWorld);
                    }

                }
            }
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

        private static void SearchMinSeparate(RigidBody rigidBody, List<Vector3> listPoints, ref Vector3 v3RetMinSeparateNormal, ref float fRetMinDist) 
        {
            if (0 == listPoints.Count()) 
            {
                return;
            }

            v3RetMinSeparateNormal = new Vector3();
            fRetMinDist = float.MaxValue;

            int nId = 0;
            for (int id = 0; id < rigidBody.m_listIndices.Count; id += 3, nId++)
            {
                // plane pos
                Vector3 v3A = rigidBody.m_listPoints[rigidBody.m_listIndices[id + 0]];
                Vector3 v3AInWorld = Vector4.Transform(new Vector4(v3A, 1), rigidBody.m_m4World).Xyz;

                // plane normal
                Vector3 v3N = rigidBody.m_listTriangleNormals[nId];
                Vector3 v3NInWorld = Vector4.Transform(new Vector4(v3N, 0), rigidBody.m_m4World).Xyz;
                v3NInWorld.Normalize();

                Plane plane = new Plane(v3AInWorld, v3NInWorld);

                Vector3 v3LocalMaxSeparateNormal = new Vector3();
                float fLocalMaxDist = float.MinValue;

                foreach (Vector3 v3PointInWorld in listPoints) 
                {
                    float fDist = -plane.GetDistance(v3PointInWorld);

                    if (fDist > fLocalMaxDist) 
                    {
                        v3LocalMaxSeparateNormal = v3NInWorld;
                        fLocalMaxDist = fDist;
                    }
                }

                if (fLocalMaxDist < fRetMinDist)
                {
                    v3RetMinSeparateNormal = v3LocalMaxSeparateNormal;
                    fRetMinDist = fLocalMaxDist;
                }
            }
        }
    }
}
