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
        public static bool RigidBodyAndPlane(RigidBody rigidBody, Plane plane, ref List<Hit> listHits, ref Vector3 v3Separate)
        {
            listHits.Clear();

            foreach (Vector3 v3Point in rigidBody.m_listPoints)
            {
                Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), rigidBody.m_m4World).Xyz;
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

        public static bool RigidBodyAndRigidBody(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Hit> listHits, ref Vector3 v3Separate) 
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
            List<Vector3> listPoints2 = new List<Vector3>();
            GetPointsInConvexMesh(rigidBody1, rigidBody2, ref listPoints2);
            List<Vector3> listPoints1 = new List<Vector3>();
            GetPointsInConvexMesh(rigidBody2, rigidBody1, ref listPoints1);

            // separate
            listHits.Clear();
            ;

            // calc hits
            ;

            return true;
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
            if ( (fXPart1 + fXPart2) > fFull ) { bCollX = true; }

            // Y
            bool bCollY = false;
            float fYMin = Math.Min(v3RigidBody1Min.Y, v3RigidBody2Min.Y);
            float fYMax = Math.Max(v3RigidBody1Max.Y, v3RigidBody2Max.Y);
            fFull = fYMax - fYMin;
            float fYPart1 = v3RigidBody1Max.Y - v3RigidBody1Min.Y;
            float fYPart2 = v3RigidBody2Max.Y - v3RigidBody2Min.Y;
            if ((fYPart1 + fYPart2) > fFull) { bCollY = true; }

            // Z
            bool bCollZ = false;
            float fZMin = Math.Min(v3RigidBody1Min.Z, v3RigidBody2Min.Z);
            float fZMax = Math.Max(v3RigidBody1Max.Z, v3RigidBody2Max.Z);
            fFull = fZMax - fZMin;
            float fZPart1 = v3RigidBody1Max.Z - v3RigidBody1Min.Z;
            float fZPart2 = v3RigidBody2Max.Z - v3RigidBody2Min.Z;
            if ((fZPart1 + fZPart2) > fFull) { bCollZ = true; }

            return (bCollX && bCollY && bCollZ);
        }

        private static void GetPointsInConvexMesh(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Vector3> listPoints)
        {
            listPoints.Clear();

            Matrix4 m4FinalTransform = Matrix4.Mult(rigidBody2.m_m4World, rigidBody1.m_m4World.Inverted());

            foreach (Vector3 v3Point in rigidBody2.m_listPoints)
            {
                Vector3 v3PointInLocal = Vector4.Transform(new Vector4(v3Point, 1), m4FinalTransform).Xyz;

                int nId = 0;
                for (int id = 0; id < rigidBody1.m_listIndices.Count; id += 3, nId++) 
                {
                    Vector3 v3AInLocal = rigidBody1.m_listPoints[ rigidBody1.m_listIndices[id + 0] ];
                    Vector3 v3NLocal = rigidBody1.m_listTriangleNormals[nId];

                    Vector3 v3Dir = (v3PointInLocal - v3AInLocal).Normalized();

                    if (Vector3.CalculateAngle(v3NLocal, v3Dir) > ToRadian(90.0f)) 
                    {
                        listPoints.Add(v3Point);
                    }
                }
            }
        }

        private static float ToRadian(float fDegree)
        {
            return (fDegree / 180.0f * (float)Math.PI);
        }
    }
}
