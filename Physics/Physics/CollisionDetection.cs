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
        public static bool RigidBodyAndPlane(RigidBody rigidBody, Plane plane, List<Hit> listHits)
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

            return (listHits.Count() > 0);
        }

        public static bool RigidBodyAndRigidBody(RigidBody rigidBody1, RigidBody rigidBody2, List<Hit> listHits) 
        {
            listHits.Clear();

            if
            (
                true == CollisionDetectionAABB(rigidBody1, rigidBody2, listHits)
             && true == CollisionDetectionAABB(rigidBody2, rigidBody1, listHits)
            )
            {
                return true;
            }

            return false;
        }

        private static bool CollisionDetectionAABB(RigidBody rigidBody1, RigidBody rigidBody2, List<Hit> listHits) 
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
    }
}
