﻿using System;
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
        public static int step = 10;
        public static float margin = 0.01f;

        public static bool RigidBodyAndPlane(RigidBody rigidBody, Plane plane, ref List<Hit> listHits)
        {
            listHits.Clear();

            int nIdN = 0;
            for (int id = 0; id < rigidBody.m_listIndices.Count(); id += 3, nIdN++)
            {
                Vector3 v3A = rigidBody.m_listPoints[rigidBody.m_listIndices[id + 0]];
                Vector3 v3B = rigidBody.m_listPoints[rigidBody.m_listIndices[id + 1]];
                Vector3 v3C = rigidBody.m_listPoints[rigidBody.m_listIndices[id + 2]];

                Vector3 v3Normal = rigidBody.m_listTriangleNormals[nIdN];

                for (int i = 0; i <= step; i++)
                {
                    float u = (float)i / (float)step;

                    Vector3 v3Start = ((1.0f - u) * v3A) + (u * v3B);
                    Vector3 v3End = ((1.0f - u) * v3A) + (u * v3C);

                    for (int j = 0; j <= /*step*/1; j++)
                    {
                        float v = (float)j / (float)step;
                        
                        Vector3 v3Point = ((1.0f - v) * v3Start) + (v * v3End);

                        Vector3 v3PointInWorld = Vector4.Transform(new Vector4(v3Point, 1), rigidBody.m_m4World).Xyz;
                        float dist = plane.GetDistance(v3PointInWorld);
                        if (dist < 0.01f)
                        {
                            Hit hit = new Hit();
                            hit.m_v3Normal = Vector4.Transform(new Vector4(v3Normal, 0), rigidBody.m_m4World).Xyz;/*plane.m_v3Normal;*/ 
                            hit.m_v3PositionInWorld = v3PointInWorld;
                            hit.m_v3SeparateDir = -hit.m_v3Normal;

                            listHits.Add(hit);
                        }
                    }
                    
                }
            }

            return (listHits.Count() > 0);
        }

        public static bool RigidBodyAndRigidBody(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Hit> listHits) 
        {
            // points in other body
            listHits.Clear();
            GetPointsInConvexMesh(rigidBody1, rigidBody2, ref listHits);

            return (listHits.Count() > 0);
        }

        private static void GetPointsInConvexMesh(RigidBody rigidBody1, RigidBody rigidBody2, ref List<Hit> listHits)
        {
            Matrix4 m4FinalTransform = Matrix4.Mult(rigidBody2.m_m4World, rigidBody1.m_m4World.Inverted());

            int nIdN = 0;
            for (int id = 0; id < rigidBody2.m_listIndices.Count(); id += 3, nIdN++)
            {
                Vector3 v3A = rigidBody2.m_listPoints[rigidBody2.m_listIndices[id + 0]];
                Vector3 v3B = rigidBody2.m_listPoints[rigidBody2.m_listIndices[id + 1]];
                Vector3 v3C = rigidBody2.m_listPoints[rigidBody2.m_listIndices[id + 2]];
                //Vector3 v3Normal = Vector3.Cross(v3B - v3A, v3C - v3A).Normalized();

                Vector3 v3Normal = rigidBody2.m_listTriangleNormals[nIdN];

                for (int i = 0; i <= step; i++)
                {
                    float u = (float)i / (float)step;

                    Vector3 v3Start = ((1.0f - u) * v3A) + (u * v3B);
                    Vector3 v3End = ((1.0f - u) * v3A) + (u * v3C);

                    for (int j = 0; j <= /*step*/1; j++)
                    {
                        float v = (float)j / (float)step;

                        Vector3 v3Point = ((1.0f - v) * v3Start) + (v * v3End);

                        Vector3 v3PointInLocal = Vector4.Transform(new Vector4(v3Point, 1), m4FinalTransform).Xyz;

                        bool bIsIn = true;
                        int nId = 0;
                        for (int id2 = 0; id2 < rigidBody1.m_listIndices.Count; id2 += 3, nId++)
                        {
                            if (false == bIsIn) { continue; }

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
                            Hit hit = new Hit();
                            
                            hit.m_v3Normal = Vector4.Transform(new Vector4(v3Normal, 0), rigidBody2.m_m4World).Xyz;
                            hit.m_v3PositionInWorld = Vector4.Transform(new Vector4(v3Point, 1), rigidBody2.m_m4World).Xyz + hit.m_v3Normal * 0.2f;
                            hit.m_v3SeparateDir = -hit.m_v3Normal;

                            listHits.Add(hit);
                        }
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
    }
}
