﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK.Graphics.OpenGL;
using OpenTK;

namespace Physics
{
    public class CollisionResponse
    {
        public static void Apply(RigidBody rigidBody, List<Hit> listHits, float dt)
        {
            foreach (Hit hit in listHits)
            {
                // calc contact velocity
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody.m_v3Position;
                Vector3 v3RelVelocity = rigidBody.GetPointVelocity(rA);
                float fProjVelocity = Vector3.Dot(v3RelVelocity, hit.m_v3Normal);

                // calc inertia
                float nominator = -(1.0f + rigidBody.m_fRestitution) * fProjVelocity;
                float term1 = 1.0f / rigidBody.m_fMass;
                float term2 = 0.0f;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = 0.0f;
                float J = nominator / (term1 + term2 + term3 + term4);

                // apply velocity
                rigidBody.m_v3LinearVelocity += (J * 0.5f) * hit.m_v3Normal;
                rigidBody.m_v3AngularVelocity += (J * 0.5f) * Vector3.Cross(rA, hit.m_v3Normal);
            }

            // separate
            if (listHits.Count > 0) 
            {
                Vector3 v3Separate = new Vector3();
                foreach (Hit hit in listHits)
                {
                    v3Separate += hit.m_v3Normal;
                }
                rigidBody.m_v3Position += v3Separate.Normalized() * dt * 0.025f;
            }
            
        }

        public static void Apply(RigidBody rigidBody1, RigidBody rigidBody2, List<Hit> listHits, float dt)
        {
            foreach (Hit hit in listHits)
            {
                // calc contact velocity
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody1.m_v3Position;
                Vector3 rB = hit.m_v3PositionInWorld - rigidBody2.m_v3Position;
                Vector3 v3RelVelocity = rigidBody1.GetPointVelocity(rA) - rigidBody2.GetPointVelocity(rB);
                float fProjVelocity = Vector3.Dot(v3RelVelocity, hit.m_v3Normal);

                // calc inertia
                float nominator = -(1.0f + rigidBody1.m_fRestitution) * fProjVelocity;
                float term1 = 1.0f;
                float term2 = 1.0f;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rB, hit.m_v3Normal), rB));
                float J = nominator / (term1 + term2 + term3 + term4);

                // apply velocity
                rigidBody1.m_v3LinearVelocity += (J * 0.5f) * hit.m_v3Normal;
                rigidBody1.m_v3AngularVelocity += (J * 0.5f) * Vector3.Cross(rA, hit.m_v3Normal);

                rigidBody2.m_v3LinearVelocity -= (J * 0.5f) * hit.m_v3Normal;
                rigidBody2.m_v3AngularVelocity -= (J * 0.5f) * Vector3.Cross(rB, hit.m_v3Normal);
            }

            // separate
            if (listHits.Count > 0)
            {
                Vector3 v3Separate = new Vector3();
                foreach (Hit hit in listHits)
                {
                    v3Separate += hit.m_v3Normal;
                }
                rigidBody1.m_v3Position -= v3Separate.Normalized() * dt * 0.05f;
            }
        }
    }
}
