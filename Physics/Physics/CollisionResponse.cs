using System;
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
            // normal
            foreach (Hit hit in listHits)
            {
                // calc contact velocity
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody.m_v3Position;
                Vector3 v3RelVelocity = rigidBody.GetPointVelocity(rA);
                float fProjVelocity = Vector3.Dot(v3RelVelocity, hit.m_v3Normal);

                if (fProjVelocity >= 0.0f) 
                {
                    continue;
                }

                // calc impulse
                float nominator = -(1.0f + rigidBody.m_fRestitution) * fProjVelocity;
                float term1 = 1.0f / rigidBody.m_fMass;
                float term2 = 0.0f;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = 0.0f;
                float J = nominator / (term1 + term2 + term3 + term4);

                J /= (float)listHits.Count();

                // apply velocity
                rigidBody.m_v3LinearVelocity += (J) * hit.m_v3Normal;
                rigidBody.m_v3AngularVelocity += (J) * Vector3.Cross(rA, hit.m_v3Normal);
            }

            // friction
            foreach (Hit hit in listHits)
            {
                // calc contact velocity
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody.m_v3Position;
                Vector3 v3RelVelocity = rigidBody.GetPointVelocity(rA);
                float fProjVelocity = Vector3.Dot(v3RelVelocity, hit.m_v3Normal);

                if (fProjVelocity >= 0.0f)
                {
                    continue;
                }

                Vector3 v3Tangent = v3RelVelocity - (Vector3.Dot(v3RelVelocity, hit.m_v3Normal) * hit.m_v3Normal);
                if (v3Tangent.Length > 0.001f)
                {
                    v3Tangent.Normalize();
                }
                else 
                {
                    v3Tangent = new Vector3(0, 0, 0);
                }

                // calc impulse
                float nominator = -Vector3.Dot(v3RelVelocity, v3Tangent);
                float term1 = 1.0f / rigidBody.m_fMass;
                float term2 = 0.0f;
                float term3 = Vector3.Dot(v3Tangent, Vector3.Cross(Vector3.Cross(rA, v3Tangent), rA));
                float term4 = 0.0f;
                float T = nominator / (term1 + term2 + term3 + term4);

                T /= (float)listHits.Count();

                T *= rigidBody.m_fFriction;

                // apply velocity
                rigidBody.m_v3LinearVelocity += (T) * v3Tangent;
                rigidBody.m_v3AngularVelocity += (T) * Vector3.Cross(rA, v3Tangent);
            }

            // separate
            if (listHits.Count > 0)
            {
                Hit hit = new Hit();
                foreach (Hit hit2 in listHits)
                {
                    if (hit2.m_fPenetration > hit.m_fPenetration)
                    {
                        hit = hit2;
                    }
                }

                rigidBody.m_v3Position += hit.m_v3Normal * hit.m_fPenetration;
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

                //if (fProjVelocity >= 0.0f)
                //{
                //    continue;
                //}

                // calc impulse
                float nominator = -(1.0f + rigidBody1.m_fRestitution) * fProjVelocity;
                float term1 = 1.0f / rigidBody1.m_fMass;
                float term2 = 1.0f / rigidBody2.m_fMass;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rB, hit.m_v3Normal), rB));
                float J = nominator / (term1 + term2 + term3 + term4);

                J /= (float)listHits.Count();

                // apply velocity
                rigidBody1.m_v3LinearVelocity += (J) * hit.m_v3Normal;
                rigidBody1.m_v3AngularVelocity += (J) * Vector3.Cross(rA, hit.m_v3Normal);

                rigidBody2.m_v3LinearVelocity -= (J) * hit.m_v3Normal;
                rigidBody2.m_v3AngularVelocity -= (J) * Vector3.Cross(rB, hit.m_v3Normal);
            }

            // friction
            foreach (Hit hit in listHits)
            {
                // calc contact velocity
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody1.m_v3Position;
                Vector3 rB = hit.m_v3PositionInWorld - rigidBody2.m_v3Position;
                Vector3 v3RelVelocity = rigidBody1.GetPointVelocity(rA) - rigidBody2.GetPointVelocity(rB);
                float fProjVelocity = Vector3.Dot(v3RelVelocity, hit.m_v3Normal);

                //if (fProjVelocity >= 0.0f)
                //{
                //    continue;
                //}

                Vector3 v3Tangent = v3RelVelocity - (Vector3.Dot(v3RelVelocity, hit.m_v3Normal) * hit.m_v3Normal);
                if (v3Tangent.Length > 0.001f)
                {
                    v3Tangent.Normalize();
                }
                else
                {
                    v3Tangent = new Vector3(0, 0, 0);
                }

                // calc impulse
                float nominator = -Vector3.Dot(v3RelVelocity, v3Tangent);
                float term1 = 1.0f / rigidBody1.m_fMass;
                float term2 = 1.0f / rigidBody2.m_fMass;
                float term3 = Vector3.Dot(v3Tangent, Vector3.Cross(Vector3.Cross(rA, v3Tangent), rA));
                float term4 = Vector3.Dot(v3Tangent, Vector3.Cross(Vector3.Cross(rB, v3Tangent), rB));
                float T = nominator / (term1 + term2 + term3 + term4);

                T /= (float)listHits.Count();

                float T1 = T * rigidBody1.m_fFriction;
                float T2 = T * rigidBody2.m_fFriction;

                // apply velocity
                rigidBody1.m_v3LinearVelocity += (T1) * v3Tangent;
                rigidBody1.m_v3AngularVelocity += (T1) * Vector3.Cross(rA, v3Tangent);

                rigidBody2.m_v3LinearVelocity -= (T2) * v3Tangent;
                rigidBody2.m_v3AngularVelocity -= (T2) * Vector3.Cross(rB, v3Tangent);
            }

            // separate
            if (listHits.Count > 0)
            {
                Hit hit = new Hit();
                foreach (Hit hit2 in listHits)
                {
                    if (hit2.m_fPenetration > hit.m_fPenetration)
                    {
                        hit = hit2;
                    }
                }

                rigidBody1.m_v3Position += hit.m_v3Normal * hit.m_fPenetration * 0.95f * 0.5f;
                rigidBody2.m_v3Position -= hit.m_v3Normal * hit.m_fPenetration * 0.95f * 0.5f;
            }
        }
    }
}
