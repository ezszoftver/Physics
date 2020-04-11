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
        public static void Apply(RigidBody rigidBody, List<Hit> listHits, Vector3 v3Separate, float dt)
        {
            //rigidBody.m_v3Position -= rigidBody.m_v3LinearVelocity * dt;

            foreach (Hit hit in listHits)
            {
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody.m_v3Position;
                Vector3 v3Velocity = rigidBody.GetPointVelocity(rA);

                float fRelVelocity = Vector3.Dot(v3Velocity, hit.m_v3Normal);

                float nominator = -(1.0f + rigidBody.m_fRestitution) * fRelVelocity;
                float term1 = 1.0f / rigidBody.m_fMass;
                float term2 = 0.0f;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = 0.0f;
                float J = nominator / (term1 + term2 + term3 + term4);

                rigidBody.m_v3LinearVelocity += (J / rigidBody.m_fMass) * hit.m_v3Normal;
                rigidBody.m_v3AngularVelocity += (J / rigidBody.m_fMass) * Vector3.Cross(rA, hit.m_v3Normal);

                // separate
                Vector3 v3Force = hit.m_v3Normal * 2.0f;
                //
                rigidBody.m_v3LinearVelocity += v3Force * dt;
                //
                rigidBody.m_v3AngularVelocity += Vector3.Cross(rA, v3Force) * dt;
                //
                rigidBody.m_v3Position += 1.0f * v3Force * dt * dt;
            }

            //rigidBody.m_v3Position += v3Separate;
        }

        public static void Apply(RigidBody rigidBody1, RigidBody rigidBody2, List<Hit> listHits, Vector3 v3Separate, float dt)
        {
            //rigidBody1.m_v3Position -= rigidBody1.m_v3LinearVelocity * dt;
            //rigidBody2.m_v3Position -= rigidBody2.m_v3LinearVelocity * dt;

            foreach (Hit hit in listHits)
            {
                Vector3 rA = hit.m_v3PositionInWorld - rigidBody1.m_v3Position;
                Vector3 rB = hit.m_v3PositionInWorld - rigidBody2.m_v3Position;
                Vector3 v3Velocity = rigidBody1.GetPointVelocity(rA) - rigidBody2.GetPointVelocity(rB);

                float fRelVelocity = Vector3.Dot(v3Velocity, hit.m_v3Normal);

                float nominator = -(1.0f + rigidBody1.m_fRestitution) * fRelVelocity;
                float term1 = 1.0f / rigidBody1.m_fMass;
                float term2 = 1.0f / rigidBody2.m_fMass;
                float term3 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rA, hit.m_v3Normal), rA));
                float term4 = Vector3.Dot(hit.m_v3Normal, Vector3.Cross(Vector3.Cross(rB, hit.m_v3Normal), rB));
                float J = nominator / (term1 + term2 + term3 + term4);

                rigidBody1.m_v3LinearVelocity += (J / rigidBody1.m_fMass) * hit.m_v3Normal;
                rigidBody1.m_v3AngularVelocity += (J / rigidBody1.m_fMass) * Vector3.Cross(rA, hit.m_v3Normal);

                rigidBody2.m_v3LinearVelocity -= (J / rigidBody2.m_fMass) * hit.m_v3Normal;
                rigidBody2.m_v3AngularVelocity -= (J / rigidBody2.m_fMass) * Vector3.Cross(rB, hit.m_v3Normal);

                // separate
                Vector3 v3Force = hit.m_v3Normal * 2.0f;
                //
                rigidBody1.m_v3LinearVelocity += v3Force * dt;
                rigidBody2.m_v3LinearVelocity -= v3Force * dt;
                //
                rigidBody1.m_v3AngularVelocity += Vector3.Cross(rA, v3Force) * dt;
                rigidBody2.m_v3AngularVelocity -= Vector3.Cross(rB, v3Force) * dt;
                //
                rigidBody1.m_v3Position += 1.0f * v3Force * dt * dt;
                rigidBody2.m_v3Position -= 1.0f * v3Force * dt * dt;
            }

            //rigidBody1.m_v3Position += v3Separate;
            //rigidBody2.m_v3Position -= v3Separate;
        }
    }
}
