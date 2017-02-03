package com.bowie.javagl;

public class SpeculativeContact {
	public RigidBody bA, bB;	// the pair of bodies, body A and body B
	public Vector3 normal;		// the normal. must point toward body A

	public float toi, dn;			// the time of impact and distance along normal
	public float kMass, rN, accum;	// the contact mass, and accumulation?
	
	
	public SpeculativeContact(float dn, Vector3 n, float t, RigidBody bodyA, RigidBody bodyB) {
		
		normal = new Vector3(n);
		
		toi = t;
		bA = bodyA;
		bB = bodyB;
		
		accum = 0;
	}
	
	public void preCalculate(float dt, float baumgarte, float slop) {
		// mass of constraint
		kMass = bA.getInvMass() + bB.getInvMass();
		kMass = 1.f/kMass;
		
		// vA - vB
		Vector3 vAB = new Vector3(bA.getVel(), bB.getVel());
		
		// the removed velocity?
		rN = baumgarte * (dn+1) / dt;
	}
	
	public void solve() {
		// get velocity
		Vector3 vAB = new Vector3(bA.getVel(), bB.getVel());
		float vn = Vector3.dot(vAB, normal);
		
		// the impulse only remove velocity so both bodies are touching after it's applied
		float jN = kMass * (-vn-rN);
		
//		// do smart clamping
		float oldAccum = accum;
		accum = Math.max(0.f, accum + jN);
		float impulse = accum - oldAccum;
//		float impulse = Math.max(0.f, jN);
		
		Vector3 j = new Vector3(normal);
		j.scale(impulse);
		
		bA.applyImpulse(j);
		bB.applyImpulse(j.inverse());
		
	}
}
