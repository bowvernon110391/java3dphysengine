package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class DistanceJoint implements Joint {
	protected RigidBody bodyA, bodyB;
	protected float length;	// joint length
	protected float stiffness = 0.1f;
	protected Vector3 localA, localB;
	protected Vector3 worldA, worldB;
	protected Vector3 n;	// the normal (pointing to A)
	protected float kMass;	// the mass of the constraint
//	protected Vector3 accumP;	// acuumulated
	protected float bias;	// bias term

	public DistanceJoint(RigidBody bA, RigidBody bB, Vector3 lA, Vector3 lB, float dist, float stiffness) {
		bodyA = bA;
		bodyB = bB;
		
		localA = new Vector3(lA);
		localB = new Vector3(lB);
		
		worldA = new Vector3();
		worldB = new Vector3();
		n = new Vector3();
//		accumP = new Vector3();
		length = dist;
		this.stiffness = stiffness;
	}
	
	@Override
	public void preCalculate(float dt, float baumgarte) {
		// update position
		worldA = bodyA.toWorld(localA);
		worldB = bodyB.toWorld(localB);
		
		Vector3.sub(worldA, worldB, n);
		// grab bias. no slop needed
		bias = (length - n.length()) * stiffness * baumgarte / dt;	// totally stiff
		
		// normalize direction
		n.normalize();
		
		// calculate mass?
		kMass = bodyA.getInvMass() + bodyB.getInvMass();
		
		Vector3 r = new Vector3();
		Vector3 rn = new Vector3();
		
		bodyA.getRot().transformVector(localA, r);
		Vector3.cross(r, n, rn);
		Vector3.cross(rn, r, rn);
		bodyA.getWorldInvInertia().transformVector3(rn, rn);
		kMass += Vector3.dot(n, rn);
		
		bodyB.getRot().transformVector(localB, r);
		Vector3.cross(r, n, rn);
		Vector3.cross(rn, r, rn);
		bodyB.getWorldInvInertia().transformVector3(rn, rn);
		kMass += Vector3.dot(n, rn);
		
		kMass = kMass > 0 ? 1.0f/kMass : 0;
		
		// should do warmstart here, applying old impulse,
		// but the joint would blow away!!
//		bodyA.applyImpulse(accumP, worldA);
//		bodyB.applyImpulse(accumP.inverse(), worldB);
	}

	@Override
	public void solve() {
		Vector3 j = new Vector3();
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldB), j);
		
		float jMag = (-Vector3.dot(j, n) + bias) * kMass;
		
		j.setTo(n);
		j.scale(jMag);
		
		bodyA.applyImpulse(j, worldA);
		bodyB.applyImpulse(j.inverse(), worldB);
	}

	@Override
	public void debugDraw(GL2 gl) {
		gl.glBegin(GL2.GL_POINTS);
			gl.glColor3f(1, 0.5f, 0.2f);
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			
			gl.glColor3f(0.2f, 0.5f, 1);
			gl.glVertex3f(worldB.x, worldB.y, worldB.z);
		gl.glEnd();
		
		// draw single line
		gl.glBegin(GL2.GL_LINES);
			gl.glColor3f(0.25f, 1, 0.25f);
			
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			gl.glVertex3f(worldB.x, worldB.y, worldB.z);
		gl.glEnd();
	}

	@Override
	public void positionSolve() {
		// let's do split impulse
		/*Vector3 j = new Vector3();
		Vector3.sub(bodyA.getBiasVelWS(worldA), bodyB.getBiasVelWS(worldB), j);
		
		float jMag = (-Vector3.dot(j, n) + bias) * kMass;
		j.setTo(n);
		j.scale(jMag);
		
		bodyA.applyBiasImpulse(j, worldA);
		bodyB.applyBiasImpulse(j.inverse(), worldB);*/
	}

}
