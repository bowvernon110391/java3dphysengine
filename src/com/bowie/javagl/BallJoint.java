package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class BallJoint implements Joint {
	protected RigidBody bodyA, bodyB;
	protected Vector3 localA, localB;
	
	protected Vector3 worldA, worldB;
	
	private Vector3 bias;	// for bias term
	
	private Matrix3 M;		// this is the constraint mass
	
	private Vector3 accumP;	// accumulated impulse?
	
	public BallJoint(RigidBody bA, Vector3 lA, RigidBody bB, Vector3 lB) {
		// just copy
		bodyA = bA;
		bodyB = bB;
		
		localA = new Vector3(lA);
		localB = new Vector3(lB);
		
		worldA = new Vector3();
		worldB = new Vector3();
		
		bias = new Vector3();
		accumP = new Vector3();
	}
	
	@Override
	public void preCalculate(float dt, float baumgarte) {
		// update world pos
		worldA = bodyA.toWorld(localA);
		worldB = bodyB.toWorld(localB);
		
		// n is pointing to A
		Vector3.sub(worldA, worldB, bias);
		// meanwhile, bias is pointing there
		bias.scale(-baumgarte/dt);
		// normalize direction (otherwise it blows off)
		
		// calculate mass
		Vector3 r1 = new Vector3();
		Vector3 r2 = new Vector3();
		
		bodyA.getRot().transformVector(localA, r1);
		bodyB.getRot().transformVector(localB, r2);
		
		// let's compute mass matrix
		Matrix3 K1 = Matrix3.diagonal(bodyA.getInvMass() + bodyB.getInvMass());
		
		Matrix3 RA = Matrix3.skew(r1);
		
		Matrix3 RB = Matrix3.skew(r2);
		
		Matrix3 K2 = new Matrix3();
		Matrix3 K3 = new Matrix3();
		
		// compute RA * Ia * RAT
		Matrix3.mul(RA, bodyA.getWorldInvInertia(), K2);
		Matrix3.mul(K2, RA, K2);
		
		Matrix3.mul(RB, bodyB.getWorldInvInertia(), K3);
		Matrix3.mul(K3, RB, K3);
		
		// sum or sub? confused
		K1.sub(K2);
		K1.sub(K3);
		K1.invert();
		
		M = K1;
		
		// now, apply old impulse
		bodyA.applyImpulse(accumP, worldA);
		bodyB.applyImpulse(accumP.inverse(), worldB);
	}

	@Override
	public void solve() {
		Vector3 j = new Vector3();
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldB), j);
		
		Vector3.add(bias, j.inverse(), j);	// equal: -dv + bias
		M.transformVector3(j, j);		// P = M * (-dv + bias)
		
		// apply to both bodies
		bodyA.applyImpulse(j, worldA);
		bodyB.applyImpulse(j.inverse(), worldB);
		
		// accumulate for warm starting
		Vector3.add(accumP, j, accumP);
	}

	@Override
	public void debugDraw(GL2 gl) {
		// draw points first
		gl.glBegin(GL2.GL_POINTS);
		// 1st on bodyA
		gl.glColor3f(1, 1, 0);
		gl.glVertex3f(worldA.x, worldA.y, worldA.z);
		
		gl.glColor3f(1, 0, 1);
		gl.glVertex3f(worldB.x, worldB.y, worldB.z);
		gl.glEnd();
		
		// draw lines connecting them
		gl.glBegin(GL2.GL_LINES);
		// from bodyA
		gl.glColor3f(1, 0, 0);
		gl.glVertex3f(bodyA.getPos().x, bodyA.getPos().y, bodyA.getPos().z);
		gl.glColor3f(1, 1, 0);
		gl.glVertex3f(worldA.x, worldA.y, worldA.z);
		
		//from bodyB
		gl.glColor3f(0, 0, 1);
		gl.glVertex3f(bodyB.getPos().x, bodyB.getPos().y, bodyB.getPos().z);
		gl.glColor3f(1, 0, 1);
		gl.glVertex3f(worldB.x, worldB.y, worldB.z);
		gl.glEnd();
	}

	@Override
	public void positionSolve() {
		// now we apply bias impulse and see
		/*Vector3 j = new Vector3();
		Vector3.sub(bodyA.getBiasVelWS(worldA), bodyB.getBiasVelWS(worldB), j);
		
		Vector3.add(j.inverse(), bias, j);
		M.transformVector3(j, j);
		
		// apply to both
		bodyA.applyBiasImpulse(j, worldA);
		bodyB.applyBiasImpulse(j.inverse(), worldB);*/
	}

}
