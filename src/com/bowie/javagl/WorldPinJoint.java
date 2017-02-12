package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class WorldPinJoint implements Joint {
	protected RigidBody body;
	protected Vector3 worldPinPos;
	protected Vector3 bodyLocalPos;
	
	protected Vector3 n, bodyWorldPos;
	
	private Vector3 accumP, bias;
	
	private Matrix3 M;	// the constraint mass
	
	public WorldPinJoint(Vector3 worldPinPos, RigidBody b, Vector3 bodyLocalPos) {
		this.bodyLocalPos = new Vector3(bodyLocalPos);
		this.worldPinPos = new Vector3(worldPinPos);
		this.body = b;
		
		this.accumP = new Vector3();
		this.bias = new Vector3();
		this.n = new Vector3();
		this.bodyWorldPos = new Vector3();
	}
	
	@Override
	public void preCalculate(float dt, float baumgarte, float slop) {
		// update body position?
		bodyWorldPos = body.toWorld(bodyLocalPos);
		
		// update n
		Vector3.sub(bodyWorldPos, worldPinPos, n);
		
		// set as bias
		bias.setTo(n);
		bias.scale(-baumgarte/dt);
		
		// calculate r
		Vector3 r = new Vector3();
		body.getRot().transformVector(bodyLocalPos, r);
		
		Matrix3 K1 = Matrix3.diagonal(body.getInvMass());
		Matrix3 K2 = new Matrix3();
		
		Matrix3 RA = Matrix3.skew(r);
		Matrix3.mul(RA, body.getWorldInvInertia(), K2);
		Matrix3.mul(K2, RA, K2);
		
		K1.sub(K2);
		K1.invert();
		
		M = K1;
		
		// apply old impulse
		body.applyImpulse(accumP, bodyWorldPos);
	}

	@Override
	public void solve() {
		Vector3 vel = body.getVelWS(bodyWorldPos);
		
		Vector3 j = new Vector3();
		
		Vector3.add(vel.inverse(), Vector3.ZERO, j);	// equal: (-dv + bias)
		M.transformVector3(j, j);				// equal: M * (-dv + bias)
		
		body.applyImpulse(j, bodyWorldPos);
		
		// accumulate
		Vector3.add(accumP, j, accumP);
	}

	@Override
	public void debugDraw(GL2 gl, float dt) {
		// draw both point in red and blue
		Vector3 worldBodyPos = body.toWorld(bodyLocalPos);
		gl.glBegin(GL2.GL_POINTS);
			gl.glColor3f(1, 0, 0);
			gl.glVertex3f(worldPinPos.x, worldPinPos.y, worldPinPos.z);
			
			gl.glColor3f(0, 1, 0);
			gl.glVertex3f(worldBodyPos.x, worldBodyPos.y, worldBodyPos.z);
		gl.glEnd();
		
		// draw line
		gl.glBegin(GL2.GL_LINES);
			// joint line
			gl.glColor3f(1, 0, 0);
			gl.glVertex3f(worldPinPos.x, worldPinPos.y, worldPinPos.z);
			
			gl.glColor3f(0, 1, 0);
			gl.glVertex3f(worldBodyPos.x, worldBodyPos.y, worldBodyPos.z);
			
			// body line
			gl.glColor3f(0, 0, 1);
			gl.glVertex3f(body.getPos().x, body.getPos().y, body.getPos().z);
			
			gl.glColor3f(0, 1, 0);
			gl.glVertex3f(worldBodyPos.x, worldBodyPos.y, worldBodyPos.z);
		gl.glEnd();
	}

	@Override
	public void positionSolve() {
		Vector3 vel = body.getBiasVelWS(bodyWorldPos);
		
		Vector3 j = new Vector3();
		Vector3.add(vel.inverse(), bias, j);
		M.transformVector3(j, j);
		
		body.applyBiasImpulse(j, bodyWorldPos);
	}

	public Vector3 getWorldPinPos() {
		return worldPinPos;
	}

	public void setWorldPinPos(Vector3 worldPinPos) {
		this.worldPinPos = worldPinPos;
	}

}
