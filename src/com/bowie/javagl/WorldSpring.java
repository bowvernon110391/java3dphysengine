package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class WorldSpring implements SimForce {
	private Vector3 wa, ba;
	private RigidBody b;
	private float l;
	private float k;
	private float c;
	private Vector3 worldBA = new Vector3();
	private boolean active = true;
	
	public Vector3 getBodyAnchor() {
		return ba;
	}
	
	public Vector3 getWorldAnchor() {
		return wa;
	}
	
	public WorldSpring(Vector3 worldAnchor, Vector3 bodyAnchor, RigidBody body, float len, float kConst, float cVel) {
		wa = new Vector3(worldAnchor);
		ba = new Vector3(bodyAnchor);
		b = body;
		l = len;
		k = kConst;
		c = cVel;
	}
	
	public void simulate(float dt) {
		if (!active)
			return;
		// update world body anchor
		worldBA = b.toWorld(ba);
		// get velocity
		Vector3 v = b.getVelWS(worldBA);
		
		// compute force magnitude
		Vector3 ab = new Vector3();
		Vector3.sub(wa, worldBA, ab);
		
		// avoid singularity
		if (Vector3.dot(ab, ab) < Vector3.EPSILON)
			return;
		
		// grab length
		float len = ab.length();
		ab.scale(1.0f/l);
		
		float fMag = k * (len - l) - Vector3.dot(v, ab) * c;

		ab.scale(fMag);
		b.applyForce(ab, worldBA);
	}
	
	public void render(GL2 gl) {
		if (!active)
			return;
		gl.glBegin(GL2.GL_LINES);
			gl.glColor3f(1, 0, 0);
			gl.glVertex3f(wa.x, wa.y, wa.z);
			gl.glColor3f(1, 1, 0);
			gl.glVertex3f(worldBA.x, worldBA.y, worldBA.z);
		gl.glEnd();
	}

	public boolean isActive() {
		return active;
	}

	public void setActive(boolean active) {
		this.active = active;
	}
}
