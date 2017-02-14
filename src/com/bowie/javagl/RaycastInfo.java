package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class RaycastInfo {
	public Vector3 rayhitN, rayhitP;
	public float rayT;
	
	public float rayRadius;	// usually zero
	public Vector3 rayStart, rayEnd;
	
	public Polygon hitPoly;
	
	public RaycastInfo() {
		rayhitN = new Vector3();
		rayhitP = new Vector3();
		rayStart = new Vector3();
		rayEnd = new Vector3();
		
		hitPoly = null;
		
		reset();
	}
	
	public void reset() {
		// fill default value
		rayT = -1.f;
		rayhitN.setTo(0, 0, 0);
		rayhitP.setTo(0, 0, 0);
		rayStart.setTo(0, 0, 0);
		rayEnd.setTo(0, 0, 0);
	}
	
	public void debugDraw(GL2 gl) {
		gl.glBegin(GL2.GL_LINES);
		gl.glColor3f(1, 0,0);
		gl.glVertex3f(rayStart.x, rayStart.y, rayStart.z);
		gl.glColor3f(0,0,1);
		gl.glVertex3f(rayEnd.x, rayEnd.y, rayEnd.z);
		gl.glEnd();
		
		// if t value is sensible, draw hit point
		if (rayT >= 0 && rayT <= 1) {			
			gl.glBegin(GL2.GL_POINTS);
			gl.glColor3f(1, 0, 1);
			gl.glVertex3f(rayhitP.x, rayhitP.y, rayhitP.z);
			gl.glEnd();
			
			// if so, the normal must be valid too
			gl.glBegin(GL2.GL_LINES);
			gl.glColor3f(1, 0, 0);
			gl.glVertex3f(rayhitP.x, rayhitP.y, rayhitP.z);
			
			gl.glColor3f(0, 1, 1);
			gl.glVertex3f(rayhitP.x + rayhitN.x, rayhitP.y + rayhitN.y, rayhitP.z + rayhitN.z);
			gl.glEnd();
		}
		
		// if polygon is not null, draw
		if (hitPoly != null) {
			gl.glColor3f(0, 1, 1);
			hitPoly.debugDraw(gl);
		}
	}
}
