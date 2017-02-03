package com.bowie.javagl;

import com.jogamp.opengl.GL2;

/**
 * AABB - represents a bounding volume
 * @author Bowie
 *
 */
public class AABB {
	public Vector3 min = new Vector3();
	public Vector3 max = new Vector3();
	
	/**
	 * overlap - check if two AABB overlap
	 * @param b	- the other AABB
	 * @return true if overlap, false otherwise
	 */
	public boolean overlap(AABB b) {
		return min.x <= b.max.x && min.y <= b.max.y && min.z <= b.max.z
				&& max.x >= b.min.x && max.y >= b.min.y && max.z >= b.min.z; 
	}
	
	public void move(Vector3 m) {
		Vector3.add(min, m, min);
		Vector3.add(max, m, max);
	}
	
	public void encompass(Vector3 a, Vector3 b, boolean init) {
		// this will try to encompass both vectors
		if (init) {
			// this means aabb is unitialized
			min.x	= Math.min(a.x, b.x);
			min.y	= Math.min(a.y, b.y);
			min.z	= Math.min(a.z, b.z);
			
			max.x	= Math.max(a.x, b.x);
			max.y	= Math.max(a.y, b.y);
			max.z	= Math.max(a.z, b.z);
		} else {
			// this means aabb is already initialized, 
			// so we only need to expand when necessary
			min.x	= Math.min(min.x, Math.min(a.x, b.x));
			min.y	= Math.min(min.y, Math.min(a.y, b.y));
			min.z	= Math.min(min.z, Math.min(a.z, b.z));
			
			max.x	= Math.max(max.x, Math.max(a.x, b.x));
			max.y	= Math.max(max.y, Math.max(a.y, b.y));
			max.z	= Math.max(max.z, Math.max(a.z, b.z));
		}
	}
	
	/**
	 * grow - grow the AABB by a scalar d
	 * @param d	- how much to expand
	 */
	public void grow(float d) {
		min.x -= d;
		min.y -= d;
		min.z -= d;
		
		max.x += d;
		max.y += d;
		max.z += d;
	}
	
	/**
	 * grow - grow it using the vector v
	 * @param v
	 */
	public void grow(Vector3 v) {
		if (v.x > 0)
			max.x += v.x;
		else
			min.x += v.x;
		
		if (v.y > 0)
			max.y += v.y;
		else
			min.y += v.y;
		
		if (v.z > 0)
			max.z += v.z;
		else
			min.z += v.z;
	}
	
	/**
	 * grow by another aabb
	 * @param b
	 */
	public void grow(AABB b) {
		// grow by new aabb
		Vector3 bmax = b.max;
		Vector3 bmin = b.min;
		
		if (min.x < bmin.x) min.x = bmin.x;
		if (min.y < bmin.y) min.y = bmin.y;
		if (min.z < bmin.z) min.z = bmin.z;
		
		if (max.x > bmax.x) max.x = bmax.x;
		if (max.y > bmax.y) max.y = bmax.y;
		if (max.z > bmax.z) max.z = bmax.z;
	}
	
	public void debugDraw(GL2 gl) {
		
		// front face
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glVertex3f(min.x, min.y, max.z);
			gl.glVertex3f(max.x, min.y, max.z);
			gl.glVertex3f(max.x, max.y, max.z);
			gl.glVertex3f(min.x, max.y, max.z);
		gl.glEnd();
		
		// back face
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glVertex3f(max.x, min.y, min.z);
			gl.glVertex3f(min.x, min.y, min.z);
			gl.glVertex3f(min.x, max.y, min.z);
			gl.glVertex3f(max.x, max.y, min.z);
		gl.glEnd();
		
		// ridges in-between
		gl.glBegin(GL2.GL_LINES);
			gl.glVertex3f(min.x, min.y, min.z);
			gl.glVertex3f(min.x, min.y, max.z);
			
			gl.glVertex3f(max.x, min.y, min.z);
			gl.glVertex3f(max.x, min.y, max.z);
			
			gl.glVertex3f(max.x, max.y, min.z);
			gl.glVertex3f(max.x, max.y, max.z);
			
			gl.glVertex3f(min.x, max.y, min.z);
			gl.glVertex3f(min.x, max.y, max.z);
		gl.glEnd();
	}
}
