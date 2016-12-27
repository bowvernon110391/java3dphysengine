package com.bowie.javagl;

import com.bowie.javagl.Polytope.EPATriangle;
import com.jogamp.opengl.GL2;

public class EPAInfo {
	private Vector3 mtd;
	private Vector3 normal;
	private CSOVertex [] closest = new CSOVertex[3];
	private Vector3 barycentric;
	
	public EPAInfo(Vector3 vMtd, Vector3 vNormal, EPATriangle t, Vector3 bary) {
		mtd = vMtd;
		normal = vNormal;
		barycentric = bary;
		
		closest[0] = new CSOVertex(t.a);
		closest[1] = new CSOVertex(t.b);
		closest[2] = new CSOVertex(t.c);
		/*closest[0] = new Vector3(t.a);
		closest[1] = new Vector3(t.b);
		closest[2] = new Vector3(t.c);*/
	}
	
	public Vector3 calcContactA() {
		// just do by barycentric coord
		float x,y,z;
		x = closest[0].a.x * barycentric.x + closest[1].a.x * barycentric.y + closest[2].a.x * barycentric.z;
		y = closest[0].a.y * barycentric.x + closest[1].a.y * barycentric.y + closest[2].a.y * barycentric.z;
		z = closest[0].a.z * barycentric.x + closest[1].a.z * barycentric.y + closest[2].a.z * barycentric.z;
		
		return new Vector3(x, y, z);
	}
	
	public Vector3 calcContactB() {
		float x,y,z;
		x = closest[0].b.x * barycentric.x + closest[1].b.x * barycentric.y + closest[2].b.x * barycentric.z;
		y = closest[0].b.y * barycentric.x + closest[1].b.y * barycentric.y + closest[2].b.y * barycentric.z;
		z = closest[0].b.z * barycentric.x + closest[1].b.z * barycentric.y + closest[2].b.z * barycentric.z;
		
		return new Vector3(x, y, z);
	}

	public Vector3 getMtd() {
		return mtd;
	}

	public void setMtd(Vector3 mtd) {
		this.mtd = mtd;
	}

	public Vector3 getNormal() {
		return normal;
	}

	public void setNormal(Vector3 normal) {
		this.normal = normal;
	}

	public CSOVertex[] getClosest() {
		return closest;
	}

	public void setClosest(CSOVertex[] closest) {
		this.closest = closest;
	}
	
	public void drawDebug(GL2 gl) {
		CSOVertex a = closest[0];
		CSOVertex b = closest[1];
		CSOVertex c = closest[2];
		
		// draw the configuration space shit
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glColor3f(0.2f, 0.6f, 0.8f);
			gl.glVertex3f(a.p.x, a.p.y, a.p.z);
			gl.glVertex3f(b.p.x, b.p.y, b.p.z);
			gl.glVertex3f(c.p.x, c.p.y, c.p.z);
		gl.glEnd();
		
		// draw the object A's shit
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glColor3f(1.0f, 0.1f, 0.1f);
			gl.glVertex3f(a.a.x, a.a.y, a.a.z);
			gl.glVertex3f(b.a.x, b.a.y, b.a.z);
			gl.glVertex3f(c.a.x, c.a.y, c.a.z);
		gl.glEnd();
		
		// draw the object B's shit
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glColor3f(0.1f, 1.0f, 0.1f);
			gl.glVertex3f(a.b.x, a.b.y, a.b.z);
			gl.glVertex3f(b.b.x, b.b.y, b.b.z);
			gl.glVertex3f(c.b.x, c.b.y, c.b.z);
		gl.glEnd();
	}

	public Vector3 getBarycentric() {
		return barycentric;
	}

	public void setBarycentric(Vector3 barycentric) {
		this.barycentric = barycentric;
	}
}
