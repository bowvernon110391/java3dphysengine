package com.bowie.javagl;

import java.util.Vector;

import com.jogamp.opengl.GL2;

/**
 * Simplex - class representing a simplex (collection of points)
 * @author Bowie
 *
 */
public class Simplex {
	private Vector<CSOVertex> points;
	public Vector3 lastDir = new Vector3(1, 0, 0);
	
	public Simplex() {
		points = new Vector<>(4);
	}
	
	public void reset() {
		points.clear();
	}
	
	public void addSupport(CSOVertex v) {
		points.add(v);
	}
	
	public void removeLast() {
		if (points.size() == 4) {
			// gotta remove the vertex with lowest barycentric coord
			Quaternion bary = MathHelper.computeBarycentric(Vector3.ZERO, points.get(0).p, points.get(1).p, 
					points.get(2).p, points.get(3).p);
			
			if (bary.x <= 0) {
				points.remove(0);
			} else if (bary.y <= 0) {
				points.remove(1);
			} else if (bary.z <= 0) {
				points.remove(2);
			} else {
				points.remove(3);
			}
		}
	}
	
	public void addSupportConservatively(CSOVertex v) {
		// gotta add, while removing unnecessary vertex
		if (points.size() >= 4) {
			System.out.println("removing shit simplex");
			// gotta remove when we're full
			CSOVertex a, b, c, d;
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			
			// which vertex is redundant?
			Quaternion bary = MathHelper.computeBarycentric(v.p, a.p, b.p, c.p, d.p);
			
			System.out.println("bary: " + bary.x+", "+bary.y+", "+bary.z+", "+bary.w);
			
			if (bary.x <= Vector3.EPSILON) {
				// a
				points.remove(a);
			} else if (bary.y <= Vector3.EPSILON) {
				// b
				points.remove(b);
			} else if (bary.z <= Vector3.EPSILON) {
				// c
				points.remove(c);
			} else if (bary.w <= Vector3.EPSILON){
				// force d
				points.remove(d);
			} else {
				// contain origin, return
				System.out.println("contain origin> shiiit");
				return;
			}
			// otherwise, it might be inside, which makes the caller a stupid person
		} else {
			System.out.println("normal add");
		}
		
		// safe to add
		points.add(v);
	}
	
	public Vector3 closestToOrigin() {
		// gotta give the closest CSOVertex to origin
		
		if (points.size() == 1) {
			return new Vector3(points.firstElement().p);
		} else if (points.size() == 2) {
			// return closest to line segment
			return MathHelper.getClosestToLine(Vector3.ZERO, points.get(0).p, points.get(1).p, true);
		} else if (points.size() == 3) {
			// return closest to triangle
			return MathHelper.getClosestToTriangle(Vector3.ZERO, points.get(0).p, points.get(1).p, points.get(2).p);
		} else if (points.size() == 4) {
			//return closest to tetrahedron
			return MathHelper.getClosestToTetrahedron(Vector3.ZERO, points.get(0).p, points.get(1).p, 
					points.get(3).p, points.get(2).p);
		}
		System.out.println("Whaaat!!!" + points.size());
		return null;
	}

	
	public boolean getClosestPoint(Vector3 bA, Vector3 bB) {
		// depending on the simplex size
		CSOVertex a, b, c, d;
		if (points.size() == 1) {
			System.out.println("point case!");
			// simplest one
			a = points.get(0);
			
			bA.setTo(a.a);
			bB.setTo(a.b);
			
			return true;
		} else if (points.size() == 2) {
			System.out.println("line case!");
			// calculate our fraction on the line
			a = points.get(0);
			b = points.get(1);
			
			Vector3 u = a.p.inverse();
			Vector3 v = new Vector3();
			Vector3.sub(b.p, a.p, v);
			
			float uv = Vector3.dot(u, v);
			float vv = Vector3.dot(v, v);
			
			if (Math.abs(vv) < Vector3.EPSILON) {
				// parallel cuk
				bA.setTo(a.a);
				bB.setTo(a.b);
				return true;
			}
			
			// good number
			uv /= vv;
			
			// clamp them (usually not needed)
			if (uv <= 0) {
				// on a
				bA.setTo(a.a);
				bB.setTo(a.b);
			} else if (uv >= 1) {
				// on b
				bA.setTo(b.a);
				bB.setTo(b.b);
			} else {
				// in between
				Vector3 v1 = a.a;
				Vector3 v2 = b.a;
				
				// for A
				bA.x = (1.0f-uv) * v1.x + uv * v2.x;
				bA.y = (1.0f-uv) * v1.y + uv * v2.y;
				bA.z = (1.0f-uv) * v1.z + uv * v2.z;
				
				// for B
				v1 = b.b;
				v2 = b.b;
				
				bB.x = (1.0f-uv) * v1.x + uv * v2.x;
				bB.y = (1.0f-uv) * v1.y + uv * v2.y;
				bB.z = (1.0f-uv) * v1.z + uv * v2.z;
			}
			
			return true;
		} else if (points.size() == 3) {
			System.out.println("triangle case!");
			// from a triangle!!
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);

			// grab closest to it
			Vector3 closest = MathHelper.getClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p);
			// compute barycentric coordinate
			Vector3 bary = MathHelper.computeBarycentric(closest, a.p, b.p, c.p);
			
			// use it to get real coords
			Vector3 v1, v2, v3;
			
			// On A
			v1 = a.a;
			v2 = b.a;
			v3 = c.a;
			
			bA.x = v1.x * bary.x + v2.x * bary.y + v3.x * bary.z;
			bA.y = v1.y * bary.x + v2.y * bary.y + v3.y * bary.z;
			bA.z = v1.z * bary.x + v2.z * bary.y + v3.z * bary.z;
			
			// On B
			v1 = a.b;
			v2 = b.b;
			v3 = c.b;
			
			bB.x = v1.x * bary.x + v2.x * bary.y + v3.x * bary.z;
			bB.y = v1.y * bary.x + v2.y * bary.y + v3.y * bary.z;
			bB.z = v1.z * bary.x + v2.z * bary.y + v3.z * bary.z;
			
			return true;
		} else if (points.size() == 4) {
			System.out.println("tetrahedron case!");
			// usually this is it. gotta get closest on the tetrahedron
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			
			Vector3 closest = MathHelper.getClosestToTetrahedron(Vector3.ZERO, a.p, b.p, c.p, d.p);
			Quaternion bary = MathHelper.computeBarycentric(closest, a.p, b.p, c.p, d.p);
			
			
			System.out.println("bary: " + bary.x + ", " + bary.y + ", " + bary.z + ", " + bary.w);
			
//			return false;
			bA.x = a.a.x * bary.x + b.a.x * bary.y + c.a.x * bary.z + d.a.x * bary.w;
			bA.y = a.a.y * bary.x + b.a.y * bary.y + c.a.y * bary.z + d.a.y * bary.w;
			bA.z = a.a.z * bary.x + b.a.z * bary.y + c.a.z * bary.z + d.a.z * bary.w;
			
			bB.x = a.b.x * bary.x + b.b.x * bary.y + c.b.x * bary.z + d.b.x * bary.w;
			bB.y = a.b.y * bary.x + b.b.y * bary.y + c.b.y * bary.z + d.b.y * bary.w;
			bB.z = a.b.z * bary.x + b.b.z * bary.y + c.b.z * bary.z + d.b.z * bary.w;
			
			return true;
		}
		return false;
	}
	
	public CSOVertex getLast() {
		return points.lastElement();
	}
	
	public void debugDraw(GL2 gl) {		
		// now draw the line loop
		if (points.size() == 2) {
			Vector3 a = points.get(0).a;
			Vector3 b = points.get(1).a;
			
			gl.glBegin(GL2.GL_LINES);
				gl.glVertex3f(a.x, a.y, a.z);
				gl.glVertex3f(b.x, b.y, b.z);
			gl.glEnd();
			
			a = points.get(0).b;
			b = points.get(1).b;
			
			gl.glBegin(GL2.GL_LINES);
				gl.glVertex3f(a.x, a.y, a.z);
				gl.glVertex3f(b.x, b.y, b.z);
			gl.glEnd();
		} else if (points.size() >=3 ) {
			// draw line loop
			Vector3 a = points.get(0).a;
			Vector3 b = points.get(1).a;
			Vector3 c = points.get(2).a;
			
			gl.glBegin(GL2.GL_LINE_LOOP);
				gl.glVertex3f(a.x, a.y, a.z);
				gl.glVertex3f(b.x, b.y, b.z);
				gl.glVertex3f(c.x, c.y, c.z);
			gl.glEnd();
			
			a = points.get(0).b;
			b = points.get(1).b;
			c = points.get(2).b;
			
			gl.glBegin(GL2.GL_LINE_LOOP);
				gl.glVertex3f(a.x, a.y, a.z);
				gl.glVertex3f(b.x, b.y, b.z);
				gl.glVertex3f(c.x, c.y, c.z);
			gl.glEnd();
		} 
		
		if (points.size() == 4) {
			// draw ridges
			Vector3 a = points.get(0).a;
			Vector3 b = points.get(1).a;
			Vector3 c = points.get(2).a;
			Vector3 d = points.get(3).a;
			
			gl.glBegin(GL2.GL_LINES);
				gl.glVertex3f(a.x, a.y, a.z);
				gl.glVertex3f(d.x, d.y, d.z);
				
				gl.glVertex3f(b.x, b.y, b.z);
				gl.glVertex3f(d.x, d.y, d.z);
				
				gl.glVertex3f(c.x, c.y, c.z);
				gl.glVertex3f(d.x, d.y, d.z);
			gl.glEnd();
			
			a = points.get(0).b;
			b = points.get(1).b;
			c = points.get(2).b;
			d = points.get(3).b;
			
			
			gl.glBegin(GL2.GL_LINES);
				gl.glVertex3f(a.x, a.y, a.z);
				gl.glVertex3f(d.x, d.y, d.z);
				
				gl.glVertex3f(b.x, b.y, b.z);
				gl.glVertex3f(d.x, d.y, d.z);
				
				gl.glVertex3f(c.x, c.y, c.z);
				gl.glVertex3f(d.x, d.y, d.z);
			gl.glEnd();
		}
		
		float [][] color = new float[][] {
				{1,0,0},
				{0,1,0},
				{0,0,1},
				{0.5f,1,1}
			};
			gl.glBegin(GL2.GL_POINTS);
			for (int i=0; i<points.size(); i++) {
				Vector3 p = points.get(i).a;
				Vector3 q = points.get(i).b;
				gl.glColor3f(color[i][0], color[i][1], color[i][2]);
				gl.glVertex3f(p.x, p.y, p.z);
				gl.glVertex3f(q.x, q.y, q.z);
			}
			gl.glEnd();
	}
	
	public int size() {
		return points.size();
	}
	
	public CSOVertex getSupport(int idx) {
		return points.get(idx);
	}
	
	public boolean hasOrigin() {
		
		/*if (points.size() == 3) {
			// get closest
			Vector3 cp = MathHelper.getClosestToTriangle(new Vector3(), points.get(0), points.get(1), points.get(2));
			
			// is it inside?
			if (Vector3.dot(cp, cp) <= Vector3.EPSILON)
				return true;
		} */
		// cannot contain origin with triangle only, ignore degenerate line/triangle case
		// we're more interested in tetrahedron
		if (points.size() == 4) {
			// compute barycentric
			Quaternion bary = MathHelper.computeBarycentric(Vector3.ZERO, points.get(0).p, points.get(1).p, points.get(2).p, points.get(3).p);
			// it's inside if all is positive
			return bary.x > 0 && bary.y > 0 && bary.z > 0 && bary.w > 0;
		}
//		System.out.println("not having O: " + points.size());
		return false;
	}
	
	public Vector3 calcNewDir() {
		Vector3 newDir = null;
		// are we a triangle? or line
		/*if (points.size() == 3) {
			// triangle. Gotta remove on or two points, I guess?
			Vector3 a = points.get(0);
			Vector3 b = points.get(1);
			Vector3 c = points.get(2);
			// get closest point
			Vector3 cp = MathHelper.getClosestToTriangle(new Vector3(), a, b, c);
			// remove unnecessary point
			Vector3 pAB = MathHelper.getClosestToLine(new Vector3(), a, b, true);
			Vector3.sub(cp, pAB, Vector3.tmp0);
			if (Vector3.dot(Vector3.tmp0, Vector3.tmp0) <= Vector3.EPSILON) {
				// remove C
				points.remove(2);
			} else {
				Vector3 pBC = MathHelper.getClosestToLine(new Vector3(), b, c, true);
				Vector3.sub(cp, pBC, Vector3.tmp0);
				if (Vector3.dot(Vector3.tmp0, Vector3.tmp0) <= Vector3.EPSILON) {
					// remove A
					points.remove(0);
				} else {
					// remove B
					points.remove(1);
				}
			}
			// recalculate direction based on the new line
			a = points.get(0);
			b = points.get(1);
			// get projection on line, and return
			cp = MathHelper.getClosestToLine(new Vector3(), a, b, true);
			cp.scale(-1.0f);
			newDir = cp;
		} else if (points.size() >= 2) {
			// line. get projection
			Vector3 a = points.get(0);
			Vector3 b = points.get(1);
			// get projection on line, and return
			Vector3 cp = MathHelper.getClosestToLine(new Vector3(), a, b, true);
			cp.scale(-1.0f);
			newDir = cp;
		}*/
		
		// only test for negative voronoi region (because the vertex region is handled by gjk dotproduct already)
		// direction depends on how many vertex we have [2..4]
		// if 2 --> DIRECTION IS NEGATIVE PROJECTION
		// if 3 -->	DIRECTION IS NEGATIVE NORMAL
		// if 4 --> CHECK BARYCENTRIC, REMOVE NON-CONTRIBUTING POINT, RECALCULATE TRIANGLE
		Vector3 o = new Vector3();	// origin
		if (points.size() == 2) {
			// LINE CASE
			Vector3 proj = MathHelper.getClosestToLine(o, points.get(0).p, points.get(1).p, true);
			// NEGATE
			proj.scale(-1.0f);
			newDir = proj;
		} else if (points.size() == 3) {
			// TRIANGLE CASE
			Vector3 AB = new Vector3();
			Vector3 AC = new Vector3();
			Vector3 n = new Vector3();
			// For Academic Purpose!!
			CSOVertex a = points.get(0);
			CSOVertex b = points.get(1);
			CSOVertex c = points.get(2);
			
			// compute normal (RAW)
			Vector3.sub(b.p, a.p, AB);
			Vector3.sub(c.p, a.p, AC);
			Vector3.cross(AB, AC, n);
			
			// check sign
			float dp = Vector3.dot(a.p, n);
			
			if (dp < 0 /*= Vector3.EPSILON*/) {
				// TRIANGLE IS FRONT FACING ORIGIN!!
				// FLIP ORDER
				points.remove(c);
				points.remove(b);
				
				points.add(c);
				points.add(b);
				// DIRECTION IS FINE
			} else {
				// TRIANGLE IS BACK FACING ORIGIN <GOOOD>
				// REVERSE DIRECTION THOUGH
				n.scale(-1.0f);
			}
			newDir = n;
		} else if (points.size() == 4) {
			// TETRAHDERON CASE
			// 1. compute barycentric
			// 2. analyze voronoi only (vertex region is handled by gjkColDet already)
			// 3. remove non-contributing vertex
			// 4. calculate triangle normal, reversing when necessary blabla
			
			CSOVertex a = points.get(0);
			CSOVertex b = points.get(1);
			CSOVertex c = points.get(2);
			CSOVertex d = points.get(3);
			
			Quaternion bary = MathHelper.computeBarycentric(o, a.p, b.p, c.p, d.p);
			
			// analyze Voronoi region
			// PS: if we reach here, that means the current tetrahedron doesn't contain origin (safe)
			if (bary.x < 0) {
				points.remove(a);
			} else if (bary.y < 0) {
				points.remove(b);
			} else if (bary.z < 0) {
				points.remove(c);
			} else {
				points.remove(d);
			}
			
			// now it's a triangle, recurse simply
			return calcNewDir();
		}
		return newDir;
	}
}
