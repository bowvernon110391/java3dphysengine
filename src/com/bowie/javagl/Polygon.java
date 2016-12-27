package com.bowie.javagl;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GL2;

/**
 * Polygon - representing a polygon (containing points and normal)
 * the polygon also has to be co-planar and convex
 * @author Bowie
 *
 */
public class Polygon {
	public Vector3 n = new Vector3();
	public List<Vector3> p = new ArrayList<>();
	
	public Polygon() {
		// do nothing
	}
	
	public Polygon(Vector3 [] pts) {
		// initialize from set of points
		p.clear();
		for (Vector3 v : pts) {
			p.add(v);
		}
		calcNormal();
	}
	
	public Polygon(List<Vector3> pts) {
		// initialize from set of points
		p.clear();
		for (Vector3 v : pts) {
			p.add(v);
		}
		calcNormal();
	}
	
	public void addPoint(Vector3 pt) {
		p.add(new Vector3(pt));
	}
	
	public void setNormal(Vector3 normal) {
		n.setTo(normal);
	}
	
	/**
	 * this will clip against another polygon
	 * @param p	- this is the incidence
	 * @return
	 */
	public List<Vector3> clip(Polygon inc) {
		// make a plane.
		// ppos = point
		// pnormal = N X E --> points toward inside
		Vector3 pPos = new Vector3();
		Vector3 pNormal = new Vector3();
		Vector3 e = new Vector3();
		
		List<Vector3> pts = new ArrayList<>(inc.p);	// gotta copy
		
		int np = p.size();
		for (int i=0; i<p.size(); i++) {
			int j = (i+1) % np;
			// build an edge
			Vector3.sub(p.get(j), p.get(i), e);
			// calculate normal
			Vector3.cross(n, e, pNormal);
//			pNormal.normalize();
			// set plane position
			pPos.setTo(p.get(i));
			// clip em and reduce em
			pts = clip(pts, pPos, pNormal);
		}
		return pts;
	}
	
	/**
	 * clip - this one will clip a set of points against a plane
	 * @param pts
	 * @param pPos
	 * @param pNormal
	 * @return
	 */
	public static List<Vector3> clip(List<Vector3> pts, Vector3 pPos, Vector3 pNormal) {
		List<Vector3> result = new ArrayList<>();
		// loop over line segment
		int np = pts.size();
		for (int i=0; i<np; i++) {
			// ensure j index is wrapped
			int j = (i+1)%np;
			
			// store plane dist
			float dA = Vector3.planeDist(pPos, pNormal, pts.get(i));
			float dB = Vector3.planeDist(pPos, pNormal, pts.get(j));
			
			// sutherland clipping

			if (dA > 0 && dB > 0) {
				// both in front, store endpoint only
				result.add(pts.get(j));
			} else if (dA > 0 && dB < 0) {
				// a in front, b in back
				// store intersection point only
				Vector3 v = Vector3.intesectLinePlane(pts.get(i), pts.get(j), pPos, pNormal);
				result.add(v);
			} else if (dA < 0 && dB > 0) {
				// a in back, b in front
				// store intersection point, then endpoint
				Vector3 v = Vector3.intesectLinePlane(pts.get(i), pts.get(j), pPos, pNormal);
				result.add(v);
				result.add(pts.get(j));
			}
		}
		return result;
	}
	
	/**
	 * clip - perform sutherland clipping over a plane
	 * @param pPos
	 * @param pNormal
	 * @return clipped points
	 */
	public List<Vector3> clip(Vector3 pPos, Vector3 pNormal) {
		List<Vector3> result = new ArrayList<>();
		// loop over line segment
		int np = p.size();
		for (int i=0; i<np; i++) {
			// ensure j index is wrapped
			int j = (i+1)%np;
			
			// store plane dist
			float dA = Vector3.planeDist(pPos, pNormal, p.get(i));
			float dB = Vector3.planeDist(pPos, pNormal, p.get(j));
			
			// sutherland clipping

			if (dA > 0 && dB > 0) {
				// both in front, store endpoint only
				result.add(p.get(j));
			} else if (dA > 0 && dB < 0) {
				// a in front, b in back
				// store intersection point only
				Vector3 v = Vector3.intesectLinePlane(p.get(i), p.get(j), pPos, pNormal);
				result.add(v);
			} else if (dA < 0 && dB > 0) {
				// a in back, b in front
				// store intersection point, then endpoint
				Vector3 v = Vector3.intesectLinePlane(p.get(i), p.get(j), pPos, pNormal);
				result.add(v);
				result.add(p.get(j));
			}
		}
		return result;
	}
	
	public List<Vector3> generateContactPoints(Polygon inc) {
		// simply clip first
		List<Vector3> ctcs = this.clip(inc);
		
		// now clip point behind our normal
		Vector3 invN = n.inverse();
		
		return clip(ctcs, p.get(0), invN);
	}
	
	/**
	 * transform - transform all the data into world space
	 * @param pos	- worldspace pos
	 * @param rot	- worldspace rot
	 */
	public void transform(Vector3 pos, Quaternion rot) {
		Vector3 tmp = new Vector3();
		
		// loop over points
		for (int i=0; i<p.size(); i++) {
			rot.transformVector(p.get(i), tmp);
			// add it
			tmp.x += pos.x;
			tmp.y += pos.y;
			tmp.z += pos.z;
			// store it back
			p.get(i).setTo(tmp);
		}
		// transform normal
		rot.transformVector(n, tmp);
		n.setTo(tmp);
	}
	
	/**
	 * calcNormal - calculates the polygon normal
	 */
	public void calcNormal() {
		// provide a way to calculate normal
		// we must calculate normal using all points
		// storing longest normal along the way
		Vector3 tempN = new Vector3();
		
		Vector3 normal = new Vector3();
		float nLength = -1.0f;	// first timer
		
		int numPts = p.size();
		
		// loop over all points
		for (int i=0; i<numPts; i++) {
			int j = (i+1)%numPts;
			int k = (i+2)%numPts;
			// construct a triangle
			Vector3 AB = new Vector3();
			Vector3 BC = new Vector3();
			
			Vector3.sub(p.get(i), p.get(j), AB);
			Vector3.sub(p.get(j), p.get(k), BC);
			
			// cross product is normal
			Vector3.cross(AB, BC, tempN);
			
			// keep the longest normal
			float lSquare = tempN.lengthSquared();
			if (lSquare > nLength) {
				nLength = lSquare;
				normal.setTo(tempN);
			}
		}
		
		if (nLength > 0.0f) {
			// we got em. normalize
			normal.normalize();
			n.setTo(normal);
		}
	}
	
	public void debugDraw(GL2 gl) {
		gl.glBegin(GL2.GL_LINE_LOOP);
		for (Vector3 v : p)
			gl.glVertex3f(v.x, v.y, v.z);
		gl.glEnd();
	}
	
	public void debugDraw(GL2 gl, boolean wireframe) {
		if (p.size() == 0)
			return;
		
		// calculate mid point
		Vector3 tmp = new Vector3();
		for (Vector3 v : p) {
			tmp.x += v.x;
			tmp.y += v.y;
			tmp.z += v.z;
		}
		
		float div = (float)p.size();
		tmp.x /= div;
		tmp.y /= div;
		tmp.z /= div;
		
		// debug drawing
		int mode = GL2.GL_LINE_LOOP;
		
		if (!wireframe)
			mode = GL2.GL_POLYGON;
		
		// draw main shape
		gl.glBegin(mode);
			for (Vector3 v : p)
				gl.glVertex3f(v.x, v.y, v.z);
		gl.glEnd();
		
		// draw normal
		gl.glBegin(GL2.GL_LINES);
			gl.glColor3f(0, 1, 0);
			
			gl.glVertex3f(tmp.x, tmp.y, tmp.z);
			gl.glVertex3f(tmp.x+n.x, tmp.y+n.y, tmp.z+n.z);
		gl.glEnd();
		
		// draw mid point?
		gl.glBegin(GL2.GL_POINTS);
			gl.glColor3f(1, 1, 0);
			gl.glVertex3f(tmp.x, tmp.y, tmp.z);
		gl.glEnd();
	}
}
