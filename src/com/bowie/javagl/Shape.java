package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public abstract class Shape {
	protected int id = 0;
	
	abstract public void render(GL2 gl);
	
	abstract public void render(GL2 gl, Vector3 pos, Quaternion ori);
	/**
	 * supportPoint
	 * @param worldDir	the direction of search (in World)
	 * @param worldPos	the position of the shape (in World)
	 * @param worldOri	the orientation of the shape (in World)
	 * @return the support point in world reference
	 */
	abstract public Vector3 supportPoint(Vector3 worldDir, Vector3 worldPos, Quaternion worldOri);
	
	public Polygon getFace(Vector3 dir, Vector3 worldPos, Quaternion worldRot) {
		return null;
	}
	
	abstract public void getAABB(Vector3 worldPos, Quaternion worldOri, AABB oldbbox);
	
	/**
	 * getInvInertiaTensor - calculate the inverse inertia
	 * @param invMass	- the inverse mass for multiplier
	 * @return the 3x3 inverse inertia tensor
	 */
	abstract public Matrix3 getInvInertiaTensor(float invMass);
	
	/**
	 * minkowskiDiff - perform minkowski difference along a direction 
	 * @param sA
	 * @param posA
	 * @param rotA
	 * @param sB
	 * @param posB
	 * @param rotB
	 * @param worldDir
	 * @return the minkowski difference
	 */
	static public CSOVertex minkowskiDiff(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, Vector3 worldDir) {
		// generate negation of dir
		Vector3 supA = sA.supportPoint(worldDir, posA, rotA);
		Vector3 supB = sB.supportPoint(worldDir.inverse(), posB, rotB);
		
		return new CSOVertex(supA, supB);
	}
	
	/**
	 * getFeature - return the outmost feature, suitable for contact generation
	 * @param dir - the search direction (in local frame)
	 * @return return the outmost feature according to dir
	 */
	abstract public FeatureEdge getFeature(Vector3 dir); 
	
	public static FeatureEdge [] getFeaturePair(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, Vector3 dir) {
		
		Vector3 localDir = new Vector3();
		
		FeatureEdge fA, fB;
		// SHAPE A Because dir is already pointing away from A
		// create local dir		
		rotA.conjugated().transformVector(dir, localDir);
		fA = sA.getFeature(localDir);
		fA.transform(posA, rotA);
		
		// dir is pointing to sA, so reverse
		dir.scale(-1.0f);
		// SHAPE B 
		rotB.conjugated().transformVector(dir, localDir);
		fB = sB.getFeature(localDir);
		fB.transform(posB, rotB);
		
		
		FeatureEdge [] ret = new FeatureEdge[2];
		ret[0] = fA;
		ret[1] = fB;
		return ret;
		
	}
	
	private static Vector3 [] clip(Vector3 ppos, Vector3 pnorm, Vector3[] pts) {
		if (pts.length < 2)
			return null;	// Can't do!!
		
		// allright, let's modify the point
		float d0 = Vector3.planeDist(ppos, pnorm, pts[0]);
		float d1 = Vector3.planeDist(ppos, pnorm, pts[1]);
		
		Vector3 a = pts[0], b = pts[1];
		
		System.out.println("clip start: ");
		if (d0 < Vector3.EPSILON && d1 < Vector3.EPSILON) {
			System.out.println("both behind!");
			// both are behind the plane, give it that
			a = pts[0];
			b = pts[1];
		} else if (d1 < Vector3.EPSILON && d0 >= Vector3.EPSILON) {
			System.out.println("clipAB, b");
			// b is behind, a is in front
			// a is clipped
			a = Vector3.intesectLinePlane(pts[0], pts[1], ppos, pnorm);
			// b is as it is
			b = pts[1];	
		} else if (d0 < Vector3.EPSILON && d1 >= Vector3.EPSILON) {
			System.out.println("a, clipAB");
			// a is behind, b is in front
			// a is as it is
			a = pts[0];
			// b is clipped
			b = Vector3.intesectLinePlane(pts[0], pts[1], ppos, pnorm);
		}
		System.out.println("clip end: ");
		
		pts[0] = new Vector3(a);
		pts[1] = new Vector3(b);
		
		return pts;
	}
	
	/**
	 * clip - clip incident edge against reference edge
	 * @param inc
	 * @param ref
	 * @return clipped points, in CCW order (inc modified)
	 */
	public static Vector3 [] clip(FeatureEdge inc, FeatureEdge ref) {
		// build initial edges
		Vector3 [] pts = new Vector3 []{
				new Vector3(inc.e[0]),
				new Vector3(inc.e[1])
		};
		
		// must build 3 planes
		Vector3 ppos = ref.e[0];
		Vector3 pnorm = ref.n;
		
		// 1st, test with default planes
		pts = clip(ppos, pnorm, pts);
		
		// 2nd, ppos = 1st edge vert, pnorm = pnorm rotated -90
		pnorm = new Vector3(ref.n.y, -ref.n.x, 0);
		pts = clip(ppos, pnorm, pts);
		
		// 3rd, ppos = 2nd edge vert, pnorm = pnorm rotated 90
		ppos = ref.e[1];
		pnorm = new Vector3(-ref.n.y, ref.n.x, 0);
		pts = clip(ppos, pnorm, pts);
		
		return pts;
	}
	
	/**
	 * FeatureEdge - represents an edge 'feature'
	 * @author Bowie
	 * the edge must be CCW
	 */
	public class FeatureEdge {
		public Vector3 [] e = new Vector3[2];
		public Vector3 n;
		
		public FeatureEdge(Vector3 e1, Vector3 e2) {
			// copy vertices (CCW)
			e[0] = new Vector3(e1);
			e[1] = new Vector3(e2);
			
			// let's calculate normal (it's edge direction ROTATED CW 90)
			float ex = e2.x-e1.x;
			float ey = e2.y-e1.y;
			
			float tmp = ex;
			ex = ey;
			ey = -tmp;
			n = new Vector3(ex, ey, 0).normalized();
		}
		
		// debug drawing
		public void debugDraw(GL2 gl) {
			// calculate midpoint
			float mx = (e[0].x + e[1].x) * 0.5f;
			float my = (e[0].y + e[1].y) * 0.5f;
			float mz = (e[0].z + e[1].z) * 0.5f;
			
			// calculate end point
			float ex = mx + n.x;
			float ey = my + n.y;
			float ez = mz + n.z;
			
			// draw
			gl.glBegin(GL2.GL_LINES);
				// draw edge first
				gl.glColor3f(0.5f, 0.2f, 1.0f);
				gl.glVertex3f(e[0].x, e[0].y, e[0].z);
				gl.glVertex3f(e[1].x, e[1].y, e[1].z);
				
				// draw normal
				gl.glColor3f(0.1f, 0.62f, 1.0f);
				gl.glVertex3f(mx, my, mz);
				gl.glVertex3f(ex, ey, ez);
			gl.glEnd();
		}
		
		// transform to world space
		public void transform(Vector3 pos, Quaternion rot) {
			Vector3 tE1 = new Vector3();
			Vector3 tE2 = new Vector3();
			
			rot.transformVector(e[0], tE1);
			rot.transformVector(e[1], tE2);
			
			Vector3.add(tE1, pos, tE1);
			Vector3.add(tE2, pos, tE2);
			
			// set to new
			e[0] = tE1;
			e[1] = tE2;
			
			// re-calculate normal
			rot.transformVector(n, n);
		}
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}
}
