package com.bowie.javagl;

import java.util.Iterator;
import java.util.Vector;

import com.jogamp.opengl.GL2;

public class Polytope {
	static final private float debugAlpha = 0.852f;
	static final public float [][] colors = new float [][] {
		{1, 0, 0, debugAlpha},
		{0, 1, 0, debugAlpha},
		{0, 0, 1, debugAlpha},
		{1, 1, 0, debugAlpha},
		{0, 1, 1, debugAlpha},
		{1, 0, 1, debugAlpha},
		{1, 0.5f, 0.5f, debugAlpha},
		{0.5f, 1, 0.5f, debugAlpha},
		{0.5f, 0.5f, 1, debugAlpha},
		{1, 1, 0.5f, debugAlpha},
		{0.5f, 1, 1, debugAlpha},
		{1, 0.5f, 1, debugAlpha}
	};
	
	static public float [] getColor(int hash) {
		return colors[hash % colors.length];
	}
	
	// hold triangles
	private Vector<EPATriangle> tris;
	
	public Polytope(Simplex s) {
		init(s);
	}
	
	/**
	 * initialize our polytope
	 * @param s - simplex returned by GJK (must contain origin and is a tetrahedron)
	 */
	public void init(Simplex s) {
		new Vector<>();
		tris = new Vector<EPATriangle>();
		
		// make sure it's 4 in the beginning, or else it would fail
		CSOVertex a = s.getSupport(0);
		CSOVertex b = s.getSupport(1);
		CSOVertex c = s.getSupport(2);
		CSOVertex d = s.getSupport(3);
		
		// add new triangle on the way
		tris.add(new EPATriangle(a, b, c));	// ABC
		tris.add(new EPATriangle(d, b, a));	// DBA
		tris.add(new EPATriangle(d, a, c));	// DAC
		tris.add(new EPATriangle(d, c, b));	// DCB
	}
	
	/**
	 * this does the necessary bookkeeping when we're expanding our
	 * polytope. it removes visible faces, keeping edges to be rebuilt
	 * @param p - the new point. it MUST BE OUTSIDE the polytope
	 */
	public void addPoint(CSOVertex p) {
//		System.out.println("adding: " + p.x + "," + p.y+"," +p.z);
		// holds our edge list
		EdgeList el = new EdgeList();
		// first, we gotta remove all triangles visible from our p
		Iterator<EPATriangle> iter = tris.iterator();
		
		Vector3 tmp = new Vector3();
		while (iter.hasNext()) {
			EPATriangle t = iter.next();
			
			// normal is calculated already
			Vector3.sub(p.p, t.a.p, tmp);
			float dist = Vector3.dot(tmp, t.n);
//			System.out.println("dist: " + dist);
			// it's VISIBLE!! REMOVE KEBAAAB!!!
			if (dist >= 0) {
				// add to edge list
				el.addTriangle(t);
				// remove from our triangle
				iter.remove();
			}
		}
		
//		System.out.println("now tri_count: " + tris.size());
		
		// at this point, all visible triangles are culled away
		// now we're safe to rebuild a triangle fan sharing common center
		// let's see how much edges we got
		el.buildFan(p, tris);
	}
	
	/**
	 * debug drawing function
	 * @param gl
	 */
	public void debugDraw(GL2 gl) {
		// we debug draw all triangles
		int i=0;
		
//		synchronized (tris) {
			for (EPATriangle t : tris) {
				float [] triCol = getColor(i++);
				
				gl.glColor4f(triCol[0], triCol[1], triCol[2], triCol[3]);
				gl.glBegin(GL2.GL_TRIANGLES);
					gl.glVertex3f(t.a.a.x, t.a.a.y, t.a.a.z);
					gl.glVertex3f(t.b.a.x, t.b.a.y, t.b.a.z);
					gl.glVertex3f(t.c.a.x, t.c.a.y, t.c.a.z);
				gl.glEnd();
				
				triCol = getColor(i++);
				gl.glColor4f(triCol[0], triCol[1], triCol[2], triCol[3]);
				gl.glBegin(GL2.GL_TRIANGLES);
					gl.glVertex3f(t.a.b.x, t.a.b.y, t.a.b.z);
					gl.glVertex3f(t.b.b.x, t.b.b.y, t.b.b.z);
					gl.glVertex3f(t.c.b.x, t.c.b.y, t.c.b.z);
				gl.glEnd();
				
//				// draw normal
//				triCol = getColor(i);
//				gl.glColor3f(triCol[0], triCol[1], triCol[2]);
//				gl.glBegin(GL2.GL_LINES);
//					gl.glVertex3f(t.mid.x, t.mid.y, t.mid.z);
//					gl.glVertex3f(t.mid.x+t.n.x, t.mid.y+t.n.y, t.mid.z+t.n.z);
//				gl.glEnd();
			}
//		}
		
	}
	
	/**
	 * return closest point from said point
	 * @param p
	 * @return
	 */
	public Vector3 getClosestPoint(Vector3 p) {
		return null;
	}
	
	/**
	 * this function gets closest triangle, also the mtd will contain its projection of p
	 * @param p		- point of query
	 * @param mtd	- projected p on the closest triangle
	 * @return	- the closest triangle
	 */
	public EPATriangle getClosestTriangle(Vector3 p, Vector3 projected) {
		// we will look for closest triangle
		if (tris.size() < 1)
			return null;
		
		// grab from first triangle
		EPATriangle t = tris.get(0);
		
		// get projected p on the triangle
		Vector3 proj = MathHelper.getClosestToTriangle(p, t.a.p, t.b.p, t.c.p);
		float dist = Vector3.lengthSquared(p, proj);
		
		projected.setTo(proj);
		
		// refine our result so far
		for (int i=1; i<tris.size(); i++) {
			EPATriangle tr = tris.get(i);
			proj = MathHelper.getClosestToTriangle(p, tr.a.p, tr.b.p, tr.c.p);
			float d = Vector3.lengthSquared(p, proj);
			
			if (d < dist) {
				dist = d;
				t = tr;
				projected.setTo(proj);
			}
		}
		return t;
	}
	
	/**
	 * This will hold our EPA Face 
	 * That makes up the polytope
	 * @author Bowie
	 *
	 */
	public class EPATriangle {
		public CSOVertex a, b, c;
		
		public Vector3 mid = new Vector3();
		public Vector3 n = new Vector3();
		
		public EPATriangle(CSOVertex a_, CSOVertex b_, CSOVertex c_) {
			a=a_;
			b=b_;
			c=c_;
			
			calcDrawData();
		}
		
		public void calcDrawData() {
			Vector3.add(a.p, b.p, mid);
			Vector3.add(mid, c.p, mid);
			mid.scale(0.3333f);
			
			Vector3.cross(new Vector3(b.p.x-a.p.x,b.p.y-a.p.y,b.p.z-a.p.z), 
					new Vector3(c.p.x-a.p.x,c.p.y-a.p.y,c.p.z-a.p.z), n);
//			n.normalize();
		}
	}
	
	/**
	 * This will hold our EPA Edge
	 * used when removing and adding triangle
	 * @author Bowie
	 *
	 */
	public class EPAEdge {
		public CSOVertex a, b;
		
		public EPAEdge(CSOVertex a2, CSOVertex b2) {
			a = a2;
			b = b2;
		}

		public boolean equal(EPAEdge e) {
			// compare reference should be enough, but I'm not sure
			return (a.equals(e.b) && b.equals(e.a)) || (a == e.b && b == e.a);
		}
	}
	
	/**
	 * this class manages Edge list when we're removing triangle
	 * it also builds  the new triangles
	 * @author Bowie
	 *
	 */
	public class EdgeList {
		private Vector<EPAEdge> el = new Vector<Polytope.EPAEdge>();
		
		public void addTriangle(EPATriangle t) {
			addEdge(new EPAEdge(t.a, t.b));	// AB
			addEdge(new EPAEdge(t.b, t.c));	// BC
			addEdge(new EPAEdge(t.c, t.a));	// CA
		}
		
		public void addEdge(EPAEdge e) {
			// first check if it's in our list
			for (EPAEdge oe : el) {
				if (e.equal(oe)) {
					// remove, and exit (to avoid adding shit)
					el.remove(oe);
					return;
				}
			}
			// safe
			el.add(e);
		}
		
		/**
		 * builds a triangle fan, sharing a common center
		 * @param common	- the common point
		 * @param tris		- the triangle list to be modified
		 */
		public void buildFan(CSOVertex common, Vector<EPATriangle> tris) {
			for (EPAEdge e : el) {
				tris.add(new EPATriangle(common, e.a, e.b));
			}
		}
	}
}
