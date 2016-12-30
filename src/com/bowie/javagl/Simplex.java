package com.bowie.javagl;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.jogamp.opengl.GL2;

/**
 * Simplex - class representing a simplex (collection of points)
 * @author Bowie
 *
 */
public class Simplex {
	private List<CSOVertex> points;
	public Vector3 lastDir = new Vector3(1, 0, 0);
	
	
	//=============STRICTLY TEST DATA=====================================
	public Shape sA, sB;
	public Vector3 posA, posB;
	public Quaternion rotA, rotB;
	
	public int iter = 0;	// the first iteration
	public boolean converged = false;
	public float closestDist = Float.MAX_VALUE;
	public Vector3 currentDir = new Vector3(1, 0, 0);
	public Vector3 closestPointToOrigin = null;
	
	public Vector3 cA, cB;
	//=============STRICTLY TEST DATA=====================================
	
	
	public Simplex(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, Vector3 initDir) {
		this();
		
		// then do another things
		this.sA	= sA;
		this.sB	= sB;
		this.posA = posA;
		this.posB = posB;
		this.rotA = rotA;
		this.rotB = rotB;
		
		// make sure direction is valid
		if (initDir.lengthSquared() > Vector3.EPSILON)
			this.currentDir.setTo(initDir);
		else
			this.currentDir.setTo(1,0,0);
		
		iter = 0;
		closestDist = Float.MAX_VALUE;
		converged = false;
		closestPointToOrigin = null;
		cA = null;
		cB = null;
		
		// add 2 points
		this.addSupportConservatively(grabMinkowskiDiff());
		// flip
		this.currentDir.scale(-1.0f);
	}
	
	public CSOVertex grabMinkowskiDiff() {
		// based on current direction..find it
		return Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, currentDir);
	}
	
	public boolean advance() {
		// do not do anything if we have converged
		if (converged)
			return true;
		// increment counter
		System.out.println("SIMPLEX.ADVANCE -- " + iter);
		iter++;
		// we add point
		this.addSupportConservatively(grabMinkowskiDiff());
		// now we find closest from origin..but we must check first if we're degenerate
		if (isDegenerate()) {
			// warn user
			System.out.println("SIMPLEX WARNING!! DEGENERATE!!! ANY DIRECTION OR DISTANCE MIGHT NOT BE VALID NOW!! @ " + points.size());
		}
		
		// we have at least 2 point, so simply add shit based on closest direction
		closestPointToOrigin = closestToOrigin();
		float d = closestPointToOrigin.lengthSquared();
		
		// check if we're close
		if (Math.abs(closestDist - d) < MathHelper.GJK_MARGIN_ERROR) {
			System.out.printf("SIMPLEX CONVERGED WITH SEP_DIST: %f%n", Math.abs(closestDist - d));
			converged = true;
			
			// make sure our simplex isn't degenerate
			ceaseDegeneracy();
			
			// compute closest point
			cA = new Vector3();
			cB = new Vector3();
			
			this.getClosestPoint(cA, cB);
			
			return true;
		}
		
		// Gotta warn too if distance is shit
		if (d > closestDist) {
			System.out.println("SIMPLEX WARNING!! RECENT DISTANCE IS FARTHER!!: " + d + " > " + closestDist);
		} else {
			System.out.printf("SIMPLEX MSG: updating closest distance from %.4f -> %.4f %n", closestDist, d);
			// we're getting better, update
			closestDist = d;
		}
		
		// welp, no luck. so simply refine direction
		// check if the direction is valid though
		if (d < Vector3.EPSILON)
			this.currentDir.setTo(calcNewDir());
		else
			this.currentDir.setTo(closestPointToOrigin.inverse());
		
		// we have not yet converged
		return false;
	}
	
	//=============STRICTLY TEST DATA=====================================
	
	public Simplex() {
		points = new ArrayList<>(4);
	}
	
	public void reset() {
		points.clear();
		
		//=============STRICTLY TEST DATA=====================================
		iter = 0;
		closestDist = Float.MAX_VALUE;
		currentDir.setTo(1, 0, 0);
		converged = false;
		closestPointToOrigin = null;
		cA = null;
		cB = null;
		//=============STRICTLY TEST DATA=====================================
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
	
	public void removeVertex(CSOVertex v) {
		points.remove(v);
	}
	
	public void addSupportConservatively(CSOVertex v) {
		// gotta add, while removing unnecessary vertex
		if (points.size() >= 4) {
			System.out.println("Simplex.asc: removing shit simplex");
			// gotta remove when we're full
			CSOVertex a, b, c, d;
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			
			// which vertex is redundant?
			
			// THIS WILL FAIL IF TETRAHEDRON IS DEGENERATE!!!
			// BUT IF IT'S THE CASE, THEN REMOVING ANY POINT IS FINE!!!
			Vector3 cp = MathHelper.naiveClosestToTetrahedron(Vector3.ZERO, a.p, b.p, c.p, d.p);
			Quaternion bary = MathHelper.computeBarycentric(cp, a.p, b.p, c.p, d.p);
			
			/*float [] dots = new float[]{
					-Vector3.dot(cp, a.p),
					-Vector3.dot(cp, b.p),
					-Vector3.dot(cp, c.p),
					-Vector3.dot(cp, d.p)
			};*/
			
			System.out.printf("Simplex.asc: bary: %.6f, %.6f, %.6f, %.6f%n", bary.x, bary.y, bary.z, bary.w);
//			System.out.printf("Simplex.asc: dots: %.6f, %.6f, %.6f, %.6f%n", dots[0], dots[1], dots[2], dots[3]);
			
			// remove the minima
			float maxD = Float.MAX_VALUE;
			CSOVertex r = null;
			
			if (bary.x < maxD) {
				maxD = bary.x;
				r = a;
			}
			
			if (bary.y < maxD) {
				maxD = bary.y;
				r = b;
			}
			
			if (bary.z < maxD) {
				maxD = bary.z;
				r = c;
			}
			
			if (bary.w < maxD) {
				maxD = bary.w;
				r = d;
			}
			
			if (r != null) {
				// gotta remove
				System.out.printf("SIMPLEX.ASC: gotta remove %d -> %f%n", points.indexOf(r), maxD);
				points.remove(r);
			} else {
				System.out.println("SIMPLEX.ASC: well, just remove first point");
				points.remove(0);
			}
			// otherwise, it might be inside, which makes the caller a stupid person
		} else {
			System.out.println("Simplex.asc: normal add");
		}
		
		// safe to add
		points.add(v);
	}
	
	public Vector3 closestToOrigin() {
		// gotta give the closest CSOVertex to origin
		System.out.println("closest orig: " + points.size());
		if (points.size() == 1) {
			return new Vector3(points.get(0).p);
		} else if (points.size() == 2) {
			// return closest to line segment
			return MathHelper.getClosestToLine(Vector3.ZERO, points.get(0).p, points.get(1).p, true);
		} else if (points.size() == 3) {
			// return closest to triangle
			float triArea = MathHelper.calcTriangleArea(points.get(0).p, points.get(1).p, points.get(2).p);
			
			if (triArea < Vector3.EPSILON) {
				System.out.println("closest to origin: degenerate simplex triangle case: " + triArea);
			}
//			return MathHelper.getClosestToTriangle(Vector3.ZERO, points.get(0).p, points.get(1).p, points.get(2).p);
			return MathHelper.naiveClosestToTriangle(Vector3.ZERO, points.get(0).p, points.get(1).p, points.get(2).p);
		} else if (points.size() == 4) {
			//return closest to tetrahedron
			
			// WAIITTT!!! DOESN'T WORK IF SIMPLEX IS DEGENERATE!!!!
			float tetraVol = MathHelper.calcTetraVolume(points.get(0).p, points.get(1).p, points.get(2).p, points.get(3).p);
			
			if (Math.abs(tetraVol) < Vector3.EPSILON) {
				System.out.println("closest to origin: degenerate simplex tetrahedron case:" + tetraVol);
			}
			
//			return MathHelper.getClosestToTetrahedron(Vector3.ZERO, points.get(0).p, points.get(1).p, 
//					points.get(3).p, points.get(2).p);
			
			return MathHelper.naiveClosestToTetrahedron(Vector3.ZERO, points.get(0).p, points.get(1).p, 
					points.get(3).p, points.get(2).p);
		}
		System.out.println("Whaaat!!!" + points.size());
		return null;
	}
	
	public void ceaseDegeneracy() {
		// let's stop this once and for all
		while (isDegenerate()) {
			// start removing point. depending on the case though
			if (points.size() == 2) {
				System.out.println("ceaseDegeneracy: line case. removing point b");
				// line case. simply remove any point
				// let's say point b
				points.remove(1);
			} else if (points.size() == 3) {
				// triangle is quite involved. remove incident edge
				CSOVertex a = points.get(0);
				CSOVertex b = points.get(1);
				
				// removing one at a time
				// if still degenerate, will be refined at next iteration
				if (Vector3.distBetween(a.p, b.p) < Vector3.EPSILON) {
					System.out.println("ceaseDegeneracy: triangle case. removing point b");
					// remove b
					points.remove(1);
				} else {
					System.out.println("ceaseDegeneracy: triangle case. removing point c");
					// remove either a or c. in this case we pick c
					points.remove(2);
				}
			} else if (points.size() == 4) {
				// we keep the best triangle in this case. We could remove degenerate triangle
				// instead, but nope. This will do, albeit more expensive
				CSOVertex removed = null;
				float closestDist = Float.MAX_VALUE;
				float dist;
				
				CSOVertex a = points.get(0);
				CSOVertex b = points.get(1);
				CSOVertex c = points.get(2);
				CSOVertex d = points.get(3);
				
				// check origin against abc
				dist = MathHelper.naiveClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p).lengthSquared();
				if (dist < closestDist) {
					closestDist = dist;
					removed = d;
				}
				
				// check origin against bcd
				dist = MathHelper.naiveClosestToTriangle(Vector3.ZERO, b.p, c.p, d.p).lengthSquared();
				if (dist < closestDist) {
					closestDist = dist;
					removed = a;
				}
				
				// check origin against cda
				dist = MathHelper.naiveClosestToTriangle(Vector3.ZERO, c.p, d.p, a.p).lengthSquared();
				if (dist < closestDist) {
					closestDist = dist;
					removed = b;
				}
				
				// check origin against abd
				dist = MathHelper.naiveClosestToTriangle(Vector3.ZERO, a.p, b.p, d.p).lengthSquared();
				if (dist < closestDist) {
					closestDist = dist;
					removed = c;
				}
				
				// we remove it, hopefully it gets better next time
				if (removed != null) {
					System.out.println("ceaseDegeneracy: tetrahedron case. removing point id -> " + points.indexOf(removed));
					points.remove(removed);
				}
			}
		}
	}
	
	public boolean isDegenerate() {
		// only a triangle or tetrahedron could be degenerate
		// WAIT!! A LINE IS POSSIBLE
		if (points.size() == 2) {
			// line
			return Vector3.distBetween(points.get(0).p, points.get(1).p) < Vector3.EPSILON;
		} else if (points.size() == 3) {
			//triangle
			return MathHelper.calcTriangleArea(points.get(0).p, points.get(1).p, points.get(2).p) < Vector3.EPSILON;
		} else if (points.size() == 4) {
			// tetrahedron
			return Math.abs(
					MathHelper.calcTetraVolume(points.get(0).p, points.get(1).p, points.get(2).p, points.get(3).p)
					) < Vector3.EPSILON;
		}		
		
		// point cannot be degenerate
		return false;
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
			
			Vector3 u = new Vector3();
			Vector3 v = new Vector3();
			
			Vector3.sub(closestPointToOrigin, a.p, u);
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
			
			System.out.println("a: " + a.p.x+", "+a.p.y+", "+a.p.z);
			System.out.println("b: " + b.p.x+", "+b.p.y+", "+b.p.z);
			System.out.println("c: " + c.p.x+", "+c.p.y+", "+c.p.z);

			// grab closest to it
			/*Vector3 p0 = MathHelper.getClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p);
			Vector3 p1 = MathHelper.getClosestToLine(Vector3.ZERO, a.p, b.p, true);
			Vector3 p2 = MathHelper.getClosestToLine(Vector3.ZERO, b.p, c.p, true);
			Vector3 p3 = MathHelper.getClosestToLine(Vector3.ZERO, c.p, a.p, true);
			
			
			Vector3 [] candidates = new Vector3[]{
					p0, p1, p2, p3
			};
			Vector3 closest = p0;
			float dist = closest.lengthSquared();
			
			for (int i=1; i<4; i++) {
				float canDist = candidates[i].lengthSquared();
				if (canDist < dist) {
					dist = canDist;
					closest = candidates[i];
				}
			}*/
			// WAITT!!! SEE IF IT'S DEGENERATE
			float triArea = MathHelper.calcTriangleArea(a.p, b.p, c.p);
			
			if (triArea < Vector3.EPSILON) {
				System.out.println("Simplex.getClosestPoint: degenerate triangle!! removing degenerate point...");
				// we need to remove offending points
				if (Vector3.distBetween(a.p, b.p) < Vector3.EPSILON) {
					// a and b are coincident. remove one, then recurse
					points.remove(a);
					System.out.println("Simplex.getClosestPoint: a and b are coincident!!");
				} else if (Vector3.distBetween(b.p, c.p) < Vector3.EPSILON) {
					// b and c
					points.remove(b);
					System.out.println("Simplex.getClosestPoint: b and c are coincident!!");
				} else {
					// must be between c and a
					points.remove(c);
					System.out.println("Simplex.getClosestPoint: c and a are coincident!!");
				}
				
				return this.getClosestPoint(bA, bB);
			}
			
			Vector3 closest = closestPointToOrigin; //MathHelper.naiveClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p);
			// compute barycentric coordinate
			Vector3 bary = MathHelper.computeBarycentric(closest, a.p, b.p, c.p);
			
			System.out.println("bary: " + bary.x+", "+bary.y+", "+bary.z);
			
//			System.out.println("cl: " + closest.x+ ", "+closest.y+ ", "+closest.z+ ", "+closest.length());
//			System.out.println("p1: " + p1.x+ ", "+p1.y+ ", "+p1.z+ ", "+p1.length());
//			System.out.println("p2: " + p2.x+ ", "+p2.y+ ", "+p2.z+ ", "+p2.length());
//			System.out.println("p3: " + p3.x+ ", "+p3.y+ ", "+p3.z+ ", "+p3.length());
			
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
			
//			List<CSOVertex> better = new ArrayList<>();	// we make new list, that is not degenerate
////			
//			// for every current points
//			for (CSOVertex pt : points) {
//				// if the new list is empty, just insert
//				if (better.isEmpty())
//					better.add(pt);
//				else {
//					// by default, it's safe to add
//					boolean safeToAdd = true;
//					// let's compare with every vertex of better list
//					// OPTIMIZATION!! BEGIN FROM THE BACK!!
//					for (int j = better.size()-1; j >= 0; --j) {
//						CSOVertex pt2 = better.get(j);
//						Vector3 dv = new Vector3(pt.p, pt2.p);
//						if (dv.lengthSquared() < Vector3.EPSILON) {
//							System.out.println("singularity a: " + pt.p.x + ", " + pt.p.y + ", " + pt.p.z);
//							System.out.println("singularity b: " + pt2.p.x + ", " + pt2.p.y + ", " + pt2.p.z);
//							
//							safeToAdd = false;
//							break;	// break for (EARLY OUT)
//						}
//					}
//					// is it safe?
//					if (safeToAdd)
//						better.add(pt);
//				}
//			}
//			
//			System.out.println("Tetrahedron simplified: " + better.size());
//			
//			// replace
//			this.points = better;
//			
//			// if this is indeed degenerate tetrahedro, recurse
//			if (points.size() < 4) {
//				return this.getClosestPoint(bA, bB);
//			}
			/*// usually this is it. gotta get closest on the tetrahedron
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			
			// compute closest point ON THE FACE OF THE TETRAHEDRON!!! (DISCARDING ANYTHING INSIDE)
			Vector3 c1 = MathHelper.getClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p);
			Vector3 c2 = MathHelper.getClosestToTriangle(Vector3.ZERO, b.p, c.p, d.p);
			Vector3 c3 = MathHelper.getClosestToTriangle(Vector3.ZERO, c.p, d.p, a.p);
			Vector3 c4 = MathHelper.getClosestToTriangle(Vector3.ZERO, a.p, b.p, d.p);
			
			float d1 = c1.lengthSquared();
			float d2 = c2.lengthSquared();
			float d3 = c3.lengthSquared();
			float d4 = c4.lengthSquared();
			
			Vector3 closest = null;
			
			if (d1 < d2 && d1 < d3 && d1 < d4) {
				closest = c1;
				// face is abc
				a = points.get(0);
				b = points.get(1);
				c = points.get(2);
				
			} else if (d2 < d1 && d2 < d3 && d2 < d4) {
				closest = c2;
				// face is bcd
				a = points.get(1);
				b = points.get(2);
				c = points.get(3);
			} else if (d3 < d1 && d3 < d2 && d3 < d4) {
				closest = c3;
				// face is cda
				a = points.get(2);
				b = points.get(3);
				c = points.get(0);
			} else {
				closest = c4;
				// face is abd
				a = points.get(0);
				b = points.get(1);
				c = points.get(3);
			}
			
			System.out.printf("abc: %.4f,  bcd: %.4f, cda: %.4f, abd: %.4f", c1.lengthSquared(), c2.lengthSquared(),
					c3.lengthSquared(), c4.lengthSquared());
			
			Vector3 bary = MathHelper.computeBarycentric(closest, a.p, b.p, c.p);
			System.out.println("bary: " + bary.x+", "+ bary.y+", "+ bary.z);
			
			if (bary.lengthSquared() < Vector3.EPSILON) {
				System.out.println("DEGENERATE CASE!! FALLBACK TO LINE VS LINE");
				
				// check which point is degenerate, and throw them away
				Vector3 AB = new Vector3(b.p, a.p);
				Vector3 BC = new Vector3(c.p, b.p);
				Vector3 CA = new Vector3(a.p, c.p);
				
				Vector3 lineU = null;
				Vector3 lineV = null;
				
				CSOVertex va, vb;
				
				if (AB.lengthSquared() < Vector3.EPSILON) {
					// AB is degenerate!!!
					lineV = BC;
					lineU = new Vector3(closest, b.p);
					
					va = b;
					vb = c;
				} else if (BC.lengthSquared() < Vector3.EPSILON) {
					// BC is degenerate
					lineV = CA;
					lineU = new Vector3(closest, c.p);
					
					va = c;
					vb = a;
				} else {
					// CA is degenerate
					lineV = AB;
					lineU = new Vector3(closest, a.p);
					
					va = a;
					vb = b;
				}
				// compute interval
				float uv = Vector3.dot(lineU, lineV);
				float vv = Vector3.dot(lineV, lineV);
				
				// make sure it's not degenerate too
				if (Math.abs(vv) < Vector3.EPSILON) {
					System.out.println("SHEIIIITTT THE LINE IS DEGENERATE TOO!!! RETURNING POINT!!");
					bA.setTo(a.a);
					bB.setTo(a.b);
				} else {
					uv /= vv;
					// no need to clamp, already clamped
					bA.x	= va.a.x * (1.0f-uv) + vb.a.x * uv;
					bA.y	= va.a.y * (1.0f-uv) + vb.a.y * uv;
					bA.z	= va.a.z * (1.0f-uv) + vb.a.z * uv;
					
					bB.x	= va.b.x * (1.0f-uv) + vb.b.x * uv;
					bB.y	= va.b.y * (1.0f-uv) + vb.b.y * uv;
					bB.z	= va.b.z * (1.0f-uv) + vb.b.z * uv;
				}
			} else {
				// safe to compute
				bA.x	= bary.x * a.a.x + bary.y * b.a.x + bary.z * c.a.x;
				bA.y	= bary.x * a.a.y + bary.y * b.a.y + bary.z * c.a.y;
				bA.z	= bary.x * a.a.z + bary.y * b.a.z + bary.z * c.a.z;
				
				bB.x	= bary.x * a.b.x + bary.y * b.b.x + bary.z * c.b.x;
				bB.y	= bary.x * a.b.y + bary.y * b.b.y + bary.z * c.b.y;
				bB.z	= bary.x * a.b.z + bary.y * b.b.z + bary.z * c.b.z;
			}*/
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			
			
			float volTetra = MathHelper.calcTetraVolume(a.p, b.p, c.p, d.p);
			
			if (Math.abs(volTetra) < Vector3.EPSILON) {
				// this shouldn't be possible
				System.out.println("SHEEIIIITTT DEGENERATE TETRAHEDRON!!!");
				return false;
			} else {
				// non degenerate would be fine
				System.out.println("NON DEGENERATE TETRAHEDRON!! GOOD!!");
				
				Quaternion bary = MathHelper.computeBarycentric(closestPointToOrigin, a.p, b.p, c.p, d.p);
				
				bA.x = a.a.x * bary.x + b.a.x * bary.y + c.a.x * bary.z + d.a.x * bary.w;
				bA.y = a.a.y * bary.x + b.a.y * bary.y + c.a.y * bary.z + d.a.y * bary.w;
				bA.z = a.a.z * bary.x + b.a.z * bary.y + c.a.z * bary.z + d.a.z * bary.w;
				
				bB.x = a.b.x * bary.x + b.b.x * bary.y + c.b.x * bary.z + d.b.x * bary.w;
				bB.y = a.b.y * bary.x + b.b.y * bary.y + c.b.y * bary.z + d.b.y * bary.w;
				bB.z = a.b.z * bary.x + b.b.z * bary.y + c.b.z * bary.z + d.b.z * bary.w;
				
				return true;
			}
			/*float volTetra = MathHelper.calcTetraVolume(a.p, b.p, c.p, d.p);
			
			if (Math.abs(volTetra) < Vector3.EPSILON) {
				System.out.println("SHEEIIIITTT DEGENERATE TETRAHEDRON!!!");				
			} else {
				// simply grab closest to tetrahedron
//				Vector3 cl = MathHelper.naiveClosestToTetrahedron(Vector3.ZERO, a.p, b.p, c.p, d.p);
				System.out.println("NON DEGENERATE TETRAHEDRON: Still only interested in best face though!!");
				
//				// remove unncessary point
//				if (tb.x < Vector3.EPSILON) {
//					points.remove(a);
//					System.out.println("POINT A IS BACKFACING ORIGIN!");
//				} else if (tb.y < Vector3.EPSILON) { 
//					points.remove(b);
//					System.out.println("POINT A IS BACKFACING ORIGIN!");
//				} else if (tb.z < Vector3.EPSILON) {
//					points.remove(c);
//					System.out.println("POINT A IS BACKFACING ORIGIN!");
//				} else if (tb.w < Vector3.EPSILON) {
//					points.remove(d);
//					System.out.println("POINT A IS BACKFACING ORIGIN!");
//				} else {
//					System.out.println("ORIGIN INSIDE, IDIOOOOOOOT!!!");
//					return false;
//				}
//				
//				// recurse
//				return this.getClosestPoint(bA, bB);
			}
			
			CSOVertex [][] tris = new CSOVertex[][] {
					{a, b, c},
					{b, c, d},
					{c, d, a},
					{a, b, d}
			};	
			
			String [] tri_name = new String[]{
					"tri ABC",
					"tri BCD",
					"tri CDA",
					"tri ABD"
			};
			
			int id = 0;
			float dist = -1.0f;	// will update
			float area = Vector3.EPSILON;
			Vector3 closest = null;
			for (int i=0; i<4; i++) {
				Vector3 cl = MathHelper.naiveClosestToTriangle(Vector3.ZERO, tris[i][0].p, tris[i][1].p, 
						tris[i][2].p);
				float na = MathHelper.calcTriangleArea(tris[i][0].p, tris[i][1].p, tris[i][2].p);
				float nd = cl.lengthSquared();
				
				if (na < Vector3.EPSILON)
					continue;
				
				if (nd < dist || dist < 0) {
					System.out.println("got better triangle: " + tri_name[i] + ", l:" + nd +", a:" + na);
					dist = nd;
					area = na;
					id = i;
					closest = cl;
				}
			}
			// we got the good triangle. use that
			System.out.println("got best triangle: " + tri_name[id]);
			
			// compute barycentric
			Vector3 bary = MathHelper.computeBarycentric(closest, tris[id][0].p, tris[id][1].p, tris[id][2].p);
			
			System.out.println("best bary: " + bary.x+", "+bary.y+", "+bary.z);
			
			bA.x	= tris[id][0].a.x * bary.x + tris[id][1].a.x * bary.y + tris[id][2].a.x * bary.z; 
			bA.y	= tris[id][0].a.y * bary.x + tris[id][1].a.y * bary.y + tris[id][2].a.y * bary.z;
			bA.z	= tris[id][0].a.z * bary.x + tris[id][1].a.z * bary.y + tris[id][2].a.z * bary.z;
			
			bB.x	= tris[id][0].b.x * bary.x + tris[id][1].b.x * bary.y + tris[id][2].b.x * bary.z; 
			bB.y	= tris[id][0].b.y * bary.x + tris[id][1].b.y * bary.y + tris[id][2].b.y * bary.z;
			bB.z	= tris[id][0].b.z * bary.x + tris[id][1].b.z * bary.y + tris[id][2].b.z * bary.z;
			
			return true;*/
			
//			// must remove degenerate point
//			List<CSOVertex> better = new ArrayList<>();	// we make new list, that is not degenerate
//			
//			// for every current points
//			for (CSOVertex pt : points) {
//				// if the new list is empty, just insert
//				if (better.isEmpty())
//					better.add(pt);
//				else {
//					// by default, it's safe to add
//					boolean safeToAdd = true;
//					// let's compare with every vertex of better list
//					// OPTIMIZATION!! BEGIN FROM THE BACK!!
//					for (int j = better.size()-1; j >= 0; --j) {
//						CSOVertex pt2 = better.get(j);
//						Vector3 dv = new Vector3(pt.p, pt2.p);
//						if (dv.lengthSquared() < Vector3.EPSILON) {
//							System.out.println("singularity a: " + pt.p.x + ", " + pt.p.y + ", " + pt.p.z);
//							System.out.println("singularity b: " + pt2.p.x + ", " + pt2.p.y + ", " + pt2.p.z);
//							
//							safeToAdd = false;
//							break;	// break for (EARLY OUT)
//						}
//					}
//					// is it safe?
//					if (safeToAdd)
//						better.add(pt);
//				}
//			}
//			
//			System.out.println("Tetrahedron simplified: " + better.size());
//			
//			// replace
//			this.points = better;
//			
//			// if this is indeed degenerate tetrahedro, recurse
//			if (points.size() < 4) {
//				return this.getClosestPoint(bA, bB);
//			}
//			
//			// whoa!! a valid tetrahedron!!
//			// simply get closest to the tetrahedron
//			
//			
//			Vector3 closest = MathHelper.getClosestToTetrahedron(Vector3.ZERO, a.p, b.p, c.p, d.p);
//			
//			Quaternion bary = MathHelper.computeBarycentric(closest, a.p, b.p, c.p, d.p);
//			
//			System.out.println("valid tetra bary: " + bary.x+ ", "+ bary.y+ ", "+ bary.z+ ", "+ bary.w);
//			
//			// valid tetrahedron...hmm weird
//			bA.x = a.a.x * bary.x + b.a.x * bary.y + c.a.x * bary.z + d.a.x * bary.w;
//			bA.y = a.a.y * bary.x + b.a.y * bary.y + c.a.y * bary.z + d.a.y * bary.w;
//			bA.z = a.a.z * bary.x + b.a.z * bary.y + c.a.z * bary.z + d.a.z * bary.w;
//			
//			bB.x = a.b.x * bary.x + b.b.x * bary.y + c.b.x * bary.z + d.b.x * bary.w;
//			bB.y = a.b.y * bary.x + b.b.y * bary.y + c.b.y * bary.z + d.b.y * bary.w;
//			bB.z = a.b.z * bary.x + b.b.z * bary.y + c.b.z * bary.z + d.b.z * bary.w;
//			
//			return true;
			
//			return true;
		}
		return false;
	}
	
	public CSOVertex getLast() {
		return points.get(points.size()-1);
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
			
			a = points.get(0).p;
			b = points.get(1).p;
			
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
			
			a = points.get(0).p;
			b = points.get(1).p;
			c = points.get(2).p;
			
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
			
			
			a = points.get(0).p;
			b = points.get(1).p;
			c = points.get(2).p;
			d = points.get(3).p;
			
			
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
				{1,1,0}
			};
			gl.glBegin(GL2.GL_POINTS);
			for (int i=0; i<points.size(); i++) {
				Vector3 p = points.get(i).a;
				Vector3 q = points.get(i).b;
				Vector3 r = points.get(i).p;
				gl.glColor3f(color[i][0], color[i][1], color[i][2]);
				gl.glVertex3f(p.x, p.y, p.z);
				gl.glVertex3f(q.x, q.y, q.z);
				gl.glVertex3f(r.x, r.y, r.z);
			}
			// draw closest SHIT
			gl.glColor3f(.2f, .6f, .5f);
			Vector3 co = closestPointToOrigin;
			
			if (co != null) {
				gl.glVertex3f(co.x, co.y, co.z);
			}
			
			gl.glEnd();
			
			// draw line from closest to the origin
			if (co != null) {
				gl.glBegin(GL2.GL_LINES);
				
				gl.glColor3f(1, 1, 0.4f);
				
				gl.glVertex3f(0,0,0);
				gl.glVertex3f(co.x, co.y, co.z);
				
				gl.glEnd();
			}
			
			// draw line connecting closest point
			if (cA != null && cB != null) {
				gl.glBegin(GL2.GL_LINES);
				gl.glColor3f(1, 0.4f, 1);
				
				gl.glVertex3f(cA.x, cA.y, cA.z);
				gl.glVertex3f(cB.x, cB.y, cB.z);
				
				gl.glEnd();
			}
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
