package com.bowie.javagl;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GL2;

/**
 * Simplex - class representing a simplex (collection of points)
 * @author Bowie
 *
 */
public class Simplex {
	private static final boolean debug = false;
	
	private List<CSOVertex> points;
	private List<CSOVertex> history;
	public Vector3 lastDir = new Vector3(1, 0, 0);
	
	static public final int SIMPLEX_CANNOT_EVOLVE = 0;
	static public final int SIMPLEX_CONTAIN_ORIGIN = 1;
	static public final int SIMPLEX_EVOLVED = 2;
	
//	static public final int RAY_NOT_HIT = 0;
//	static public final int RAY_HIT = 1;
//	static public final int RAY_INSIDE = 2;
	
	// raycast result
	static public final int RAY_WONT_HIT = 0;		// caused by parallel ray
	static public final int RAY_HIT = 1;			// it hits!
	static public final int RAY_START_INSIDE = 2;	// it starts inside
	static public final int RAY_STILL_MARCHING = 3;	// it still needs to march, no conclusion yet
	
	static public final float RAY_MARGIN_SQUARED = 0.01f * 0.01f;
	//=============RAYCAST DATA===========================================
	public Vector3 rayhitPos;
	public Vector3 rayhitNormal;
	public Vector3 rayOrigin;
	public Vector3 rayCurrentO;
	public Vector3 rayEnd;
	public float rayT, rayR;
	
	//=============STRICTLY TEST DATA=====================================
	public Shape sA, sB;
	public Vector3 posA, posB;
	public Quaternion rotA, rotB;
	
	public int iter = 0;	// the first iteration
	public boolean converged = false, containsOrigin = false;
	public float closestDist = Float.MAX_VALUE;
	public Vector3 currentDir = new Vector3(1, 0, 0);
	public Vector3 closestPointToOrigin = null;
	
	public Vector3 cA, cB;
	
	private CSOVertex lastRemoved = null;
	//=============STRICTLY TEST DATA=====================================
	
	
	public Simplex(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB) {
		// allocate data
		points = new ArrayList<>(4);
		history = new ArrayList<>(4);
		
		// then do another things
		this.sA	= sA;
		this.sB	= sB;
		this.posA = posA;
		this.posB = posB;
		this.rotA = rotA;
		this.rotB = rotB;
		
		// set current direction to posA - posB
		/*Vector3 pAB = new Vector3(posA, posB);
		if (pAB.lengthSquared() > Vector3.EPSILON)
			this.currentDir.setTo(pAB);
		else*/ 
		
		iter = 0;
		closestDist = Float.MAX_VALUE;
		converged = false;
		containsOrigin = false;
		closestPointToOrigin = null;
		cA = null;
		cB = null;
		
		// init raycast data
		rayhitPos = new Vector3();
		rayhitNormal = new Vector3();
		rayOrigin = new Vector3();
		rayEnd = new Vector3();
		rayCurrentO = new Vector3();
		rayT = -1.f;
	}
	
	public boolean containOrigin() {
		// check case depending on num of point
		int numPoint = points.size();
		
		CSOVertex a,b,c,d;
		Vector3 cp;
		switch (numPoint) {
		
		case 1:
			a = points.get(0);
			
			return a.p.lengthSquared() < Vector3.EPSILON;
		case 2:
			a = points.get(0);
			b = points.get(1);
			
			cp = MathHelper.getClosestToLine(Vector3.ZERO, a.p, b.p, true);
			return cp.lengthSquared() < Vector3.EPSILON;
		case 3:
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			
			cp = MathHelper.getClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p);
			return cp.lengthSquared() < Vector3.EPSILON;
		case 4:
			// compute barycentric coordinate
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			
			Quaternion bary = MathHelper.computeBarycentric(Vector3.ZERO, a.p, b.p, c.p, d.p);
			return bary.x > 0 && bary.y > 0 && bary.z > 0 && bary.w > 0;
		}
		
		return false;
	}
	
	/**
	 * this function computes new direction that will bring us closer
	 * to origin
	 * @param dir		- current direction
	 * @param origin	- the origin to where we are heading
	 * @param tolerance	- the EPSILON value, to which we assume the origin distance is too close
	 * @param autoremove- if TRUE, prevent the simplex from forming a tetrahedron
	 * @return			- true if we can find new direction, false if origin is contained
	 */
	public boolean computeDirection(Vector3 dir, Vector3 origin, float tolerance, boolean autoremove) {
		Vector3 o = origin;
		if (o == null) {
			o = Vector3.ZERO;
		}
		// depending on number of points we got
		int numP = points.size();
//		System.out.println("computeDir: " + numP);
		
		CSOVertex a,b,c,d;
		Vector3 cp;
		
		switch (numP) {
		case 1:
			a = points.get(0);
			// check if it's degenerate
			if (Vector3.distBetween(o, a.p) < tolerance/*Vector3.EPSILON*/) {
				// that means a is origin
				if (debug)
					System.out.printf("COMPUTEDIR TOO CLOSE @ 0-SIMP : %.4f%n", Vector3.distBetween(o, a.p));
				return false;
			} else {
				dir.setTo(o.x-a.p.x, o.y-a.p.y, o.z-a.p.z);
			}
			return true;
		case 2:
			a = points.get(0);
			b = points.get(1);
			
			cp = MathHelper.getClosestToLine(Vector3.ZERO, a.p, b.p, true);
			
			if (Vector3.distBetween(o, cp) < tolerance/*Vector3.EPSILON*/) {
				// that means origin lies on AB
				if (debug)
					System.out.printf("COMPUTEDIR TOO CLOSE @ 1-SIMP : %.4f%n", Vector3.distBetween(o, cp));
				return false;
			} else {
				dir.setTo(o.x-cp.x, o.y-cp.y, o.z-cp.z);
			}
			return true;
		case 3:
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			
			cp = MathHelper.getClosestToTriangle(Vector3.ZERO, a.p, b.p, c.p);
			
			if (Vector3.distBetween(o, cp) < tolerance/*Vector3.EPSILON*/) {
				// that means origin is on plane ABC
				if (debug)
					System.out.printf("COMPUTEDIR TOO CLOSE @ 2-SIMP : %.4f%n", Vector3.distBetween(o, cp));
				return false;		
			} else {
				// safe to use
				dir.setTo(o.x-cp.x, o.y-cp.y, o.z-cp.z);
			}
			return true;
		case 4:
			a = points.get(0);
			b = points.get(1);
			c = points.get(2);
			d = points.get(3);
			// ensure that we're not inside
			Quaternion bary = MathHelper.computeBarycentric(o, a.p, b.p, c.p, d.p);
			if (bary.x > 0 && bary.y > 0 && bary.z > 0 && bary.w > 0) {
				// that means origin is contained within tetrahedron
				if (debug)
					System.out.printf("COMPUTEDIR TOO CLOSE @ 3-SIMP : {%.4f, %.4f, %.4f, %.4f}%n", 
						bary.x, bary.y, bary.z, bary.w);
				if (autoremove) {
					if (debug)
						System.out.println("UNIMPORTANT POINT REMOVED");
					// do the removal
					if (bary.x < bary.y && bary.x < bary.z && bary.x < bary.w) {
						// remove a
						points.remove(0);
					} else if (bary.y < bary.x && bary.y < bary.z && bary.y < bary.w) {
						// remove b
						points.remove(1);
					} else if (bary.z < bary.x && bary.z < bary.y && bary.z < bary.w) {
						// remove c
						points.remove(2);
					} else {
						// remove d
						points.remove(3);
					}
				}
				return false;
			}
			
			// grab closest			
			Vector3 abc = MathHelper.getClosestToTriangle(o, a.p, b.p, c.p);
			Vector3 bcd = MathHelper.getClosestToTriangle(o, b.p, c.p, d.p);
			Vector3 cda = MathHelper.getClosestToTriangle(o, c.p, d.p, a.p);
			Vector3 abd = MathHelper.getClosestToTriangle(o, a.p, b.p, d.p);
			
			CSOVertex removed = d;
			Vector3 closest = abc;
			float closestD = Vector3.distBetween(o, abc);//abc.lengthSquared();
			
			// test bcd
			float dist = Vector3.distBetween(o, bcd);//bcd.lengthSquared();
			if (dist < closestD) {
				removed = a;
				closestD = dist;
				closest = bcd;
			}
			
			// test cda
			dist = Vector3.distBetween(o, cda);//cda.lengthSquared();
			if (dist < closestD) {
				removed = b;
				closestD = dist;
				closest = cda;
			}
			
			// test abd
			dist = Vector3.distBetween(o, abd);//abd.lengthSquared();
			if (dist < closestD) {
				removed = c;
				closestD = dist;
				closest = abd;
			}
			
			// keep entry on the last removed
			lastRemoved = removed;
			points.remove(removed);
			dir.setTo(o.x-closest.x, o.y-closest.y, o.z-closest.z);
			return true;
		}
		return false;
	}
	
	public void GJK_Init() {
		// init direction, and clear shits
		points.clear();
		history.clear();
		
		lastDir.x = posB.x - posA.x;
		lastDir.y = posB.y - posA.y;
		lastDir.z = posB.z - posA.z;
		
		if (lastDir.lengthSquared() < Vector3.EPSILON) {
			// give good direction
			lastDir.setTo(1, 0, 0);
		}
		
		currentDir.setTo(lastDir);
		
		closestDist = Float.MAX_VALUE;
		closestPointToOrigin = null;
		
		cA = null;
		cB = null;
	}
	
	public int GJK_Evolve() {
		CSOVertex v = Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, lastDir);
		
		if (!addSupportPoint(v)) {
			
			if (debug)
				System.out.printf("Simplex.GJK_Evolve: Cant evolve more! pt: %d%n", points.size());
			
			cA = new Vector3();
			cB = new Vector3();
//			
			getClosestPoint(cA, cB);
			
			return SIMPLEX_CANNOT_EVOLVE;
		}
		
		// is it better?
		float dot = Vector3.dot(lastDir, v.p);
		if (dot <= 0) {
			if (debug)
				System.out.println("the last point is not better. ISA GJK POTENTIAL EARLY OUT!! " + dot);
		}
		
		closestPointToOrigin = closestToOrigin();
		float d = closestPointToOrigin.lengthSquared();
		
		if (d < closestDist) {
			if (debug)
				System.out.printf("Simplex.GJK_Evolve: getting closer from %.4f -> %.4f%n", closestDist, d);
			closestDist = d;
		} else {
			if (debug)
				System.out.printf("Simplex.GJK_Evolve: WTF!!! DIDN'T GET CLOSER!! %.4f -> %.4f%n", closestDist, d);
			// remove last point
			points.remove(v);
			
			cA = new Vector3();
			cB = new Vector3();
//			
			getClosestPoint(cA, cB);
			
			return SIMPLEX_CANNOT_EVOLVE;
		}
		
		
		// change direction. only fail if origin is contained
		if (!computeDirection(lastDir, Vector3.ZERO, Vector3.EPSILON, false)) {
			// dang, we've contained
			if (debug)
				System.out.printf("Simplex.GJK_Evolve: Origin CONTAINEDD!!%n");
			return SIMPLEX_CONTAIN_ORIGIN;
		} else {
			if (debug)
				System.out.printf("Simplex: new dir-> %.4f,  %.4f, %.4f%n", lastDir.x, lastDir.y, lastDir.z);
		}
		
		return SIMPLEX_EVOLVED;
	}
	
	public boolean GJK(boolean ISAGJK) {
		// perform gjk
		// grab initial direction
		lastDir.x = posB.x - posA.x;
		lastDir.y = posB.y - posA.y;
		lastDir.z = posB.z - posA.z;
		
		if (lastDir.lengthSquared() < Vector3.EPSILON) {
			// give good direction
			lastDir.setTo(1, 0, 0);
		}
		
		// now we loop all over
		int iter = 0;
		while (iter++ < MathHelper.GJK_MAX_ITERATION) {
			// push initial support point
			CSOVertex v = Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, lastDir);
			
			// is this new point?
			if (!addSupportPoint(v)) {
				// failing to add point means that point is already added
				// which means we can't get better
				return false;
			}
			
			// ISA GJK BEGIN HERE
			// check if last . dir < EPSILON, then return false
			if (ISAGJK) {
				float dot = Vector3.dot(v.p, lastDir);
				if (dot < Vector3.EPSILON) {
					// that means it won't even reach past origin
					return false;
				}
			}
			// ISA GJK END HERE
			
			// now we check if we contain origin
			
			if (!computeDirection(lastDir, Vector3.ZERO, Vector3.EPSILON, false)) {
				// failing to compute new direction means origin is contained
				// welp, blow simplex if it's not tetrahedron yet
				if (points.size() < 4)
					blow();
				return true;
			}
			
		}
		
		if (iter >= MathHelper.GJK_MAX_ITERATION) {
			if (debug)
				System.out.println("GJK: FAIL HARD!!! " + iter);
		}
		
		return false;
	}
	
	public void blow() {
		// we blow until it becomes tetrahedron
		int numP = points.size();
		while (numP < 4) {
			if (numP == 1) {
				// point case
				// just shoot at random direction
				lastDir.setTo(1, 0, 0);
				CSOVertex v = Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, lastDir);
				points.add(v);
			} else if (numP == 2) {
				// line case
				// make another direction that is perpendicular
				
				Vector3 n = Vector3.tmp0;
				Vector3.sub(points.get(1).p, points.get(0).p, n);
				
				if (Math.abs(n.x) >= 0.57735f) {
					lastDir.x = n.y;
					lastDir.y = -n.z;
					lastDir.z = 0;
				} else {
					lastDir.x = 0;
					lastDir.y = n.z;
					lastDir.z = -n.y;
				}
				CSOVertex v = Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, lastDir);
				points.add(v);
			} else if (numP == 3) {
				// triangle case
				// compute normal
				Vector3 AB = Vector3.tmp0;
				Vector3 AC = Vector3.tmp1;
				Vector3 n = Vector3.tmp2;
				
				Vector3.sub(points.get(1).p, points.get(0).p, AB);
				Vector3.sub(points.get(2).p, points.get(0).p, AB);
				Vector3.cross(AB, AC, n);
				
				lastDir.setTo(n);
				CSOVertex v = Shape.minkowskiDiff(sA, AB, rotA, sB, AC, rotB, lastDir);
				if (!addSupportPoint(v)) {
					// welp, reverse it
					lastDir.scale(-1);
					v = Shape.minkowskiDiff(sA, AB, rotA, sB, AC, rotB, lastDir);
					addSupportPoint(v);
				}
			}
			
			// update num point
			numP = points.size();
		}
	}
	
	public boolean advance() {
		// do not do anything if we have converged
		if (converged)
			return true;
		if (containsOrigin)
			return true;
		// increment counter
//		System.out.println("SIMPLEX.ADVANCE -- " + iter);
		iter++;
		// we add point
		if (!addSupportPoint(Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, currentDir))) {
			System.out.println("SIMPLEX TRIED ADDING OLD POINT!!");
			return false;
		}
//		this.addSupportConservatively(grabMinkowskiDiff());
		// now we find closest from origin..but we must check first if we're degenerate
//		if (isDegenerate()) {
//			// warn user
////			System.out.println("SIMPLEX WARNING!! DEGENERATE!!! ANY DIRECTION OR DISTANCE MIGHT NOT BE VALID NOW!! @ " + points.size());
//		}
		
		// we have at least 2 point, so simply add shit based on closest direction
		
		
		// check if we have origin inside
		if (containOrigin()) {
//			System.out.println("SIMPLEX CONTAIN ORIGIN!! EXITING...");
			containsOrigin = true;
			return true;
		}
		
		closestPointToOrigin = closestToOrigin();
		float d = closestPointToOrigin.lengthSquared();
		// check if we're close
		if (Math.abs(closestDist - d) < MathHelper.GJK_MARGIN_ERROR) {
			System.out.printf("SIMPLEX CONVERGED WITH SEP_DIST: %f%n", Math.abs(closestDist - d));
			converged = true;
			
			// make sure our simplex isn't degenerate
//			ceaseDegeneracy();
			
			// compute closest point
			cA = new Vector3();
			cB = new Vector3();
			
			this.getClosestPoint(cA, cB);
			
			// grab distance
			Vector3 n = new Vector3(cA, cB);
			float dist = n.length();
			n.normalize();
			System.out.printf("closest distance: %.4f, n = [%.4f %.4f %.4f]%n", dist, n.x, n.y, n.z);
			
			return true;
		}
		
		// Gotta warn too if distance is shit
		if (d > closestDist) {
			System.out.println("SIMPLEX WARNING!! RECENT DISTANCE IS FARTHER!!: " + d + " > " + closestDist);
			return false;
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
		return true;
	}
	
	//=============STRICTLY TEST DATA=====================================
	
	public void reset() {
		points.clear();
		history.clear();
		
		//=============STRICTLY TEST DATA=====================================
		iter = 0;
		closestDist = Float.MAX_VALUE;
		currentDir.setTo(1, 0, 0);
		converged = false;
		containsOrigin = false;
		closestPointToOrigin = null;
		cA = null;
		cB = null;
		//=============STRICTLY TEST DATA=====================================
	}
	
	// init raycast data (which is not covered by simplex)
	public void raycastInit(Vector3 rS, Vector3 rE, float radius) {
		// set initial ray position
		rayCurrentO.setTo(rS);
		rayOrigin.setTo(rS);
		rayEnd.setTo(rE);
		// ignore radius for now
		rayhitNormal.setTo(Vector3.ZERO);
		
		// set rayT to 0
		rayT = 0;
		rayR = radius;
		
		// last dir set to > rayCurrentO - posA
		Vector3.sub(rS, posA, lastDir);
//		Vector3.sub(rE, rS, lastDir);
		
		// reset gjk info
		points.clear();
		history.clear();
		lastRemoved = null;
	}
	
	/**
	 * raycastStep - do a single step of GJK raycast
	 * @return ray hit status code
	 */
	public int raycastStep() {
		// use rayCurrentO as b's point	
		boolean collide = false;
		// do GJK step
		if (debug)
			System.out.println("GJK RAYCAST: evolving simplex...");
		int iter = 0;
		while (iter++ < MathHelper.GJK_MAX_ITERATION) {
			// push initial support point
			Vector3 supA = sA.supportPoint(lastDir, posA, rotA);
			
			// gotta inflate it along the direction (IF RAY HAS RADIUS)
			// only do it if it's really necessary (cause round shapes = slow convergence)
			// better use subdivided cylinder (polygon-shaped cylinder/round shapes)
			// this means better do convex cast than sphere cast
			Vector3 supB;			
			if (rayR > Vector3.EPSILON) {
				// lastDir always point to origin
				Vector3 n = lastDir.normalized();
				supB = new Vector3(
						rayCurrentO.x - n.x * rayR,
						rayCurrentO.y - n.y * rayR,
						rayCurrentO.z - n.z * rayR
						);
			} else {
				supB = new Vector3(rayCurrentO);
			}
			
			// build CSOVertex
			CSOVertex v = new CSOVertex(supA, supB);
			
			// is this new point?
			if (!addSupportPoint(v)) {
				// failing to add point means that point is already added
				// which means we can't get better
				collide = false;
				break;
			}
			
			// use this direction as normal?
			rayhitNormal.setTo(lastDir);
			
			// now we check if we contain origin
			if (!computeDirection(lastDir, Vector3.ZERO, RAY_MARGIN_SQUARED, true)) {
				collide = true;
				break;
			}			
		}
		
		if (debug)
			System.out.println("GJK RAYCAST: simplex evolved in : " + iter);
		
		if (!collide) {
			// compute segment from current origin
			Vector3 seg = new Vector3(rayEnd, rayCurrentO);
			
			// use last direction as normal
			float uv = lastDir.lengthSquared();
			float vv = -Vector3.dot(seg, lastDir);
			
			if (uv < RAY_MARGIN_SQUARED) {
				// we've hit!!
				if (debug)
					System.out.printf("GJK RAYCAST: RAY HIT!!! SUCCESS!! @ %.4f  dist(%.4f)%n", rayT, uv);
				
				// ALREADY VALID
				// SIMPLY CALCULATE DATA: NORMAL, RAYHITPOS
				
				return RAY_HIT;
			}
			
			if (Math.abs(vv) < Vector3.EPSILON) {
				// ray is parallel!!!
				if (debug)
					System.out.println("GJK RAYCAST: RAY IS PARALLEL!! QUITTING...");
				return RAY_WONT_HIT;
			}
			
			float t = uv / vv;
			// it must be valid
			if (t < 0 || t > 1) {
				if (debug)
					System.out.printf("GJK RAYCAST: invalid ray segment : %.4f%n", t);
				return RAY_WONT_HIT;
			}
			
			
			
			// move rayCurrentO by that amount
			rayT += (1.f-rayT) * t;
			
			if (debug)
				System.out.printf("GJK RAYCAST: we can advance ray by %.4f --> %.4f%n", t, rayT);
			
			rayCurrentO.x += (rayEnd.x - rayCurrentO.x) * t;
			rayCurrentO.y += (rayEnd.y - rayCurrentO.y) * t;
			rayCurrentO.z += (rayEnd.z - rayCurrentO.z) * t;
			
			// shift direction
			computeDirection(lastDir, new Vector3(rayCurrentO, rayOrigin), RAY_MARGIN_SQUARED, false);
			
		} else {
			if (debug)
				System.out.printf("GJK RAYCAST: RAY START INSIDE!! @ %.4f%n", rayT);
			
			// VALID CRITERIA:
			// 0 <= t <= 1
			if (rayT > Vector3.EPSILON && rayT <= 1.f)
				return RAY_HIT;
			else
				return RAY_START_INSIDE;
		}
		
		// we still need to march the ray
		return RAY_STILL_MARCHING;
	}
	
	public int doRayCast(Vector3 rS, Vector3 rE, float radius) {
		// init raycast data
		raycastInit(rS, rE, radius);
		
		// keep advancing
		int retVal = RAY_STILL_MARCHING;
		
		do {
			retVal = raycastStep();
		} while (retVal == RAY_STILL_MARCHING);
		
		// check return values, we need to calculate something here
		if (retVal == RAY_HIT) {
			// calculate some data
			if (debug)
				System.out.println("GJKRAYCAST: ray hit!! calculating some data...");
			
			if (points.size() >= 3) {
				// this is the only valid stats tho
				CSOVertex a = points.get(0);
				CSOVertex b = points.get(1);
				CSOVertex c = points.get(2);
				
				// first, the only known data is rayT
				// so let's calculate normal (easy)
				Vector3 ab = Vector3.tmp0;
				Vector3 ac = Vector3.tmp1;
				
				Vector3.sub(b.p, a.p, ab);
				Vector3.sub(c.p, a.p, ac);
				
				Vector3.cross(ab, ac, rayhitNormal);
				
				// make sure it's facing the ray
				Vector3.sub(rE, rS, ab);
				
				float d = Vector3.dot(ab, rayhitNormal);
				
				if (d > 0.0f) {
					// flip
					rayhitNormal.scale(-1.f);
				}
				
				// normalize
				rayhitNormal.normalize();
				
				// now let's calculate ray hit point
				// 1. calculate segment position
				rayhitPos.setTo(
						rS.x * (1.f-rayT) + rE.x * rayT, 
						rS.y * (1.f-rayT) + rE.y * rayT, 
						rS.z * (1.f-rayT) + rE.z * rayT
						);
				// 2. offset it along the radius * normal
				if (radius > Vector3.EPSILON) {
					rayhitPos.x -= rayhitNormal.x * radius;
					rayhitPos.y -= rayhitNormal.y * radius;
					rayhitPos.z -= rayhitNormal.z * radius;
				}
				
			}
		}
		
		if (debug)
			System.out.println("GJKRAYCAST: SHIT retcode= " + retVal);
		
		return retVal;
	}
	
	// For convex cast
	public void convexCastInit(Vector3 vA, Vector3 vB) {
		// all ray data is now in GJK space, different than the raycast one
		// ray start is posA - posB
		rayOrigin.setTo(posA.x-posB.x, posA.y-posB.y, posA.z-posB.z);
		rayCurrentO.setTo(rayOrigin);
		// the ray end
		rayEnd.setTo(rayOrigin.x + vB.x-vA.x, 
				rayOrigin.y + vB.y-vA.y, 
				rayOrigin.z + vB.z-vA.z);
		
		// the remaining data is set like raycastInit
		// set rayT to 0
		rayT = 0;
		
		// last dir set to ---> inverse of ray direction
		Vector3.sub(rayOrigin, rayEnd, lastDir);		
		
		// reset gjk info
		points.clear();
		history.clear();
		lastRemoved = null;
	}
	
	/**
	 * convexCastStep - this function perform a step of the convex cast algorithm
	 * @return one of the return code of raycast
	 */
	public int convexCastStep() {
		// use rayCurrentO as b's point	
		boolean collide = false;
		// do GJK step
		if (debug)
			System.out.println("GJK CONVEXCAST: evolving simplex...");
		int iter = 0;
		while (iter++ < MathHelper.GJK_MAX_ITERATION) {
			// push initial support point
			Vector3 supA = sA.supportPoint(lastDir, posA, rotA);
			Vector3 supB = sB.supportPoint(lastDir.inverse(), posB, rotB);
			
			// either support point a or b needs to be adjusted according to current ray start position
//			supA.x -= (rayCurrentO.x-rayOrigin.x);
//			supA.y -= (rayCurrentO.y-rayOrigin.y);
//			supA.z -= (rayCurrentO.z-rayOrigin.z);
			
			supB.x += (rayCurrentO.x-rayOrigin.x);
			supB.y += (rayCurrentO.y-rayOrigin.y);
			supB.z += (rayCurrentO.z-rayOrigin.z);
			
			
			// build CSOVertex
			CSOVertex v = new CSOVertex(supA, supB);
			
			// is this new point?
			if (!addSupportPoint(v)) {
				// failing to add point means that point is already added
				// which means we can't get better
				collide = false;
				break;
			}
			
			// now we check if we contain origin
			if (!computeDirection(lastDir, Vector3.ZERO, RAY_MARGIN_SQUARED, true)) {
				collide = true;
				break;
			}			
		}
		
		if (debug)
			System.out.println("GJK CONVEXCAST: simplex evolved in : " + iter);
		
		if (!collide) {
			// compute segment from current origin
			Vector3 seg = new Vector3(rayEnd, rayCurrentO);
			
			// use last direction as normal
			float uv = lastDir.lengthSquared();
			float vv = -Vector3.dot(seg, lastDir);
			
			if (uv < RAY_MARGIN_SQUARED) {
				// we've hit!!
				if (debug)
					System.out.printf("GJK CONVEXCAST: RAY HIT!!! SUCCESS!! @ %.4f  dist(%.4f)%n", rayT, uv);
				
				// ALREADY VALID
				// SIMPLY CALCULATE DATA: NORMAL, RAYHITPOS
				
				return RAY_HIT;
			}
			
			if (Math.abs(vv) < Vector3.EPSILON) {
				// ray is parallel!!!
				if (debug)
					System.out.println("GJK CONVEXCAST: RAY IS PARALLEL!! QUITTING...");
				return RAY_WONT_HIT;
			}
			
			float t = uv / vv;
			// it must be valid
			if (t < 0 || t > 1) {
				if (debug)
					System.out.printf("GJK CONVEXCAST: invalid ray segment : %.4f%n", t);
				return RAY_WONT_HIT;
			}
			
			
			
			// move rayCurrentO by that amount
			rayT += (1.f-rayT) * t;
			
			if (debug)
				System.out.printf("GJK CONVEXCAST: we can advance ray by %.4f --> %.4f%n", t, rayT);
			
			rayCurrentO.x += (rayEnd.x - rayCurrentO.x) * t;
			rayCurrentO.y += (rayEnd.y - rayCurrentO.y) * t;
			rayCurrentO.z += (rayEnd.z - rayCurrentO.z) * t;
			
			// shift direction
			computeDirection(lastDir, new Vector3(rayCurrentO, rayOrigin), RAY_MARGIN_SQUARED, false);
			
		} else {
			if (debug)
				System.out.printf("GJK CONVEXCAST: RAY START INSIDE!! @ %.4f%n", rayT);
			
			// VALID CRITERIA:
			// 0 <= t <= 1
			if (rayT > Vector3.EPSILON && rayT <= 1.0f) 
				return RAY_HIT;
			else
				return RAY_START_INSIDE;
		}
		// gotta return still needs marching
		return RAY_STILL_MARCHING;
	}
	
	public int doConvexCast(Vector3 vA, Vector3 vB) {
		// first, we init data
		convexCastInit(vA, vB);
		
		// then we march the ray as long as it still can
		int retVal = RAY_STILL_MARCHING;
		do {
			retVal = convexCastStep();
		} while (retVal == RAY_STILL_MARCHING);
		
		// preview the result, decide potential action
		if (retVal == RAY_HIT) {
			if (debug)
				System.out.println("CONVEXCAST: ray hit!! we can compute normal now with pts= " + points.size());
			// now we compute shit in here
			// first known thing is rayT
			// next, compute the raynormal (using the normal of simplex)
			// NOTE:theoritically the simplex would terminate with n >= 3
			if (points.size() >= 3) {
				// cool
				// rayNormal
				Vector3 ab = Vector3.tmp0;
				Vector3 ac = Vector3.tmp1;
				
				CSOVertex a = points.get(0);
				CSOVertex b = points.get(1);
				CSOVertex c = points.get(2);
				
				ab.setTo(b.p.x-a.p.x, b.p.y-a.p.y, b.p.z-a.p.z);	
				ac.setTo(c.p.x-a.p.x, c.p.y-a.p.y, c.p.z-a.p.z);
				
				// cross
				Vector3.cross(ab, ac, rayhitNormal);
				
				// make sure it's facing the ray
				Vector3.sub(rayEnd, rayOrigin, ab);
				float d = Vector3.dot(ab, rayhitNormal);
				
				if (d < 0.0f) {
					// flip
					rayhitNormal.scale(-1.f);
				}
				
				// normalize
				rayhitNormal.normalize();
				
				// now the rayhit position
				// use shape A's space, and offset by vA * rayT (we can also use B, same shit really)
				// 1. compute barycentric coord
				Vector3 bary = MathHelper.computeBarycentric(Vector3.ZERO, a.p, b.p, c.p);
				
				// 2. use it to interpolate along sA's coord
				rayhitPos.setTo(
						a.a.x * bary.x + b.a.x * bary.y + c.a.x * bary.z, 
						a.a.y * bary.x + b.a.y * bary.y + c.a.y * bary.z, 
						a.a.z * bary.x + b.a.z * bary.y + c.a.z * bary.z);
				
				// 3. offset by vA * rayT
				rayhitPos.x += vA.x * rayT;
				rayhitPos.y += vA.y * rayT;
				rayhitPos.z += vA.z * rayT;
			}
		} 
		
		if (debug) 
			System.out.println("CONVEXCAST: retval is = " + retVal);
		
		return retVal;
	}
	
	public boolean addSupportPoint(CSOVertex v) {
		// adding already added point means we can't get better
		if (history.contains(v))
			return false;
		
		// it's new!!
		points.add(v);
		history.add(v);
		return true;
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
//			System.out.println("Simplex.asc: removing shit simplex");
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
			
//			System.out.printf("Simplex.asc: bary: %.6f, %.6f, %.6f, %.6f%n", bary.x, bary.y, bary.z, bary.w);

			
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
//				System.out.printf("SIMPLEX.ASC: gotta remove %d -> %f%n", points.indexOf(r), maxD);
				points.remove(r);
			} else {
//				System.out.println("SIMPLEX.ASC: well, just remove first point");
				points.remove(0);
			}
			// otherwise, it might be inside, which makes the caller a stupid person
		} else {
//			System.out.println("Simplex.asc: normal add");
		}
		
		// safe to add
		points.add(v);
	}
	
	public Vector3 closestToOrigin() {
		// gotta give the closest CSOVertex to origin
//		System.out.println("closest orig: " + points.size());
		if (points.size() == 1) {
			return new Vector3(points.get(0).p);
		} else if (points.size() == 2) {
			// return closest to line segment
			return MathHelper.getClosestToLine(Vector3.ZERO, points.get(0).p, points.get(1).p, true);
		} else if (points.size() == 3) {
			// return closest to triangle
			/*float triArea = MathHelper.calcTriangleArea(points.get(0).p, points.get(1).p, points.get(2).p);
			
			if (triArea < Vector3.EPSILON) {
//				System.out.println("closest to origin: degenerate simplex triangle case: " + triArea);
			}*/
			return MathHelper.getClosestToTriangle(Vector3.ZERO, points.get(0).p, points.get(1).p, points.get(2).p);
//			return MathHelper.naiveClosestToTriangle(Vector3.ZERO, points.get(0).p, points.get(1).p, points.get(2).p);
		} else if (points.size() == 4) {
			//return closest to tetrahedron
			
			// WAIITTT!!! DOESN'T WORK IF SIMPLEX IS DEGENERATE!!!!
//			float tetraVol = MathHelper.calcTetraVolume(points.get(0).p, points.get(1).p, points.get(2).p, points.get(3).p);
			
//			if (Math.abs(tetraVol) < Vector3.EPSILON) {
//				System.out.println("closest to origin: degenerate simplex tetrahedron case:" + tetraVol);
//			}
			
			return MathHelper.naiveClosestToTetrahedron(Vector3.ZERO, points.get(0).p, points.get(1).p, 
					points.get(3).p, points.get(2).p);
			
			// which triangle to keep? the closest one
		}
//		System.out.println("Whaaat!!!" + points.size());
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
		int numP = points.size();
		
		// update closest
		closestPointToOrigin = closestToOrigin();
		
		switch (numP) {
		case 1:
			CSOVertex a = points.get(0);
			
			bA.setTo(a.a);
			bB.setTo(a.b);
			return true;
		case 2:
			a = points.get(0);
			CSOVertex b = points.get(1);
			// compute closest point interval
			Vector3 u = Vector3.tmp0;
			Vector3 v = Vector3.tmp1;
			
			Vector3.sub(closestPointToOrigin, a.p, u);
			Vector3.sub(b.p, a.p, v);
			
			float scale = Vector3.dot(u, v);
			float det = Vector3.dot(v, v);
			if (Math.abs(det) < Vector3.EPSILON) {
				// segment too close, return a
				scale = 0;
			} else {
				scale /= det;
			}
			
			// now scale = [0..1]
			bA.setTo(
					(1.0f-scale) * a.a.x + scale * b.a.x, 
					(1.0f-scale) * a.a.y + scale * b.a.y, 
					(1.0f-scale) * a.a.z + scale * b.a.z
					);
			bB.setTo(
					(1.0f-scale) * a.b.x + scale * b.b.x, 
					(1.0f-scale) * a.b.y + scale * b.b.y, 
					(1.0f-scale) * a.b.z + scale * b.b.z
					);
			
			return true;
		case 3:
			a = points.get(0);
			b = points.get(1);
			CSOVertex c = points.get(2);
			Vector3 bary = MathHelper.computeBarycentric(closestPointToOrigin, a.p, b.p, c.p);
			
			float sum = bary.x + bary.y + bary.z;
//			System.out.printf("closest point: bary = %.4f, %.4f, %.4f = %.4f%n", bary.x, bary.y, bary.z, sum);
			
			bA.setTo(
					a.a.x * bary.x + b.a.x * bary.y + c.a.x * bary.z, 
					a.a.y * bary.x + b.a.y * bary.y + c.a.y * bary.z, 
					a.a.z * bary.x + b.a.z * bary.y + c.a.z * bary.z
					);
			bB.setTo(
					a.b.x * bary.x + b.b.x * bary.y + c.b.x * bary.z, 
					a.b.y * bary.x + b.b.y * bary.y + c.b.y * bary.z, 
					a.b.z * bary.x + b.b.z * bary.y + c.b.z * bary.z
					);
			
			return true;
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
		Vector3 o = Vector3.ZERO;	// origin
		if (points.size() == 2) {
			// LINE CASE
			Vector3 proj = MathHelper.getClosestToLine(o, points.get(0).p, points.get(1).p, true);
			// NEGATE
			proj.scale(-1.0f);
			newDir = proj;
		} else if (points.size() == 3) {
			// TRIANGLE CASE
			
			// For Academic Purpose!!
			CSOVertex a = points.get(0);
			CSOVertex b = points.get(1);
			CSOVertex c = points.get(2);
			
			// compute normal (RAW)
//			Vector3.sub(b.p, a.p, AB);
//			Vector3.sub(c.p, a.p, AC);
//			Vector3.cross(AB, AC, n);
			
			Vector3 AB = new Vector3(b.p, a.p);
			Vector3 AC = new Vector3(c.p, a.p);
			
			Vector3 n = new Vector3();
			Vector3.cross(AB, AC, n);
			
			// check sign
			float dp = Vector3.dot(a.p, n);
			
			if (dp < 0) {
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
			if (bary.x < Vector3.EPSILON) {
				points.remove(0);
			} else if (bary.y < Vector3.EPSILON) {
				points.remove(1);
			} else if (bary.z < Vector3.EPSILON) {
				points.remove(2);
			} else {
				points.remove(3);
			} 
			
			// now it's a triangle, recurse simply
			return calcNewDir();
		}
		return newDir;
	}
}
