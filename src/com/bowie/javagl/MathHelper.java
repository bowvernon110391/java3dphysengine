package com.bowie.javagl;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.bowie.javagl.Polytope.EPATriangle;

public class MathHelper {	
	static public int GJK_MAX_ITERATION = 32;
	static public int EPA_MAX_ITERATION = 32;
	static public float GJK_MARGIN_ERROR = 0.001f;
	
	static public Random rng = new Random(System.currentTimeMillis());
	
	public MathHelper() {
	}
	
	public static float pacejkaMFLat(float degSA, float B, float C, float D, float E) {
		// gotta wrap value around		
		float term1 = B * (1-E) * degSA;
		float term2 = (float) (E * Math.atan(B * degSA));
		float term3 = (float) (C * Math.atan(term1 + term2));
		float yVal = (float) (D * Math.sin(term3));
		return yVal;
	}
	
	public static float lerp(float a, float b, float mu) {
		return (1.0f-mu) * a + mu * b;
	}
	
	public static float cosineInterpolation(float a, float b, float mu) {
		float mu2;

		mu2 = (float) ((1 - Math.cos(mu * Math.PI)) / 2.0f);
		return (a * (1 - mu2) + b * mu2);
	}
	
	// will push the vertices into m- margin
	public static Convex getDeflatedConvex(Convex s, float m) {
		// first, we grab all faces, and push them inward
		List<Polygon> faces = new ArrayList();
		for (Polygon p : s.getFaces()) {
			faces.add(new Polygon(p));
		}
		
		// now we push them
		for (Polygon p : faces) {
//			p.pushInward(m);
		}
		
		List<Polygon> clpFaces = new ArrayList<>();
		
		// we push all vertices, using brute forces.....
		List<Vector3> pts = s.getPoints();
		List<Vector3> newPts = new ArrayList<>();
		
		float multFactor = 1.0f / (float) Math.max(64, pts.size() * pts.size());
//		System.out.println("mult factor: " +multFactor);
		for (int p_id=0; p_id<pts.size(); p_id++) {
			
		
			Vector3 tmpPt = new Vector3(pts.get(p_id));
//			System.out.printf("o -> %.4f %.4f %4f%n", tmpPt.x, tmpPt.y, tmpPt.z);
			int i=0;
			float maxD = m * 2;
			int iterMax = 200;
			int iterCnt = 0;
			while (maxD+m > Vector3.EPSILON || iterCnt++ < iterMax) {
				int pol_id = 0;
				for (Polygon pol : faces) {
					// push back along face
					Vector3 ap = new Vector3(tmpPt, pol.p.get(0));
					float d= Vector3.dot(ap, pol.n);
					
					if (d > -m) {
						float pushD = -m -d;
						tmpPt.x += pol.n.x * pushD * multFactor;
						tmpPt.y += pol.n.y * pushD * multFactor;
						tmpPt.z += pol.n.z * pushD * multFactor;
						
//						System.out.printf("d[%d] = %f%n", pol_id, d);
						maxD = Math.min(maxD, d);
					}
					
					pol_id++;
				}
				
			}
//			System.out.printf("n -> %.4f %.4f %4f%n", tmpPt.x, tmpPt.y, tmpPt.z);
			newPts.add(new Vector3(tmpPt));
		}
		
		Convex ret = new Convex(s, newPts);
		return ret;
	}
	
	public static float randomRanged(float min, float max) {
		return min + (max-min) * rng.nextFloat();
	}
	
	public static void computeTangents(Vector3 n, Vector3 t1, Vector3 t2) {
		// normal has to be normalized already
		// t1 and t2 has to be allocated already
		if (Math.abs(n.x) >= 0.57735f) {
			t1.x = n.y;
			t1.y = -n.z;
			t1.z = 0;
		} else {
			t1.x = 0;
			t1.y = n.z;
			t1.z = -n.y;
		}
		t1.normalize();
		Vector3.cross(n, t1, t2);
	}
	
	public static void calcSphereCoord(int x, int y, int [] screen, Vector3 res) {
		float vX = (float) (x-screen[0]) / (float) screen[2];
		float vY = (float) (y-screen[1]) / (float) screen[3];
		//map from [0..1] to [-1..1]
		
		vX = vX * 2.0f - 1.0f;
		vY = vY * 2.0f - 1.0f;
		
		//okay, z is simple
		float vZ = 1.0f - vX*vX - vY*vY;
		
		res.x = vX;
		res.y = vY;
		res.z = vZ;
		
		res.normalize();
	}
	
	/**
	 * Calculate quaternion rotation from 2 sphere coordinates
	 * @param scA the beginning point
	 * @param scB the end point
	 * @return a quaternion represent rotation from scA to scB
	 */
	public static Quaternion calcQuatDifference(Vector3 scA, Vector3 scB) {
		//get angle (in radians)
		float angle = (float) Math.acos(Vector3.dot(scA, scB));
		//get axis
		Vector3 axis = new Vector3();
		Vector3.cross(scA, scB, axis);
		axis.normalize();
		
		return Quaternion.makeAxisRot(axis, angle);
	}
	
	/**
	 * this wrap values to be between 0..1
	 * @param v
	 * @return value between 0..1
	 */
	public static float wrapFloat01(float v) {
		
		return (float) (v - Math.floor(v));
	}
	
	/**
	 * This converts phase [0..1] to appropriate frame time
	 * @param phase value
	 * @param trackTime [min..max] of frametime
	 * @return correct frametime
	 */
	public static float phaseToRenderTime(float phase, float [] trackTime) {
		if (trackTime.length >= 2) {
			return trackTime[0] * (1.0f-phase) + trackTime[1] * phase;
		}
		return 1.0f;
	}
	
	public static float clamp(float v, float min, float max) {
		return v < min ? min : v > max ? max : v;
	}
	
	/**
	 * This return projection of p on AB
	 * @param p 
	 * @param a
	 * @param b
	 * @return projection of p on AB
	 */
	public static Vector3 getClosestToLine(Vector3 p, Vector3 a, Vector3 b, boolean capped) {
		Vector3 ab = new Vector3(b.x-a.x, b.y-a.y, b.z-a.z);
		Vector3 ap = new Vector3(p.x-a.x, p.y-a.y, p.z-a.z);
		float vv = Vector3.dot(ab, ab);
		float uv = Vector3.dot(ap, ab);
		
		if (vv <= Vector3.EPSILON)
			return a;	// close to parallel
		
		uv/=vv;
		
		if (capped)
			uv = MathHelper.clamp(uv, 0, 1);
		
		Vector3 proj = new Vector3(
				a.x + ab.x * uv, a.y + ab.y * uv, a.z + ab.z * uv
				);
		return proj;
	}
	
	public static Vector3 naiveClosestToTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {		
		// list of candidates
		Vector3 [] candidate = new Vector3[4];
		int canSize = 3;
		
		candidate[0] = getClosestToLine(p, a, b, true);
		candidate[1] = getClosestToLine(p, b, c, true);
		candidate[2] = getClosestToLine(p, c, a, true);
		
		// only add if triangle area is valid
		float tArea = MathHelper.calcTriangleArea(a, b, c); 
		if (tArea > Vector3.EPSILON) {
			candidate[3] = getClosestToTriangle(p, a, b, c);
			canSize = 4;
		} /*else {
			System.out.println("naiveClosestTriangle: degenerate triangle!!" + tArea);
		}*/
		
		
		Vector3 ret = candidate[0];
		float dist = Vector3.distBetween(p, candidate[0]);
		
		for (int i=1; i<canSize; i++) {
			float newDist = Vector3.distBetween(p, candidate[i]);
			
			if (newDist < dist) {
				dist = newDist;
				ret = candidate[i];
			}
		}
		
		return ret;
	}
	
	public static Vector3 naiveClosestToTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
		// list of candidates
		Vector3 [] candidate = new Vector3[5];
		int canSize	= 4;
		
		candidate[0] = naiveClosestToTriangle(p, a, b, c);
		candidate[1] = naiveClosestToTriangle(p, b, c, d);
		candidate[2] = naiveClosestToTriangle(p, c, d, a);
		candidate[3] = naiveClosestToTriangle(p, a, b, d);
		
		float tetraVol = Math.abs( MathHelper.calcTetraVolume(a, b, c, d) );
		
		if (tetraVol > Vector3.EPSILON) {
			candidate[4] = getClosestToTetrahedron(p, a, b, c, d);
			canSize = 5;
		} /*else {
			System.out.println("naiveClosestTetrahedron: degenerate tetrahedron!!" + tetraVol);
		}*/
		
		
		Vector3 ret = candidate[0];
		float dist = Vector3.distBetween(p, candidate[0]);
		
		for (int i=1; i<canSize; i++) {
			float newDist = Vector3.distBetween(p, candidate[i]);
			
			if (newDist < dist) {
				dist = newDist;
				ret = candidate[i];
			}
		}
		
		return ret;
	}
	
	public static Vector3 getClosestToTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
		Vector3 edge0 = Vector3.tmp0;
		Vector3 edge1 = Vector3.tmp1;
		Vector3 v0 = Vector3.tmp2;
		
		Vector3.sub(b, a, edge0);
		Vector3.sub(c, a, edge1);
		Vector3.sub(a, p, v0);
		
		float fa = Vector3.dot(edge0, edge0);
		float fb = Vector3.dot(edge0, edge1);
		float fc = Vector3.dot(edge1, edge1);
		float fd = Vector3.dot(edge0, v0);
		float fe = Vector3.dot(edge1, v0);
		
		float det = fa*fc - fb*fb;
		float s = fb*fe - fc*fd;
		float t = fb*fd - fa*fe;
		
		if (s+t < det) {
			if (s < 0.0f) {
				if (t < 0.0f) {
					if (fd < 0.0f) {
						s = MathHelper.clamp(-fd/fa,  0, 1);
						t = 0;
					} else {
						s = 0;
						t = MathHelper.clamp(-fe/fc, 0, 1);
					}
				} else {
					s = 0;
					t = MathHelper.clamp(-fe/fc, 0, 1);
				}
			} else if (t < 0.0f) {
				s = MathHelper.clamp(-fd/fa, 0, 1);
				t = 0;
			} else {
				float invDet = 1.0f/det;
				s *= invDet;
				t *= invDet;
			}
		} else {
			if (s < 0.0f) {
				float tmp0 = fb+fd;
				float tmp1 = fc+fe;
				if (tmp1 > tmp0) {
					float numer = tmp1 - tmp0;
					float denom = fa-2*fb+fc;
					s = MathHelper.clamp(numer/denom, 0, 1);
					t = 1.0f-s;
				} else {
					t = MathHelper.clamp(-fe/fc, 0, 1);
					s = 0.0f;
				}
			} else if (t < 0.0f) {
				if (fa+fd > fb+fe) {
					float numer = fc+fe-fb-fd;
					float denom = fa-2*fb+fc;
					s = MathHelper.clamp(numer/denom, 0, 1);
					t = 1.0f-s;
				} else {
					s = MathHelper.clamp(-fe/fc, 0, 1);
					t = 0.0f;
				}
			} else {
				float numer = fc+fe-fb-fd;
				float denom = fa-2*fb+fc;
				s = MathHelper.clamp(numer/denom, 0, 1);
				t = 1.0f-s;
			}
		}
		
		return new Vector3(
				a.x + s * edge0.x + t * edge1.x,
				a.y + s * edge0.y + t * edge1.y,
				a.z + s * edge0.z + t * edge1.z
				);
	}
	
	public static Vector3 getClosestToTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
		// closest on tetrahedron must be closest to triangle too
		Quaternion bary = computeBarycentric(p, a, b, c, d);
		
		// depending on the returned value, it might be inside too
		if (bary.x >= 0 && bary.y >= 0 && bary.z >= 0 && bary.w >= 0) {
			//inside!!
			return new Vector3(
					bary.x * a.x + bary.y * b.x + bary.z * c.x + bary.w * d.x,
					bary.x * a.y + bary.y * b.y + bary.z * c.y + bary.w * d.y,
					bary.x * a.z + bary.y * b.z + bary.z * c.z + bary.w * d.z
					);
		} else if (bary.x < 0) {
			// on triangle BCD
			return getClosestToTriangle(p, b, c, d);
		} else if (bary.y < 0) {
			// on triangle ACD
			return getClosestToTriangle(p, a, c, d);
		} else if (bary.z < 0) {
			// on triangle ABD
			return getClosestToTriangle(p, a, b, d);
		} else {
			// force it on triangle ABC
			return getClosestToTriangle(p, a, b, c);
		}
	}
	
	public static Vector3 oldGetClosestToTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
		Vector3 ab = new Vector3();
		Vector3 ac = new Vector3();
		
		Vector3.sub(b, a, ab);
		Vector3.sub(c, a, ac);
		
		Vector3 rawNormal = new Vector3();
		Vector3.cross(ab, ac, rawNormal);
		
		Vector3 aNP = MathHelper.getClosestToLine(p, a, new Vector3(a.x + rawNormal.x, a.y + rawNormal.y, a.z + rawNormal.z), false);
		
		Vector3 flatP = new Vector3();
		flatP.x = p.x + a.x - aNP.x;
		flatP.y = p.y + a.y - aNP.y;
		flatP.z = p.z + a.z - aNP.z;
		
		// test a against BC
		Vector3 pBC = MathHelper.getClosestToLine(p, b, c, true);
		Vector3.sub(a, pBC, Vector3.tmp0);
		Vector3.sub(p, pBC, Vector3.tmp1);
		float d = Vector3.dot(Vector3.tmp0, Vector3.tmp1);
		if (d < Vector3.EPSILON)
			return pBC;
		
		// test b against CA
		Vector3 pCA = MathHelper.getClosestToLine(p, c, a, true);
		Vector3.sub(b, pCA, Vector3.tmp0);
		Vector3.sub(p, pCA, Vector3.tmp1);
		d = Vector3.dot(Vector3.tmp0, Vector3.tmp1);
		if (d < Vector3.EPSILON)
			return pCA;
		
		// test c against AB
		Vector3 pAB = MathHelper.getClosestToLine(p, a, b, true);
		Vector3.sub(c, pAB, Vector3.tmp0);
		Vector3.sub(p, pAB, Vector3.tmp1);
		d = Vector3.dot(Vector3.tmp0, Vector3.tmp1);
		if (d < Vector3.EPSILON)
			return pAB;
		
		// it's inside, just return it
		return flatP;
	}
	
	/**
	 * computeBarycentric - compute barycentric coordinate of a point w.r.t triangle
	 * @param p		- the point in question
	 * @param a		- triangle point A
	 * @param b		- triangle point B
	 * @param c		- triangle point C
	 * @return	the u,v,w (barycentric coordinate in range [0..1])
	 */
	public static Vector3 computeBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
		Vector3 AB = new Vector3();
		Vector3 AC = new Vector3();
		
		Vector3.sub(b, a, AB);
		Vector3.sub(c, a, AC);
		
		// compute triangle normal
		Vector3 n = new Vector3();
		Vector3.cross(AB, AC, n);
		
		float dsABC = Vector3.dot(n, n);	// dot squared of ABC area
		
		if (dsABC < Vector3.EPSILON)
			return new Vector3();	// return 0!! --> DEGENERATE TRIANGLE CASE!!
		
		// for temporarily hold triangle edge
		Vector3 v0 = new Vector3();
		Vector3 v1 = new Vector3();
		Vector3 tN = new Vector3();
		float u,v,w;
		
		// AREA apc	--> barycentric for b aka v
		Vector3.sub(p, a, v0);
		v1.setTo(AC);
		
		Vector3.cross(v0, v1, tN);
		
		v = Vector3.dot(tN, n) / dsABC;
		
		// AREA abp --> barycentric for c aka w
		v1.setTo(v0);
		v0.setTo(AB);
		
		Vector3.cross(v0, v1, tN);
		
		w = Vector3.dot(tN, n) / dsABC;
		
		// AREA bcp --> barycentric for a aka u
		Vector3.sub(c, b, v0);
		Vector3.sub(p, b, v1);
		
		Vector3.cross(v0, v1, tN);
		
		u = Vector3.dot(tN, n) / dsABC;
		
		return new Vector3(u, v, w);
	}
	
	public static Vector3 flatten(Vector3 p, Vector3 pPos, Vector3 pNormal) {
		// normal doesn't need to be normalized
		Vector3 AP = new Vector3(p, pPos);
		
		float uv = Vector3.dot(AP, pNormal);
		float vv = Vector3.dot(pNormal, pNormal);
		
		if (Math.abs(vv) < Vector3.EPSILON) {
			// whoa, dangerous!!
			return null;
		}
		
		uv /= vv;
		
		return new Vector3(
				p.x - pNormal.x * uv,
				p.y - pNormal.y * uv,
				p.z - pNormal.z * uv
				);
	}
	
	public static float calcTetraVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
		Vector3 v0 = new Vector3();
		Vector3 v1 = new Vector3();
		Vector3 v2 = new Vector3();
		
		Vector3.sub(b, a, v1);	// AB
		Vector3.sub(c, a, v0);	// AC
		Vector3.sub(d, a, v2);	// AD
		
		Vector3.cross(v0, v1, Vector3.tmp0);	// AC x AB (so result will face AD)
		float volABCD = Vector3.dot(Vector3.tmp0, v2);
		
		return volABCD;
	}
	
	/**
	 * computeBarycentric - this one computes barycentric coord of a point to a tetrahedron
	 * @param p
	 * @param a
	 * @param b
	 * @param c
	 * @param d
	 * @return the x,y,z,w of each point
	 */
	public static Quaternion computeBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
		// in gjk-produced simplex, tetrahedron ABC + D would be:
		//	ABC is facing away from origin
		//	D is in the back of ABC, shared by A,B,C
		//	FRONT FACES ARE CCW!!
		Vector3 tmp = Vector3.tmp0;
		// Compute main volume
		Vector3 v0 = new Vector3();
		Vector3 v1 = new Vector3();
		Vector3 v2 = new Vector3();
		
		Vector3.sub(b, a, v1);	// AB
		Vector3.sub(c, a, v0);	// AC
		Vector3.sub(d, a, v2);	// AD
		
		Vector3.cross(v0, v1, tmp);	// AC x AB (so result will face AD)
		float volABCD = Vector3.dot(tmp, v2);
		
		// if this is too small, we cannot do anything
		if (Math.abs(volABCD) < Vector3.EPSILON)
			return new Quaternion(0,0,0,0);
		
		// compute volume ABC + P
		Vector3.sub(p, a, v2);	// AP
		float volABCP = Vector3.dot(tmp, v2);
		
		// compute volume BCD + P
		v2.setTo(v0);			// AC --> save computation :p
		Vector3.sub(b, c, v0);	// CB
		Vector3.sub(d, c, v1);	// CD
		Vector3.sub(p, c, v2);	// PC
		
		Vector3.cross(v1, v0, tmp);
		float volBCDP = Vector3.dot(tmp, v2);
		
		// compute volume ABD + P
		Vector3.sub(a, b, v0);	// BA
		Vector3.sub(d, b, v1);	// BD
		Vector3.sub(p, b, v2);	// BP
		
		Vector3.cross(v1, v0, tmp);
		float volABDP = Vector3.dot(tmp, v2);
		
		// compute volume ADC + P
		Vector3.sub(c, a, v0);	// AC
		Vector3.sub(d, a, v1);	// AD
		Vector3.sub(p, a, v2);	// AP
		
		Vector3.cross(v1, v0, tmp);
		float volADCP = Vector3.dot(tmp, v2);
		
//		System.out.println("ABCD: " + volABCD);
//		System.out.println("ADCP: " + volADCP);
//		System.out.println("ABDP: " + volABDP);
//		System.out.println("ABCP: " + volABCP);
		
		// now we normalize them
		volBCDP /= volABCD;		// this is A's strength
		volADCP /= volABCD;		// this is B's
		volABDP /= volABCD;		// this is C's
		volABCP /= volABCD;		// this is D's
		
		return new Quaternion(volBCDP, volADCP, volABDP, volABCP);
	}
	
//	public static Vector3 [] gjkClosestPoint(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, Simplex simp) {
//		// reset simplex
//		if (simp == null)
//			simp = new Simplex();
//		else
//			simp.reset();
//		// give initial support
//		Vector3 dir = new Vector3(1, 0, 0);
//		
//		simp.addSupportConservatively(Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir));
//		
//		// reverse direction
//		dir.scale(-1.0f);
//		
//		// this is to ensure termination (INCREASE FOR MORE accuracy for curved shapes)
//		int iter = 0;
//		
//		float dist = 999999.f;
//		
//		while (iter++ < GJK_MAX_ITERATION) {
//			// add support point. remove unimportant one from simplex
//			simp.addSupportConservatively(Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir));
//			
//			Vector3 cl = simp.closestToOrigin();
//			float d = cl.lengthSquared();
//			
//			// means we won't get better
//			if (Math.abs(d-dist) < 0.001f) {
//				// early out. Would be better to keep track of last direction?
//				// wait. ensure we get triangle
//				dist = Math.min(d, dist);
//				break;
//			} else {
//				// do we have origins?
//				if (simp.hasOrigin()) {
//					// stop here
////							System.out.println("gjk terminate iter: " + iter);
//					break;
//				} else {
//					// welp, get new direction then
//					dir = simp.calcNewDir();
//					simp.lastDir = dir;
//				}
//			}
//		}
//		Vector3 [] cp = new Vector3[]{
//				new Vector3(),
//				new Vector3()
//		};
//		
//		simp.getClosestPoint(cp[0], cp[1]);
//		return cp;
//	}
	

	
//	public static boolean gjkClosestPoint(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, Vector3 dir, Simplex simp) {
//		if (simp == null)
//			simp = new Simplex();
//		else
//			simp.reset();
//		
//		System.out.println("=============GJK CLOSEST START==================");
//		
//		// we add single support point first
//		simp.addSupportConservatively(Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir));
////		simp.addSupport(Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir));
//		
//		// direction reversed
//		dir.scale(-1.0f);
//		
//		// last result
//		float closestDist = Float.MAX_VALUE;	// set to maximum. we will improve our distance
//		
//		int iter = 0;
//		
//		while (iter ++ < GJK_MAX_ITERATION) {
//			// potential to get stuck in infinite loop
//			System.out.println("gjk closest @ " + iter);
//			// add point to simplex
//			CSOVertex v = Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir);
//			simp.addSupportConservatively(v);
////			simp.addSupport(Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir));
//			
//			// grab closest point to track our distance so far
//			Vector3 closest = simp.closestToOrigin();
//			
//			System.out.println("gjk closest: closest= " + closest.x+", "+closest.y+", "+closest.z);
//			
//			if (simp.hasOrigin()) {
//				// welp, cannot be true
//				System.out.println("gjk closest: simplex has origin");
//				return false;
//			} else {
//				// well, let's see
//				float d = closest.lengthSquared();
//				
//				System.out.println("gjk closest: dist => " + d + ", " + closestDist+ " @ iter " + (iter));
//				
//				
//				// how close?
//				float diff = Math.abs(closestDist - d);
//				
//				if (diff < GJK_MARGIN_ERROR) {
//					System.out.println("============GJK END============= @ " + iter);
//					System.out.printf("gjk closest: terminate with separation distance -> %.6f %n", Math.abs(closestDist-d));
//					return true;
//				} else {
//					// the separation is too far... update distance instead
//					if (d < closestDist) {
//						closestDist = d;
//					} else if (d > closestDist) {
//						System.out.println("gjk closest: we cannot get better than this -> " + closestDist + " vs " + d);
//						
//						// well, bad vertex added....maybe remove it?
//						System.out.println("gjk closest: BAD VERTEX { " + v.p.x + ", "+v.p.y+","+v.p.z + "}");
////						simp.removeVertex(v);
//						
//						
////						return true;
//					}
//					
//				} 
//				// get new direction
////				Vector3 normalDir = simp.calcNewDir();
////				dir = closest.inverse();
//				
//				// check direction though				
//				if (closest.lengthSquared() < Vector3.EPSILON) {
//					System.out.println("gjk closest: UNSAFEEE dir --> " + closest.lengthSquared());
//					dir = simp.calcNewDir();
//				} else {
//					System.out.println("gjk closest: safe dir");
//					dir = closest.inverse();
//				}
//			}
//		}
//		return false;
//	}
	
	
	public static float calcTriangleArea(Vector3 a, Vector3 b, Vector3 c) {
		Vector3 AB = new Vector3();
		Vector3 AC = new Vector3();
		Vector3.sub(b, a, AB);
		Vector3.sub(c, a, AC);
		Vector3.cross(AB, AC, AC);
		return Vector3.dot(AC, AC);
	}
	
//	public static Vector3 triRawNormal(Vector3 a, Vector3 b, Vector3 c) {
//		
//	}
	
	/**
	 * epaExecute	- execute EPA on pair of shapes
	 * DO NOTE that this MUST ONLY BE CALLED IF GJK RETURNS POSITIVE COLLISION!!!
	 * POSITIVE COLLISION MEANS THE POLYTOPE CONTAINS ORIGIN INITIALLY
	 * @param sA	-- shape A
	 * @param posA	-- position of shape A
	 * @param rotA	-- rotation of shape A
	 * @param sB	-- shape B
	 * @param posB	-- position of shape B
	 * @param rotB	-- rotation of shape B
	 * @param poly	- initial polytope from resulting gjk (right, you must build it yerself)
	 * @return	- the information (MTD and normal of collision)
	 */
	public static EPAInfo epaExecute(Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, 
			Polytope poly) {
		// when an idiot uses this
		if (poly == null) {
			return null;
		}
		
		// smart user found. give him blowjob
		int iterCnt = 0;
		Vector3 proj = new Vector3();
		Vector3 o = Vector3.ZERO;
		while (iterCnt++ < EPA_MAX_ITERATION) {
			// Grab closest triangle from origin + projection
			EPATriangle t = poly.getClosestTriangle(o, proj);
			
			// if proj is too close, use normal
			if (t == null) {
				System.out.println("MATH EPA!! IMPOSSIBLE ERROR!!!!  " + poly.tris.size());
				return null;
			}
			Vector3 dir = t.n;
			/*if (proj.lengthSquared() < Vector3.EPSILON) {
				dir = t.n;
			} else {
				dir = proj;
			}*/
			
			// we get direction, query for support point. triangle normal will point away from origin
			CSOVertex sup = Shape.minkowskiDiff(sA, posA, rotA, sB, posB, rotB, dir);
			
			// are we close?
			float d = Vector3.dot(sup.p, dir) - Vector3.dot(proj, dir);
			if (Math.abs(d) <= Vector3.EPSILON) {
				// won't get better!!. This is IT!!
				// let's calculate projections' barycentric
				Vector3 bary = computeBarycentric(proj, t.a.p, t.b.p, t.c.p);
				
				if (dir.lengthSquared() < Vector3.EPSILON) {
					System.out.println("Uh oh! worrying normal shit!!");
				}
				
				return new EPAInfo(new Vector3(proj), dir.normalized(), t, bary);
			} else {
				// add the new support point
				poly.addPoint(sup);
			}
		}
		return null;
	}
}
