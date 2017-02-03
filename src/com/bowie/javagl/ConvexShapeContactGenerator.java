package com.bowie.javagl;

public class ConvexShapeContactGenerator implements ContactGenerator {
	private static final boolean debug = false;
	
	public Simplex simp;
	public Polytope poly;
	
	public Vector3 cA, cB;
	public Vector3 rS, rE;
	
	public float speculativeMargin;
	
	public ConvexShapeContactGenerator() {
		simp = new Simplex(null, null, null, null, null, null);
		poly = new Polytope(null);
		
		cA = new Vector3();
		cB = new Vector3();
		
		rS = new Vector3();
		rE = new Vector3();
		
		speculativeMargin = 0.1f;
	}

	@Override
	public int generateContact(PersistentManifold m, Shape sA, Vector3 posA,
			Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB,
			RigidBody bA, RigidBody bB) {
		// first, this works for anything that is convex, so make sure
		int id0 = sA.getShapeID();
		int id1 = sB.getShapeID();
		
		// make sure it's valid shape
		if (id0 != Shape.SHAPE_BOX && id0 != Shape.SHAPE_CONVEX || id1 != Shape.SHAPE_BOX && id1 != Shape.SHAPE_CONVEX)
			return 0;
		
		// welp, let's ask help
//		Simplex simp = new Simplex(sA, posA, rotA, sB, posB, rotB);
		// gib data!!
		simp.sA = sA;
		simp.sB = sB;
		
		simp.posA = posA;
		simp.posB = posB;
		
		simp.rotA = rotA;
		simp.rotB = rotB;
		
		// DO ISA GJK IF NONE IS CONTINUOUS
		// performCCD -> doISAGJK
		boolean useISAGJK = !bA.isContinuous() && !bB.isContinuous();
		boolean performCCD = bA.isContinuous() || bB.isContinuous();
		
		simp.reset();		
		boolean collide = simp.GJK(useISAGJK);
		
		int nc = 0;
		
		if (collide) {
			poly.reset();
			poly.init(simp);
			
			EPAInfo info = MathHelper.epaExecute(sA, posA, rotA, sB, posB, rotB, poly);
			
			if (info != null) {
				Vector3 cpA = info.calcContactA();
				Vector3 cpB = info.calcContactB();
				Vector3 n = info.getNormal().inverse();
				
				Contact c = new Contact(cpA, cpB, n, bA, bB);
				m.add(c);
				nc++;
			}
		} 
		
		// ccd is always performed on continuous body regardless of discrete cd result
		if (performCCD) {
			// we only make sure it's between a or b
			if (bA.isContinuous()) {				
				// we'll make body B as our "target"
				simp.sA = sB;
				simp.posA = posB;
				simp.rotA = rotB;
				
				// grab body A linear velocity
				// because this is the true velocity of the embedded sphere
				Vector3 vel = new Vector3(bA.getVel(), bB.getVel());
				
				// generate ray (from posA to posA+vel)
				rS.setTo(posA);
				rE.setTo(
						posA.x + vel.x,
						posA.y + vel.y,
						posA.z + vel.z
						);
				
				// do raycast
				int retVal = simp.doRayCast(rS, rE, bA.getCcdRadius());
				
				if (retVal == Simplex.RAY_HIT) {
					// add speculative contact
					// grab teh distance
					Vector3 pN = Vector3.tmp0;
					Vector3.sub(rS, simp.rayhitPos, pN);
					float dN = Vector3.dot(pN, simp.rayhitNormal);
					
					if (debug)
						System.out.printf("A CCD: dn = %.4f, toi = %.4f%n", dN, simp.rayT);
					
					// add speculative contact
					SpeculativeContact sc = new SpeculativeContact(dN, simp.rayhitNormal, simp.rayT, bA, bB);
					m.addSpeculativeContact(sc);
					nc++;
				}
			}
			if (bB.isContinuous()) {
				// also perform similar test
				
				// we'll make body A as our "target"
				simp.sA = sA;
				simp.posA = posA;
				simp.rotA = rotA;
				
				// grab body B linear velocity
				// because this is the true velocity of the embedded sphere
				Vector3 vel = new Vector3(bB.getVel(), bA.getVel());
				
				// generate ray (from posB to posB+vel)
				rS.setTo(posB);
				rE.setTo(
						posB.x + vel.x,
						posB.y + vel.y,
						posB.z + vel.z
						);
				
				// do raycast
				int retVal = simp.doRayCast(rS, rE, bB.getCcdRadius());
				
				if (retVal == Simplex.RAY_HIT) {
					// add speculative contact
					// grab teh distance
					Vector3 pN = Vector3.tmp0;
					Vector3.sub(rS, simp.rayhitPos, pN);
					float dN = Vector3.dot(pN, simp.rayhitNormal);
					
					if (debug)
						System.out.printf("B CCD: dn = %.4f, toi = %.4f%n", dN, simp.rayT);
					
					// add speculative contact (reverse body position)
					SpeculativeContact sc = new SpeculativeContact(dN, simp.rayhitNormal, simp.rayT, bB, bA);
					m.addSpeculativeContact(sc);
					nc++;
				}
			}
		}
		
		return nc;
	}

}
