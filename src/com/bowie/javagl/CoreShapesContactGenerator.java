package com.bowie.javagl;

public class CoreShapesContactGenerator implements ContactGenerator {
	private static final boolean debug = false;
	
	public static int gjkCount = 0;
	public static int epaCount = 0;
	
	public Simplex simp;
	public Polytope poly;
	
	public Vector3 rS = new Vector3();
	public Vector3 rE = new Vector3();
	
	public CoreShapesContactGenerator() {
		simp = new Simplex(null, null, null, null, null, null);
		poly = new Polytope(null);
	}
	
	public Contact generate(Shape sA, Vector3 posA,
			Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB,
			RigidBody bA, RigidBody bB) {
		// welp, make sure both are core shapes
		if (sA.getShapeID() != sB.getShapeID() && sA.getShapeID() != Shape.SHAPE_CORE)
			return null;
		
//		System.out.println("CSCG: start....");
		// wokaaay, we're saved. build simplex
		CoreShape csA = (CoreShape) sA;
		CoreShape csB = (CoreShape) sB;
		
		// early out. check outer shapes, are they touching?
//		Simplex simp = new Simplex();
		
		simp.sA = csA.outerShape;
		simp.sB = csB.outerShape;
		
		simp.posA = posA;
		simp.posB = posB;
		
		simp.rotA = rotA;
		simp.rotB = rotB;
		
		// do fast ISA GJK (boolean test) of outer shapes
		simp.reset();
		boolean collide = simp.GJK(true);
		if (!collide) {
			// outer shape's didn't even touch
			return null;
		} else {
			// outer shapes intersect, now do pure GJK using core shapes
			simp.reset();
			
			simp.sA = csA.innerShape;
			simp.sB = csB.innerShape;
			
			// check if inner shapes are colliding, using pure GJK
			collide = simp.GJK(false);
			if (collide) {
				// fallback to epa using outer shapes
				poly.reset();
				poly.init(simp);
				EPAInfo info = MathHelper.epaExecute(csA.outerShape, posA, rotA, csB.outerShape, posB, rotB, poly);
				
				if (info != null) {
//					System.out.println("CSCG: EPA SUCCESS!!");
					
					Vector3 cpA = info.calcContactA();
					Vector3 cpB = info.calcContactB();
					Vector3 n = info.getNormal().inverse();
					
					Contact c = new Contact(cpA, cpB, n, bA, bB);
					
					return c;

				}
			} else {
				// well, outer shapes are colliding, but inner shapes don't
				// that means it's safe to use closest points
				
				Vector3 cpA = new Vector3();
				Vector3 cpB = new Vector3();
				
				simp.getClosestPoint(cpA, cpB);
				
				Vector3 n = new Vector3(cpA, cpB);
				float dist = n.lengthSquared();
				float margins = csA.margin + csB.margin;
				
				if (dist < margins*margins) {
					// construct contact
					n.normalize();
					
					cpA.x -= n.x * csA.margin;
					cpA.y -= n.y * csA.margin;
					cpA.z -= n.z * csA.margin;
					 
					cpB.x += n.x * csB.margin;
					cpB.y += n.y * csB.margin;
					cpB.z += n.z * csB.margin;
					
					Contact c = new Contact(cpA, cpB, n, bA, bB);
					return c;
				} 
			}
		}
		
		
		return null;
	}

	@Override
	public int generateContact(PersistentManifold m, Shape sA, Vector3 posA,
			Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB,
			RigidBody bA, RigidBody bB) {
		// welp, make sure both are core shapes
		if (sA.getShapeID() != sB.getShapeID() && sA.getShapeID() != Shape.SHAPE_CORE)
			return 0;
		
		// let's build contact
		
		Contact c = this.generate(sA, posA, rotA, sB, posB, rotB, bA, bB);
		
		int nc=0;
		if (c != null && m != null) {
			m.add(c);
			nc++;
		}
		
		// ccd is always performed on continuous body regardless of discrete cd result
		if (bA.isContinuous() || bB.isContinuous()) {
			// we only make sure it's between a or b
			if (bA.isContinuous()) {
				if (debug)
					System.out.println("Body A is continuous!!");
				
				// we'll make body B's outer shape as our "target"
				simp.sA = ((CoreShape)sB).outerShape;
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
						System.out.printf("CCD: dn = %.4f, toi = %.4f%n", dN, simp.rayT);
					
					// add speculative contact
					SpeculativeContact sc = new SpeculativeContact(dN, simp.rayhitNormal, simp.rayT, bA, bB);
					m.addSpeculativeContact(sc);
					nc++;
				} else {
					if (debug)
						System.out.println("But rayhit test = " + retVal);
				}
			}
			if (bB.isContinuous()) {
				// also perform similar test
				if (debug)
					System.out.println("Body B is continuous!!");
				
				// we'll make body A as our "target"
				simp.sA = ((CoreShape)sA).outerShape;
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
						System.out.printf("CCD: dn = %.4f, toi = %.4f%n", dN, simp.rayT);
					
					// add speculative contact (reverse body position)
					SpeculativeContact sc = new SpeculativeContact(dN, simp.rayhitNormal, simp.rayT, bB, bA);
					m.addSpeculativeContact(sc);
					nc++;
				} else {
					if (debug)
						System.out.println("But rayhit test = " + retVal);
				}
			}
		}
		
		return 0;
	}

}
