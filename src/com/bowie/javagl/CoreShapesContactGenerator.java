package com.bowie.javagl;

public class CoreShapesContactGenerator implements ContactGenerator {
	
	public static int gjkCount = 0;
	public static int epaCount = 0;
	
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
		Simplex simp = new Simplex();
		
		Vector3 dir = new Vector3(1,0,0);
		boolean collide = MathHelper.gjkColDet(csA.outerShape, posA, rotA, csB.outerShape, posB, rotB, dir, simp);
		if (!collide) {
			return null;
		} else {
			simp.reset();
			dir.setTo(1,0,0);
			
			// check if inner shapes are colliding
			collide = MathHelper.gjkColDet(csA.innerShape, posA, rotA, csB.innerShape, posB, rotB, dir, simp);
			if (collide) {
				// fallback to epa
				Polytope poly = new Polytope(simp);
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
				// run gjk closest point detection
				simp = new Simplex(csA.innerShape, posA, rotA, csB.innerShape, posB, rotB, new Vector3(1,0,0));
				
				while (!simp.converged) {
					if (!simp.advance())
						break;
					if (simp.containsOrigin)
						break;
				}
				
				// check result
				if (simp.converged) {
//					gjkCount++;
//					System.out.println("CSCG: got closest distance, but how far?");
					 Vector3 n = new Vector3(simp.cA, simp.cB);
					 
					 float distance = n.length();
					 
					 // total distance within margin
					 if (distance < csA.margin + csB.margin) {
//						 System.out.println("CSCG: distance < MARGIN!! WE COLLIDE!!");
						 // we collide, generate contact information
						 n.normalize();
						 
						 // grab the real contactpoints
						 Vector3 cpA = new Vector3(simp.cA);
						 cpA.x -= n.x * csA.margin;
						 cpA.y -= n.y * csA.margin;
						 cpA.z -= n.z * csA.margin;
						 
						 Vector3 cpB = new Vector3(simp.cB);
						 cpB.x += n.x * csB.margin;
						 cpB.y += n.y * csB.margin;
						 cpB.z += n.z * csB.margin;
						 
						 // we get all necessary stuffs now, store it
						 Contact c = new Contact(cpA, cpB, n, bA, bB);
						 
						return c;
					 } /*else {
						 System.out.println("CSCG: distance is too far... -> " + distance);
					 }*/
					 
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
		
		if (c != null && m != null) {
			m.add(c);
			return 1;
		}
		
		return 0;
	}

}
