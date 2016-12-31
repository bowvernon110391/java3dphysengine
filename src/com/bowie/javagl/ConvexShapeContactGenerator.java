package com.bowie.javagl;

public class ConvexShapeContactGenerator implements ContactGenerator {

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
		Simplex simp = new Simplex();
		boolean collide = MathHelper.gjkColDet(sA, posA, rotA, sB, posB, rotB, new Vector3(1,0,0), simp);
		
		if (collide) {
			Polytope poly = new Polytope(simp);
			
			EPAInfo info = MathHelper.epaExecute(sA, posA, rotA, sB, posB, rotB, poly);
			
			if (info != null) {
				Vector3 cpA = info.calcContactA();
				Vector3 cpB = info.calcContactB();
				Vector3 n = info.getNormal().inverse();
				
				Contact c = new Contact(cpA, cpB, n, bA, bB);
				m.add(c);
				return 1;
			}
		}
		
		return 0;
	}

}
