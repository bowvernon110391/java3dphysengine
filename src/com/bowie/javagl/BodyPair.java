package com.bowie.javagl;

public class BodyPair {
	private RigidBody bodyA, bodyB;
	
	public BodyPair(RigidBody b1, RigidBody b2) {
		bodyA = b1;
		bodyB = b2;
		if (b2.getId() < b1.getId()) {
			// flip
			bodyA = b2;
			bodyB = b1;
		}
	}
	
	@Override
	public boolean equals(Object arg0) {
		BodyPair p = (BodyPair)arg0;
		
		return 	(p.bodyA == bodyA && p.bodyB == bodyB) || 
				(p.bodyA.getId() == bodyA.getId() && p.bodyB.getId() == bodyB.getId()) ||
				(p.bodyA == bodyB && p.bodyB == bodyA) ||
				(p.bodyA.getId() == bodyB.getId() && p.bodyB.getId() == bodyA.getId()) ||
				(arg0 == this);
	}
	
	@Override
	public int hashCode() {
		// first 16 bit is bodyA id, next 16 bit is bodyB's
		return (bodyA.getId() << 16) | (bodyB.getId() & 0xFFFF);
	}
	
	public boolean stillInProximity() {
		return bodyA.getBbox().overlap(bodyB.getBbox());
	}

	public RigidBody getBodyA() {
		return bodyA;
	}

	public RigidBody getBodyB() {
		return bodyB;
	}
}
