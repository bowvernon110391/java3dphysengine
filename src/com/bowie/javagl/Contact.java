package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class Contact {
	public static float MIN_RESTITUTION = 0.03f;		// minimum velocity to bounce off (0.03 m/s)
	public static float RESTITUTION_DISSIPATE = 0.01f;	// .1% dissipated every solver iteration
	
	public Vector3 localA, localB;	// contact point in local space
	public Vector3 worldA, worldB;	// contact point in world space
	public float depth;		// negative means penetration
	public Vector3 normal;	// points toward A
	public Vector3 localNormalA, localNormalB, positionNormal;
	public Vector3 tangent1, tangent2;
	
	public float accumPN, accumPT1, accumPT2;	// accumulated impulse
	public float accumBias;
	public float bias, friction, restitution, kRestitution;				// bias impulse and friction constant, and restitution
	public float massN, massT1, massT2;	// effective mass of each constraint
	
	public RigidBody bodyA, bodyB;	// rigid body
	
	
	
	public Contact(Vector3 worldA, Vector3 worldB, Vector3 normal, RigidBody bA, RigidBody bB) {
		this.worldA = new Vector3(worldA);
		this.worldB = new Vector3(worldB);
		this.normal = new Vector3(normal);
		this.bodyA = bA;
		this.bodyB = bB;
		
		this.accumPN = 0;
		this.accumPT1 = 0;
		this.accumPT2 = 0;
		this.accumBias = 0;
		this.bias = 0;
		// compute local and depth
		this.computeLocalAndDepth();
	}
	
	//--------------------------------------------------------
	// EXPERIMENTAL!! FOR POSITION CORRECTION!!!
	/*public void updateWorldPosAndDepth() {
		// update world contact position
		this.worldA = bodyA.toWorld(localA);
		this.worldB = bodyB.toWorld(localB);
		
		// depth = (worldA - worldB) * normal
		// however, we can't use normal from collision detection
		// because both bodies have moved
		// so --> recalculate normal (averaging absolute normal)
		Vector3 wNA = new Vector3(this.localNormalA);
		bodyA.getRot().transformVector(wNA, wNA);
		Vector3 wNB = new Vector3(this.localNormalB);
		bodyB.getRot().transformVector(wNB, wNB);
		
		Vector3.add(wNA, wNB, wNA);
		// normalize
		wNA.normalize();
		this.positionNormal = new Vector3(wNA);
		
		Vector3 AB = new Vector3();
		Vector3.sub(worldA, worldB, AB);
		this.depth = Vector3.dot(AB, positionNormal);
	}
	
	public float computePseudoImpulse(float baumgarte, float slop) {
		// next, compute effective mass
		// check if depth is good
		float C = Math.max(-depth-slop, 0) * baumgarte;
		// compute effective mass
		Vector3 r1 = new Vector3();
		Vector3 r2 = new Vector3();
		
		bodyA.getRot().transformVector(localA, r1);
		bodyB.getRot().transformVector(localB, r2);
		
		Vector3 rn1 = new Vector3();
		Vector3 rn2 = new Vector3();
		
		// calculate mass normal
		Vector3.cross(r1, positionNormal, rn1);
		Vector3.cross(rn1, r1, rn1);
		bodyA.getInvInertia().transformVector3(rn1, rn1);
		
		Vector3.cross(r2, positionNormal, rn2);
		Vector3.cross(rn2, r2, rn2);
		bodyB.getInvInertia().transformVector3(rn2, rn2);
		
		Vector3.add(rn1, rn2, rn1);
		
		float cMass = bodyA.getInvMass() + bodyB.getInvMass();
		cMass += Vector3.dot(rn1, positionNormal);
		cMass = 1.0f/cMass;
		
		return cMass * C;
	}
	
	public void applyPseudoImpulse(float baumgarte, float slop) {
		// holy shit this is it!!
		this.updateWorldPosAndDepth();
		float jN = computePseudoImpulse(baumgarte, slop);
		
		if (jN >= Vector3.EPSILON)
			System.out.println("position impulse: " + jN);
		Vector3 j = new Vector3(positionNormal);
		j.scale(jN);
		
		// for both bodies, I hope there is already function for it
		bodyA.applyBiasImpulse(j, worldA, 1);
		bodyB.applyBiasImpulse(j.inverse(), worldB, 1);
	}*/
	// EXPERIMENTAL!!! FOR POSITION CORRECTION
	//--------------------------------------------------------
	
	public void applyBiasImpulse() {
		// calculate impulse
		Vector3 vAB = new Vector3();
		Vector3.sub(bodyA.getBiasVelWS(worldA), bodyB.getBiasVelWS(worldB), vAB);
		
		float jMag = (-Vector3.dot(vAB, normal)+ bias ) * massN ;
		
		float pBias = accumBias;
		accumBias = Math.max(accumBias + jMag, 0);
		pBias = accumBias - pBias;
		
		Vector3 jBias = new Vector3(normal);
		jBias.scale(pBias);
		
		bodyA.applyBiasImpulse(jBias, worldA);
		bodyB.applyBiasImpulse(jBias.inverse(), worldB);
	}
	
	public void computeLocalAndDepth() {
		this.localA = bodyA.toLocal(worldA);
		this.localB = bodyB.toLocal(worldB);
		
		this.localNormalA = new Vector3(normal);
		bodyA.getRot().conjugated().transformVector(localNormalA, localNormalA);
		this.localNormalB = new Vector3(normal);
		bodyB.getRot().conjugated().transformVector(localNormalB, localNormalB);
		
		// depth = (worldA - worldB) * normal
		Vector3 AB = new Vector3();
		Vector3.sub(worldA, worldB, AB);
		this.depth = Vector3.dot(AB, normal);
		
		this.tangent1 = new Vector3();
		this.tangent2 = new Vector3();
		MathHelper.computeTangents(normal, tangent1, tangent2);
	}
	
	private void calcFrictionBiasRestitution(float dt, float baumgarte, float slop) {
		friction = Math.min(bodyA.getFriction(), bodyB.getFriction());
		bias = Math.max(-depth-slop, 0) * baumgarte / dt;
		kRestitution = Math.min(bodyA.getRestitution(), bodyB.getRestitution());
		
		// calculate restitution target, set to zero when it becomes too small
		Vector3 vel = new Vector3();
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldB), vel);
		
		// clamp our restitution (slop in restitution)
		// the slop is proportional to time step. so it's consistent over frames
		float vn = Math.max(0, -Vector3.dot(vel, normal) - (MIN_RESTITUTION/dt) );
		
		restitution = kRestitution * vn;
		
//		if (kRestitution > Vector3.EPSILON)
//			System.out.println("k, restitution: " + kRestitution + ", " +restitution);
	}
	
	private void calcNormalMass(Vector3 r1, Vector3 r2) {
		massN = bodyA.getInvMass() + bodyB.getInvMass();
		
		Matrix3 invIA = bodyA.getWorldInvInertia();
		Matrix3 invIB = bodyB.getWorldInvInertia();
		
		// r1 and r2 is in world space
		Vector3 rn = new Vector3();
		
		Vector3.cross(r1, normal, rn);
		Vector3.cross(rn, r1, rn);
		invIA.transformVector3(rn, rn);
		massN += Vector3.dot(normal, rn);
		
		Vector3.cross(r2, normal, rn);
		Vector3.cross(rn, r2, rn);
		invIB.transformVector3(rn, rn);
		massN += Vector3.dot(normal, rn);
		
		massN = 1.0f/massN;
	}
	
	private void calcTangentMass(Vector3 r1, Vector3 r2) {
		massT1 = bodyA.getInvMass() + bodyB.getInvMass();
		
		Matrix3 invIA = bodyA.getWorldInvInertia();
		Matrix3 invIB = bodyB.getWorldInvInertia();
		
		// r1 and r2 is in world space
		Vector3 rt = new Vector3();
		
		Vector3.cross(r1, tangent1, rt);
		Vector3.cross(rt, r1, rt);
		invIA.transformVector3(rt, rt);
		massT1 += Vector3.dot(tangent1, rt);
		
		Vector3.cross(r2, tangent1, rt);
		Vector3.cross(rt, r2, rt);
		invIB.transformVector3(rt, rt);
		massT1 += Vector3.dot(tangent1, rt);
		
		massT1 = 1.0f/massT1;
		// now for mass tangent 2
		massT2 = bodyA.getInvMass() + bodyB.getInvMass();
		
		Vector3.cross(r1, tangent2, rt);
		Vector3.cross(rt, r1, rt);
		invIA.transformVector3(rt, rt);
		massT2 += Vector3.dot(tangent2, rt);
		
		Vector3.cross(r2, tangent2, rt);
		Vector3.cross(rt, r2, rt);
		invIB.transformVector3(rt, rt);
		massT2 += Vector3.dot(tangent2, rt);
		
		massT2 = 1.0f/massT2;
	}
	
	/**
	 * Precalculate Normal, and 2 Tangent masses
	 * computation in local space of each body, because transforming inertia is tricky
	 * @param dt		- the simulation timestep
	 * @param baumgarte	- coefficient for position correction. [0..1]
	 * @param slop		- slop allowed for penetration
	 */
	public void preCalculate(float dt, float baumgarte, float slop) {
		// compute friction constant and bias term
		calcFrictionBiasRestitution(dt, baumgarte, slop);
		
		// compute displacement vector
		Vector3 r1 = new Vector3();
		Vector3 r2 = new Vector3();
		
		Vector3.sub(/*bodyA.toWorld(localA)*/worldA, bodyA.getPos(), r1);
		Vector3.sub(/*bodyB.toWorld(localB)*/worldB, bodyB.getPos(), r2);

		// compute Jacobian / effective mass
		calcNormalMass(r1, r2);		
		calcTangentMass(r1, r2);
		
		// apply old impulse (warmstarting)
		Vector3 j = new Vector3();
	
		// apply old impulse (warm start)
		j.setTo(normal);
		j.scale(accumPN);
		bodyA.applyImpulse(j, worldA);
		bodyB.applyImpulse(j.inverse(), worldB);
		
		j.setTo(tangent1);
		j.scale(accumPT1);
		bodyA.applyImpulse(j, worldA);
		bodyB.applyImpulse(j.inverse(), worldB);
		
		j.setTo(tangent2);
		j.scale(accumPT2);
		bodyA.applyImpulse(j, worldA);
		bodyB.applyImpulse(j.inverse(), worldB);
	}
	
//	public void preCalculate(float dt, float baumgarte, float slop) {
//		// set friction
//		friction = Math.min(bodyA.getFriction(), bodyB.getFriction());
//		// compute bias
//		bias = Math.max(-depth-slop, 0) * baumgarte / dt;
//		// compute Jacobian / effective mass
//		Vector3 r1 = new Vector3();
//		Vector3 r2 = new Vector3();
//		
////		Vector3.sub(worldA, bodyA.getPos(), r1);
////		Vector3.sub(worldB, bodyB.getPos(), r2);
//		bodyA.getRot().transformVector(localA, r1);
//		bodyB.getRot().transformVector(localB, r2);
//		
//		Vector3 rn1 = new Vector3();
//		Vector3 rn2 = new Vector3();
//		
//		// calculate mass normal
//		
//		Vector3.cross(r1, normal, rn1);
//		Vector3.cross(rn1, r1, rn1);
//		bodyA.getRot().conjugated().transformVector(rn1, rn1);
//		bodyA.getInvInertia().transformVector3(rn1, rn1);
//		bodyA.getRot().transformVector(rn1, rn1);
//		
//		Vector3.cross(r2, normal, rn2);
//		Vector3.cross(rn2, r2, rn2);
//		bodyB.getRot().conjugated().transformVector(rn2, rn2);
//		bodyB.getInvInertia().transformVector3(rn2, rn2);
//		bodyB.getRot().transformVector(rn2, rn2);
//		
//		Vector3.add(rn1, rn2, rn1);
//		
//		massN = bodyA.getInvMass() + bodyB.getInvMass();
//		massN += Vector3.dot(rn1, normal);
//		massN = 1.0f/massN;
//		
//		// calculate mass tangent1
//		Vector3.cross(r1, tangent1, rn1);
//		Vector3.cross(rn1, r1, rn1);
//		bodyA.getRot().conjugated().transformVector(rn1, rn1);
//		bodyA.getInvInertia().transformVector3(rn1, rn1);
//		bodyA.getRot().transformVector(rn1, rn1);
//		
//		Vector3.cross(r2, tangent1, rn2);
//		Vector3.cross(rn2, r2, rn2);
//		bodyB.getRot().conjugated().transformVector(rn2, rn2);
//		bodyB.getInvInertia().transformVector3(rn2, rn2);
//		bodyB.getRot().transformVector(rn2, rn2);
//		
//		Vector3.add(rn1, rn2, rn1);
//		
//		massT1 = bodyA.getInvMass() + bodyB.getInvMass();
//		massT1 += Vector3.dot(rn1, tangent1);
//		massT1 = 1.0f/massT1;
//		
//		// calculate mass tangent2
//		Vector3.cross(r1, tangent2, rn1);
//		Vector3.cross(rn1, r1, rn1);
//		bodyA.getRot().conjugated().transformVector(rn1, rn1);
//		bodyA.getInvInertia().transformVector3(rn1, rn1);
//		bodyA.getRot().transformVector(rn1, rn1);
//		
//		Vector3.cross(r2, tangent2, rn2);
//		Vector3.cross(rn2, r2, rn2);
//		bodyB.getRot().conjugated().transformVector(rn2, rn2);
//		bodyB.getInvInertia().transformVector3(rn2, rn2);
//		bodyB.getRot().transformVector(rn2, rn2);
//		
//		Vector3.add(rn1, rn2, rn1);
//		
//		massT2 = bodyA.getInvMass() + bodyB.getInvMass();
//		massT2 += Vector3.dot(rn1, tangent2);
//		massT2 = 1.0f/massT2;
//		
//		// let's apply old impulse
//		rn1.setTo(normal);
//		rn1.scale(accumPN);
//		bodyA.applyImpulse(rn1, worldA);
//		bodyB.applyImpulse(rn1.inverse(), worldB);
//		
//		rn1.setTo(tangent1);
//		rn1.scale(accumPT1);
//		bodyA.applyImpulse(rn1, worldA);
//		bodyB.applyImpulse(rn1.inverse(), worldB);
//		
//		rn1.setTo(tangent2);
//		rn1.scale(accumPT2);
//		bodyA.applyImpulse(rn1, worldA);
//		bodyB.applyImpulse(rn1.inverse(), worldB);
//	}
	/*public void preCalculate2(float dt, float baumgarte, float slop) {
		// set friction
		friction = (bodyA.getFriction() * bodyB.getFriction());
		// compute bias
		bias = Math.max(-depth-slop, 0) * baumgarte / dt;
		// compute Jacobian / effective mass
		Vector3 r1 = new Vector3();
		Vector3 r2 = new Vector3();
		
		Vector3.sub(worldA, bodyA.getPos(), r1);
		Vector3.sub(worldA, bodyB.getPos(), r2);
		
		Vector3 rn1 = new Vector3();
		Vector3 rn2 = new Vector3();
		
		// calculate mass normal
		Vector3.cross(r1, normal, rn1);
		Vector3.cross(rn1, r1, rn1);
		bodyA.getInvInertia().transformVector3(rn1, rn1);
		
		Vector3.cross(r2, normal, rn2);
		Vector3.cross(rn2, r2, rn2);
		bodyB.getInvInertia().transformVector3(rn2, rn2);
		
		Vector3.add(rn1, rn2, rn1);
		
		massN = bodyA.getInvMass() + bodyB.getInvMass();
		massN += Vector3.dot(rn1, normal);
		massN = 1.0f/massN;
		
		// calculate mass tangent1
		Vector3.cross(r1, tangent1, rn1);
		Vector3.cross(rn1, r1, rn1);
		bodyA.getInvInertia().transformVector3(rn1, rn1);
		
		Vector3.cross(r2, tangent1, rn2);
		Vector3.cross(rn2, r2, rn2);
		bodyB.getInvInertia().transformVector3(rn2, rn2);
		
		Vector3.add(rn1, rn2, rn1);
		
		massT1 = bodyA.getInvMass() + bodyB.getInvMass();
		massT1 += Vector3.dot(rn1, tangent1);
		massT1 = 1.0f/massT1;
		
		// calculate mass tangent2
		Vector3.cross(r1, tangent2, rn1);
		Vector3.cross(rn1, r1, rn1);
		bodyA.getInvInertia().transformVector3(rn1, rn1);
		
		Vector3.cross(r2, tangent2, rn2);
		Vector3.cross(rn2, r2, rn2);
		bodyB.getInvInertia().transformVector3(rn2, rn2);
		
		Vector3.add(rn1, rn2, rn1);
		
		massT2 = bodyA.getInvMass() + bodyB.getInvMass();
		massT2 += Vector3.dot(rn1, tangent2);
		massT2 = 1.0f/massT2;
		
		// let's apply old impulse
		rn1.setTo(normal);
		rn1.scale(accumPN);
		bodyA.applyImpulse(rn1, worldA);
		bodyB.applyImpulse(rn1.inverse(), worldA);
		
		rn1.setTo(tangent1);
		rn1.scale(accumPT1);
		bodyA.applyImpulse(rn1, worldA);
		bodyB.applyImpulse(rn1.inverse(), worldA);
		
		rn1.setTo(tangent2);
		rn1.scale(accumPT2);
		bodyA.applyImpulse(rn1, worldA);
		bodyB.applyImpulse(rn1.inverse(), worldA);
	}
	*/
	public void applyImpulse() {
		// first, normal impulse
		Vector3 vAB = new Vector3();
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldB), vAB);
		
		float pN = (-Vector3.dot(vAB, normal) + restitution) * massN;
		
		// smoothen out restitution
		restitution *= (1.0f-RESTITUTION_DISSIPATE);
		
		float dPN = accumPN;
		accumPN = Math.max(dPN + pN, 0);
		dPN = accumPN - dPN;
		
		Vector3 jN = new Vector3(normal);
		jN.scale(dPN);
		
		bodyA.applyImpulse(jN, worldA);
		bodyB.applyImpulse(jN.inverse(), worldB);
		
		// second, tangent impulse 1
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldB), vAB);
		
		float pT = -Vector3.dot(vAB, tangent1) * massT1;
		
		// maximum friction constant
		float maxPT = accumPN * friction;
		
		float dPT = accumPT1;
		accumPT1 = MathHelper.clamp(accumPT1 + pT, -maxPT, maxPT);
		dPT = accumPT1 - dPT;
		
		Vector3 jT = new Vector3(tangent1);
		jT.scale(dPT);
		
		bodyA.applyImpulse(jT, worldA);
		bodyB.applyImpulse(jT.inverse(), worldB);
		
		// third, tangent impulse 2
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldB), vAB);
		
		pT = -Vector3.dot(vAB, tangent2) * massT2;
		
		dPT = accumPT2;
		accumPT2 = MathHelper.clamp(accumPT2 + pT, -maxPT, maxPT);
		dPT = accumPT2 - dPT;
		
		jT.setTo(tangent2);
		jT.scale(dPT);
		
		bodyA.applyImpulse(jT, worldA);
		bodyB.applyImpulse(jT.inverse(), worldB);
	}
	/*public void applyImpulse2() {
		// first, normal impulsee
		Vector3 vAB = new Vector3();
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldA), vAB);
		
		float pN = -Vector3.dot(vAB, normal) * massN + bias;
		
		float dPN = accumPN;
		accumPN = Math.max(dPN + pN, 0);
		dPN = accumPN - dPN;
		
		Vector3 jN = new Vector3(normal);
		jN.scale(dPN);
		
		bodyA.applyImpulse(jN, worldA);
		bodyB.applyImpulse(jN.inverse(), worldA);
		
		// second, tangent impulse 1
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldA), vAB);
		
		float pT = -Vector3.dot(vAB, tangent1) * massT1;
		
		// maximum friction constant
		float maxPT = accumPN * friction;
		
		float dPT = accumPT1;
		accumPT1 = MathHelper.clamp(accumPT1 + pT, -maxPT, maxPT);
		dPT = accumPT1 - dPT;
		
		Vector3 jT = new Vector3(tangent1);
		jT.scale(dPT);
		
		bodyA.applyImpulse(jT, worldA);
		bodyB.applyImpulse(jT.inverse(), worldA);
		
		// third, tangent impulse 2
		Vector3.sub(bodyA.getVelWS(worldA), bodyB.getVelWS(worldA), vAB);
		
		pT = -Vector3.dot(vAB, tangent2) * massT2;
		
		dPT = accumPT2;
		accumPT2 = MathHelper.clamp(accumPT2 + pT, -maxPT, maxPT);
		dPT = accumPT2 - dPT;
		
		jT.setTo(tangent2);
		jT.scale(dPT);
		
		bodyA.applyImpulse(jT, worldA);
		bodyB.applyImpulse(jT.inverse(), worldA);
	}*/
	
	public void refresh() {
		// this one computes world data + depth only
		this.worldA = bodyA.toWorld(localA);
		this.worldB = bodyB.toWorld(localB);
		
		Vector3 AB = new Vector3();
		Vector3.sub(worldA, worldB, AB);
		this.depth = Vector3.dot(AB, normal);
	}
	
	public void debugDraw(GL2 gl, boolean drawNormal, boolean drawTangents) {
		// debug draw contacts
		
		// draw contact point first		
		gl.glBegin(GL2.GL_POINTS);
			gl.glColor3f(1, 0, 0);
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			
			gl.glColor3f(0, 0, 1);
			gl.glVertex3f(worldB.x, worldB.y, worldB.z);
		gl.glEnd();
		
		// draw with normal now
		gl.glBegin(GL2.GL_LINES);
		if (drawNormal) {
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			gl.glColor3f(0, 1, 0);
			gl.glVertex3f(worldA.x+normal.x, worldA.y+normal.y, worldA.z+normal.z);
		}
			
		if (drawTangents) {
			// draw tangents 1 (assume x)
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			gl.glColor3f(1, 0, 0);
			gl.glVertex3f(worldA.x+tangent1.x, worldA.y+tangent1.y, worldA.z+tangent1.z);
			
			// draw tangents 2 (assume z)
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			gl.glColor3f(0, 0, 1);
			gl.glVertex3f(worldA.x+tangent2.x, worldA.y+tangent2.y, worldA.z+tangent2.z);
		}
			
			// draw the contact points
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(worldA.x, worldA.y, worldA.z);
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(worldB.x, worldB.y, worldB.z);
		gl.glEnd();
	}
}
