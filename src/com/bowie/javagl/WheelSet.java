package com.bowie.javagl;

import java.util.ArrayList;

import com.jogamp.opengl.GL2;

/**
 * This class encompass a set of wheel for a vehicle
 * This controls and contains all wheel
 * @author Bowie
 *
 */
public class WheelSet implements SimForce, Joint {
	public class RayHitData {
		public RayHitData(Vector3 hitPos, Vector3 hitNormal, float t) {
			hitP = new Vector3(hitPos);
			hitN = new Vector3(hitNormal);
			rayT = t;
		}
		
		public Vector3 hitP, hitN;	// hit position and normal
		public float rayT;			// the fraction of the ray when hitting
	}
	
	// now set data
	private RigidBody chassis;				// the car chassis. We do not shoot ray against it.
	private ArrayList<RayWheel> wheels;		// the wheels of course
	private AABB bbox;						// the bbox encompass all wheels
	private Physics world;					// the physics engine
	
	// this is shared for temporary calculation
	static private Simplex s = new Simplex(null, null, null, null, null, null);
	
	public WheelSet(RigidBody b) {
		bbox = new AABB();
		chassis = b;
		wheels  = new ArrayList<>();
		world = null;
	}
	
	public WheelSet setWorld(Physics wrld) {
		world = wrld;
		return this;
	}
	
	public WheelSet addWheel(RayWheel w) {
		wheels.add(w);
		return this;
	}
	
	private void updateAABB() {
		// first, update all wheels
		int i=0;
		for (RayWheel w : wheels) {
			w.calcAbsData(chassis.getPos(), chassis.getRot());
			
			if (i == 0) {
				// init with this
				bbox.encompass(w.absRayStart, w.absRayEnd, true);
			} else {
				// continue with this
				bbox.encompass(w.absRayStart, w.absRayEnd, false);
			}
			
			i++;
		}
	}

	@Override
	public void simulate(float dt) {
		// first, we must update bounding box
		updateAABB();
		
		// then, we gather all bodies
		ArrayList<RigidBody> bodies = new ArrayList<>();
		
		if (world != null) {
			// make sure world is accessible
			world.grabPotentialColliders(bbox, bodies);
			// make sure it's not empty
//			System.out.printf("Wheelset: potential wheel colliders: %d %n", bodies.size());
			// now we gotta update each wheel's data
			for (RayWheel w : wheels) {
				// let's update our position here
				w.wheelAngPos += w.wheelAngVel * dt;
				// first, move last T
				w.lastT = w.rayT;
				// reset current T
				w.rayT = 1.f;
				// store last ground object (to reset accumulated impulse)
				RigidBody lastGroundObj = w.groundObj;
				w.groundObj = null;
				float t = -1.f;
				
				for (RigidBody b : bodies) {
					if (b != chassis) {
						// skip chassis
						
						// since a body may consist of multiple shapes, we must accommodate it here too
						// but for now, a body only consist of one shape, so it's fine
						s.sA = b.getShape();
						s.posA = b.getPos();
						s.rotA = b.getRot();
						
						int rayTest = s.doRayCast(w.absRayStart, w.absRayEnd, 0.f);
						if (rayTest == Simplex.RAY_HIT) {
							// well, the ray hit, record closest shit
							if (s.rayT < t || t < 0.f) {
								t = s.rayT;
								w.rayHitNormal.setTo(s.rayhitNormal.normalized());
								w.rayHitPos.setTo(s.rayhitPos);
								w.rayT = t;
								w.groundObj = b;	// mark as ground object to affect
							}
						}
					}
				}
				
				// shall we reset accumulated impulse
				if (lastGroundObj != w.groundObj) {
					// reset
					w.accumN = 0;
					w.accumSide = 0;
					w.accumForward = 0;
				}
				
				// calculate absolute wheel position (always available)
				w.absWheelPos.x = w.absRayEnd.x * w.rayT + w.absRayStart.x * (1.f-w.rayT);
				w.absWheelPos.y = w.absRayEnd.y * w.rayT + w.absRayStart.y * (1.f-w.rayT);
				w.absWheelPos.z = w.absRayEnd.z * w.rayT + w.absRayStart.z * (1.f-w.rayT);
				
				w.absWheelPos.x -= w.absRayDir.x * w.wheelRadius;
				w.absWheelPos.y -= w.absRayDir.y * w.wheelRadius;
				w.absWheelPos.z -= w.absRayDir.z * w.wheelRadius;
				
				// calculate position error
				w.posError = (1.0f - w.rayT) * ( w.suspensionLength + w.wheelRadius );
				
				// compute rest of data
				if (w.groundObj != null) {
					// calculate rest of needed data
					Vector3 vTmp = Vector3.tmp0;
					// forward vector
					Vector3.cross(w.rayHitNormal, w.absRight, w.rayHitForward);
					w.rayHitForward.normalize();
					// absolute right vector
					Vector3.cross(w.rayHitForward, w.rayHitNormal, w.rayHitSide);
					w.rayHitSide.normalize();
					
					// scale position error
					w.posError *= -Vector3.dot(w.absRayDir, w.rayHitNormal);
				}
				
			}
		}
	}
	
	public RayWheel getWheel(int id) {
		if (id >= wheels.size())
			return null;
		return wheels.get(id);
	}
	
	public void debugDraw(GL2 gl, float dt) {
		for (RayWheel w : wheels) {
			w.debugDraw(gl, dt);
		}
		
		// draw bounding box
//		bbox.debugDraw(gl);
	}

	@Override
	public void preCalculate(float dt, float baumgarte, float slop) {
		for (RayWheel w : wheels) {
			// gotta precalculate stuffs
			w.massN = 0;
			
			if (w.groundObj != null) {
				// collide, compute contact impulse
				// the beta term
				w.beta = Math.max(w.posError - slop, 0) * baumgarte / dt;
				
				// term
				Vector3 r1 = new Vector3(w.rayHitPos, chassis.getPos());
				Vector3 r2 = new Vector3(w.rayHitPos, w.groundObj.getPos());
				Vector3 rn = new Vector3();
				
				Matrix3 invI1 = chassis.getWorldInvInertia();
				Matrix3 invI2 = w.groundObj.getWorldInvInertia();
				
				// the mass
				float invMass = chassis.getInvMass() + w.groundObj.getInvMass();
				
				Vector3.cross(r1, w.rayHitNormal, rn);
				Vector3.cross(rn, r1, rn);
				invI1.transformVector3(rn, rn);
				invMass += Vector3.dot(rn, w.rayHitNormal);
				
				Vector3.cross(r2, w.rayHitNormal, rn);
				Vector3.cross(rn, r2, rn);
				invI2.transformVector3(rn, rn);
				invMass += Vector3.dot(rn, w.rayHitNormal);
				
				w.massN = invMass > 0.0f ? 1.0f / invMass : 0.0f;
				
				// let's compute gamma and beta				
				// let's also apply accumulated impulse
				Vector3 j = new Vector3(w.rayHitNormal);
				j.scale(w.accumN);
				
				chassis.applyImpulse(j, w.rayHitPos);
				w.groundObj.applyImpulse(j.inverse(), w.rayHitPos);
			}
		}
	}

	@Override
	public void solve() {
		for (RayWheel w : wheels) {
			// gotta apply shit here
			if (w.groundObj != null) {
				Vector3 vel = new Vector3(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
				
				float pN = (-Vector3.dot(vel, w.rayHitNormal)  + w.beta) * w.massN;
				
				// let's try accumulated
				float dpN = w.accumN;
				w.accumN = Math.max(w.accumN + pN, 0.0f);
				pN = w.accumN - dpN;
				
				Vector3 j = new Vector3(w.rayHitNormal);
				j.scale(pN);
				
				chassis.applyImpulse(j, w.rayHitPos);
				w.groundObj.applyImpulse(j.inverse(), w.rayHitPos);
			}
		}
	}

	@Override
	public void debugDraw(GL2 gl) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void positionSolve() {
		// TODO Auto-generated method stub
		
	}

	
}
