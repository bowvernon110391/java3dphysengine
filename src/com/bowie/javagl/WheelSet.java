package com.bowie.javagl;

import java.util.ArrayList;

import com.jogamp.opengl.GL2;
import com.sun.xml.internal.bind.v2.runtime.Name;

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
		
		// for various reusage
		Vector3 r1 = new Vector3();
		Vector3 r2 = new Vector3();
		Vector3 rn = new Vector3();
		Matrix3 invIA = null;
		Matrix3 invIB = null;
		float invMass = 0.0f;
		
		if (world != null) {
			// make sure world is accessible
			world.grabPotentialColliders(bbox, bodies);
			// make sure it's not empty
//			System.out.printf("Wheelset: potential wheel colliders: %d %n", bodies.size());
			// now we gotta update each wheel's data
			for (RayWheel w : wheels) {
				// let's update our position here
				// apply torque here
				float angAccel = w.wheelTorque * w.wheelInvInertia;
//				w.wheelAngVel += angAccel * dt;
				// zero out torque
//				w.wheelTorque = 0;
				// update position (for rendering)
				w.wheelAngPos += (w.wheelAngVel - angAccel * dt) * dt;
				// dissipate wheel rotation
				
				// wrap the value around
				w.wheelAngPos = (float) Math.toRadians( Math.toDegrees(w.wheelAngPos) );
//				w.wheelAngPos += -angAccel * dt;
				// first, move last T
				w.lastT = w.rayT;
				// reset current T
				w.rayT = 1.f;
				// store last ground object (to reset accumulated impulse)
				RigidBody lastGroundObj = w.groundObj;
				w.groundObj = null;
				float t = -1.f;
				
				// do collision detection (raycast) against potential colliders
				for (RigidBody b : bodies) {
					// skip chassis
					if (b != chassis) {						
						// since a body may consist of multiple shapes, we must accommodate it here too
						// but for now, a body only consist of one shape, so it's fine
						s.sA = b.getShape();
						s.posA = b.getPos();
						s.rotA = b.getRot();
						
						int rayTest = s.doRayCast(w.absRayStart, w.absRayEnd, 0.f);
						// only account valid hit (must handle RAY_INSIDE) in separate stuffs
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
				
				// calculate absolute wheel position (always available)
				w.absWheelPos.x = w.absRayEnd.x * w.rayT + w.absRayStart.x * (1.f-w.rayT);
				w.absWheelPos.y = w.absRayEnd.y * w.rayT + w.absRayStart.y * (1.f-w.rayT);
				w.absWheelPos.z = w.absRayEnd.z * w.rayT + w.absRayStart.z * (1.f-w.rayT);
				
				w.absWheelPos.x -= w.absRayDir.x * w.wheelRadius;
				w.absWheelPos.y -= w.absRayDir.y * w.wheelRadius;
				w.absWheelPos.z -= w.absRayDir.z * w.wheelRadius;
				
				// set default data
				// calculate position error (this always available)
				w.posError = (1.0f - w.rayT) * ( w.suspensionLength + w.wheelRadius );
				
				// compute rest of data
				if (w.groundObj != null) {
					// calculate rest of needed data
					Vector3 vTmp = Vector3.tmp0;
					// forward vector (friction dir 1 aka longitudinal)
					Vector3.cross(w.rayHitNormal, w.absRight, w.rayHitForward);
					w.rayHitForward.normalize();
					// absolute right vector (friction dir 2 aka lateral)
					Vector3.cross(w.rayHitForward, w.rayHitNormal, w.rayHitSide);
					w.rayHitSide.normalize();
					
					// grab velocity @ contact point
//					Vector3.sub(w.getVelWS(chassis, w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos), w.rayHitVel);
					Vector3.sub(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos), w.rayHitVel);
					
					// calculate position error (scaled)
					float normScale = -Vector3.dot(w.absRayDir, w.rayHitNormal);	// so direct hit = stronger forces
					w.posError *= normScale;
					w.posError = Math.max(w.posError - 0.04f, 0.0f);
					
					// calculate speed of compression (scaled)
					// v = x / t
					w.springSpd = (w.lastT - w.rayT) * (w.suspensionLength + w.wheelRadius) / dt * normScale;
//					w.springSpd = -Vector3.dot(w.rayHitNormal, w.rayHitVel);
					
					// common used data for mass calculation
					Vector3.sub(w.rayHitPos, chassis.getPos(), r1);
					Vector3.sub(w.rayHitPos, w.groundObj.getPos(), r2);
					invIA = chassis.getWorldInvInertia();
					invIB = w.groundObj.getWorldInvInertia();
					
					// calculating effective mass (Normal)
					invMass = chassis.getInvMass() + w.groundObj.getInvMass();
					
					Vector3.cross(r1, w.rayHitNormal, rn);
					Vector3.cross(rn, r1, rn);
					invIA.transformVector3(rn, rn);
					invMass += Vector3.dot(w.rayHitNormal, rn);
					
					Vector3.cross(r2, w.rayHitNormal, rn);
					Vector3.cross(rn, r2, rn);
					invIB.transformVector3(rn, rn);
					invMass += Vector3.dot(w.rayHitNormal, rn);
					
					w.massN = invMass > 0.0f ? 1.0f / invMass : 0.0f;
					
					// calculate effective mass (Side)
					invMass = chassis.getInvMass() + w.groundObj.getInvMass();
					
					Vector3.cross(r1, w.rayHitSide, rn);
					Vector3.cross(rn, r1, rn);
					invIA.transformVector3(rn, rn);
					invMass += Vector3.dot(rn, w.rayHitSide);
					
					Vector3.cross(r2, w.rayHitSide, rn);
					Vector3.cross(rn, r2, rn);
					invIB.transformVector3(rn, rn);
					invMass += Vector3.dot(rn, w.rayHitSide);
					
					w.massSide = invMass > 0.0f ? 1.0f / invMass : 0.0f;
					
					// calculate effective mass (Forward)
					invMass = chassis.getInvMass() + w.groundObj.getInvMass();
					
					Vector3.cross(r1, w.rayHitForward, rn);
					Vector3.cross(rn, r1, rn);
					invIA.transformVector3(rn, rn);
					invMass += Vector3.dot(rn, w.rayHitForward);
					
					Vector3.cross(r2, w.rayHitForward, rn);
					Vector3.cross(rn, r2, rn);
					invIB.transformVector3(rn, rn);
					invMass += Vector3.dot(rn, w.rayHitForward);
					
					w.massForward = invMass > 0.0f ? 1.0f / invMass : 0.0f;
					
					// calculated k and d
					w.constK = w.massN * w.kStrength / (dt * dt);
					w.constD = w.massN * w.kDampRatio / (dt);
					
					// compute slip angle  (no need to account wheel angvel)
//					Vector3 slipVel = new Vector3(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
					Vector3 slipVel = w.rayHitVel;
					
					float vLat = Vector3.dot(slipVel, w.rayHitSide);
					float vLong = Vector3.dot(slipVel, w.rayHitForward);
					
					float slipAngle = (float) Math.toDegrees( Math.atan2(-vLat, -vLong) );
					
					slipAngle = slipAngle < -90.f ? slipAngle + 180.f : slipAngle > 90.f ? slipAngle - 180.f : slipAngle;
					
					w.beta = slipAngle;
//					if (w.name == "FR" || w.name == "BR")
//						System.out.printf("%s sA %.4f%n", w.name, slipAngle);
				}
				
			}
			
			// here, we gotta apply necessary forces
			// apply suspension force + longitudinal force
			float wheelLoad = 0.0f;
			for (RayWheel w : wheels) {
				if (w.groundObj != null) {
					// compute suspension forces
					float normScale = -Vector3.dot(w.absRayDir, w.rayHitNormal);
					float fSuspensionMag = (w.constK * w.posError + w.constD * w.springSpd);
					
					Vector3 fSuspension = new Vector3(w.rayHitNormal);
					fSuspension.scale(fSuspensionMag);
					
					// apply it if ground and chassis aren't sleeping
					chassis.applyForce(fSuspension, w.rayHitPos);
					w.groundObj.applyForce(fSuspension.inverse(), w.rayHitPos);
					
					
					// log suspension force
//					System.out.printf("%s : %.4f%n", w.name, fSuspensionMag);
					
					w.accumN = Math.abs(fSuspensionMag);
					wheelLoad += w.accumN;
					
					// now compute longitudinal force
					float longAccel = -w.wheelTorque * w.wheelInvInertia;
					w.wheelTorque = 0;
					float fLongMag = longAccel * w.massForward / dt;
					
					// gotta clamp
					fLongMag = MathHelper.clamp(fLongMag, -w.accumN * 1.5f, w.accumN * 1.5f);
					
					Vector3 fLong = new Vector3(w.rayHitForward);
					fLong.scale(fLongMag);
					
					// apply it
					chassis.applyForce(fLong, w.rayHitPos);
					w.groundObj.applyForce(fLong.inverse(), w.rayHitPos);
					
					// gotta apply reversal torque (modify wheel angular velocity)
					float angVel = -Vector3.dot(w.rayHitVel, w.rayHitForward) / w.wheelRadius;
					w.wheelAngVel = angVel;	// gotta apply some traction control here
					
					// gotta recompute beta it seems
//					w.rayHitVel.setTo(chassis.getVelWS(w.absWheelPos));
					
					// now we compute lateral force limit					
					float latMag =Math.abs( Vector3.dot(w.rayHitVel, w.rayHitSide) );
					if (w.name == "BR" || w.name == "BL")
					System.out.printf("%s groundvel: %.2f %.2f %.2f | %.2f, slip: %.2f%n",
							w.name,
							w.rayHitVel.x, w.rayHitVel.y, w.rayHitVel.z,
							latMag, w.beta);
					
					float tireFriction = Math.abs( MathHelper.pacejkaMFLat(w.beta, .1f, 2.0f, w.frictionS, -0.7f) );
					// try to zero out when speed is low
//					if (w.rayHitVel.lengthSquared() <  Vector3.EPSILON) {
//						tireFriction = 0;
//						System.out.println("low speed: use static friction model");
//					}
					// if we reach speed limit, we gotta switch though
					w.accumSide = Math.min(w.accumN * tireFriction, 2800) * dt;
//					w.accumSide = Math.min(w.accumN, 2000) * tireFriction * dt;
					
					// now apply our counter torque
//					w.applyTorque(fLongMag);
					// update our angular velocity
//					Vector3 vel = new Vector3(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
//					w.wheelAngVel = Vector3.dot(vel, w.rayHitForward) / w.wheelRadius; 
				}
			}
			// log something
			System.out.printf("wheelLoad : %.4f%n", wheelLoad);
			// normalize it
			for (RayWheel w : wheels) {
				if (wheelLoad > .0f)
					w.accumN /= wheelLoad;
				else
					w.accumN = 0;
				
//				System.out.printf("%s : %.2f%n", w.name, w.accumN);
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
			w.debugDraw(gl, chassis, dt);
		}
		
		// draw bounding box
//		bbox.debugDraw(gl); 
	}

	@Override
	public void preCalculate(float dt, float baumgarte, float slop) {
		
	}

	@Override
	public void solve() {
		// apply side friction
		for (RayWheel w : wheels) {
			if (w.groundObj == null)
				continue;
			// compute velocity
			Vector3 vel = new Vector3(w.getVelWS(chassis, w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
			
			float pLatMag = -Vector3.dot(vel, w.rayHitSide) * w.massSide;
			
			// clamp
			pLatMag = MathHelper.clamp(pLatMag, -w.accumSide, w.accumSide);
			
			// apply impulse
			Vector3 pLat = new Vector3(w.rayHitSide);
			pLat.scale(pLatMag);
			
			chassis.applyImpulse(pLat, w.rayHitPos);
			w.groundObj.applyImpulse(pLat.inverse(), w.rayHitPos);
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
