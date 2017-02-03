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
				w.wheelAngVel += angAccel * dt;
				// update position (for rendering)
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
				
				// set default data
				// calculate position error
				w.posError = (1.0f - w.rayT) * ( w.suspensionLength + w.wheelRadius );
				w.massN = 0.0f;
				
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
					// allow small slop
					w.posError = Math.max(w.posError - 0.03f, 0.0f);
					
					// calculate velocity along normal
					float vN = (w.lastT-w.rayT) * (w.suspensionLength+w.wheelRadius);
					Vector3 vNorm = new Vector3(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
					float vN2 = -Vector3.dot(vNorm, w.rayHitNormal);
					
//					System.out.printf("naive, real: %.4f, %.4f%n", vN, vN2);
					
					
					// refine normal mass
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
					
					// apply our force here
					float k = (w.kStrength) * w.massN / (dt * dt);
					float d = (w.kDampRatio) * w.massN / (dt);
					
					float fSuspensionMag = ( k * w.posError + d * (vN2) );
					Vector3 fSuspension = new Vector3(w.rayHitNormal);
					fSuspension.scale(fSuspensionMag);
					
					chassis.applyForce(fSuspension, w.rayHitPos);
					w.groundObj.applyForce(fSuspension.inverse(), w.rayHitPos);
					
					// store this as pN (for limiting friction)
					w.accumN = fSuspensionMag *  1.4f * dt;
					
					// also apply torque
					// torque = r x F
					// F = torque / r
					float fForwardMag = -w.wheelTorque / w.wheelRadius;
					Vector3 fForward = new Vector3(w.rayHitForward);
					fForward.scale(fForwardMag);
					w.wheelTorque = 0;//reset
					
					chassis.applyForce(fForward, w.rayHitPos);
					w.groundObj.applyForce(fForward.inverse(), w.rayHitPos);
					
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
					
					// test if we should use which max friction
					Vector3 vGround = new Vector3(w.getVelWS(chassis, w.absWheelPos), w.groundObj.getVelWS(w.rayHitPos));
//					Vector3 vGround = new Vector3(w.getVelWS(chassis, w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
					float pT = -Vector3.dot(vGround, w.rayHitSide) * w.massSide;
					
					w.gamma = Math.abs(fSuspensionMag) * w.frictionS * dt;
					if (w.name == "FR" || w.name == "FL") {
						System.out.printf("%s | susp fric sta dyn : %.4f %.4f %.4f %.4f%n", 
								w.name, fSuspensionMag, pT, fSuspensionMag * w.frictionS, fSuspensionMag * w.frictionD);
					}
					
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
		int i = 0;
		for (RayWheel w : wheels) {
			i++;
			if (w.groundObj != null) {
				// we only have to deal with friction (side friction)
				Vector3 vel = new Vector3(w.getVelWS(chassis, w.absWheelPos), w.groundObj.getVelWS(w.rayHitPos));
//				Vector3 vel = new Vector3(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
				
				float fSideMag = -(Vector3.dot(vel, w.rayHitSide)) * w.massSide;
				
				float maxFriction = w.gamma;
				
//				if (Math.abs(fSideMag) > maxFrictionS) {
//					maxFriction = maxFrictionD;
//					
////					System.out.printf("%s | using dynamic friction!%n", w.name);
//				}
				
//				System.out.printf("%s | maxS, maxD, RAW, fric: %.4f %.4f %.4f %.4f%n", w.name, maxFrictionS, maxFrictionD, w.accumN, fSideMag);
				fSideMag = MathHelper.clamp(fSideMag, -maxFriction, maxFriction);
				
				Vector3 j = new Vector3(w.rayHitSide);
				j.scale(fSideMag);
				
				chassis.applyImpulse(j, w.rayHitPos);
				w.groundObj.applyImpulse(j.inverse(), w.rayHitPos);
				
				// calculate angular velocity of the wheel
				// only do this when wheel is not locked (brake)
				Vector3.sub(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos), vel);
				float vForward = Vector3.dot(w.rayHitForward, vel);
				
				w.wheelAngVel = vForward / w.wheelRadius;
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
