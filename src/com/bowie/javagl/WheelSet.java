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
				// update wheel positon regardless of collision status 
				w.wheelAngVel += w.wheelTorque * w.wheelInvInertia * dt;	// this is uncorrected velocity
				w.wheelAngPos += w.wheelAngVel * dt;
				// first, move last T to current T
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
					w.posError = Math.max(w.posError - 0.02f, 0.0f);
					
					// calculate speed of compression (scaled)
					// v = x / t
//					w.springSpd = (w.lastT - w.rayT) * (w.suspensionLength + w.wheelRadius) / dt * normScale;
					w.springSpd = -Vector3.dot(w.rayHitNormal, w.rayHitVel);
//					w.springSpd = Math.max(0, w.springSpd);
					
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
				}
				
			}
			
			// here, we gotta apply necessary forces
			// apply suspension force + longitudinal force
			float wheelLoad = 0.0f;
			for (RayWheel w : wheels) {
				if (w.groundObj != null) {
					// compute suspension forces
					float fSuspensionMag = (w.constK * w.posError + w.constD * w.springSpd);
					fSuspensionMag = Math.max(0, fSuspensionMag);
					
					Vector3 fSuspension = new Vector3(w.rayHitNormal);
					fSuspension.scale(fSuspensionMag);
					
					// apply it if ground and chassis aren't sleeping
					chassis.applyForce(fSuspension, w.rayHitPos);
					w.groundObj.applyForce(fSuspension.inverse(), w.rayHitPos);
					
					// compute load for the wheel
					w.accumN = fSuspensionMag;
					wheelLoad += w.accumN;	// total load for log purpose
					
					float vLat = -Vector3.dot(w.rayHitVel, w.rayHitSide);
					float vLong = -Vector3.dot(w.rayHitVel, w.rayHitForward);
					
					float slipAngle = (float) Math.toDegrees( Math.atan2(vLat, vLong) );
					float gndVel = -Vector3.dot(w.rayHitForward, w.rayHitVel);
					float wVel = (w.wheelTorque * w.wheelInvInertia * w.wheelRadius) * dt;
					float slipRatio;
					
					if (w.wheelTorque > Vector3.EPSILON)
						slipRatio = 13;
					else if (w.wheelTorque < -Vector3.EPSILON)
						slipRatio = -13;
					else
						slipRatio = 0;
					
					// compute longitudinal force
					float vLen = (float) Math.sqrt(vLong*vLong + vLat*vLat);
					float latLoadScaler = Math.min(2.4f,  vLen / 20.f);	// cap at 2x
					float longLoadScaler = Math.min(2.f, vLen / 10.f);
					float fLongPMF = MathHelper.pacejkaMFLat(slipRatio, .1f, 1.3f, .95f + latLoadScaler, .2f) * MathHelper.clamp(w.accumN, 400.f, 12500.f);
					// compute lateral force
					float fLatPMF =MathHelper.pacejkaMFLat(slipAngle, .2f, 1.4f, .98f + longLoadScaler, .02f) * MathHelper.clamp(w.accumN, 400.f, 12500.f);
					// let's clamp lateral force?
					float fMaxLat = Math.abs(vLat) * w.massSide / dt;
					fLatPMF = MathHelper.clamp(fLatPMF, -fMaxLat, fMaxLat);
										
					float fCombined = (float) Math.sqrt(fLatPMF*fLatPMF + fLongPMF*fLongPMF);
					if (w.name == "BR") {
						System.out.printf("%.4f %.4f %.4f%n", vLat, vLong, vLen);
					}
					// clamp against friction circle?
					float maxF = fSuspensionMag * (1.f+longLoadScaler);
					
					if (fCombined > 0) {
						float scale = maxF / fCombined;
						scale = Math.min(1.f, scale);
						
						fLatPMF *= scale;
						fLongPMF *= scale;
					}
					
					// apply it
					// it's only valid for high speed cornering
					w.lowSpeedMode = true;	// assume true
					w.staticLoad = w.accumN * w.frictionS;	// when speed is low, use this
					// only apply if speed is > 15 km/h
					if (vLen > 4.2f) {
						w.lowSpeedMode = false;
						Vector3 fLateral = new Vector3(w.rayHitSide);
						fLateral.scale(fLatPMF);
						chassis.applyForce(fLateral, w.rayHitPos);
						w.groundObj.applyForce(fLateral.inverse(), w.rayHitPos);
					}
					
					Vector3 fLong = new Vector3(w.rayHitForward);
					fLong.scale(-fLongPMF);
					
					chassis.applyForce(fLong, w.rayHitPos);
					w.groundObj.applyForce(fLong.inverse(), w.rayHitPos);
					
					// finally, we correct velocity in contact
					// w = v / r
					float angVel = -Vector3.dot(w.rayHitVel, w.rayHitForward) / w.wheelRadius;
					w.wheelAngVel = angVel;
				}
				
				// this happens no matter what
				// reset brake and torque
				w.wheelTorque = 0;
				w.brakeStrength = 0;
			}
			// log something
//			System.out.printf("wheelLoad : %.4f%n", wheelLoad);
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
		for (RayWheel w : wheels) {
			w.staticLoad *= dt;	// P = F x dt
			w.accumSide = 0;	// reset accumulator
		}
	}

	@Override
	public void solve() {
		for (RayWheel w : wheels) {
			// don't do shit if not on ground
			// or is not in low speed mode
			if (w.groundObj == null || !w.lowSpeedMode)
				continue;
			
			Vector3 vel = new Vector3(chassis.getVelWS(w.rayHitPos), w.groundObj.getVelWS(w.rayHitPos));
			
			float pT = -Vector3.dot(vel, w.rayHitSide) * w.massSide;
			
			// clamp
			float dpT = w.accumSide;
			w.accumSide = MathHelper.clamp(w.accumSide + pT, -w.staticLoad, w.staticLoad);
			pT = w.accumSide - dpT;
			
			Vector3 jT = new Vector3(w.rayHitSide);
			jT.scale(pT);
			
			chassis.applyImpulse(jT, w.rayHitPos);
			w.groundObj.applyImpulse(jT.inverse(), w.rayHitPos);
		}
	}

	@Override
	public void debugDraw(GL2 gl) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void positionSolve() {
	}

	
}
