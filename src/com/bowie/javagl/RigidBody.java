package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class RigidBody {
	private static int guid = 0;
	public static synchronized int generateID() {
		return guid++;
	}
	public static  synchronized void resetGUID() {
		guid = 0;
	}
	// rigid body data
	private Matrix3 invInertia;	// inverse inertia (in local frame)
	private float invMass;		// inverse mass
	
	private Matrix3 worldInvInertia;	// inverse inertia (in world frame)
	
	private Shape shape;		// shape
	private float restitution;	// usually 0
	private float friction;		// friction coefficient
	
	private int id = generateID();
	
	private AABB bbox = new AABB();			// bounding box
	
	private Vector3 accel = new Vector3();		// linear acceleration -- ZEROED EVERY STEP
	private Vector3 angAccel = new Vector3();	// angular acceleration -- ZEROED EVERY STEP
	
	private Vector3 vel = new Vector3();		// linear velocity
	private Vector3 angVel = new Vector3();		// angular velocity
	
	private Vector3 linBias = new Vector3();	// linear bias velocity
	private Vector3 angBias = new Vector3();	// angular bias velocity
	
	private Vector3 pos = new Vector3();		// position
	private Quaternion rot = new Quaternion();		// rotation
	
	private boolean fixed = false;
	
	public RigidBody(float mass, Shape s) {
		// Inertia and Mass calculation
		if (mass <= 0) {
			this.invMass = 0;
		} else {
			this.invMass = 1.0f/mass;
		}
		
		// give default friction
		setFriction(0.2f);
		// calculate inverse inertia
		this.invInertia = s.getInvInertiaTensor(invMass);
		// set shape
		this.shape = s;
		// update?
		this.worldInvInertia = new Matrix3();
		refreshWorldInvInertia();
	}
	
	public void debugDraw(GL2 gl) {
		shape.render(gl, pos, rot);
	}
	
	public void debugDraw(GL2 gl, float dt) {
		
		
		if (dt < Vector3.EPSILON) {
			shape.render(gl, pos, rot);
		} else {
			Vector3 p = Vector3.tmp0;
			Quaternion r = Quaternion.tmp0;
			Quaternion spin = Quaternion.tmp1;
			
			spin.x = angVel.x;
			spin.y = angVel.y;
			spin.z = angVel.z;
			spin.w = 0;
			
			spin.scale(.5f);
			Quaternion.mul(spin, rot, spin);
			
			r.x = rot.x + spin.x * dt;
			r.y = rot.y + spin.y * dt;
			r.z = rot.z + spin.z * dt;
			r.w = rot.w + spin.w * dt;
			
			r.normalize();
			
//			// angular is quite hard
//			// 1. calculate spin
//			Quaternion scaledAngvel = new Quaternion(angVel.x+angBias.x, angVel.y+angBias.y, angVel.z+angBias.z, 0);
//			scaledAngvel.scale(0.5f);
//			Quaternion.mul(scaledAngvel, rot, scaledAngvel);
//			// 2. scale to timestep
//			scaledAngvel.scale(dt);
//			// 3. add to rotation
//			Quaternion.add(rot, scaledAngvel, rot);
//			rot.normalize();
			
			// compute interpolated properties
			p.x	= pos.x + vel.x * dt;
			p.y	= pos.y + vel.y * dt;
			p.z	= pos.z + vel.z * dt;
			
			shape.render(gl, p, r);
		}
		
	}
	
	public Matrix3 getInvInertia() {
		return invInertia;
	}
	
	public void applyGravity(Vector3 g) {
		// gravity is equal for all obj,
		// just add
		Vector3.add(accel, g, accel);
	}
	
	public void applyLinearDamping(float d) {
		vel.scale(1.0f-d);
	}
	
	public void applyAngularDamping(float d) {
		angVel.scale(1.0f-d);
	}

	public void setInvInertia(Matrix3 invInertia) {
		this.invInertia = new Matrix3(invInertia);
	}

	public float getInvMass() {
		return invMass;
	}

	public void setInvMass(float invMass) {
		this.invMass = invMass;
	}

	public Shape getShape() {
		return shape;
	}

	public void setShape(Shape shape) {
		this.shape = shape;
	}

	public float getRestitution() {
		return restitution;
	}

	public void setRestitution(float restitution) {
		this.restitution = restitution;
	}

	public float getFriction() {
		return friction;
	}

	public void setFriction(float fricion) {
		this.friction = fricion;
	}

	public AABB getBbox() {
		return bbox;
	}

	public void setBbox(AABB bbox) {
		this.bbox = bbox;
	}

	public Vector3 getAccel() {
		return accel;
	}

	public void setAccel(Vector3 accel) {
		this.accel.setTo(accel);
	}

	public Vector3 getAngAccel() {
		return angAccel;
	}

	public void setAngAccel(Vector3 angAccel) {
		this.angAccel.setTo(angAccel);
	}

	public Vector3 getVel() {
		return vel;
	}

	public void setVel(Vector3 vel) {
		this.vel.setTo(vel);
	}

	public Vector3 getAngVel() {
		return angVel;
	}

	public void setAngVel(Vector3 angVel) {
		this.angVel.setTo(angVel);
	}

	public Vector3 getPos() {
		return pos;
	}

	public void setPos(Vector3 pos) {
		this.pos.setTo(pos);
	}

	public Quaternion getRot() {
		return rot;
	}

	public void setRot(Quaternion rot) {
		this.rot.setTo(rot);
		// update?
		refreshWorldInvInertia();
	}
	
	public void updateBBox(float dt) {
		shape.getAABB(pos, rot, bbox);
		// grow it by fraction of velocity
		Vector3 gVel = new Vector3(vel);
		gVel.scale(dt);
		bbox.grow(gVel);
		
		// add small margin to maximize cache
		bbox.grow(Vector3.EPSILON);
	}
	
	public Vector3 toLocal(Vector3 p) {
		Vector3 local = new Vector3();
		Vector3.sub(p, pos, local);
		rot.conjugated().transformVector(local, local);
		return local;
	}
	
	public Vector3 toWorld(Vector3 p) {
		Vector3 world = new Vector3();
		rot.transformVector(p, world);
		Vector3.add(pos, world, world);
		return world;
	}
	
	public Vector3 getVelWS(Vector3 wp) {
		// get particular velocity of a point
		// in world space
		Vector3 worldVel = new Vector3();
		Vector3 r = new Vector3();
		Vector3.sub(wp, pos, r);
		Vector3.cross(angVel, r, worldVel);
		Vector3.add(worldVel, vel, worldVel);
		return worldVel;
	}
	
	public Vector3 getBiasVelWS(Vector3 wp) {
		// get bias velocity of a point
		// in world space
		Vector3 worldVel = new Vector3();
		Vector3 r = new Vector3();
		Vector3.sub(wp, pos, r);
		Vector3.cross(angBias, r, worldVel);
		Vector3.add(worldVel, linBias, worldVel);
		return worldVel;
	}
	
	public void updateVelocity(float dt) {
		// linear is easy
		Vector3 scaledAccel = new Vector3(accel);
		scaledAccel.scale(dt);
		Vector3.add(scaledAccel, vel, vel);
		
		// now, for the rotational
		scaledAccel.setTo(angAccel);
		scaledAccel.scale(dt);
		Vector3.add(scaledAccel, angVel, angVel);
	}
	
	public void updatePosition(float dt) {
		// linear is easy
		Vector3 scaledVel = new Vector3(vel);
		Vector3.add(scaledVel, linBias, scaledVel);
		scaledVel.scale(dt);
		Vector3.add(scaledVel, pos, pos);
		
		// angular is quite hard
		// 1. calculate spin
		Quaternion scaledAngvel = new Quaternion(angVel.x+angBias.x, angVel.y+angBias.y, angVel.z+angBias.z, 0);
		scaledAngvel.scale(0.5f);
		Quaternion.mul(scaledAngvel, rot, scaledAngvel);
		// 2. scale to timestep
		scaledAngvel.scale(dt);
		// 3. add to rotation
		Quaternion.add(rot, scaledAngvel, rot);
		rot.normalize();
		
		// update inverse world inertia
		refreshWorldInvInertia();
		
		// clear bias
		linBias.setTo(Vector3.ZERO);
		angBias.setTo(Vector3.ZERO);
	}
	
	public void refreshWorldInvInertia() {
		// gotta refresh
		Matrix3 R = new Matrix3();
		R.setToIdentity();
		
		// grab rotation matrix
		rot.toMatrix3(R);
		// rotate inverse inertia by world matrix
		Matrix3.mul(R, invInertia, worldInvInertia);
		// transpose rotation (invert it)
		R.transpose();
		// redo
		Matrix3.mul(worldInvInertia, R, worldInvInertia);
	}
	
	public Matrix3 getWorldInvInertia() {
		return worldInvInertia;
	}
	
	public void clearForces() {
		accel.setTo(Vector3.ZERO);
		angAccel.setTo(Vector3.ZERO);
	}
	
	public void applyForce(Vector3 f) {
		// apply force thru origin
		Vector3 scaledForce = new Vector3(f);
		scaledForce.scale(invMass);
		Vector3.add(accel, scaledForce, accel);
	}
	
	public void applyTorque(Vector3 t) {
		// transform into local
		/*Vector3 localTorque = new Vector3();
		rot.conjugated().transformVector(t, localTorque);
		// multiply by inverse inertia
		invInertia.transformVector3(localTorque, localTorque);
		// transform back to worldspace
		rot.transformVector(localTorque, localTorque);*/
		
		// now just add
//		Vector3.add(angAccel, localTorque, angAccel);
		
		Vector3 scaledTorque = new Vector3();
		worldInvInertia.transformVector3(t, scaledTorque);
		Vector3.add(angAccel, scaledTorque, angAccel);
	}
	
	public void applyForce(Vector3 f, Vector3 wp) {
		// apply force at world position wp
		Vector3 r = new Vector3();
		Vector3.sub(wp, pos, r);
		
		Vector3 torque = new Vector3();
		// torque is r x f
		Vector3.cross(r, f, torque);
		
		// apply both force and torque
		applyForce(f);
		applyTorque(torque);
	}
	
	public void applyImpulse(Vector3 p) {
		// apply impulse thru center of mass
		Vector3 scaledImpulse = new Vector3(p);
		scaledImpulse.scale(invMass);
		Vector3.add(vel, scaledImpulse, vel);
	}
	
	public void applyAngularImpulse(Vector3 w) {
//		Vector3 localW = new Vector3();
//		// transform to local
//		rot.conjugated().transformVector(w, localW);
//		// multiply by inverse inertia
//		invInertia.transformVector3(localW, localW);
//		// transform back to world
//		rot.transformVector(localW, localW);
//		// add
//		Vector3.add(angVel, localW, angVel);
		
		Vector3 scaledAngularImpulse = new Vector3();
		worldInvInertia.transformVector3(w, scaledAngularImpulse);
		Vector3.add(angVel, scaledAngularImpulse, angVel);
	}
	
	public void applyImpulse(Vector3 p, Vector3 wp) {
		// apply impulse at world position
		Vector3 r = new Vector3();
		Vector3.sub(wp, pos, r);
		
		// calculate angular impulse (r x p)
		Vector3 w = new Vector3();
		Vector3.cross(r, p, w);
		
		// apply both
		applyImpulse(p);
		applyAngularImpulse(w);
	}
	
	public void applyBiasImpulse(Vector3 p, Vector3 wp) {
		/*//========================================
		// ANGULAR
		// calculate angular impulse first
		Vector3 r = new Vector3();
		Vector3.sub(wp, pos, r);
		
		Vector3.cross(r, p, r);
		
		// r is angular impulse
		// manual integration
		rot.conjugated().transformVector(r, r); 	// now r is in local space.
		invInertia.transformVector3(r, r); 			// now r is already scaled down by inertia
		rot.transformVector(r, r);
		// now we calculate spin
		// 1. calculate spin
		Quaternion scaledAngvel = new Quaternion(r.x, r.y, r.z, 0);
		scaledAngvel.scale(0.5f);
		Quaternion.mul(scaledAngvel, rot, scaledAngvel);
		// 2. scale to timestep
		scaledAngvel.scale(dt);
		// 3. add to rotation
		Quaternion.add(rot, scaledAngvel, rot);
		rot.normalize();
		
		//=============================================================
		// LINEAR 
		// it's easy
		r.setTo(p);
		r.scale(invMass * dt);
		Vector3.add(pos, r, pos);*/
		
		// LINEAR
		Vector3 scaledLinBias = new Vector3(p);
		scaledLinBias.scale(invMass);
		Vector3.add(linBias, scaledLinBias, linBias);
		
		// ANGULAR is a bit involved
		Vector3 r = new Vector3();
		Vector3.sub(wp, pos, r);
		
		Vector3.cross(r, p, r);
		worldInvInertia.transformVector3(r, r);
		
		// now add
		Vector3.add(angBias, r, angBias);
	}

	public boolean isFixed() {
		return fixed;
	}

	public void setFixed(boolean fixed) {
		this.fixed = fixed;
	}

	public int getId() {
		return id;
	}
}
