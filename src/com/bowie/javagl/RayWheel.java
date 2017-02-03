package com.bowie.javagl;

import java.util.ArrayList;

import com.jogamp.opengl.GL2;

public class RayWheel {
	// basic data
	// local data
	private Vector3 localRayStart;		// suspension position (in local frame)
	private Vector3 localRight;			// local right vector (in local frame)
	private Vector3 localRayDir;		// local ray direction (in local frame) usually (0,-1,0)
	
	// absolute data (used by collision detection)
	public Vector3 absRayStart;		// suspension start, in absolute frame
	public Vector3 absRayEnd;			// suspension end + wheel radius, in absolute frame
	public Vector3 absRight;			// absolute right direction, plays in determining forward dir
	public Vector3 absRayDir;			// absolute ray direction
	public Vector3 absWheelPos;			// absolute wheel position
	
	// now calculated data from collision detection routine
//	public ArrayList<RayHitData> rayHits;	// this is to hold ray hit data
	public float rayT, lastT;			// this means invalid rayT initially
	public Vector3 rayHitPos;			// where the ray hits?
	public Vector3 rayHitSide;			// side friction direction
	public Vector3 rayHitNormal;		// the hit normal vector
	public Vector3 rayHitForward;		// the forward direction
	public RigidBody groundObj;			// what did we hit?
	
	// another dynamic data
	public float wheelAngPos;		// the wheel angular position (mostly for rendering purpose)
	public float wheelAngVel;		// the wheel angular velocity (mostly for rendering purpose)
	public float wheelInvMass;		// the inverse mass
	public float wheelInvInertia;	// the wheel inverted inertia tensor (for angular velocity mod)
	public float wheelRadius, wheelThickness;	// wheel radius and thickness
	public float wheelSteer;		// current steering. Defaults to zero
	
	// for impulse calculation
	public float massN, massSide, massForward;
	public float accumN, accumSide, accumForward;
	public float posError;	// displacement error?
	
	public float frictionS, frictionD;	// static and dynamic friction constant
	
	// static data
	public float suspensionLength;			// together form the ray length
	public float kFrequency, kDampRatio, kStrength;	// the spring frequency and damping constant
	
	// calculated data
	public float beta, gamma;		// for soft constraint
	
	// constructor
	public RayWheel(float mass, float radius, float thickness) {
		// simple constructor
		// inertia of a cylinder
		float I = .5f * mass * radius * radius;
		wheelInvInertia = 1.f/I;
		wheelInvMass	= 1.f/mass;
		
		wheelRadius = radius;
		wheelThickness = thickness;
		wheelSteer = 0.f;
		
		// allocate common data
		localRayStart = new Vector3();
		localRayDir = new Vector3();
		localRight = new Vector3(1, 0, 0);
		
		absRayEnd = new Vector3();
		absRayStart = new Vector3();
		absRight = new Vector3();
		absRayDir = new Vector3();
		absWheelPos = new Vector3();
		
//		rayHits = new ArrayList<>();
		rayT = lastT = 1.f;	// fully extended
		rayHitPos = new Vector3();
		rayHitNormal = new Vector3();
		rayHitForward = new Vector3();
		rayHitSide = new Vector3();
		
		groundObj = null;
		kStrength = 1.f;
	}
	
	// builder pattern
	public RayWheel setConstant(float freqHz, float dampRatio, float strength) {
		this.kDampRatio = dampRatio;
		this.kFrequency = freqHz;
		this.kStrength = strength;
		return this;
	}
	
	public RayWheel setFriction(float frictionS, float frictionD) {
		this.frictionS = frictionS;
		this.frictionD = frictionD;
		return this;
	}
	
	public RayWheel setRayDir(Vector3 dir) {
		this.localRayDir.setTo(dir.normalized());
		return this;
	}
	
	public RayWheel setWheelRight(Vector3 right) {
		this.localRight.setTo(right);
		return this;
	}
	
	public RayWheel setRayStart(Vector3 rS) {
		this.localRayStart.setTo(rS);
		return this;
	}
	
	public RayWheel setSuspensionLength(float l) {
		this.suspensionLength = l;
		return this;
	}
	
	public RayWheel setSteerAngle(float rad) {
		this.wheelSteer = rad;
		return this;
	}
	
	public Vector3 getVelWS(RigidBody c, Vector3 wp) {
		Vector3 vel = c.getVelWS(wp);
		
		if (c.isSleeping())
			vel.setTo(Vector3.ZERO);
		// add our angular velocity data (if possible)
		if (groundObj != null) {
			Vector3 velForward = new Vector3(rayHitForward);
			velForward.scale(wheelAngVel * wheelRadius);
			
			Vector3.add(vel, velForward, vel);
		}
		return vel;
	}
	
	public void drawWheel(GL2 gl, float radius, int numPts) {
		// gotta draw wheel in z direction
		gl.glBegin(GL2.GL_LINES);
		gl.glVertex3f(0, 0, 0);
		gl.glVertex3f(0, 0, radius);
		gl.glEnd();
		
		float add = (float) (Math.PI * 2.f / (float)numPts);
		float rad = 0;
		gl.glBegin(GL2.GL_LINE_LOOP);
		for (int i=0; i<numPts; i++) {
			gl.glVertex3f(0, (float)Math.sin(rad) * radius, (float)Math.cos(rad) * radius);
			rad += add;
		}
		gl.glEnd();
	}
	
	public void calcAbsData(Vector3 chassisPos, Quaternion chassisRot) {
		// will calculate absolute data
		// 1. absolute raystart position
		chassisRot.transformVector(localRayStart, absRayStart);
		Vector3.add(absRayStart, chassisPos, absRayStart);
		
		// 2. ray direction in absolute frame
//		chassisRot.transformVector(localRayDir, absRayEnd);
//		absRayEnd.scale(suspensionLength+wheelRadius);
		// then offset it by absRayStart
		Vector3.add(absRayStart, absRayEnd, absRayEnd);
		
		// 3. right vector in absolute frame
		// first, we rotate along the suspension travel in local frame
		Vector3 tmpAxis = Vector3.tmp0;
		Vector3.sub(localRayStart, localRayDir, tmpAxis);
		Quaternion axisRot = Quaternion.makeAxisRot(tmpAxis, wheelSteer);
		Vector3 tmpAxle = Vector3.tmp1;
		axisRot.transformVector(localRight, tmpAxle);
		
		chassisRot.transformVector(tmpAxle, absRight);
		
		// 4. absolute ray direction
		chassisRot.transformVector(localRayDir, absRayDir);
		
		float rL = suspensionLength + wheelRadius;
		absRayEnd.setTo(
				absRayStart.x + absRayDir.x * rL, 
				absRayStart.y + absRayDir.y * rL, 
				absRayStart.z + absRayDir.z * rL
				);
	}
	
	// gotta feed data
	
	// debug render function
	public void debugDraw(GL2 gl, float dt) {
		// we don't draw the wheel cylinder yet
		// draw start, end, and current hit
		
		gl.glBegin(GL2.GL_POINTS);
		
		// ray start
		gl.glColor3f(1, 0, 0);
		gl.glVertex3f(absRayStart.x, absRayStart.y, absRayStart.z);
			
		// ray end
		gl.glColor3f(0, 0, 1);
		gl.glVertex3f(absRayEnd.x, absRayEnd.y, absRayEnd.z);
		
		// in between (if we collide)
		if (groundObj != null) {
			gl.glColor3f(0, 1, 0);
			gl.glVertex3f(
					absRayStart.x * (1.f-rayT) + absRayEnd.x * rayT, 
					absRayStart.y * (1.f-rayT) + absRayEnd.y * rayT, 
					absRayStart.z * (1.f-rayT) + absRayEnd.z * rayT
					);
		}
		
		gl.glEnd();
		
		// gotta draw the wheel @ the wheel position
		
		Vector3 absRayZ = new Vector3();
		Vector3.cross(absRight, absRayDir, absRayZ);
		
		Matrix4 m = new Matrix4();
		m.setPosition(absWheelPos);
		m.setRotation(absRight.normalized(), absRayDir.inverse(), absRayZ.normalized());
		
		gl.glPushMatrix();
		gl.glMultMatrixf(m.m, 0);
		// rotate along the angular position
		gl.glRotatef((float) Math.toDegrees(wheelAngPos), 1, 0, 0);
//		drawWheel(gl, wheelRadius, 8);
		gl.glPopMatrix();
		
		// draw the ray
		gl.glBegin(GL2.GL_LINES);
		// ray start
		gl.glColor3f(1, 0, 0);
		gl.glVertex3f(absRayStart.x, absRayStart.y, absRayStart.z);
			
		// ray end
		gl.glColor3f(0, 0, 1);
		gl.glVertex3f(absRayEnd.x, absRayEnd.y, absRayEnd.z);
		
		// draw side slip friction
		float disp = wheelThickness * .5f;
		
		gl.glColor3f(1, .2f, 0.f);
		gl.glVertex3f(absRayEnd.x-absRight.x * disp, 
				absRayEnd.y-absRight.y * disp, 
				absRayEnd.z-absRight.z * disp);
		gl.glColor3f(1, .8f, 0.f);
		gl.glVertex3f(absRayEnd.x+absRight.x * disp, 
				absRayEnd.y+absRight.y * disp, 
				absRayEnd.z+absRight.z * disp);
		
		// draw forward vector
		if (groundObj != null) {
			Vector3 tmp = Vector3.tmp0;
			tmp.setTo(absRayStart.x * (1.f-rayT) + absRayEnd.x * rayT, 
					absRayStart.y * (1.f-rayT) + absRayEnd.y * rayT, 
					absRayStart.z * (1.f-rayT) + absRayEnd.z * rayT);
			
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(
					tmp.x, tmp.y, tmp.z
					);
			
			gl.glColor3f(1,1,1);
			gl.glVertex3f(tmp.x + rayHitForward.x, 
					tmp.y + rayHitForward.y, 
					tmp.z + rayHitForward.z);
			
			// also side vector
			gl.glColor3f(1, 1, 1);
			gl.glVertex3f(
					tmp.x, tmp.y, tmp.z
					);
			
			gl.glColor3f(0,1,1);
			gl.glVertex3f(tmp.x + rayHitSide.x, 
					tmp.y + rayHitSide.y, 
					tmp.z + rayHitSide.z);
		}
		
		gl.glEnd();
	}
}