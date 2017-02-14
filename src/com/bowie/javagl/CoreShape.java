package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class CoreShape extends Shape {
	// it contains convex
	public Shape outerShape = null;
	public Shape innerShape = null;
	public float margin = 0;
	
	public CoreShape(Shape s, float m) {
		outerShape = s;
		margin = m;
		
		// generate inner shape
		if (margin > Vector3.EPSILON)
			innerShape = outerShape.getDeflated(margin);
		else
			innerShape = outerShape;
	}
	
	@Override
	public void render(GL2 gl) {
		innerShape.render(gl);
		outerShape.render(gl);
	}

	@Override
	public int getShapeID() {
		return Shape.SHAPE_CORE;
	}

	@Override
	public void render(GL2 gl, Vector3 pos, Quaternion ori) {
		innerShape.render(gl, pos, ori);
		outerShape.render(gl, pos, ori);
	}

	@Override
	public Vector3 supportPoint(Vector3 worldDir, Vector3 worldPos,
			Quaternion worldOri) {
		return outerShape.supportPoint(worldDir, worldPos, worldOri);
	}

	@Override
	public void getAABB(Vector3 worldPos, Quaternion worldOri, AABB oldbbox) {
		outerShape.getAABB(worldPos, worldOri, oldbbox);
	}

	@Override
	public Shape getDeflated(float margin) {
		return innerShape;
	}

	@Override
	public Matrix3 getInvInertiaTensor(float invMass) {
		return outerShape.getInvInertiaTensor(invMass);
	}

	@Override
	public FeatureEdge getFeature(Vector3 dir) {
		return outerShape.getFeature(dir);
	}

	@Override
	public boolean raycast(Vector3 sPos, Quaternion sRot, RaycastInfo r) {
		return outerShape.raycast(sPos, sRot, r);
	}

}
