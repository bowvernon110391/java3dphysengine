package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public class Box extends Shape {
	private float width, height, depth;
	
	public Box(float w, float h, float d) {
		width = w;
		height = h;
		depth = d;
	}
	
	@Override
	public
	void render(GL2 gl) {
		float halfWidth = width * 0.5f;
		float halfHeight = height * 0.5f;
		float halfDepth = depth * 0.5f;
		
		// front face
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glVertex3f(-halfWidth, -halfHeight, halfDepth);
			gl.glVertex3f( halfWidth, -halfHeight, halfDepth);
			gl.glVertex3f( halfWidth,  halfHeight, halfDepth);
			gl.glVertex3f(-halfWidth,  halfHeight, halfDepth);
		gl.glEnd();
		
		// front face
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glVertex3f( halfWidth, -halfHeight, -halfDepth);
			gl.glVertex3f(-halfWidth, -halfHeight, -halfDepth);
			gl.glVertex3f(-halfWidth,  halfHeight, -halfDepth);
			gl.glVertex3f( halfWidth,  halfHeight, -halfDepth);
		gl.glEnd();
		
		// ridges
		gl.glBegin(GL2.GL_LINES);
			gl.glVertex3f(-halfWidth, -halfHeight, -halfDepth);
			gl.glVertex3f(-halfWidth, -halfHeight,  halfDepth);
			
			gl.glVertex3f( halfWidth, -halfHeight, -halfDepth);
			gl.glVertex3f( halfWidth, -halfHeight,  halfDepth);
			
			gl.glVertex3f( halfWidth,  halfHeight, -halfDepth);
			gl.glVertex3f( halfWidth,  halfHeight,  halfDepth);
			
			gl.glVertex3f(-halfWidth,  halfHeight, -halfDepth);
			gl.glVertex3f(-halfWidth,  halfHeight,  halfDepth);
		gl.glEnd();
	}

	

	public float getWidth() {
		return width;
	}

	public void setWidth(float width) {
		this.width = width;
	}

	public float getHeight() {
		return height;
	}
	
	public void setHeight(float height) {
		this.height = height;
	}

	
	@Override
	public
	Vector3 supportPoint(Vector3 worldDir, Vector3 worldPos, Quaternion worldOri) {
		// transform into local direction
		Vector3 localDir = Vector3.tmp0;
		worldOri.conjugated().transformVector(worldDir, localDir);
		
		// it's a matter of which is which
		float halfWidth = width * 0.5f;
		float halfHeight = height * 0.5f;
		float halfDepth = depth * 0.5f;
		// set default point (bottom left)
		Vector3 retPoint = new Vector3(-halfWidth, -halfHeight, -halfDepth);
		
		// which is which
		if (localDir.x < 0)
			retPoint.x = -halfWidth;
		else
			retPoint.x = halfWidth;
		
		if (localDir.y < 0)
			retPoint.y = -halfHeight;
		else
			retPoint.y = halfHeight;
		
		if (localDir.z < 0)
			retPoint.z = -halfDepth;
		else
			retPoint.z = halfDepth;
		
		// got the point, now rotate it
		worldOri.transformVector(retPoint, retPoint);
		
		// offset it
		/*Vector3.add(worldPos, retPoint, retPoint);
		
		// return it
		return retPoint;*/
		return new Vector3(
				worldPos.x + retPoint.x,
				worldPos.y + retPoint.y,
				worldPos.z + retPoint.z
				);
	}
	
	@Override
	public Polygon getFace(Vector3 dir, Vector3 worldPos, Quaternion worldRot) {
		Vector3 localDir = new Vector3();
		// get local direction
		worldRot.conjugated().transformVector(dir, localDir);
		
		// where are we pointing?
		float aX = Math.abs(localDir.x);
		float aY = Math.abs(localDir.y);
		float aZ = Math.abs(localDir.z);
		
		Polygon pol = new Polygon();
		
		// halfsizes
		float halfW = width * 0.5f;
		float halfH = height * 0.5f;
		float halfD = depth * 0.5f;
		
		// up/down?
		if (aY > aX && aY > aZ) {
			// up/down
			if (localDir.y > 0) {
				// up
				pol.setNormal(new Vector3(0, 1, 0));
				
				pol.addPoint(new Vector3(-halfW, halfH, -halfD));
				pol.addPoint(new Vector3(-halfW, halfH,  halfD));
				pol.addPoint(new Vector3( halfW, halfH,  halfD));
				pol.addPoint(new Vector3( halfW, halfH, -halfD));
			} else {
				// down
				pol.setNormal(new Vector3(0, -1, 0));
				
				pol.addPoint(new Vector3(-halfW, -halfH,  halfD));
				pol.addPoint(new Vector3(-halfW, -halfH, -halfD));
				pol.addPoint(new Vector3( halfW, -halfH, -halfD));
				pol.addPoint(new Vector3( halfW, -halfH,  halfD));
			}
		} else if (aX > aY && aX > aZ) {
			// left/right
			if (localDir.x > 0) {
				// right
				pol.setNormal(new Vector3(1, 0, 0));
				
				pol.addPoint(new Vector3( halfW, -halfH, halfD));
				pol.addPoint(new Vector3( halfW, -halfH,-halfD));
				pol.addPoint(new Vector3( halfW,  halfH,-halfD));
				pol.addPoint(new Vector3( halfW,  halfH, halfD));
			} else {
				// left
				pol.setNormal(new Vector3(-1, 0, 0));
				
				pol.addPoint(new Vector3(-halfW, -halfH, -halfD));
				pol.addPoint(new Vector3(-halfW, -halfH,  halfD));
				pol.addPoint(new Vector3(-halfW,  halfH,  halfD));
				pol.addPoint(new Vector3(-halfW,  halfH, -halfD));
			}
		} else {
			// must be front/back
			if (localDir.z > 0) {
				// front
				pol.setNormal(new Vector3(0, 0, 1));
				
				pol.addPoint(new Vector3(-halfW, -halfH, halfD));
				pol.addPoint(new Vector3( halfW, -halfH, halfD));
				pol.addPoint(new Vector3( halfW,  halfH, halfD));
				pol.addPoint(new Vector3(-halfW,  halfH, halfD));
			} else {
				// back
				pol.setNormal(new Vector3(0, 0, -1));
				
				pol.addPoint(new Vector3( halfW, -halfH, -halfD));
				pol.addPoint(new Vector3(-halfW, -halfH, -halfD));
				pol.addPoint(new Vector3(-halfW,  halfH, -halfD));
				pol.addPoint(new Vector3( halfW,  halfH, -halfD));
			}
		}
		// transform back
		pol.transform(worldPos, worldRot);
		return pol;
	}

	@Override
	public
	void render(GL2 gl, Vector3 pos, Quaternion ori) {
		// now we got full info on how to render
		// first, get matrix
		Matrix4 mat = new Matrix4(ori, pos);
		
		gl.glPushMatrix();
			gl.glMultMatrixf(mat.m, 0);
			
			this.render(gl);
		gl.glPopMatrix();
	}

	@Override
	public FeatureEdge getFeature(Vector3 dir) {
		// check direction
		FeatureEdge e = null;
		
		float halfW = width * 0.5f;
		float halfH = height * 0.5f;
		
		if (Math.abs(dir.x) > Math.abs(dir.y)) {
			// either left or right
			if (dir.x > 0) {
				// right
				e = new FeatureEdge(new Vector3(halfW, -halfH, 0), new Vector3(halfW, halfH, 0));
			} else {
				// left
				e = new FeatureEdge(new Vector3(-halfW,  halfH, 0), new Vector3(-halfW, -halfH, 0));
			}
		} else {
			// either top or bottom
			if (dir.y > 0) {
				// top
				e = new FeatureEdge(new Vector3(halfW,  halfH, 0), new Vector3(-halfW, halfH, 0));
			} else {
				// bottom
				e = new FeatureEdge(new Vector3(-halfW,  -halfH, 0), new Vector3(halfW, -halfH, 0));
			}
		}
		
		return e;
	}

	@Override
	public Matrix3 getInvInertiaTensor(float invMass) {
		float Ixx = (height*height + depth*depth)/12.0f;
		float Iyy = (width*width + depth*depth)/12.0f;
		float Izz = (width*width + height*height)/12.0f;
		
		Matrix3 I = new Matrix3(
				Ixx, 0, 0, 
				0, Iyy, 0, 
				0, 0, Izz);
		I.invert();
		I.scale(invMass);
		return I;
	}
	
	@Override
	public void getAABB(Vector3 worldPos, Quaternion worldOri, AABB oldbbox) {
		if (oldbbox == null)
			return;
		// construct 4 halfsizes
		Vector3 v1 = new Vector3(width*0.5f, height*0.5f, depth*0.5f);
		Vector3 v2 = new Vector3(v1.x, -v1.y, -v1.z);
		Vector3 v3 = new Vector3(-v1.x,  v1.y, -v1.z);
		Vector3 v4 = new Vector3(-v1.x, -v1.y, v1.z);
		
		// create 3 transformed axis
		Matrix3 mat = new Matrix3();
		worldOri.conjugated().toMatrix3(mat);
		
		Vector3 xAxis = new Vector3(mat.m[0], mat.m[1], mat.m[2]);
		Vector3 yAxis = new Vector3(mat.m[3], mat.m[4], mat.m[5]);
		Vector3 zAxis = new Vector3(mat.m[6], mat.m[7], mat.m[8]);
		
		// get maximum extent in each axis
		float d1, d2, d3, d4;
		
		// X AXIS
		d1 = Math.abs( Vector3.dot(v1, xAxis) );
		d2 = Math.abs( Vector3.dot(v2, xAxis) );
		d3 = Math.abs( Vector3.dot(v3, xAxis) );
		d4 = Math.abs( Vector3.dot(v4, xAxis) );
		
		float maxX = Math.max(Math.max(d1, d2), Math.max(d3, d4));
		
		// Y AXIS
		d1 = Math.abs( Vector3.dot(v1, yAxis) );
		d2 = Math.abs( Vector3.dot(v2, yAxis) );
		d3 = Math.abs( Vector3.dot(v3, yAxis) );
		d4 = Math.abs( Vector3.dot(v4, yAxis) );
		
		float maxY = Math.max(Math.max(d1, d2), Math.max(d3, d4));
		
		// Z AXIS
		d1 = Math.abs( Vector3.dot(v1, zAxis) );
		d2 = Math.abs( Vector3.dot(v2, zAxis) );
		d3 = Math.abs( Vector3.dot(v3, zAxis) );
		d4 = Math.abs( Vector3.dot(v4, zAxis) );
		
		float maxZ = Math.max(Math.max(d1, d2), Math.max(d3, d4));
		
		// update aabb
		oldbbox.max.x = maxX;
		oldbbox.max.y = maxY;
		oldbbox.max.z = maxZ;
		
		oldbbox.min.x = -maxX;
		oldbbox.min.y = -maxY;
		oldbbox.min.z = -maxZ;
		
		oldbbox.move(worldPos);
	}

	@Override
	public Shape getDeflated(float margin) {
		return new Box(width-margin * 2, height-margin * 2, depth-margin * 2);
	}
	
	@Override
	public int getShapeID() {
		return Shape.SHAPE_BOX;
	}
}
