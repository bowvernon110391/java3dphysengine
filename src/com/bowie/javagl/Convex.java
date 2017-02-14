package com.bowie.javagl;

import java.util.ArrayList;
import java.util.List;

import com.jogamp.opengl.GL2;

public class Convex extends Shape {
	
	private List<Vector3> points;
	private List<Polygon> faces;
	private List<List<Integer>> face_ids;
	
	public Convex(Convex master, List<Vector3> newPts) {
		// this thing only copies master data
		points = new ArrayList<>(newPts);
		faces = new ArrayList<>();
		face_ids = new ArrayList<>();
		
		// copy face
		for (int i=0; i<master.getFaces().size(); i++) {
			faces.add(new Polygon(master.getFaces().get(i)));
		}
		
		// copy face index
		for (int i=0; i<master.face_ids.size(); i++) {
			face_ids.add(new ArrayList<Integer>());
			
			for (int j=0; j<master.face_ids.get(i).size(); j++) {
				face_ids.get(i).add(master.face_ids.get(i).get(j));
			}
		}
		
		// refine face data
		for (int i=0; i<faces.size(); i++) {
			Polygon p = faces.get(i);
			// now loop over all vertices
			for (int j=0; j<p.p.size(); j++) {
				p.p.get(j).setTo(points.get(face_ids.get(i).get(j)));
			}
		}
	}
	
	public Convex(Vector3[] pts, int[][] face_idx) {
		points = new ArrayList<>();
		faces = new ArrayList<>();
		face_ids = new ArrayList<>();
		
		// add vertices
		for (int i=0; i<pts.length; i++) {
			points.add(new Vector3(pts[i]));
		}
		
		// create faces
		for (int i=0; i<face_idx.length; i++) {
			// gotta copy face_ids
			face_ids.add(new ArrayList<Integer>());
			
			// this is it
			Polygon p = new Polygon();
			for (int j=0; j<face_idx[i].length; j++) {
				p.addPoint(points.get(face_idx[i][j]));
				
				// add face vertex id
				face_ids.get(i).add(face_idx[i][j]);
			}
			p.calcNormal();
			faces.add(p);
		}
	}
	
	@Override
	public void render(GL2 gl) {
		/*gl.glBegin(GL2.GL_LINE_LOOP);
		for (int i=0; i<points.size(); i++) {
			Vector3 p = points.get(i);
			gl.glVertex3f(p.x, p.y, p.z);
		}
		gl.glEnd();*/
		for (Polygon p : faces) {
			p.debugDraw(gl, true);
		}
	}

	@Override
	public void render(GL2 gl, Vector3 pos, Quaternion ori) {
		Matrix4 mat = new Matrix4(ori, pos);
		
		gl.glPushMatrix();
			gl.glMultMatrixf(mat.m, 0);
			
			this.render(gl);
		gl.glPopMatrix();
	}

	@Override
	public Vector3 supportPoint(Vector3 worldDir, Vector3 worldPos,
			Quaternion worldOri) {
		// transform into local direction
		Vector3 localDir = new Vector3();
		worldOri.conjugated().transformVector(worldDir, localDir);
		
		Vector3 maxPoint = points.get(0);
		float maxDist = Vector3.dot(maxPoint, localDir);
		
		for (int i = 1; i<points.size(); i++ ) {
			float dist = Vector3.dot(points.get(i), localDir);
			if (dist > maxDist) {
				maxDist = dist;
				maxPoint = points.get(i);
			}
		}
		// sp must not be null
		Vector3 retPoint = new Vector3();
		worldOri.transformVector(maxPoint, retPoint);
		Vector3.add(retPoint, worldPos, retPoint);
		
		return retPoint;
	}

	@Override
	public FeatureEdge getFeature(Vector3 dir) {
		// track edge
		FeatureEdge bestEdge = null;
		float d = 0.0f;
		boolean firstTime = true;
		
		// loop all vertex, creating an edge
		for (int i=0; i<points.size(); i++) {
			int j = (i == points.size()-1 ? 0 : i+1);
			
			// create edge
			FeatureEdge e = new FeatureEdge(points.get(i), points.get(j));
			
			// check dot product
			float dp = Vector3.dot(e.n, dir) + Vector3.dot(e.e[0], dir);
			
			// record it
			if (firstTime || dp > d) {
				d = dp;
				firstTime = false;
				
				bestEdge = e;
			}
		}
		return bestEdge;
	}

	@Override
	public Matrix3 getInvInertiaTensor(float invMass) {
		float iXX = 0;	// sqr
		float iYY = 0;	// sqr
		float iZZ = 0;	// sqr
		float iXY = 0;
		float iYZ = 0;
		float iZX = 0;
		
		for (Vector3 p : points) {
			iXX += p.y*p.y + p.z*p.z;
			iYY += p.x*p.x + p.z*p.z;
			iZZ += p.x*p.x + p.y*p.y;
			
			iXY += -(p.x*p.y);
			iYZ += -(p.y*p.z);
			iZX += -(p.z*p.x);
		}
		// normalize them
		float numPts = points.size();
		iXX /= numPts;
		iYY /= numPts;
		iZZ /= numPts;
		
		iXY /= numPts;
		iYZ /= numPts;
		iZX /= numPts;
		
		Matrix3 I = new Matrix3(
				iXX, iXY, iZX, 
				iXY, iYY, iYZ, 
				iZX, iYZ, iZZ);
		I.invert();
		I.scale(invMass);
		
		return I;
	}

	@Override
	public void getAABB(Vector3 worldPos, Quaternion worldOri, AABB oldbbox) {
		Matrix3 localRot = new Matrix3();
		worldOri.conjugated().toMatrix3(localRot);
		
		// construct axis
		Vector3 xAxis = new Vector3(localRot.m[0], localRot.m[1], localRot.m[2]);
		Vector3 yAxis = new Vector3(localRot.m[3], localRot.m[4], localRot.m[5]);
		Vector3 zAxis = new Vector3(localRot.m[6], localRot.m[7], localRot.m[8]);
		
		// grab min max
		float minX, maxX;
		float minY, maxY;
		float minZ, maxZ;
		
		Vector3 v = points.get(0);
		
		// init min max
		minX = maxX = Vector3.dot(v, xAxis);
		minY = maxY = Vector3.dot(v, yAxis);
		minZ = maxZ = Vector3.dot(v, zAxis);
		
		// loop
		for (int i=1; i<points.size(); i++) {
			v = points.get(i);
			float xL = Vector3.dot(v, xAxis);
			float yL = Vector3.dot(v, yAxis);
			float zL = Vector3.dot(v, zAxis);
			
			if (xL < minX) minX = xL;
			if (yL < minY) minY = yL;
			if (zL < minZ) minZ = zL;
			
			if (xL > maxX) maxX = xL;
			if (yL > maxY) maxY = yL;
			if (zL > maxZ) maxZ = zL;
		}
		// construct aabb
		oldbbox.min.x = minX;
		oldbbox.min.y = minY;
		oldbbox.min.z = minZ;
		
		oldbbox.max.x = maxX;
		oldbbox.max.y = maxY;
		oldbbox.max.z = maxZ;
		
		oldbbox.move(worldPos);
		return ;
	}

	public List<Vector3> getPoints() {
		return points;
	}

	public List<Polygon> getFaces() {
		return faces;
	}
	
	public void setFaces(List<Polygon> f) {
		// WAAIIITTT!!!
		// We'd better copy vertex data only
		// because we must already have index and polygonal data
		// just change vertex data, and reset polygon data
		
		for (Polygon origFace : faces) {
			for (Polygon pol : f) {
				// check if the normal is close
				float diff = 1.0f - Vector3.dot(origFace.n, pol.n);
				
				if (Math.abs(diff) < Vector3.EPSILON && origFace.p.size() == pol.p.size()) {
					Vector3 n1 = origFace.n;
					Vector3 n2 = pol.n;
					System.out.printf("potential match: [%.2f %.2f %.2f] -> [%.2f %.2f %.2f] : %d -> %d%n", n1.x, n1.y, n1.z, n2.x,
							n2.y, n2.z, origFace.p.size(), pol.p.size());
				}
			}
		}
		
		// for rendering purpose
		faces = f;
	}

	public List<List<Integer>> getFace_ids() {
		return face_ids;
	}

	@Override
	public Shape getDeflated(float margin) {
		return MathHelper.getDeflatedConvex(this, margin);
	}
	
	@Override
	public int getShapeID() {
		return Shape.SHAPE_CONVEX;
	}

	@Override
	public boolean raycast(Vector3 sPos, Quaternion sRot, RaycastInfo r) {
		// transform into local frame
		Vector3 rS_loc = new Vector3(r.rayStart, sPos);
		Vector3 rE_loc = new Vector3(r.rayEnd, sPos);
		
		// transform
		Quaternion invRot = sRot.conjugated();
		invRot.transformVector(rS_loc, rS_loc);
		invRot.transformVector(rE_loc, rE_loc);
		
		Vector3 rayDir = new Vector3(rE_loc, rS_loc);
		
		// loop over all polygon, do raycast against them
		float currentT = -1.f;	// so far
		Polygon polyHit = null;
		for (Polygon p : faces) {
			Vector3 AP = new Vector3(rS_loc, p.p.get(0));
			
			float uv = Vector3.dot(AP, p.n);
			// ignore backfaces
			if (uv < 0.f)
				continue;
			
			// go on
			float vv = Vector3.dot(rayDir, p.n);
			
			// prevent parallel
			if (Math.abs(vv) < Vector3.EPSILON) 
				continue;
			
			float t = -uv / vv;
			// is it worth it?
			if (t < 0 || t > 1)
				continue;
			
			if (t < currentT || currentT < 0) {
//				System.out.printf("better t: %.4f -> %.4f %n", currentT, t);
				// wait!! is it in the perimeter?
				// generate hit point
				Vector3 hitP = Vector3.tmp0;
				hitP.setTo(
						rS_loc.x * (1.f - t) + rE_loc.x * t,
						rS_loc.y * (1.f - t) + rE_loc.y * t,
						rS_loc.z * (1.f - t) + rE_loc.z * t
						);
				if (!p.pointInsidePerimeter(hitP)) {
//					System.out.println("too bad it's outside perimeter");
					continue;
				}
				
				// welp, it's good!
				currentT = t;
				polyHit = p;
			}
		}
		
		// ok, now we generate the real stuffs
		if (polyHit != null && currentT >= 0 && currentT <= 1) {
			r.hitPoly = new Polygon(polyHit);
			r.hitPoly.transform(sPos, sRot);
			// now calculate shit
			r.rayT = currentT;
			r.rayhitN.setTo(polyHit.n);
			sRot.transformVector(r.rayhitN, r.rayhitN);
			// compute hit position
			r.rayhitP.setTo(
					r.rayStart.x * (1.f-r.rayT) + r.rayEnd.x * r.rayT,
					r.rayStart.y * (1.f-r.rayT) + r.rayEnd.y * r.rayT,
					r.rayStart.z * (1.f-r.rayT) + r.rayEnd.z * r.rayT
					);
			return true;
		}
		
		return false;
	}
}
