package com.bowie.javagl;

public class Matrix3 {
	public static Matrix3 tmp0 = new Matrix3();
	public static Matrix3 tmp1 = new Matrix3();
	
	public float [] m = new float[9];
	
	private static float [] tmp = new float[9];
	
	public Matrix3() {
		m[0]=m[1]=m[2]=m[3]=m[4]=m[5]=m[6]=m[7]=m[8]=0;
	}
	
	public Matrix3(Matrix3 r) {
		m = r.m.clone();
	}
	
	public void add(Matrix3 mat) {
		m[0] += mat.m[0];
		m[1] += mat.m[1];
		m[2] += mat.m[2];
		
		m[3] += mat.m[3];
		m[4] += mat.m[4];
		m[5] += mat.m[5];
		
		m[6] += mat.m[6];
		m[7] += mat.m[7];
		m[8] += mat.m[8];
	}
	
	public void sub(Matrix3 mat) {
		m[0] -= mat.m[0];
		m[1] -= mat.m[1];
		m[2] -= mat.m[2];
		
		m[3] -= mat.m[3];
		m[4] -= mat.m[4];
		m[5] -= mat.m[5];
		
		m[6] -= mat.m[6];
		m[7] -= mat.m[7];
		m[8] -= mat.m[8];
	}
	
	public Matrix3(float m0, float m3, float m6,
			float m1, float m4, float m7,
			float m2, float m5, float m8) {
		m[0] = m0;
		m[1] = m1;
		m[2] = m2;
		
		m[3] = m3;
		m[4] = m4;
		m[5] = m5;
		
		m[6] = m6;
		m[7] = m7;
		m[8] = m8;
	}
	
	public void setToIdentity() {
		m[0]=m[4]=m[8]=1;
		m[1]=m[2]=m[3]=m[5]=m[6]=m[7]=0;
	}
	
	public static void mul(Matrix3 a, Matrix3 b, Matrix3 c) {
		tmp[0] = a.m[0]*b.m[0] + a.m[3]*b.m[1] + a.m[6]*b.m[2];
		tmp[3] = a.m[0]*b.m[3] + a.m[3]*b.m[4] + a.m[6]*b.m[5];
		tmp[6] = a.m[0]*b.m[6] + a.m[3]*b.m[7] + a.m[6]*b.m[8];
		
		tmp[1] = a.m[1]*b.m[0] + a.m[4]*b.m[1] + a.m[7]*b.m[2];
		tmp[4] = a.m[1]*b.m[3] + a.m[4]*b.m[4] + a.m[7]*b.m[5];
		tmp[7] = a.m[1]*b.m[6] + a.m[4]*b.m[7] + a.m[7]*b.m[8];
		
		tmp[2] = a.m[2]*b.m[0] + a.m[5]*b.m[1] + a.m[8]*b.m[2];
		tmp[5] = a.m[2]*b.m[3] + a.m[5]*b.m[4] + a.m[8]*b.m[5];
		tmp[8] = a.m[2]*b.m[6] + a.m[5]*b.m[7] + a.m[8]*b.m[8];
		
//		c.m = tmp.clone();
		c.m[0] = tmp[0];
		c.m[1] = tmp[1];
		c.m[2] = tmp[2];
		
		c.m[3] = tmp[3];
		c.m[4] = tmp[4];
		c.m[5] = tmp[5];
		
		c.m[6] = tmp[6];
		c.m[7] = tmp[7];
		c.m[8] = tmp[8];
	}
	
	public void scale(float s) {
		m[0] *= s;
		m[1] *= s;
		m[2] *= s;
		
		m[3] *= s;
		m[4] *= s;
		m[5] *= s;
		
		m[6] *= s;
		m[7] *= s;
		m[8] *= s;
	}
	
	public void transformVector3(Vector3 v, Vector3 res) {
		float rx, ry, rz;
		
		rx = m[0] * v.x + m[3] * v.y + m[6] * v.z;
		ry = m[1] * v.x + m[4] * v.y + m[7] * v.z;
		rz = m[2] * v.x + m[5] * v.y + m[8] * v.z;
		
		res.x = rx;
		res.y = ry;
		res.z = rz;
	}
	
	public float determinant() {
		return m[0]*(m[4]*m[8]-m[5]*m[7]) - m[3]*(m[1]*m[8]-m[2]*m[7]) + m[6]*(m[1]*m[5]-m[4]*m[2]);
	}
	
	static public Matrix3 skew(Vector3 v) {
		// return skew matrix of v
		return new Matrix3(
				0, 		-v.z, 	v.y, 
				v.z, 	0, 		-v.x, 
				-v.y, 	v.x, 	0);
	}
	
	static public Matrix3 diagonal(float v) {
		return new Matrix3(
				v, 0, 0,
				0, v, 0,
				0, 0, v
				);
	}
	
	public void invert() {
		float det= m[0]*(m[4]*m[8]-m[5]*m[7]) - m[3]*(m[1]*m[8]-m[2]*m[7]) + m[6]*(m[1]*m[5]-m[4]*m[2]);
		//check for zero determinant
		if (Math.abs(det) < Vector3.EPSILON)
			return;	//do nothing
		
		det=1.0f/det;
		
		tmp[0] = ( m[4]*m[8]-m[5]*m[7] ) * det;
        tmp[1] = ( m[7]*m[2]-m[1]*m[8] ) * det;
        tmp[2] = ( m[1]*m[5]-m[4]*m[2] ) * det;
        tmp[3] = ( m[6]*m[5]-m[3]*m[8] ) * det;
        tmp[4] = ( m[0]*m[8]-m[6]*m[2] ) * det;
        tmp[5] = ( m[3]*m[2]-m[0]*m[5] ) * det;
        tmp[6] = ( m[3]*m[7]-m[6]*m[4] ) * det;
        tmp[7] = ( m[6]*m[1]-m[0]*m[7] ) * det;
        tmp[8] = ( m[0]*m[4]-m[3]*m[1] ) * det;
        
        m = tmp.clone();
	}
	
	public Matrix3 inverted() {
		//return inverted version, keep self unmodified
		Matrix3 ret = new Matrix3(this);
		ret.invert();
		return ret;
	}
	
	public void transpose() {
		//modify transpose this matrix
		float f;	//to help swapping
		//swap 1 - 3
		f = m[1];
		m[1] = m[3];
		m[3] = f;
		//swap 2 - 6
		f = m[2];
		m[2] = m[6];
		m[6] = f;
		//swap 5 - 7
		f = m[5];
		m[5] = m[7];
		m[7] = f;
	}
}
