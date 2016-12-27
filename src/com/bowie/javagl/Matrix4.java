/*
 * this is opengl 4x4 Matrix
 * used only for opengl transformation
 */

package com.bowie.javagl;

public class Matrix4 {
	public static Matrix4 tmp0 = new Matrix4();
	public static Matrix4 tmp1 = new Matrix4();
	
	public float [] m = new float[16];
	
	private static Matrix3 tmpMat0 = new Matrix3();
	private static Vector3 tmpV0 = new Vector3();
	private static float [] tmp = new float[16];
	
	public Matrix4() {
		this.setToIdentity();
	}
	
	public Matrix4(Quaternion q, Vector3 v) {
		//construct from quaternion and a vector
		this.fromQuatVec3(q, v);
	}
	
	public void transformVector(Vector3 v, Vector3 res) {
		tmpV0.x = m[0]*v.x+m[4]*v.y+m[8]*v.z+m[12];
        tmpV0.y = m[1]*v.x+m[5]*v.y+m[9]*v.z+m[13];
        tmpV0.z = m[2]*v.x+m[6]*v.y+m[10]*v.z+m[14];
        
        res.x = tmpV0.x;
        res.y = tmpV0.y;
        res.z = tmpV0.z;
	}
	
	public void fromQuatVec3(Quaternion q, Vector3 v) {
		//construct from quaternion and a vector
		q.toMatrix3(tmpMat0);
				
		m[0]=tmpMat0.m[0];    m[4]=tmpMat0.m[3];    m[8]=tmpMat0.m[6];    m[12]=v.x;
        m[1]=tmpMat0.m[1];    m[5]=tmpMat0.m[4];    m[9]=tmpMat0.m[7];    m[13]=v.y;
        m[2]=tmpMat0.m[2];    m[6]=tmpMat0.m[5];    m[10]=tmpMat0.m[8];    m[14]=v.z;
        m[3]=0.0f;      m[7]=0.0f;    m[11]=0.0f;    m[15]=1.0f;
	}
	
	public void setRotation(Vector3 right, Vector3 up, Vector3 front) {
		m[0] = right.x;
		m[1] = right.y;
		m[2] = right.z;
		m[3] = 0.0f;
		
		m[4] = up.x;
		m[5] = up.y;
		m[6] = up.z;
		m[7] = 0.0f;
		
		m[8] = front.x;
		m[9] = front.y;
		m[10] = front.z;
		m[11] = 0.0f;
	}
	
	public void setPosition(Vector3 p) {
		m[12] = p.x;
		m[13] = p.y;
		m[14] = p.z;
		m[15] = 1.0f;
	}
	
	public void quickInvert() {
		//for rendering purpose, the matrix needs to be correct tho
		float f;
		//swap 1 - 4
		f=m[1];	m[1]=m[4];	m[4]=f;
		//swap 2 - 8
		f=m[2];	m[2]=m[8];	m[8]=f;
		//swap 6 - 9
		f=m[6];	m[6]=m[9];	m[9]=f;
		
		//transform vertex
		tmpV0.x = m[0]*m[12] + m[4]*m[13] + m[8]*m[14];
        tmpV0.y = m[1]*m[12] + m[5]*m[13] + m[9]*m[14];
        tmpV0.z = m[2]*m[12] + m[6]*m[13] + m[10]*m[14];
        
        //set it
        m[12] = -tmpV0.x;
        m[13] = -tmpV0.y;
        m[14] = -tmpV0.z;
	}
	
	public float[] getValues() {
		return m;
	}
	
	//simple mult
	public static void mul(Matrix4 a, Matrix4 b, Matrix4 c) {
		tmp[0] = a.m[0] * b.m[0] + a.m[4] * b.m[1] + a.m[8] * b.m[2] + a.m[12] * b.m[3];
		tmp[4] = a.m[0] * b.m[4] + a.m[4] * b.m[5] + a.m[8] * b.m[6] + a.m[12] * b.m[7];
		tmp[8] = a.m[0] * b.m[8] + a.m[4] * b.m[9] + a.m[8] * b.m[10] + a.m[12] * b.m[11];
		tmp[12] = a.m[0] * b.m[12] + a.m[4] * b.m[13] + a.m[8] * b.m[14] + a.m[12] * b.m[15];
		
		tmp[1] = a.m[1] * b.m[0] + a.m[5] * b.m[1] + a.m[9] * b.m[2] + a.m[13] * b.m[3];
		tmp[5] = a.m[1] * b.m[4] + a.m[5] * b.m[5] + a.m[9] * b.m[6] + a.m[13] * b.m[7];
		tmp[9] = a.m[1] * b.m[8] + a.m[5] * b.m[9] + a.m[9] * b.m[10] + a.m[13] * b.m[11];
		tmp[13] = a.m[1] * b.m[12] + a.m[5] * b.m[13] + a.m[9] * b.m[14] + a.m[13] * b.m[15];
		
		tmp[2] = a.m[2] * b.m[0] + a.m[6] * b.m[1] + a.m[10] * b.m[2] + a.m[14] * b.m[3];
		tmp[6] = a.m[2] * b.m[4] + a.m[6] * b.m[5] + a.m[10] * b.m[6] + a.m[14] * b.m[7];
		tmp[10] = a.m[2] * b.m[8] + a.m[6] * b.m[9] + a.m[10] * b.m[10] + a.m[14] * b.m[11];
		tmp[14] = a.m[2] * b.m[12] + a.m[6] * b.m[13] + a.m[10] * b.m[14] + a.m[14] * b.m[15];
		
		tmp[3] = a.m[3] * b.m[0] + a.m[7] * b.m[1] + a.m[11] * b.m[2] + a.m[15] * b.m[3];
		tmp[7] = a.m[3] * b.m[4] + a.m[7] * b.m[5] + a.m[11] * b.m[6] + a.m[15] * b.m[7];
		tmp[11] = a.m[3] * b.m[8] + a.m[7] * b.m[9] + a.m[11] * b.m[10] + a.m[15] * b.m[11];
		tmp[15] = a.m[3] * b.m[12] + a.m[7] * b.m[13] + a.m[11] * b.m[14] + a.m[15] * b.m[15];
		
		//to c
		c.m[0] = tmp[0];	c.m[4] = tmp[4];	c.m[8] = tmp[8];	c.m[12] = tmp[12];
		c.m[1] = tmp[1];	c.m[5] = tmp[5];	c.m[9] = tmp[9];	c.m[13] = tmp[13];
		c.m[2] = tmp[2];	c.m[6] = tmp[6];	c.m[10] = tmp[10];	c.m[14] = tmp[14];
		c.m[3] = tmp[3];	c.m[7] = tmp[7];	c.m[11] = tmp[11];	c.m[15] = tmp[15];
	}
	
	//simple ortho matrix calculation
	public static void ortho(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, Matrix4 m) {
		m.m[0] = 2.0f/(xmax-xmin);
		m.m[5] = 2.0f/(ymax-ymin);
		m.m[10] = -2.0f/(zmax-zmin);
		
		m.m[12] = -(xmax+xmin)/(xmax-xmin);
		m.m[13] = -(ymax+ymin)/(ymax-ymin);
		m.m[14] = -(zmax+zmin)/(zmax-zmin);
		
		m.m[1]=m.m[2]=m.m[3]=m.m[4]=m.m[6]=m.m[7]=m.m[8]=m.m[9]=m.m[11]=0.0f;
		m.m[15]=1.0f;
	}
	
	//simple perspective matrix calculation
	public static void perspective(float fov, float aspect, float zmin, float zmax, Matrix4 m) {
		float onepertanfovper2 = (float) Math.tan(Math.toRadians(fov/2.0f));
		
		m.m[0] = onepertanfovper2/aspect;
		m.m[1] = 0;
		m.m[2] = 0;
		m.m[3] = 0;
		
		m.m[4] = 0;
		m.m[5] = onepertanfovper2;
		m.m[6] = 0;
		m.m[7] = 0;
		
		m.m[8] = 0;
		m.m[9] = 0;
		m.m[10] = -(zmax+zmin)/(zmax-zmin);
		m.m[11] = -1.0f;
		
		m.m[12] = 0;
		m.m[13] = 0;
		m.m[14] = -(2.0f*zmax*zmin)/(zmax-zmin);
		m.m[15] = 0;
	}
	
	public void setToIdentity() {
		m[1]=m[2]=m[3]=m[4]=m[6]=m[7]=m[8]=m[9]=m[11]=m[12]=m[13]=m[14]=0.0f;
		m[0]=m[5]=m[10]=m[15]=1.0f;
	}
	
	//a.transformBy(b) is a = b * a
	public void transformBy(Matrix4 t) {
		float [] nm = new float[16];
		
		nm[0] = t.m[0] * m[0] + t.m[4] * m[1] + t.m[8] * m[2] + t.m[12] * m[3];
		nm[4] = t.m[0] * m[4] + t.m[4] * m[5] + t.m[8] * m[6] + t.m[12] * m[7];
		nm[8] = t.m[0] * m[8] + t.m[4] * m[9] + t.m[8] * m[10] + t.m[12] * m[11];
		nm[12] = t.m[0] * m[12] + t.m[4] * m[13] + t.m[8] * m[14] + t.m[12] * m[15];
		
		nm[1] = t.m[1] * m[0] + t.m[5] * m[1] + t.m[9] * m[2] + t.m[13] * m[3];
		nm[5] = t.m[1] * m[4] + t.m[5] * m[5] + t.m[9] * m[6] + t.m[13] * m[7];
		nm[9] = t.m[1] * m[8] + t.m[5] * m[9] + t.m[9] * m[10] + t.m[13] * m[11];
		nm[13] = t.m[1] * m[12] + t.m[5] * m[13] + t.m[9] * m[14] + t.m[13] * m[15];
		
		nm[2] = t.m[2] * m[0] + t.m[6] * m[1] + t.m[10] * m[2] + t.m[14] * m[3];
		nm[6] = t.m[2] * m[4] + t.m[6] * m[5] + t.m[10] * m[6] + t.m[14] * m[7];
		nm[10] = t.m[2] * m[8] + t.m[6] * m[9] + t.m[10] * m[10] + t.m[14] * m[11];
		nm[14] = t.m[2] * m[12] + t.m[6] * m[13] + t.m[10] * m[14] + t.m[14] * m[15];
		
		nm[3] = t.m[3] * m[0] + t.m[7] * m[1] + t.m[11] * m[2] + t.m[15] * m[3];
		nm[7] = t.m[3] * m[4] + t.m[7] * m[5] + t.m[11] * m[6] + t.m[15] * m[7];
		nm[11] = t.m[3] * m[8] + t.m[7] * m[9] + t.m[11] * m[10] + t.m[15] * m[11];
		nm[15] = t.m[3] * m[12] + t.m[7] * m[13] + t.m[11] * m[14] + t.m[15] * m[15];
		
		m[0]=nm[0];
		m[1]=nm[1];
		m[2]=nm[2];
		m[3]=nm[3];
		
		m[4]=nm[4];
		m[5]=nm[5];
		m[6]=nm[6];
		m[7]=nm[7];
		
		m[8]=nm[8];
		m[9]=nm[9];
		m[10]=nm[10];
		m[11]=nm[11];
		
		m[12]=nm[12];
		m[13]=nm[13];
		m[14]=nm[14];
		m[15]=nm[15];
	}
	
	//this make a translation matrix
	static public Matrix4 makeTranslate(float x, float y, float z) {
		Matrix4 t = new Matrix4();
		t.m[12] = x;
		t.m[13] = y;
		t.m[14] = z;
		return t;
	}
	
	public void fastInvert() {
		float det = 0;
		tmp[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6]
				* m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13]
				* m[7] * m[10];

		tmp[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6]
				* m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12]
				* m[7] * m[10];

		tmp[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5]
				* m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12]
				* m[7] * m[9];

		tmp[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5]
				* m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12]
				* m[6] * m[9];

		tmp[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2]
				* m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13]
				* m[3] * m[10];

		tmp[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2]
				* m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12]
				* m[3] * m[10];

		tmp[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1]
				* m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12]
				* m[3] * m[9];

		tmp[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1]
				* m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12]
				* m[2] * m[9];

		tmp[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2]
				* m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13]
				* m[3] * m[6];

		tmp[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2]
				* m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12]
				* m[3] * m[6];

		tmp[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1]
				* m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12]
				* m[3] * m[5];

		tmp[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1]
				* m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12]
				* m[2] * m[5];

		tmp[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2]
				* m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9]
				* m[3] * m[6];

		tmp[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2]
				* m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8]
				* m[3] * m[6];

		tmp[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1]
				* m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3]
				* m[5];

		tmp[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1]
				* m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2]
				* m[5];

		det = m[0] * tmp[0] + m[1] * tmp[4] + m[2] * tmp[8] + m[3] * tmp[12];
		if (Math.abs(det) < Vector3.EPSILON)
			return;
		
		det = 1.0f/det;
		
		for (int i=0; i<16; i++)
			m[i] = tmp[i] * det;
	}
}
