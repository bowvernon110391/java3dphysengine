package com.bowie.javagl;

public class CSOVertex {
	public Vector3 a, b, p;	// from body A, from body B, the resulting minkowski difference
	
	public CSOVertex(Vector3 a_, Vector3 b_) {
		p = new Vector3();
		a = new Vector3(a_);
		b = new Vector3(b_);
		
		Vector3.sub(a_, b_, p);
	}
	
	public CSOVertex(CSOVertex v) {
		p = new Vector3(v.p);
		a = new Vector3(v.a);
		b = new Vector3(v.b);
	}
	
	@Override
	public boolean equals(Object e) {
		// TODO Auto-generated method stub
		return Vector3.equal(p, ((CSOVertex)e).p);
	}
}
