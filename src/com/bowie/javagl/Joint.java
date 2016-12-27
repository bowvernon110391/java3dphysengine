package com.bowie.javagl;

import com.jogamp.opengl.GL2;

public interface Joint {
	public void preCalculate(float dt, float baumgarte);
	public void solve();
	public void debugDraw(GL2 gl);
	public void positionSolve();
}
