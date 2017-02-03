package com.bowie.javagl;

import com.jogamp.opengl.GL2;

/**
 * Joint - interface for everything joint like 
 * (i.e., solving erverything using impulse)
 * @author Bowie
 *
 */
public interface Joint {
	/**
	 * this is for precalculation, the parameters are provided by engine settings
	 * @param dt		- the timestep of physics engine
	 * @param baumgarte	- the baumgarte stabilization constant
	 */
	public void preCalculate(float dt, float baumgarte, float slop);
	
	/**
	 * this is where the constraint is solved (it's using Sequential Impulse)
	 */
	public void solve();
	
	/**
	 * general purpose render function for debugging purpose
	 * @param gl	- the opengl handle
	 */
	public void debugDraw(GL2 gl);
	
	/**
	 * this is because we use different approach for solving positional drift
	 * implement it if your joint uses it
	 */
	public void positionSolve();
}
