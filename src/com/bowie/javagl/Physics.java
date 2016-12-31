package com.bowie.javagl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.jogamp.opengl.GL2;

public class Physics {

	public static final int TIME_UPDATE_VEL = 0;
	public static final int TIME_REFRESH_CONTACT = 1;
	public static final int TIME_BROAD_PHASE = 2;
	public static final int TIME_NARROW_PHASE = 3;
	public static final int TIME_SORT_CONTACT = 4;
	public static final int TIME_PRE_STEP = 5;
	public static final int TIME_SOLVER = 6;
	public static final int TIME_POSITION_SOLVER = 7;
	public static final int TIME_INTEGRATE = 8;
	
	private int solverIteration = 10;
	private int positionIteration = 5;
	private float baumgarte = 0.2f;
	private float slop = 0.02f;
	
	private float linearDamping = 0.0000f, angularDamping = 0.0000f;	// to slow down shits
	
	private Vector3 gravity = new Vector3(0, -10, 0);
	
	// support 4 x 4 contact generator
	private ContactGenerator [][] contactGens = new ContactGenerator[][]{
			{null, null, null, null},
			{null, null, null, null},
			{null, null, null, null},
			{null, null, null, null}
	};
	
	// list of rigid bodies
	private List<RigidBody> bodies = new ArrayList<>();
	
	// list of joints
	private List<Joint> joints = new ArrayList<>();
	
	// list of simulated forces
	private List<SimForce> forces = new ArrayList<>();
	
	// list of persistent manifold
	private Map<BodyPair, PersistentManifold> manifolds = new HashMap<BodyPair, PersistentManifold>();
	
	//this gets resorted every frame
	private List<PersistentManifold> sortedManifolds = new ArrayList<>();
		
	// this is temporary stuffs
	private List<BodyPair> potentialColliders = new ArrayList<>();
	
	// this is for profiler
	private double [] perfTime;
	
	public Physics(int solverIteration, int positionIteration, float slop, float baumgarte) {	
		this.solverIteration = solverIteration;
		this.positionIteration = solverIteration;
		this.slop = slop;
		this.baumgarte = baumgarte;
		
		// initialize profiler
		perfTime = new double[10];
	}
	
	public void reset() {
		bodies.clear();
		manifolds.clear();
		forces.clear();
		joints.clear();
		sortedManifolds.clear();
	}
	
	public void registerContactGenerator(int sIdA, int sIdB, ContactGenerator cg) {
		if (sIdA >= 4 || sIdB >= 4 || sIdA < 0 || sIdB < 0)
			return;
		// it's valid
		contactGens[sIdA][sIdB] = cg;
		contactGens[sIdB][sIdA] = cg;
	}

	public void addBody(RigidBody b) {
		// simply add to list of bodies
		bodies.add(b);
	}
	
	public void addJoint(Joint j) {
		joints.add(j);
	}
	
	public void addSimForce(SimForce f) {
		forces.add(f);
	}
	
	public int getBodyCount() {
		return bodies.size();
	}
	
	public int getForceCount() {
		return forces.size();
	}
	
	public int getJointCount() {
		return joints.size();
	}
	
	public int getPairCount() {
		return manifolds.keySet().size();
	}
	
	public int getManifoldCount() {
		return sortedManifolds.size();
	}
	
	public void sortContacts() {
		// clear them
		
		// sort them
		try {
			Collections.sort(sortedManifolds, new Comparator<PersistentManifold>() {

				@Override
				public int compare(PersistentManifold m1, PersistentManifold m2) {
					return (int) (m1.getHeight(10)-m2.getHeight(10));
				}
			});
		} catch (IllegalArgumentException e) {
			System.out.println("fuck this inconsistent contact sorting!!");
		}
			
	}
	
	public void refreshContacts() {
		// here we should be removing untouching bodies entry pair (persistent manifold)
		
		Iterator<Entry<BodyPair, PersistentManifold>> iter = manifolds.entrySet().iterator();
		while (iter.hasNext()) {
			// are the bodies still touching?
			@SuppressWarnings("rawtypes")
			Map.Entry pair = (Map.Entry)iter.next();
			BodyPair k = (BodyPair) pair.getKey();
			if (!k.stillInProximity()) {
				// remove from both list/map!!
				sortedManifolds.remove(pair.getValue());
				iter.remove();
				
//				System.out.println("removing old pair!!");
			} else {
				// refresh here!!
				PersistentManifold m = (PersistentManifold) pair.getValue();
				m.refresh();
			}
		}
	
	}
	
	public void applyAllForces(float dt) {
		// first, simulate all forces
		
		for (SimForce f : forces) {
			f.simulate(dt);
		}
	}
	
	public void updateVelocity(float dt) {

		for (RigidBody b : bodies) {
			// apply gravity here (if body is not fixed)
			if (!b.isFixed())
				b.applyGravity(gravity);
			
			b.updateVelocity(dt);
			b.updateBBox(dt);
			b.clearForces();
		}
	
	}
	
	public void updatePosition(float dt) {
		// update position + apply damping
		for (RigidBody b : bodies) {
			b.updatePosition(dt);
			b.applyLinearDamping(linearDamping);
			b.applyAngularDamping(angularDamping);
		}
	}
	
	public void step(float dt) {
		long [] timer = new long[10];
		int timeId = 0;
		
		timer[timeId++] = System.nanoTime();		// 0
		
		// first, simulate all forces
		applyAllForces(dt);
		
		// next, we update all velocities + update bounding boxes + clear forces
		updateVelocity(dt);
		
		timer[timeId++] = System.nanoTime();		// 1
		
		// refresh contacts, remove invalid ones. Here, the bounding box is up to date
		refreshContacts();
		
		timer[timeId++] = System.nanoTime();		// 2
		
		// enter broadphase (AABB is up to date here, so would be safe I guess)
		broadPhase(dt);
		
		timer[timeId++] = System.nanoTime();		// 3
		
		// enter narrowphase (also contains contact generation)
		narrowPhase(dt);
		
		timer[timeId++] = System.nanoTime();		// 4

		// sort contact here
		sortContacts();
		
		timer[timeId++] = System.nanoTime();		// 5
		
		// pre step
		solverPreStep(dt);		
		
		timer[timeId++] = System.nanoTime();		// 6
		
		// solve
		solve();
		
		timer[timeId++] = System.nanoTime();		// 7
		
		// now position correction
		positionCorrection(dt);
		
		timer[timeId++] = System.nanoTime();		// 8
		// last, update position
		updatePosition(dt);		
		
		timer[timeId++] = System.nanoTime();		// 9
		// print log
//		System.out.println("potential, nbodies, nmanifolds: " + potentialColliders.size()+", " +bodies.size()+
//				", " + manifolds.size());
		
		double timeVelUpdate	= (double) (timer[1] - timer[0])/1000000;
		double timeRefreshContact	= (double) (timer[2] - timer[1])/1000000;
		double timeBroadPhase 	= (double) (timer[3] - timer[2])/1000000;
		double timeNarrowPhase	= (double) (timer[4] - timer[3])/1000000;
		double timeSortContact	= (double) (timer[5] - timer[4])/1000000;
		double timePreStep		= (double) (timer[6] - timer[5])/1000000;
		double timeSolver			= (double) (timer[7] - timer[6])/1000000;
		double timePositionSolve	= (double) (timer[8] - timer[7])/1000000;
		double timeIntegrate		= (double) (timer[9] - timer[8])/1000000;
		
		perfTime[0]	= timeVelUpdate;
		perfTime[1]	= timeRefreshContact;
		perfTime[2] = timeBroadPhase;
		perfTime[3] = timeNarrowPhase;
		perfTime[4] = timeSortContact;
		perfTime[5] = timePreStep;
		perfTime[6] = timeSolver;
		perfTime[7] = timePositionSolve;
		perfTime[8] = timeIntegrate;
		
//		perfCounter ++;
	}
	
	public double getPerformanceTimer(int id) {
		return perfTime[id];
	}
	
	public void debugDraw(GL2 gl, boolean drawContacts, boolean drawContactN, boolean drawContactT, boolean drawBBox, float dt) {
		// draw all rigid bodies, using colors from
		for (RigidBody b : bodies) {
			float [] color = Polytope.getColor(b.getId());
			
			gl.glColor3f(color[0], color[1], color[2]);
			b.debugDraw(gl, dt);
			
			if (drawBBox) {
				gl.glColor3f(0.2f, 0.0f, 0.8f);
				b.getBbox().debugDraw(gl);
			}
		}
		
		
		// draw all manifolds?
		if (drawContacts) {
			
			for (PersistentManifold m : manifolds.values()) {
				m.debugDraw(gl, drawContactN, drawContactT);
			}
			
		}	
		
		
		for (Joint j : joints) {
			j.debugDraw(gl);
		}
		
	}
	
	public void broadPhase(float dt) {
		// reset
		potentialColliders.clear();
		
		// brute force (O(n2))		
		for (int i=0; i<bodies.size()-1; i++) {
			RigidBody b1 = bodies.get(i);
			for (int j=bodies.size()-1; j>=0; --j) {
				// skip similar indices
				if (i == j)
					continue;

				RigidBody b2 = bodies.get(j);
				
				// skip fixed bodies
				if (b1.isFixed() && b2.isFixed())
					continue;
				
				// skip infinite mass
				if (b1.getInvMass() < Vector3.EPSILON && b2.getInvMass() < Vector3.EPSILON)
					continue;
				
				// are we close enough? BBox collide
				if (b1.getBbox().overlap(b2.getBbox())) {
					// add to potential colliders, also add pointer to persistent manifold
					potentialColliders.add(new BodyPair(b1, b2));
				}
			}
		} // for loop
		
	}
	
	public void narrowPhase(float dt) {
		// for each entry, gotta find persistent cache
		for (BodyPair p : potentialColliders) {				
			
			PersistentManifold m = manifolds.get(p);
			// if we cannot find, then add new
			if (m == null) {
				m = new PersistentManifold(p);
				// add to map
				manifolds.put(p, m);
				// add to to be sorted list
				sortedManifolds.add(m);
			}
			
			// grab contact generator
			// loop over each shapes, though
			ContactGenerator cg = contactGens[p.getBodyA().getShape().getShapeID()][p.getBodyB().getShape().getShapeID()];
			
			if (cg != null) {
				// execute contact generator
				RigidBody bA = p.getBodyA();
				RigidBody bB = p.getBodyB();
				
				cg.generateContact(m, bA.getShape(), bA.getPos(), bA.getRot(), bB.getShape(), bB.getPos(), bB.getRot(), bA, bB);
			} else {
				System.out.printf("Shape contact unhandled: %d, %d%n", p.getBodyA().getShape().getShapeID(), 
						p.getBodyB().getShape().getShapeID());
			}
			
			
		} // for	
	}
	
	
	public void solverPreStep(float dt) {
		/*synchronized (manifolds) {
			for (PersistentManifold m : manifolds.values()) {
				m.preStep(dt, slop, baumgarte);	// no slop and baumgarte
			}
		}*/ // sync
		
//			int i = 0;
//			System.out.println("===============PRE STEP===============");
		for (PersistentManifold m : sortedManifolds) {
			m.preStep(dt, slop, baumgarte);
//				System.out.println(i + " : " + m.getHeight(10));
//				i++;
		}
		
		
		// now for joints
		for (Joint j : joints) {
			j.preCalculate(dt, baumgarte);
		}
		
	}
	
	public void solve() {
		// solve it
		/*synchronized (manifolds) {
			for (int i=0; i<solverIteration; i++) {
				for (PersistentManifold m : manifolds.values()) {
					m.solve();
				}
			}
		}*/
		
		for (int i=0; i<solverIteration; i++) {
			
			// solve contacts
			for (PersistentManifold m : sortedManifolds) {
				m.solve();
			}
			
			
			// now for joints
			for (Joint j : joints) {
				j.solve();
			}
			
		}
		
	}
	
	public void positionCorrection(float dt) {
		/*synchronized (manifolds) {
			float strength = 1.0f/(float)positionIteration;
			for (int i=0; i<positionIteration; i++) {
				for (PersistentManifold m : manifolds.values()) {
					m.positionSolve(baumgarte, slop, strength);
				}
				
				// would be good to run precompute again here
			}
		}*/
		// we spread the impulse over several iteration
		// so for each iteration, we apply only a fraction
		// of the bias impulse
		for (int i=0; i<positionIteration; i++) {
			
			for (PersistentManifold m : sortedManifolds) {
				m.positionSolve();
			}
		
		
		
			for (Joint j : joints) {
				j.positionSolve();
			}
			
		}
	}

	public List<RigidBody> getBodies() {
		return bodies;
	}

	public List<SimForce> getForces() {
		return forces;
	}

	public Map<BodyPair, PersistentManifold> getManifolds() {
		return manifolds;
	}

	public float getLinearDamping() {
		return linearDamping;
	}

	public void setLinearDamping(float linearDamping) {
		this.linearDamping = linearDamping;
	}

	public float getAngularDamping() {
		return angularDamping;
	}

	public void setAngularDamping(float angularDamping) {
		this.angularDamping = angularDamping;
	}

	public Vector3 getGravity() {
		return gravity;
	}

	public void setGravity(Vector3 gravity) {
		this.gravity = gravity;
	}
}
