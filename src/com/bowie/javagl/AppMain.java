package com.bowie.javagl;


import java.awt.KeyboardFocusManager;
import java.util.ConcurrentModificationException;

import com.jogamp.nativewindow.WindowClosingProtocol.WindowClosingMode;
import com.jogamp.newt.Display;
import com.jogamp.newt.NewtFactory;
import com.jogamp.newt.Screen;
import com.jogamp.newt.event.KeyEvent;
import com.jogamp.newt.event.KeyListener;
import com.jogamp.newt.event.MouseEvent;
import com.jogamp.newt.event.MouseListener;
import com.jogamp.newt.opengl.GLWindow;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLContext;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.glu.GLUquadric;
import com.jogamp.opengl.glu.gl2.GLUgl2;
import com.jogamp.opengl.util.Animator;

public class AppMain {	
	
	public GLWindow glWindow = null;
	public Animator anim = null;	// GUI thread
	public Thread phys = null;	// separate physics thread? (UNUSED)
	
	// shared between thread!!
	volatile private float dt = 0;		// delta time
	volatile private float aspect = 1;
	
	
	private float fov = 80.0f;
	private Matrix4 matPers = new Matrix4();
	
	private Physics world = new Physics(7, 5, .04f, .3f);
	private RigidBody bomb;
	private WorldSpring ws;
	private WorldPinJoint wp;

	private RigidBody chassis;
	private WheelSet wheels;
	
//	private AppRes res = new AppRes();
	
	private boolean drawContacts = true;
	private boolean drawContactN = false;
	private boolean drawContactT = false;
	private boolean drawBBox = false;
	
	private int camMode = 0;
	
	private boolean useCoreShape = false;
	
	private float targetSteer = .0f;
	private float maxSteer = (float) Math.toRadians(40);	// 33 degree maximum
	private float steerStrength = .4f;
	private float currSteer = .0f;
	
	private boolean steerRight = false;
	private boolean steerLeft= false;
	private boolean forward = false;
	private boolean reverse = false;
	
	// this is our lock
//	final private ReentrantLock resLock = new ReentrantLock();
	
	private float yRot = 40;
	private float xzRot = 0.0f;
	private float dist = 6.0f;
	private float camStrength = .25f;
	private float camX, camY, camZ;
	private float camTX, camTY, camTZ;
	private float camVX, camVY, camVZ;
	
	final private float maxYRot = 89.0f;
	final private float minDist = 1.0f;
	final private float maxDist = 16.0f;
	
	private float profilerTrigger = 0;
	
	private int simFPS = 30;
	public float simDT = 1.0f/simFPS;
		
	public static void main(String[] args) {
		AppMain app = new AppMain();
		app.init();
		app.run();
	}
	
	public void init() {
		// simplex visualization
//		synchronized (res) {
//			boolean success = MathHelper.gjkClosestPoint(res.sA, res.posA, res.rotA, res.sB, res.posB, res.rotB,
//					new Vector3(1,0,0), res.simp);
//		}
		float worldSize = 250.0f;
		
		world.reset();
		
		Vector3 [] cylinder_vertex = new Vector3[] {
				new Vector3(-.5f,  .7f, -.5f),
				new Vector3(-.5f, -.7f, -.5f),
				
				new Vector3(-.7f,  .7f, -.0f),
				new Vector3(-.7f, -.7f, -.0f),
				
				new Vector3(-.5f,  .7f,  .5f),
				new Vector3(-.5f, -.7f,  .5f),
				
				new Vector3(-.0f,  .7f,  .7f),
				new Vector3(-.0f, -.7f,  .7f),
				
				new Vector3( .5f,  .7f,  .5f),
				new Vector3( .5f, -.7f,  .5f),
				
				new Vector3( .7f,  .7f, -.0f),
				new Vector3( .7f, -.7f, -.0f),
				
				new Vector3( .5f,  .7f, -.5f),
				new Vector3( .5f, -.7f, -.5f),
				
				new Vector3(-.0f,  .7f, -.7f),
				new Vector3(-.0f, -.7f, -.7f),
		};
		
		int [][] cylinder_faces = new int[][] {
				{0, 1, 3, 2},
				{2, 3, 5, 4},
				{4, 5, 7, 6},
				{6, 7, 9, 8},
				{8, 9, 11, 10},
				{10, 11, 13, 12},
				{12, 13, 15, 14},
				{14, 15, 1, 0},
				
				{0, 2, 4, 6, 8, 10, 12, 14},
				{15, 13, 11, 9, 7, 5, 3, 1}
		};
		
		Vector3 [] cone_vertex = new Vector3[] {
				new Vector3(0, 0.65f, 0),
				new Vector3(-.5f, -.5f, -.5f),
					new Vector3(-.7f, -.5f, 0),
				new Vector3(-.5f, -.5f,  .5f),
					new Vector3(0, -.5f, .7f),
				new Vector3( .5f, -.5f,  .5f),
					new Vector3(.7f, -.5f, 0),
				new Vector3( .5f, -.5f, -.5f),
					new Vector3(0, -.5f, -.7f)
		};
		
		int [][] cone_faces = new int[][] {
				{0, 1, 2},
				{0, 2, 3},
				{0, 3, 4},
				{0, 4, 5},
				{0, 5, 6},
				{0, 6, 7},
				{0, 7, 8},
				{0, 8, 1},
				{8,7,6,5,4,3,2,1}
		};
		
		// 1.5f, .5f, 2.75f
		Vector3 [] chassis_vertex = new Vector3[]{
				new Vector3(-.85f, -.001f,-1.55f),	// 
				new Vector3( .85f, -.001f,-1.55f),
				new Vector3( .85f, -.001f, 1.9f),
				new Vector3(-.85f, -.001f, 1.9f),
				
				new Vector3(-.875f,  .8f,-1.7f),	// 
				new Vector3( .875f,  .8f,-1.7f),
				new Vector3( .875f,  .5f, 1.15f),
				new Vector3(-.875f,  .5f, 1.15f),
		};
		
		int [][] chassis_faces = new int[][]{
				{0, 1, 2, 3},	// bottom
				{7, 6, 5, 4},	// top
				{3, 2, 6, 7},	// front
				{1, 0, 4, 5},	// back
				{2, 1, 5, 6},	// right
				{0, 3, 7, 4}	// left
		};
		
		float halfRampW = 4;
		float rampH = 2;
		float halfRampD = 6;
		
		Vector3 [] ramp_vertex = new Vector3[]{
				new Vector3(-halfRampW, 0, halfRampD),
				new Vector3( halfRampW, 0, halfRampD),
				new Vector3( halfRampW, 0,-halfRampD),
				new Vector3(-halfRampW, 0,-halfRampD),
				
				new Vector3(-halfRampW, rampH, -halfRampD),
				new Vector3( halfRampW, rampH, -halfRampD)
				
		};
		
		int [][] ramp_faces = new int[][]{
				{0, 3, 2, 1},
				{1, 2, 5},
				{3, 0, 4},
				{0, 1, 5, 4},
				{2, 3, 4, 5}
		};
		
		world.setAngularDamping(.085f);
		world.setLinearDamping(.075f);
		
		// add contact generator
		ContactGenerator cg = new ConvexShapeContactGenerator();
		
		world.registerContactGenerator(Shape.SHAPE_BOX, Shape.SHAPE_BOX, cg);
		world.registerContactGenerator(Shape.SHAPE_BOX, Shape.SHAPE_CONVEX, cg);
		world.registerContactGenerator(Shape.SHAPE_CONVEX, Shape.SHAPE_CONVEX, cg);
		world.registerContactGenerator(Shape.SHAPE_CORE, Shape.SHAPE_CORE, new CoreShapesContactGenerator());
		
		// a collections of shape
		Shape box1, cylinder,cone,box2,xbox,ybox,zbox, carChassis, ramp;
		// no-core version
		if (!useCoreShape) {
			box1 = new Box(2, .75f, 2.5f);
			cylinder = new Convex(cylinder_vertex, cylinder_faces);
			cone = new Convex(cone_vertex, cone_faces);
			box2 = new Box(1.5f, 1.f, 1.75f);
			
			
			ybox = new Box(worldSize, 1, worldSize);
			xbox = new Box(1, worldSize, worldSize);
			zbox = new Box(worldSize, worldSize, 1);
			
//			carChassis = new Box(1.5f, .5f, 2.75f);
			carChassis = new Convex(chassis_vertex, chassis_faces);
			
			ramp = new Convex(ramp_vertex, ramp_faces);
		} else {
			// core version
			box1 = new CoreShape(new Box(2, .75f, 2.5f), .1f);
			cylinder = new CoreShape(new Convex(cylinder_vertex, cylinder_faces),.1f);
			cone = new CoreShape(new Convex(cone_vertex, cone_faces), .1f);
			box2 = new CoreShape(new Box(1.5f, 1.f, 1.75f), .1f);
			
			
			ybox = new CoreShape(new Box(worldSize, 1, worldSize), .1f);
			xbox = new CoreShape(new Box(1, worldSize, worldSize), .1f);
			zbox = new CoreShape(new Box(worldSize, worldSize, 1), .1f);
			
			carChassis = new CoreShape(new Convex(chassis_vertex, chassis_faces), .1f);
			
			ramp = new CoreShape(new Convex(ramp_vertex, ramp_faces), .05f);
		}
		
		
		
		// setup chassis
		chassis = new RigidBody(1200.f, carChassis);
		chassis.setRestitution(0.5f);
		chassis.setContinuous(true);
		chassis.setCcdRadius(.01f);
//		chassis.setClampMotion(true);
//		chassis.setMaxAngVel(60.f);
//		chassis.setMaxLinVel(650.f);
		chassis.setPos(new Vector3(2, -1.f, 4));
		chassis.setFriction(.13f);
		chassis.setSleepingThreshold(0.00001f);
		
		world.addBody(chassis);
		
		// setup wheels
		wheels = new WheelSet(chassis).setWorld(world);
		RayWheel w;
		
		// front left
		w = new RayWheel(25.f, .3f, .25f)
			.setFriction(.75f, .1f)
			.setSuspensionLength(.3f)
			.setRayDir(new Vector3(.0f, -1, .0f))
			.setRayStart(new Vector3(.795f, .25f, 1.f))
			.setConstant(.2f, .25f)
			.setName("FL");
		wheels.addWheel(w);
		
		// front right
		w = new RayWheel(25.f, .3f, .25f)
			.setFriction(.75f, .1f)
			.setSuspensionLength(.3f)
			.setRayDir(new Vector3(-.0f, -1, .0f))
			.setRayStart(new Vector3(-.795f, .25f, 1.f))
			.setConstant(.2f, .25f)
			.setName("FR");
		wheels.addWheel(w);
		
		// back left
		w = new RayWheel(30.f, .35f, .25f)
			.setFriction(.75f, .1f)
			.setSuspensionLength(.3f)
			.setRayDir(new Vector3(.0f, -1, 0))
			.setRayStart(new Vector3(.795f, .35f, -1.f))
			.setConstant(.2f, .25f)
			.setName("BL");
		wheels.addWheel(w);
		
		// back right
		w = new RayWheel(30.f, .35f, .25f)
			.setFriction(.75f, .1f)
			.setSuspensionLength(.3f)
			.setRayDir(new Vector3(-.0f, -1, 0))
			.setRayStart(new Vector3(-.795f, .35f, -1.f))
			.setConstant(.2f, .25f)
			.setName("BR");
		wheels.addWheel(w);
		
		// add wheelset + chassis to engine
		world.addSimForce(wheels);
		world.addJoint(wheels);
		
		// build world boundaries		
		
		// bottom box
		RigidBody bA = new RigidBody(-1.0f, ybox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(0, -2, 0));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		// left wall
		bA = new RigidBody(-1.0f, xbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(-worldSize * .5f, worldSize * .5f - 2.5f, 0));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		// right wall
		bA = new RigidBody(-1.0f, xbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3( worldSize * .5f, worldSize * .5f - 2.5f, 0));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		// front wall
		bA = new RigidBody(-1.0f, zbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(0, worldSize * .5f - 2.5f, worldSize * .5f));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		// back wall
		bA = new RigidBody(-1.0f, zbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(0, worldSize * .5f - 2.5f, -worldSize * .5f));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		float halfWS = worldSize * .5f;
		float margin = 2;
		float safeWS = halfWS - margin;
		// add random bodies
		int nBodies = (int)MathHelper.randomRanged(10, worldSize);
		for (int i=0; i<nBodies; i++) {
			float val = MathHelper.randomRanged(0, 40);
			
			Shape s;
			if (val < 10) {
				s = box1;
			} else if (val < 20) {
				s = box2;
			} else if (val < 30) {
				s = cylinder;
			} else if (val < 39) {
				s = cone;
			} else {
				s = carChassis;
			}

			RigidBody b = new RigidBody(MathHelper.randomRanged(10, 1000), s);
			b.setRestitution(MathHelper.randomRanged(0, .5f));
			b.setFriction(MathHelper.randomRanged(.1f, .5f));
			b.setPos(new Vector3(MathHelper.randomRanged(-safeWS, safeWS), MathHelper.randomRanged(2, 8), MathHelper.randomRanged(-safeWS, safeWS)));
			world.addBody(b);
		}
		
		// stack of boxes?
		int nBoxes = 7;
		for (int i=0; i<nBoxes; i++) {
			RigidBody b = new RigidBody(10, box1);
			b.setRestitution(0);
			b.setFriction(0.4f);
			b.setPos(new Vector3(MathHelper.randomRanged(-.1f, .1f), -1.f + i * 0.7f, MathHelper.randomRanged(-.1f, .1f)));
			world.addBody(b);
		}
		
		// random ramps?
		int nRamp = (int) MathHelper.randomRanged(2, 20);
		float rampY = -1.6f;	// should be 1.5f but allow small sink
		for (int i=0; i<nRamp; i++) {
			RigidBody b = new RigidBody(-1.f, ramp);
			b.setFixed(true);
			b.setRestitution(0.2f);
			b.setFriction(.2f);
			b.setPos(new Vector3(MathHelper.randomRanged(-safeWS, safeWS), rampY, MathHelper.randomRanged(-safeWS, safeWS)));
			b.setRot(Quaternion.makeAxisRot(new Vector3(0,1,0),(float) MathHelper.randomRanged((float)-Math.PI, (float)Math.PI)));
			
			world.addBody(b);
		}
	}
	
	public void run() {
		Display disp = NewtFactory.createDisplay(null);
		Screen scr = NewtFactory.createScreen(disp, 0);
		GLProfile glProfile = GLProfile.getDefault();
		GLCapabilities glCap = new GLCapabilities(glProfile);
		glWindow = GLWindow.create(scr, glCap);
		
		glWindow.setSize(640, 480);
		glWindow.setUndecorated(false);
		glWindow.setPointerVisible(true);
		glWindow.confinePointer(false);
		glWindow.setTitle("SShhheeiiiittt");
		glWindow.setContextCreationFlags(GLContext.CTX_OPTION_DEBUG);
		glWindow.setVisible(true);
		glWindow.setDefaultCloseOperation(WindowClosingMode.DISPOSE_ON_CLOSE);
		
		glWindow.addGLEventListener(new GLHandler());
//		glWindow.addKeyListener(new KeyHandler());
		
		glWindow.setKeyboardFocusHandler(new KeyHandler());
		
		glWindow.addMouseListener(new MouseHandler());
		
		glWindow.setKeyboardVisible(true);
		
		System.out.println("screen: " + scr.getViewport().toString());
		glWindow.setPosition((scr.getWidth()-glWindow.getWidth())/2, (scr.getHeight()-glWindow.getHeight())/2);
		
		anim = new Animator(glWindow);
		anim.setRunAsFastAsPossible(true);
		anim.start();
		
//		// now we spawn our physics tread
		phys = new Thread(new Runnable() {
			
			@Override
			public void run() {
				long lastTick = System.nanoTime();
				long curTick;
				while (anim.isAnimating()) {
					curTick = System.nanoTime();
					dt += (curTick - lastTick) / 1000000000.f;
					lastTick = curTick;
					
					while (dt >= simDT) {
						dt -= simDT;
						
						AppMain.this.update(simDT);
					
					}
				}
			}
		});
		phys.start();
	}
	
	public void render(GL2 gl, float dt) {
		// interpolate camera
//		camX += camVX * dt;
//		camY += camVY * dt;
//		camZ += camVZ * dt;
		
		dt=0.f;
		
		// start rendering
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT|GL2.GL_DEPTH_BUFFER_BIT);
		
		Matrix4.perspective(fov, aspect, .1f, 200.0f, matPers);
		
		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glLoadMatrixf(matPers.m, 0);
		
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		GLU glu = GLUgl2.createGLU(gl);
		
		synchronized (world) {
			Vector3 base = chassis.getPos();
			glu.gluLookAt(camX, camY, camZ, camTX, camTY, camTZ, 0, 1, 0);
			
			world.debugDraw(gl, drawContacts, drawContactN, drawContactT, drawBBox, 0);
			wheels.debugDraw(gl, 0);
		}
	}
	
	public void keyDown(short keyCode) {
		
		switch (keyCode) {
		case KeyEvent.VK_ESCAPE:
			glWindow.destroy();
			break;
			
		case KeyEvent.VK_LEFT:
			steerLeft = true;
			break;
		case KeyEvent.VK_RIGHT:
			steerRight = true;
			break;
			
		case KeyEvent.VK_1:
			camMode = 0;
			break;
		case KeyEvent.VK_2:
			camMode = 1;
			break;
			
		case KeyEvent.VK_W:
		case KeyEvent.VK_UP:
			forward = true;
			break;
			
		case KeyEvent.VK_S:
		case KeyEvent.VK_DOWN:
			reverse = true;
			break;
		
		case KeyEvent.VK_K:
			synchronized (world) {
				useCoreShape = !useCoreShape;
				init();
			}
			break;
		case KeyEvent.VK_R:
			synchronized (world) {
				init();
			}
			break;
			
		case KeyEvent.VK_B:
			drawBBox = !drawBBox;
			break;
		case KeyEvent.VK_N:
			drawContactN = !drawContactN;
			break;
		case KeyEvent.VK_C:
			drawContacts = !drawContacts;
			break;
		case KeyEvent.VK_T:
			drawContactT = !drawContactT;
			break;
			
		case KeyEvent.VK_SPACE:
			synchronized (world) {
				// gotta apply random impulse
				Vector3 impulse = new Vector3(
						MathHelper.randomRanged(-1200, 1200),
						MathHelper.randomRanged(1200, 12000),
						MathHelper.randomRanged(-1200, 1200)
						);
				Vector3 point = new Vector3(
						MathHelper.randomRanged(-.5f, .5f),
						MathHelper.randomRanged(-.3f, .3f),
						MathHelper.randomRanged(-1.25f, 1.25f)
						);
				chassis.applyImpulse(impulse, chassis.toWorld(point));
			}
			break;
		}
	}
	
	public void keyUp(short keyCode) {
		switch (keyCode) {
		case KeyEvent.VK_LEFT:
			steerLeft = false;
			break;
		case KeyEvent.VK_RIGHT:
			steerRight = false;
			break;
			
		case KeyEvent.VK_W:
		case KeyEvent.VK_UP:
			forward = false;
			break;
		case KeyEvent.VK_S:
		case KeyEvent.VK_DOWN:
			reverse = false;
			break;
		}
	}
	
	public void mouseDrag(float x, float y, float z) {
//		System.out.println("mouse drag: " + x + ", " + y + " -> " + z);
		
		dist += z * 0.1f;
		xzRot += -x * 1.2f;
		yRot += y;
		
		// clamp
		dist = MathHelper.clamp(dist, minDist, maxDist);
		yRot = MathHelper.clamp(yRot, -maxYRot, maxYRot);
	}
	
	public void update(float dt) {
		
		long timeStep = System.nanoTime();
		
		float steerTarget = .0f;
		float steerDir = .0f;
		float steerVel = .0f;
		
		// revert to zero by default
		float deadCenter = 0.001f;
		steerVel = -currSteer * .2f;
		
		
		if (steerLeft) {
			steerVel += maxSteer * steerStrength;
		}
		if (steerRight) {
			steerVel += -maxSteer * steerStrength;
		}
		
		currSteer += steerVel;
		// clamp it
		currSteer = MathHelper.clamp(currSteer, -maxSteer, maxSteer);
		
		// if it's too small, just zero it out
		if (Math.abs(currSteer) < Vector3.EPSILON)
			currSteer = 0;
		
//		System.out.printf("steer: %.4f%n", currSteer);
		float torque = .0f;
		
		if (forward) {
			torque += 2.f;
		}
		
		if (reverse) {
			torque -= 3.f;
		}
		
		synchronized (world) {
			// update the wheel
			wheels.getWheel(0).setSteerAngle(currSteer);
			wheels.getWheel(1).setSteerAngle(currSteer);
			
			// apply driving force on back wheels
			float frontBrake = 0.2f;
			float rearBrake = 1.f - frontBrake;
			float torquePerSec = 300.5f;
			// if braking
			if (torque < 0.0f) {
				float axleVel = wheels.getWheel(2).wheelAngVel + wheels.getWheel(3).wheelAngVel;
				
				if (axleVel < 0.0001f) {
					wheels.getWheel(2).applyTorque(torque * torquePerSec);
					wheels.getWheel(3).applyTorque(torque * torquePerSec);
				} else {
					// apply front brake too
					wheels.getWheel(0).applyBrake(frontBrake);
					wheels.getWheel(1).applyBrake(frontBrake);
					wheels.getWheel(2).applyBrake(rearBrake);
					wheels.getWheel(3).applyBrake(rearBrake);
				}
				
			} else {
				wheels.getWheel(2).applyTorque(torque * torquePerSec);
				wheels.getWheel(3).applyTorque(torque * torquePerSec);
			}

			world.step(dt);
			
			// check our speed
			float carSpd = chassis.getVel().length() * 3.6f;
			System.out.printf("car speed: %.4f km/h%n", carSpd);
			
			// now calculate camera
			Vector3 tPos = chassis.getPos();
			
			// camera target
			camTX = tPos.x;
			camTY = tPos.y;
			camTZ = tPos.z;
			
			// if camera mode is 1 (align), then we need to align rotation
			Vector3 backVec = new Vector3(); 
			chassis.getRot().transformVector(new Vector3(0, 0, -1), backVec);
			
			// check it
			
			// the camera position			
			float cosYRot = (float) Math.cos(Math.toRadians(yRot));
			
			float camXc = (float) (Math.sin(Math.toRadians(xzRot)) * cosYRot * dist);
			float camZc = (float) (Math.cos(Math.toRadians(xzRot)) * cosYRot * dist);
			float camYc = (float) (Math.sin(Math.toRadians(yRot)) * dist);
			
			// if cam mode is 1, camXc and camZc needs to be transformed
			if (camMode == 1) {
				Vector3 trans = new Vector3(camXc, 0, camZc);
				float len = trans.length();
				chassis.getRot().transformVector(trans, trans);
				trans.normalize();
				trans.scale(len);
				
				camXc = trans.x;
				camZc = trans.z;
			}
			
			// what should it be?
			float cX = camTX + camXc;
			float cY = camTY + camYc;
			float cZ = camTZ + camZc;
			
			// calculate camera velocity
			camVX = (cX - camX) * camStrength / dt;
			camVY = (cY - camY) * camStrength / dt;
			camVZ = (cZ - camZ) * camStrength / dt;
			
			camX = camX + camVX * dt;
			camY = camY + camVY * dt;
			camZ = camZ + camVZ * dt;
		}			
		
//		System.out.printf("bomb's stat: %.4f, %.4f%n", bomb.getVel().length(), bomb.getAngVel().length());
		
		double ovl = (double)(System.nanoTime() - timeStep)/1000000;
		
		profilerTrigger += dt;
		
		// if one second has passed, update information
		if (profilerTrigger >= 5.0f) {
			profilerTrigger = 0;
			String profTxt = "upd(%.2f)ref(%.2f)bphase(%.2f)nphase(%.2f)sort(%.2f)pre(%.2f)solver(%.2f)psolver(%.2f)intg(%.2f)|ovl(%.2f)%n";
			
//			System.out.printf(profTxt, world.getPerformanceTimer(Physics.TIME_UPDATE_VEL), 
//					world.getPerformanceTimer(Physics.TIME_REFRESH_CONTACT),
//					world.getPerformanceTimer(Physics.TIME_BROAD_PHASE),
//					world.getPerformanceTimer(Physics.TIME_NARROW_PHASE),
//					world.getPerformanceTimer(Physics.TIME_SORT_CONTACT),
//					world.getPerformanceTimer(Physics.TIME_PRE_STEP),
//					world.getPerformanceTimer(Physics.TIME_SOLVER),
//					world.getPerformanceTimer(Physics.TIME_POSITION_SOLVER),
//					world.getPerformanceTimer(Physics.TIME_INTEGRATE),
//					ovl);
			
			String count = 	"body(" + world.getBodyCount() + ")" +
							"joint(" + world.getJointCount() + ")" +
							"force(" + world.getForceCount() + ")" +
							"pair(" + world.getPairCount() +")" +
							"manifold(" + world.getManifoldCount()+")";
//			System.out.println(count);
			
			
//			System.out.printf("GJK: %d,  EPA: %d%n", CoreShapesContactGenerator.gjkCount, CoreShapesContactGenerator.epaCount);
		}
		
		
	}
	
	/**
	 * GLHandler - GLEventHandler implementation
	 * @author Bowie
	 *
	 */
	
	private class GLHandler implements GLEventListener {
		float tickDT = 0;
		long curTick, lastTick = System.nanoTime();
		@Override
		public void init(GLAutoDrawable drawable) {
			GL2 gl = drawable.getGL().getGL2();
			
			// set clear color
			gl.glClearColor(0, 0, 0.25f, 1.0f);
			// enable depth testing
			gl.glEnable(GL2.GL_DEPTH_TEST);
			gl.glDepthFunc(GL2.GL_LEQUAL);
			// point size
			gl.glPointSize(5.0f);
		}

		@Override
		public void dispose(GLAutoDrawable drawable) {	
			AppMain.this.anim.stop();
			System.exit(0);
		}

		@Override
		public void display(GLAutoDrawable drawable) {
//			System.out.println("Render fire!!");
			
//			// generate delta time
//			curTick = System.nanoTime();
//			tickDT += (curTick - lastTick) / 1000000000.0f;
//			lastTick = curTick;
//			
//			while (tickDT >= AppMain.this.simDT) {
//				tickDT -= AppMain.this.simDT;
//				AppMain.this.update(AppMain.this.simDT);
//			}
			
			// render the rest of time
			AppMain.this.render(drawable.getGL().getGL2(), AppMain.this.dt);
			
			
//			System.out.println("render: " + Thread.currentThread().getName());
		}

		@Override
		public void reshape(GLAutoDrawable drawable, int x, int y, int width,
				int height) {
			GL2 gl = drawable.getGL().getGL2();			
			gl.glViewport(x, y, width, height);
			
			// reset perspective
//			Matrix4.perspective(fov, (float)width/height, 0.1f, 100.0f, matPers);
			aspect = (float)width/height;
//			Matrix4.ortho(-10, 10, -10, 10, 0.1f, 100.0f, matPers);
//			gl.glMatrixMode(GL2.GL_PROJECTION);
//			gl.glLoadMatrixf(matPers.m, 0);
			
			System.out.println("resized: " + x + ", " + y + ", " + width + ", " + height);
		}
		
	}
	
	
	private class KeyHandler implements KeyListener {
		private long [] downStamp = new long[256];
		private long [] upStamp = new long[256];
		
		@Override
		public void keyPressed(KeyEvent e) {	
			
			AppMain.this.keyDown(e.getKeyCode());
			
//			System.out.printf("press: %d @ %d%n", e.getKeyCode(), System.currentTimeMillis());
		}

		@Override
		public void keyReleased(KeyEvent e) {
			
			AppMain.this.keyUp(e.getKeyCode());
			
			
//			System.out.printf("release: %d @ %d%n", e.getKeyCode(), System.currentTimeMillis());
		}
		
	}
	
	private class MouseHandler implements MouseListener {
		int lastX = 0;
		int lastY = 0;
		
		boolean tracking = false;
		
		@Override
		public void mouseClicked(MouseEvent e) {
			// TODO Auto-generated method stub
			lastX = e.getX();
			lastY = e.getY();
			
			tracking = true;
			
//			System.out.println("click: " + e.getX() + ", " + e.getY());
		}

		@Override
		public void mouseEntered(MouseEvent e) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void mouseExited(MouseEvent e) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void mousePressed(MouseEvent e) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void mouseReleased(MouseEvent e) {
			// TODO Auto-generated method stub
			tracking = false;
			lastX = e.getX();
			lastY = e.getY();
//			System.out.println("release: " + e.getX() + ", " + e.getY());
		}

		@Override
		public void mouseMoved(MouseEvent e) {
			// TODO Auto-generated method stub
//			System.out.println("move: " + e.getX() + ", " + e.getY());
		}

		@Override
		public void mouseDragged(MouseEvent e) {
			// TODO Auto-generated method stub
			
			if (!tracking) {
				tracking = true;
				
				lastX = e.getX();
				lastY = e.getY();
			} else {
				float mX = e.getX() - lastX;
				float mY = e.getY() - lastY;
				
				lastX = e.getX();
				lastY = e.getY();
				
				AppMain.this.mouseDrag(mX, mY, 0);
			}					
			
		}

		@Override
		public void mouseWheelMoved(MouseEvent e) {
			float [] rot = e.getRotation();
			
			AppMain.this.mouseDrag(0, 0, rot[1]);
		}
		
	}
	
}
