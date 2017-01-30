package com.bowie.javagl;


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
	/*public class AppRes {
		public Simplex simp;
		public Shape sA;
		public Vector3 posA;
		public Quaternion rotA;
		
		public Shape sB;
		public Vector3 posB;
		public Quaternion rotB;
		
		public Vector3 vA, vB;
		
		public Vector3 rayhitP, rayhitN;
		
		public Vector3 rS, rE;
		public float radius;
		
		public GLUquadric sphere;
		
		public AppRes() {
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
			
			rotA = Quaternion.makeAxisRot(new Vector3(4, 3, 1), .2f);
			posA = new Vector3(0, 0, 0);
			sA = new Convex(cylinder_vertex, cylinder_faces);
			
			vA = new Vector3(3.5f, 2.f, 1.f);
//			vA = Vector3.ZERO;
			
			rotB = Quaternion.makeAxisRot(new Vector3(2, 1, 3), .1f);
			posB = new Vector3(2, 3, 0);
			sB = new Convex(cylinder_vertex, cylinder_faces);
			
			vB = new Vector3(-4.1f, -3.2f, -.005f);
			
			rS = new Vector3(1, 2, .5f);
			rE = new Vector3(-2, -1, -.7f);
			
			simp = new Simplex(sA, posA, rotA, sB, posB, rotB);
//			simp.raycastInit(rS, rE, 0);
			simp.convexCastInit(vA, vB);
			
			rayhitN = null;
			rayhitP = null;
			
			radius = 0.2f;
		}
	}*/
	
	public GLWindow glWindow = null;
	public Animator anim = null;	// GUI thread
	public Thread phys = null;	// separate physics thread? (UNUSED)
	
	// shared between thread!!
	volatile private float dt = 0;		// delta time
	volatile private float aspect = 1;
	
	
	private float fov = 80.0f;
	private Matrix4 matPers = new Matrix4();
	
	private Physics world = new Physics(7, 5, .02f, .3f);
	private RigidBody bomb;
	private WorldSpring ws;
	private WorldPinJoint wp;
	
//	private AppRes res = new AppRes();
	
	private boolean drawContacts = true;
	private boolean drawContactN = false;
	private boolean drawContactT = false;
	private boolean drawBBox = false;
	
	private boolean useCoreShape = false;
	
	// this is our lock
//	final private ReentrantLock resLock = new ReentrantLock();
	
	private float yRot = 40;
	private float xzRot = 0.0f;
	private float dist = 12.0f;
	
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
		
		world.setAngularDamping(.085f);
		world.setLinearDamping(.075f);
		
		// add contact generator
		ContactGenerator cg = new ConvexShapeContactGenerator();
		
		world.registerContactGenerator(Shape.SHAPE_BOX, Shape.SHAPE_BOX, cg);
		world.registerContactGenerator(Shape.SHAPE_BOX, Shape.SHAPE_CONVEX, cg);
		world.registerContactGenerator(Shape.SHAPE_CONVEX, Shape.SHAPE_CONVEX, cg);
		world.registerContactGenerator(Shape.SHAPE_CORE, Shape.SHAPE_CORE, new CoreShapesContactGenerator());
		
		// a collections of shape
		Shape box1, cylinder,cone,box2,xbox,ybox,zbox;
		// no-core version
		if (!useCoreShape) {
			box1 = new Box(1, .75f, 1.25f);
			cylinder = new Convex(cylinder_vertex, cylinder_faces);
			cone = new Convex(cone_vertex, cone_faces);
			box2 = new Box(1.5f, 1.f, 1.75f);
			
			
			ybox = new Box(16, 1, 16);
			xbox = new Box(1, 16, 16);
			zbox = new Box(16, 16, 1);
			
			bomb = new RigidBody(10.f, new Box(1,1.2f,1.5f));
		} else {
			// core version
			box1 = new CoreShape(new Box(1, .75f, 1.25f), .1f);
			cylinder = new CoreShape(new Convex(cylinder_vertex, cylinder_faces),.1f);
			cone = new CoreShape(new Convex(cone_vertex, cone_faces), .1f);
			box2 = new CoreShape(new Box(1.5f, 1.f, 1.75f), .1f);
			
			
			ybox = new CoreShape(new Box(16, 1, 16), .1f);
			xbox = new CoreShape(new Box(1, 16, 16), .1f);
			zbox = new CoreShape(new Box(16, 16, 1), .1f);
			
			bomb = new RigidBody(10.f, new CoreShape(new Box(1,1.2f,1.5f), .1f));
		}
		
		bomb.setContinuous(true);
		bomb.setCcdRadius(.1f);
		bomb.setClampMotion(true);
		bomb.setMaxLinVel(130.f);
		bomb.setMaxAngVel(10.f);
		bomb.setFriction(.5f);
		
		// build world boundaries		
		RigidBody bA = new RigidBody(-1.0f, ybox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(0, -2, 0));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		bA = new RigidBody(-1.0f, xbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(-8.f, 6.5f, 0));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		bA = new RigidBody(-1.0f, xbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3( 8.f, 6.5f, 0));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		bA = new RigidBody(-1.0f, zbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(0, 6.5f, 8.f));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		bA = new RigidBody(-1.0f, zbox);
		bA.setFriction(.4f);
		bA.setPos(new Vector3(0, 6.5f, -8.f));
		bA.setFixed(true);
		bA.setRestitution(.5f);
		
		world.addBody(bA);
		
		// add bodies
		
		RigidBody bB = new RigidBody(10.0f, box1);
		bB.setFriction(.2f);
		bB.setPos(new Vector3(0, 5, -5));		
		world.addBody(bB);
		
//		world.addJoint(new WorldPinJoint(new Vector3(-4, 5, 0), bB, new Vector3(-.5f, -.8f, .5f)));
		
		bA = new RigidBody(10.0f, cylinder);
		bA.setFriction(.2f);
		bA.setPos(new Vector3(0.5f, 7, -5));		
		world.addBody(bA);
		
		world.addJoint(new BallJoint(bA, new Vector3(.5f, .95f, .5f), bB, new Vector3(.5f, .8f, .5f)));
		
		bB = new RigidBody(10.0f, cone);
		bB.setFriction(.2f);
		bB.setPos(new Vector3(-.5f, 9, -5));		
		world.addBody(bB);
		
		world.addJoint(new DistanceJoint(bA, bB, new Vector3(.5f, -.5f, 0), new Vector3(.05f, .5f, 0), 1.f, .9f));
		
		/*for (int i=0; i<75; i++) {
			// add random bodies
			float rn = MathHelper.randomRanged(1, 15);
			
			Shape s = rn > 10 ? cone : rn > 5 ? cylinder : box1;
			
			bB = new RigidBody(10.0f, s);
			bB.setFriction(MathHelper.randomRanged(.1f, .8f));
			bB.setRestitution(MathHelper.randomRanged(.2f, .8f));
			bB.setPos(new Vector3(MathHelper.randomRanged(-4, 4), MathHelper.randomRanged(8, 15), MathHelper.randomRanged(-4, 4)));
			
			world.addBody(bB);
		}*/
		
		float yPos = -.7f;
		for (int i=0; i<7; i++) {
			// drop stack
			bB = new RigidBody(50.0f, box2);
			bB.setRestitution(0);
			bB.setFriction(.2f);
			bB.setPos(new Vector3(MathHelper.randomRanged(-.01f, .01f), yPos, MathHelper.randomRanged(-.01f, .01f)));
			
			world.addBody(bB);
			
			yPos += 1.0f;
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
		glWindow.addKeyListener(new KeyHandler());
		glWindow.addMouseListener(new MouseHandler());
		
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
		float cosYRot = (float) Math.cos(Math.toRadians(yRot));
		
		float camX = (float) (Math.sin(Math.toRadians(xzRot)) * cosYRot * dist);
		float camZ = (float) (Math.cos(Math.toRadians(xzRot)) * cosYRot * dist);
		float camY = (float) (Math.sin(Math.toRadians(yRot)) * dist);
		
		// start rendering
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT|GL2.GL_DEPTH_BUFFER_BIT);
		
		Matrix4.perspective(fov, aspect, .1f, 200.0f, matPers);
		
		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glLoadMatrixf(matPers.m, 0);
		
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		GLU glu = GLUgl2.createGLU(gl);
		glu.gluLookAt(camX, camY, camZ, 0, 0, 0, 0, 1, 0);
		
		synchronized (world) {
			world.debugDraw(gl, drawContacts, drawContactN, drawContactT, drawBBox, 0.f);
		}
	}
	
	/*public void render(GL2 gl, float dt) {
		float cosYRot = (float) Math.cos(Math.toRadians(yRot));
		
		float camX = (float) (Math.sin(Math.toRadians(xzRot)) * cosYRot * dist);
		float camZ = (float) (Math.cos(Math.toRadians(xzRot)) * cosYRot * dist);
		float camY = (float) (Math.sin(Math.toRadians(yRot)) * dist);
		
		// start rendering
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT|GL2.GL_DEPTH_BUFFER_BIT);
		
//		Matrix4.ortho(-dist * 2 * aspect, dist * 2 * aspect, -dist * 2, dist * 2, -1, 100.0f, matPers);
		Matrix4.perspective(fov, aspect, .1f, 200.0f, matPers);
		
		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glLoadMatrixf(matPers.m, 0);
		
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		GLU glu = GLUgl2.createGLU(gl);
		glu.gluLookAt(camX, camY, camZ, 0, 0, 0, 0, 1, 0);
		
//		synchronized(world) {
//			world.debugDraw(gl, drawContacts, drawContactN, drawContactT, drawBBox, dt);
//		}
		
		synchronized (res) {
			gl.glColor3f(.2f, .7f, .9f);
			res.sA.render(gl, res.posA, res.rotA);
			
//			gl.glColor3f(.5f, .7f, .01f);
//			res.sB.render(gl, res.posB, res.rotB);
//			
			// draw the moved shape
//			Vector3 newPos = new Vector3(
//					res.posA.x + res.vA.x * res.simp.rayT,
//					res.posA.y + res.vA.y * res.simp.rayT,
//					res.posA.z + res.vA.z * res.simp.rayT
//					);
//			gl.glColor3f(.5f, .2f, .1f);
//			res.sA.render(gl, newPos, res.rotA);
//			
//			newPos.setTo(
//					res.posB.x + res.vB.x * res.simp.rayT,
//					res.posB.y + res.vB.y * res.simp.rayT,
//					res.posB.z + res.vB.z * res.simp.rayT
//					);
//			gl.glColor3f(.1f, .2f, .5f);
//			res.sB.render(gl, newPos, res.rotB);
			
			// draw line segment to depict velocity
//			gl.glBegin(GL2.GL_LINES);
//				gl.glColor3f(1, 0, 0);
//				gl.glVertex3f(res.posA.x, res.posA.y, res.posA.z);
//				
//				gl.glColor3f(0, 0, 1);
//				gl.glVertex3f(res.posA.x + res.vA.x, 
//						res.posA.y + res.vA.y,
//						res.posA.z + res.vA.z);
//				
//				gl.glColor3f(1, 0, 0);
//				gl.glVertex3f(res.posB.x, res.posB.y, res.posB.z);
//				
//				gl.glColor3f(0, 0, 1);
//				gl.glVertex3f(res.posB.x + res.vB.x, 
//						res.posB.y + res.vB.y,
//						res.posB.z + res.vB.z);
//			gl.glEnd();
			
			// draw rayhit position + normal
			if (res.rayhitP != null && res.rayhitN != null) {
				gl.glBegin(GL2.GL_POINTS);
				gl.glColor3f(0.2f, 1, 0.5f);
				gl.glVertex3f(res.rayhitP.x, res.rayhitP.y, res.rayhitP.z);
				gl.glEnd();
				
				gl.glBegin(GL2.GL_LINES);
				gl.glColor3f(0.2f, 1, 0.5f);
				gl.glVertex3f(res.rayhitP.x, res.rayhitP.y, res.rayhitP.z);
				
				gl.glColor3f(0.8f, 1, 0.15f);
				gl.glVertex3f(res.rayhitP.x+res.rayhitN.x, res.rayhitP.y+res.rayhitN.y, res.rayhitP.z+res.rayhitN.z);
				gl.glEnd();
			}
			// draw line segment
			gl.glBegin(GL2.GL_LINES);
				gl.glColor3f(1, 0, 0);
				gl.glVertex3f(res.rS.x, res.rS.y, res.rS.z);
				
				gl.glColor3f(0, 0, 1);
				gl.glVertex3f(res.rE.x, res.rE.y, res.rE.z);
			gl.glEnd();
			
			gl.glBegin(GL2.GL_POINTS);
				gl.glColor3f(1, 0.2f, 0.1f);
				gl.glVertex3f(res.rS.x, res.rS.y, res.rS.z);
				
				gl.glColor3f(0.1f, 0.2f, 1);
				gl.glVertex3f(res.rE.x, res.rE.y, res.rE.z);
			gl.glEnd();
			
			gl.glColor3f(.2f, .2f, .9f);
			res.simp.debugDraw(gl);		
			
			// draw sphere cast
			// compute new pos here
			Vector3 sPos = new Vector3(
					res.rS.x * (1.0f - res.simp.rayT) + res.rE.x * res.simp.rayT,
					res.rS.y * (1.0f - res.simp.rayT) + res.rE.y * res.simp.rayT,
					res.rS.z * (1.0f - res.simp.rayT) + res.rE.z * res.simp.rayT
					);
			
			if (res.sphere == null) {
				res.sphere = glu.gluNewQuadric();
			}
			
			if (true) {
				gl.glPushAttrib(GL2.GL_POLYGON_BIT);
				gl.glEnable(GL2.GL_CULL_FACE);
				gl.glPolygonMode(GL2.GL_FRONT, GL2.GL_LINE);
				gl.glPushMatrix();
				gl.glTranslatef(sPos.x, sPos.y, sPos.z);
					gl.glColor3f(.2f, .84f, .1f);
					glu.gluSphere(res.sphere, res.radius, 8, 8);
					
					gl.glBegin(GL2.GL_POINTS);
					gl.glColor3f(.81f, .72f, .94f);
					gl.glVertex3f(0, 0, 0);
					gl.glEnd();
				gl.glPopMatrix();
				gl.glDisable(GL2.GL_CULL_FACE);
				gl.glPopAttrib();
			}
			
		}
		
		
//		synchronized (ws) {
//			ws.render(gl);
//		}
		
		
		
		// draw origin and sheit
		gl.glBegin(GL2.GL_POINTS);
		gl.glColor3f(1, 1, 1);
		gl.glVertex3f(0, 0, 0);
		gl.glEnd();
		
		gl.glBegin(GL2.GL_LINES);
		gl.glColor3f(1, 0, 0);
		gl.glVertex3f(0, 0, 0);
		gl.glVertex3f(1, 0, 0);
		
		gl.glColor3f(0, 1, 0);
		gl.glVertex3f(0, 0, 0);
		gl.glVertex3f(0, 1, 0);
		
		gl.glColor3f(0, 0, 1);
		gl.glVertex3f(0, 0, 0);
		gl.glVertex3f(0, 0, 1);
		gl.glEnd();
		
	}*/
	
	public void keyDown(short keyCode) {
		switch (keyCode) {
		case KeyEvent.VK_ESCAPE:
			glWindow.destroy();
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
				// we add bomb (remove first)
				world.removeBody(bomb);
				// set its position
				bomb.setPos(new Vector3(MathHelper.randomRanged(-5, 5), MathHelper.randomRanged(1, 8), MathHelper.randomRanged(-5, 5)));
				// set its rotation
				bomb.setRot(Quaternion.makeAxisRot(new Vector3(MathHelper.randomRanged(-1, 1), MathHelper.randomRanged(-1, 1), MathHelper.randomRanged(-1, 1)), 
						MathHelper.randomRanged(-4, 4)));
				// set velocity
				bomb.setVel(new Vector3(MathHelper.randomRanged(-250, 250), MathHelper.randomRanged(-250, 15), MathHelper.randomRanged(-250, 250)));
				// add it back
				world.addBody(bomb);
			}
			break;
		}
	}
	/*public void keyDown(short keyCode) {	
		
		switch (keyCode) {
		case KeyEvent.VK_ESCAPE:
			glWindow.destroy();
			break;
			
		case KeyEvent.VK_K:
			synchronized (world) {
				useCoreShape = !useCoreShape;
				init();
			}
			break;
			
		case KeyEvent.VK_B:
			drawBBox = !drawBBox;
			break;
		case KeyEvent.VK_C:
			drawContacts = !drawContacts;
			break;
		case KeyEvent.VK_N:
			drawContactN = !drawContactN;
			break;
		case KeyEvent.VK_T:
			drawContactT = !drawContactT;
			break;
			
		
		case KeyEvent.VK_3:
		case KeyEvent.VK_4:
		case KeyEvent.VK_5:
		case KeyEvent.VK_6:
			int target = (keyCode - KeyEvent.VK_0) * 10;
			System.out.println("changing tick rate to " + target + " hz");
			this.simFPS = target;
			this.simDT = 1.0f/(float)simFPS;
			break;
			
		case KeyEvent.VK_R:
			synchronized (world) {
				
//				init();
			}
			synchronized (res) {
//				res.simp.raycastInit(res.rS, res.rE, res.radius);
				res.simp.convexCastInit(res.vA, res.vB);
				
				res.rayhitN = null;
				res.rayhitP = null;
			}
			
			break;
			
		case KeyEvent.VK_A:
			synchronized (res) {
				Quaternion rot = Quaternion.makeAxisRot(new Vector3(0,1,0), .1f);
				Quaternion.mul(rot, res.rotA, res.rotA);
			}			
			break;
		case KeyEvent.VK_D:
			synchronized (res) {
				Quaternion rot = Quaternion.makeAxisRot(new Vector3(0,1,0), -.1f);
				Quaternion.mul(rot, res.rotA, res.rotA);
			}			
			break;
		case KeyEvent.VK_W:
			synchronized (res) {
				Quaternion rot = Quaternion.makeAxisRot(new Vector3(1,0,0), .1f);
				Quaternion.mul(rot, res.rotA, res.rotA);
			}			
			break;
		case KeyEvent.VK_S:
			synchronized (res) {
				Quaternion rot = Quaternion.makeAxisRot(new Vector3(1,0,0), -.1f);
				Quaternion.mul(rot, res.rotA, res.rotA);
			}			
			break;
			
		case KeyEvent.VK_G:
			synchronized (res) {
				res.rE.x-= 0.1f;
			}
			break;
		case KeyEvent.VK_H:
			synchronized (res) {
				res.rE.y-= 0.1f;
			}
			break;
		case KeyEvent.VK_J:
			synchronized (res) {
				res.rE.x+= 0.1f;
			}
			break;
		case KeyEvent.VK_Y:
			synchronized (res) {
				res.rE.y+= 0.1f;
			}
			break;
		case KeyEvent.VK_V:
			synchronized (res) {
				res.rE.z+= 0.1f;
			}
			break;
		case KeyEvent.VK_F:
			synchronized (res) {
				res.rE.z-= 0.1f;
			}
			break;
		case KeyEvent.VK_LEFT:
			synchronized (res) {
				res.posB.x -= .1f;
			}
			break;
		case KeyEvent.VK_RIGHT:
			synchronized (res) {
				res.posB.x += .1f;
			}
			break;
		case KeyEvent.VK_DOWN:
			synchronized (res) {
				res.posB.z += .1f;
			}
			break;
		case KeyEvent.VK_UP:
			synchronized (res) {
				res.posB.z -= .1f;
			}
			break;
		case KeyEvent.VK_SEMICOLON:
			synchronized (res) {
				res.posB.y -= .1f;
			}
			break;
		case KeyEvent.VK_QUOTE:
			synchronized (res) {
				res.posB.y += .1f;
			}
			break;
			
		case KeyEvent.VK_SPACE:
			synchronized (res) {
//				res.simp.raycastStep();
//				res.simp.convexCastStep();
//				int retVal= res.simp.doConvexCast(res.vA, res.vB);
				int retVal = res.simp.doRayCast(res.rS, res.rE, res.radius);
				if (retVal == Simplex.RAY_HIT) {
					res.rayhitN = res.simp.rayhitNormal;
					res.rayhitP = res.simp.rayhitPos;
				}
			}
//			synchronized (ws) {
//				ws.setActive(!ws.isActive());
//			}
//			synchronized (world) {
//				// .1T bomb
//				RigidBody b = new RigidBody(100, bomb);
//				b.setRestitution(.42f);
//				b.setFriction(.14f);
//				
//				b.setPos(new Vector3(MathHelper.randomRanged(-6,  6), MathHelper.randomRanged(5,  15), MathHelper.randomRanged(-6, 6)));
//				b.setVel(new Vector3(MathHelper.randomRanged(-42, 42), MathHelper.randomRanged(-41, -45), MathHelper.randomRanged(-42, 42)));
//				b.setContinuous(true);
//				world.addBody(b);
//			}
			
			synchronized (res) {
				res.stat = res.simp.GJK(false) ? Simplex.SIMPLEX_CONTAIN_ORIGIN : Simplex.SIMPLEX_CANNOT_EVOLVE;
				
				if (res.stat == Simplex.SIMPLEX_CONTAIN_ORIGIN) {
					res.poly = new Polytope(res.simp);
					
					EPAInfo inf = MathHelper.epaExecute(res.sA, res.posA, res.rotA, res.sB, res.posB, res.rotB, res.poly);
					
					if (inf != null) {
						res.cA = inf.calcContactA();
						res.cB = inf.calcContactB();
//						res.cA = new Vector3();
//						res.cB = new Vector3();
						
//						res.poly.getClosestPoint(res.cA, res.cB);
						
						System.out.printf("cA: %.2f,  %.2f, %.2f%n", res.cA.x, res.cA.y, res.cA.z);
						System.out.printf("cB: %.2f,  %.2f, %.2f%n", res.cB.x, res.cB.y, res.cB.z);
					}
				} else {
					res.poly = null;
					
					// cA and cB are closest points
					res.cA = new Vector3();
					res.cB = new Vector3();
					res.simp.getClosestPoint(res.cA, res.cB);
				}
			}
			
			break;
		}		
		
		// refresh
	}*/
	
	public void mouseDrag(float x, float y, float z) {
//		System.out.println("mouse drag: " + x + ", " + y + " -> " + z);
		
		dist += z * 0.1f;
		xzRot += -x * 1.2f;
		yRot += y;
		
		// clamp
		dist = MathHelper.clamp(dist, minDist, maxDist);
		yRot = MathHelper.clamp(yRot, -maxYRot, maxYRot);
	}
	
	public void keyUp(short keyCode) {
	}
	
	public void update(float dt) {
		
		long timeStep = System.nanoTime();
		
		synchronized (world) {
			world.step(dt);
		}			
		
//		System.out.printf("bomb's stat: %.4f, %.4f%n", bomb.getVel().length(), bomb.getAngVel().length());
		
		double ovl = (double)(System.nanoTime() - timeStep)/1000000;
		
		profilerTrigger += dt;
		
		// if one second has passed, update information
		if (profilerTrigger >= 5.0f) {
			profilerTrigger = 0;
			String profTxt = "upd(%.2f)ref(%.2f)bphase(%.2f)nphase(%.2f)sort(%.2f)pre(%.2f)solver(%.2f)psolver(%.2f)intg(%.2f)|ovl(%.2f)%n";
			
			System.out.printf(profTxt, world.getPerformanceTimer(Physics.TIME_UPDATE_VEL), 
					world.getPerformanceTimer(Physics.TIME_REFRESH_CONTACT),
					world.getPerformanceTimer(Physics.TIME_BROAD_PHASE),
					world.getPerformanceTimer(Physics.TIME_NARROW_PHASE),
					world.getPerformanceTimer(Physics.TIME_SORT_CONTACT),
					world.getPerformanceTimer(Physics.TIME_PRE_STEP),
					world.getPerformanceTimer(Physics.TIME_SOLVER),
					world.getPerformanceTimer(Physics.TIME_POSITION_SOLVER),
					world.getPerformanceTimer(Physics.TIME_INTEGRATE),
					ovl);
			
			String count = 	"body(" + world.getBodyCount() + ")" +
							"joint(" + world.getJointCount() + ")" +
							"force(" + world.getForceCount() + ")" +
							"pair(" + world.getPairCount() +")" +
							"manifold(" + world.getManifoldCount()+")";
			System.out.println(count);
			
			
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

		@Override
		public void keyPressed(KeyEvent e) {	
//			System.out.println("Preesssed: " + e);
			AppMain.this.keyDown(e.getKeyCode());
		}

		@Override
		public void keyReleased(KeyEvent e) {
//			System.out.println("UnPreesssed: " + e);
			AppMain.this.keyUp(e.getKeyCode());
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
