package com.bowie.javagl;

import java.util.concurrent.locks.ReentrantLock;

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
import com.jogamp.opengl.glu.gl2.GLUgl2;
import com.jogamp.opengl.util.Animator;

public class AppMain {	
	public GLWindow glWindow = null;
	public Animator anim = null;	// GUI thread
	public Thread phys = null;	// separate physics thread? (UNUSED)
	
	// shared between thread!!
	volatile private float dt = 0;		// delta time
	
	private float fov = 80.0f;
	private Matrix4 matPers = new Matrix4();
	
	private Physics world;
	private WorldSpring ws;
	private WorldPinJoint wp;
	
	private boolean drawContacts = true;
	private boolean drawContactN = false;
	private boolean drawContactT = false;
	private boolean drawBBox = false;
	
	// this is our lock
	final private ReentrantLock resLock = new ReentrantLock();
	
	private float yRot = 40;
	private float xzRot = 0.0f;
	private float dist = 10.0f;
	
	final private float maxYRot = 80.0f;
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
				{0, 7, 8},
				{8,7,6,5,4,3,2,1}
		};
		
		Box boxShape = new Box(1,1,1);
		Convex cylinderShape = new Convex(cylinder_vertex, cylinder_faces);
		Convex coneShape = new Convex(cone_vertex, cone_faces);
		
		// spawn the world
		world = new Physics(7, 5, 0.03f, 0.6f);	//  7 solver iteration, 5 position iteration, 2.5cm slop, 0.6f baumgarte stabilization
		world.setLinearDamping(0.005f);		// 0.5% linear dissipation
		world.setAngularDamping(0.0025f);	// 0.25% angular dissipation
		world.setGravity(new Vector3(0, -9.8f, 0));
		
		
//		collide = MathHelper.gjkColDet(boxA, posA, rotA, boxB, posB, rotB, new Vector3(0,1,0), simp);
		// create the world
		RigidBody bodyA, bodyB;
		// bottom plate
		bodyB = new RigidBody(-1.0f, new Box(16, 0.5f, 16));
		bodyB.setFixed(true);
		bodyB.setRestitution(0.984f);
		bodyB.setFriction(0.25f);
		world.addBody(bodyB);
		
		// right plate
		bodyB = new RigidBody(-1.0f, new Box(.5f, 16, 16));
		bodyB.setFixed(true);
		bodyB.setRestitution(0.84f);
		bodyB.setFriction(0.25f);
		bodyB.setPos(new Vector3(8, 8, 0));
		world.addBody(bodyB);
		
		// left plate
		bodyB = new RigidBody(-1.0f, new Box(.5f, 16, 16));
		bodyB.setFixed(true);
		bodyB.setRestitution(0.84f);
		bodyB.setFriction(0.25f);
		bodyB.setPos(new Vector3(-8, 8, 0));
		world.addBody(bodyB);
		
		// front plate
		bodyB = new RigidBody(-1.0f, new Box(16, 16, .5f));
		bodyB.setFixed(true);
		bodyB.setRestitution(0.84f);
		bodyB.setFriction(0.25f);
		bodyB.setPos(new Vector3(0, 8, 8));
		world.addBody(bodyB);
		
		// rear plate
		bodyB = new RigidBody(-1.0f, new Box(16, 16, .5f));
		bodyB.setFixed(true);
		bodyB.setRestitution(0.84f);
		bodyB.setFriction(0.25f);
		bodyB.setPos(new Vector3(0, 8, -8));
		world.addBody(bodyB);
		
		bodyA = new RigidBody(10.0f, cylinderShape);
		bodyA.setPos(new Vector3(-4, 4, 0));
		bodyA.setFriction(0.8f);
		world.addBody(bodyA);
		
		// add world pinner joint
		wp = new WorldPinJoint(new Vector3(-2, 8, -3), bodyA, new Vector3(1, 1, 0.5f));
		world.addJoint(wp);
		
		// add cone
		bodyB = new RigidBody(15.0f, coneShape);
		bodyB.setPos(new Vector3(-4, 1, 0));
		world.addBody(bodyB);
		
		// add ball joint
		world.addJoint(new BallJoint(bodyA, new Vector3(0.5f, -1.0f, 0), bodyB, new Vector3(0.25f, 0.75f, 0)));
		
		bodyA = new RigidBody(20.0f, cylinderShape);
		bodyA.setPos(new Vector3(-4, 1, 0));
		world.addBody(bodyA);
		
		// add ball joint
		world.addJoint(new BallJoint(bodyA, new Vector3(0.5f, .75f, 0), bodyB, new Vector3(-.5f, -.75f, 0)));
		
		// add another joint
		bodyB = new RigidBody(20.0f, cylinderShape);
		bodyB.setPos(new Vector3(-5, 1, 0));
		world.addBody(bodyB);
		
		world.addJoint(new DistanceJoint(bodyA, bodyB, new Vector3(-.5f, .7f, 0), new Vector3(.5f, .5f, .5f), 1, 1.f));
		
		RigidBody smallBox = new RigidBody(20, coneShape);
		smallBox.setFriction(0.2f);
		smallBox.setPos(new Vector3(-5, 0.5f, 0));
		world.addBody(smallBox);
		
		world.addJoint(new DistanceJoint(smallBox, bodyB, new Vector3(0,.5f,0), new Vector3(-.5f,-.5f,-.5f), 1, 1));
		// add world spring
		ws = new WorldSpring(new Vector3(-2, 7, 3), new Vector3(0.4f, -0.5f, 0.0f), smallBox, 1, 166, 0.7f);
		world.addSimForce(ws);
		
		// add more bodies?
		Box box = new Box(1.5f, 1.0f, 1.5f);
		float y = 0.5f;
		for (int i=0; i<7; i++) {
			RigidBody rb = new RigidBody(150, box);
			rb.setPos(new Vector3(MathHelper.randomRanged(-0.01f, 0.01f), y, MathHelper.randomRanged(-0.01f, 0.01f)));
			y+= 1.05f;
			rb.setFriction(0.2f);
			world.addBody(rb);
			
//			if (i == 2)
//				bodyA = rb;
			
//			if (i==6)
//				bodyB = rb;
		}
		
		for (int i=0; i<55; i++) {
			int rNumber = (int)MathHelper.randomRanged(0, 15);
			
			// add random cone
			Shape s = rNumber <= 5 ? boxShape : rNumber <=10 ? coneShape : cylinderShape;
			RigidBody rb = new RigidBody(MathHelper.randomRanged(5, 20), s);
			rb.setPos(new Vector3(MathHelper.randomRanged(-3, 3), 5 + MathHelper.randomRanged(-3, 8), MathHelper.randomRanged(-3, 3)));
			rb.setFriction(0.41f);
			rb.setRestitution(MathHelper.randomRanged(0.2f, 1.0f));
			world.addBody(rb);
			
			if (i == 30)
				bodyA = rb;
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
		
		// now we spawn our physics tread
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
						
						synchronized (world) {
							AppMain.this.update(simDT);
						}						
						
					}
				}
			}
		});
		phys.start();
	}
	
	public void render(GL2 gl, float dt) {
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT|GL2.GL_DEPTH_BUFFER_BIT);
		
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		float cosYRot = (float) Math.cos(Math.toRadians(yRot));
		
		float camX = (float) (Math.sin(Math.toRadians(xzRot)) * cosYRot * dist);
		float camZ = (float) (Math.cos(Math.toRadians(xzRot)) * cosYRot * dist);
		float camY = (float) (Math.sin(Math.toRadians(yRot)) * dist);
		
		GLU glu = GLUgl2.createGLU(gl);
		glu.gluLookAt(camX, camY, camZ, 0, 0, 0, 0, 1, 0);
		
		world.debugDraw(gl, drawContacts, drawContactN, drawContactT, drawBBox);
		ws.render(gl);
	}
	
	public void keyDown(short keyCode) {
		
		Vector3 wpPos = wp.getWorldPinPos();
		
		
		switch (keyCode) {
		case KeyEvent.VK_ESCAPE:
			glWindow.destroy();
			break;
			
		/*case KeyEvent.VK_W:
			posA.z -= 0.1f;
			break;
		case KeyEvent.VK_S:
			posA.z += 0.1f;
			break;
		case KeyEvent.VK_A:
			posA.x -= 0.1f;
			break;
		case KeyEvent.VK_D:
			posA.x += 0.1f;
			break;
		case KeyEvent.VK_Q:
			posA.y += 0.1f;
			break;
		case KeyEvent.VK_E:
			posA.y -= 0.1f;
			break;*/
			
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
			
		case KeyEvent.VK_UP:
			wpPos.z -= 0.1f;
			break;
		case KeyEvent.VK_DOWN:
			wpPos.z += 0.1f;
			break;
		case KeyEvent.VK_LEFT:
			wpPos.x -= 0.1f;
			break;
		case KeyEvent.VK_RIGHT:
			wpPos.x += 0.1f;
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
			resLock.lock();
			try {
				init();
			} finally {
				resLock.unlock();
			}
			
			break;
			
		case KeyEvent.VK_SPACE:
			synchronized (ws) {
				ws.setActive(!ws.isActive());
			}
			
			break;
		}		
		
		// refresh
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
	
	public void keyUp(short keyCode) {
	}
	
	public void update(float dt) {
		long timeStep = System.nanoTime();
		world.step(dt);	
		
		timeStep = (System.nanoTime() - timeStep)/1000000;
		
		profilerTrigger += dt;
		
		// if one second has passed, update information
		if (profilerTrigger >= 2.0f) {
			profilerTrigger = 0;
			String profTxt = 	"upd("+world.getPerformanceTimer(Physics.TIME_UPDATE_VEL)+")"+
								"ref("+world.getPerformanceTimer(Physics.TIME_REFRESH_CONTACT)+")"+
								"bphase("+world.getPerformanceTimer(Physics.TIME_BROAD_PHASE)+")"+
								"nphase("+world.getPerformanceTimer(Physics.TIME_NARROW_PHASE)+")"+
								"sort("+world.getPerformanceTimer(Physics.TIME_SORT_CONTACT)+")"+
								"pre("+world.getPerformanceTimer(Physics.TIME_PRE_STEP)+")"+
								"solver("+world.getPerformanceTimer(Physics.TIME_SOLVER)+")"+
								"psolver("+world.getPerformanceTimer(Physics.TIME_POSITION_SOLVER)+")"+
								"intg("+world.getPerformanceTimer(Physics.TIME_INTEGRATE)+")"+
								"ovl="+timeStep;
			System.out.println(profTxt);
			
			String count = 	"body(" + world.getBodyCount() + ")" +
							"joint(" + world.getJointCount() + ")" +
							"force(" + world.getForceCount() + ")" +
							"pair(" + world.getPairCount() +")" +
							"manifold(" + world.getManifoldCount()+")";
			System.out.println(count);
		}
	}
	
	/**
	 * GLHandler - GLEventHandler implementation
	 * @author Bowie
	 *
	 */
	
	private class GLHandler implements GLEventListener {
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
			// generate delta time
//			curTick = System.nanoTime();
//			tickDT += (curTick - lastTick) / 1000000000.0f;
//			lastTick = curTick;
//			
//			while (tickDT >= AppMain.this.simDT) {
//				tickDT -= AppMain.this.simDT;
//				AppMain.this.update(AppMain.this.simDT);
//			}
			
			synchronized (world) {
				AppMain.this.render(drawable.getGL().getGL2(), dt);
			}
			
			
			
//			System.out.println("render: " + Thread.currentThread().getName());
		}

		@Override
		public void reshape(GLAutoDrawable drawable, int x, int y, int width,
				int height) {
			GL2 gl = drawable.getGL().getGL2();			
			gl.glViewport(x, y, width, height);
			
			// reset perspective
			Matrix4.perspective(fov, (float)width/height, 0.1f, 100.0f, matPers);
			gl.glMatrixMode(GL2.GL_PROJECTION);
			gl.glLoadMatrixf(matPers.m, 0);
			
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
