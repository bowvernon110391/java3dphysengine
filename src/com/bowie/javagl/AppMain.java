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
	private Matrix4 matOrtho = new Matrix4();
	
	private Physics world = new Physics(7, 5, .04f, .3f);
	private RigidBody bomb;
	private WorldSpring ws;
	private WorldPinJoint wp;

	private RigidBody chassis;
	private WheelSet wheels;
	
	private Shape box = new Box(2, 3, 5);
	private Vector3 sPos = new Vector3();
	private Quaternion sRot = Quaternion.makeAxisRot(new Vector3(
			MathHelper.randomRanged(-1, 1),
			MathHelper.randomRanged(-1, 1),
			MathHelper.randomRanged(-1, 1)), MathHelper.randomRanged((float)-Math.PI, (float)Math.PI));
	
	private RaycastInfo rayInfo = new RaycastInfo();
//	private AppRes res = new AppRes();
	
	private boolean drawContacts = true;
	private boolean drawContactN = false;
	private boolean drawContactT = false;
	private boolean drawBBox = false;
	
	private int camMode = 0;
	
	private boolean useCoreShape = false;
	
	private float targetSteer = .0f;
	private float maxSteer = (float) Math.toRadians(40);	// 40 degree maximum
	private float steerStrength = .2f;
	private float currSteer = .0f;
	
	private boolean steerRight = false;
	private boolean steerLeft= false;
	private boolean forward = false;
	private boolean reverse = false;
	private boolean handbrake = false;
	
	// this is our lock
//	final private ReentrantLock resLock = new ReentrantLock();
	
	private float yRot = 40;
	private float xzRot = 130.0f;
	private float dist = 6.0f;
	private float camStrength = .25f;
	private float camX, camY, camZ;
	private float camTX, camTY, camTZ;
	private float camVX, camVY, camVZ;
	
	private int screenW, screenH;
	
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
		rayInfo.rayStart.setTo(5, 1, 1);
		rayInfo.rayEnd.setTo(-8, -2, 0);
		
		rayInfo.rayT = -1.f;
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
		// start rendering
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT|GL2.GL_DEPTH_BUFFER_BIT);
		
		Matrix4.perspective(fov, aspect, .1f, 200.0f, matPers);
		
		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glLoadMatrixf(matPers.m, 0);
		
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		GLU glu = GLU.createGLU(gl.getGL());
		glu.gluLookAt(camX, camY, camZ, camTX, camTY, camTZ, 0, 1, 0);
		
		gl.glPushMatrix();
		
		box.render(gl, sPos, sRot);
		
		gl.glPopMatrix();
		
		rayInfo.debugDraw(gl);
		
		// now we draw ui
		Matrix4.ortho(0, screenW, 0, screenH, -1, 1, matOrtho);
		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glLoadMatrixf(matOrtho.m, 0);
		
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		// draw wheel status of solver
		gl.glColor3f(1, 0, 0);
		
		if (WheelSet.useLateralImpulse)
			gl.glColor3f(1, 1, 0);
		else
			gl.glColor3f(.5f, .5f, .5f);
		gl.glPushMatrix();
		gl.glTranslated(10, 10, 0);
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glVertex2i(0, 0);
			gl.glVertex2i(20, 0);
			gl.glVertex2i(20, 20);
			gl.glVertex2i(0, 20);
		gl.glEnd();
		gl.glPopMatrix();
		
		if (WheelSet.useLongitudinalImpulse)
			gl.glColor3f(1, 0, 1);
		else
			gl.glColor3f(.25f, .5f, .25f);
		gl.glPushMatrix();
		gl.glTranslated(40, 10, 0);
		gl.glBegin(GL2.GL_LINE_LOOP);
			gl.glVertex2i(0, 0);
			gl.glVertex2i(20, 0);
			gl.glVertex2i(20, 20);
			gl.glVertex2i(0, 20);
		gl.glEnd();
		gl.glPopMatrix();
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
			
		case KeyEvent.VK_O:
			WheelSet.useLongitudinalImpulse = !WheelSet.useLongitudinalImpulse;
			break;
		case KeyEvent.VK_P:
			WheelSet.useLateralImpulse = !WheelSet.useLateralImpulse;
			break;
			
		case KeyEvent.VK_Q:
			handbrake = true;
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
			
		case KeyEvent.VK_Q:
			handbrake = false;
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
		// now calculate camera
		Vector3 tPos = new Vector3();
		
		// camera target
		camTX = tPos.x;
		camTY = tPos.y;
		camTZ = tPos.z;
		
		// the camera position			
		float cosYRot = (float) Math.cos(Math.toRadians(yRot));
		
		float camXc = (float) (Math.sin(Math.toRadians(xzRot)) * cosYRot * dist);
		float camZc = (float) (Math.cos(Math.toRadians(xzRot)) * cosYRot * dist);
		float camYc = (float) (Math.sin(Math.toRadians(yRot)) * dist);
		
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
		
		// let's do this
		rayInfo.rayT = -1.f;
		box.raycast(sPos, sRot, rayInfo);
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
			screenW = width;
			screenH = height;
			
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
			
			// now let's move it
			if (e.getButton() == MouseEvent.BUTTON1) {
				rayInfo.rayStart.setTo(camX, camY, camZ);
			}
			
			if (e.getButton() == MouseEvent.BUTTON3) {
				rayInfo.rayEnd.setTo(camX, camY, camZ);
			}
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
