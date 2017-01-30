package com.bowie.javagl;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import com.jogamp.opengl.GL2;

/**
 * PersistentManifold
 * this class will keep track of all contact points
 * this will build from single contact all the time
 * @author Bowie
 *
 */
public class PersistentManifold {
	static public int MAX_CONTACT_SIZE = 4;
	static public float DIST_TOLERANCE = 0.2f;
	static public float DEPTH_TOLERANCE = 0.002f;
	
	public List<Contact> contacts;
	public List<SpeculativeContact> specContacts;
	public RigidBody bodyA, bodyB;
	
	public PersistentManifold(BodyPair p) {
		this.bodyA = p.getBodyA();
		this.bodyB = p.getBodyB();
		this.contacts = new ArrayList<>();
		this.specContacts = new ArrayList<>();
	}
	
	public PersistentManifold(RigidBody bA, RigidBody bB) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.contacts = new ArrayList<>();
		this.specContacts = new ArrayList<>();
	}
	
	/**
	 * this will refresh all contacts, removing the invalid ones
	 * usually, the first thing to do in the loop
	 */
	public void refresh() {
		// remove speculative contacts
		specContacts.clear();
		
		// refresh persistent contact
		Iterator<Contact> iter = contacts.iterator();
		
//		int numContact = contacts.size();
//		int numRemoved = 0;
		
		while (iter.hasNext()) {
			Contact c = iter.next();
			// now, let's refresh, and ditch if it gets too far
//			Vector3 oldWorldA = new Vector3(c.worldA);
//			Vector3 oldWorldB = new Vector3(c.worldB);
			// recompute world data
			c.refresh();
			// check for separation
			boolean separating = c.depth >= DEPTH_TOLERANCE;
			// calculate 2D distance
			float dp;
			Vector3 worldAB = new Vector3();
			Vector3.sub(c.worldA, c.worldB, worldAB);
			// flatten to normal
			dp = Vector3.dot(worldAB, c.normal);
			Vector3 flatter = new Vector3(c.normal);
			flatter.scale(-dp);
			Vector3.add(worldAB, flatter, worldAB);
			dp = Vector3.dot(worldAB, worldAB);
			
			/*float distA = Vector3.lengthSquared(oldWorldA, c.worldA);
			float distB = Vector3.lengthSquared(oldWorldB, c.worldB);*/
			
			float maxDist = DIST_TOLERANCE * DIST_TOLERANCE;
			boolean closeEnough = /*distA < maxDist && distB < maxDist &&*/ dp < maxDist;
			// remove if:
			
			//  distA > DIST_TOLERANCE || distB > DIST_TOLERANCE || c.depth > (DEPTH_TOLERANCE)
//			System.out.println("depth, dist: " + c.depth +", " + dp);
			boolean keep = !separating && closeEnough;
			if ( !keep ) {
				iter.remove();
//				System.out.println("invalid contact removed!");
//				numRemoved++;
			}
			
		}
		// cool, we keep it. hopefully will get a cache hit
//		if (numRemoved > 0 && numContact > 0)
//			System.out.println("start, removed, now: " + numContact + ", " + numRemoved + ", " + contacts.size());
	}
	
	public void updateManifoldPlane(Vector3 normal) {
		for (Contact ct : contacts) {
			ct.normal.setTo(normal);
		}
	}
	
	public int numContacts() {
		return contacts.size();
	}
	
	public Contact getCacheEntry(Contact c) {
		// let's get potential cache
		// potential if:
		//	- distanceA is < DIST_TOLERANCE && distanceB is < DIST_TOLERANCE
		float closest = DIST_TOLERANCE * DIST_TOLERANCE;
		Contact cache = null;
		for (Contact ct : contacts) {
			float distA = Vector3.lengthSquared(c.worldA, ct.worldA);
			
			// well, it falls within our criteria
			if (distA < closest) {
				closest = distA;
				cache = ct;
			}
		}
		return cache;
	}
	
	public void addSpeculativeContact(SpeculativeContact sc) {
		specContacts.add(sc);
	}
	
	public void add(Contact nc) {
		if (nc == null)
			return;
		// would cache instead, if we find cache entry, we restore accumulated impulse
		Contact cache = getCacheEntry(nc);
		if (cache != null) {
			// remove cache + restore impulse
			nc.accumPN = cache.accumPN;
			nc.accumPT1 = cache.accumPT1;
			nc.accumPT2 = cache.accumPT2;
//			
			contacts.remove(cache);
//			// copy new contact point data
//			cache.localA.setTo(nc.localA);
//			cache.localB.setTo(nc.localB);
//			
//			cache.worldA.setTo(nc.worldA);
//			cache.worldB.setTo(nc.worldB);
//			
//			cache.normal.setTo(nc.normal);
//			cache.tangent1.setTo(nc.tangent1);
//			cache.tangent2.setTo(nc.tangent2);
//			
//			cache.depth = nc.depth;
			//
//			System.out.println("contact cache hit!: "+ nc.accumPN+", "+nc.accumPT1+","+nc.accumPT2);
			
		} 
		// add em
		contacts.add(nc);
		
		// welp, do we reach limit?
		if (contacts.size() > MAX_CONTACT_SIZE) {
			// simplify
			reduceContacts();
//			System.out.println("contacts reduced");
		}
	}
	
	public void preStep(float dt, float slop, float baumgarte) {
		// before we enter solver step, calculate Jacobian and shit
		for (Contact ct : contacts) {
			ct.preCalculate(dt, baumgarte, slop);
		}
		
		// precalculate for speculative contacts
		for (SpeculativeContact sc : specContacts) {
			sc.preCalculate(dt, baumgarte, slop);
		}
	}
	
	public void solve() {
		// apply speculative contacts first
		for (SpeculativeContact sc : specContacts) {
			sc.solve();
		}
		
		// apply all impulse
		for (Contact ct : contacts) {
			ct.applyImpulse();
		}		

	}
	
	public float getHeight(int granularity) {
		// do we have contact?
		if (contacts.size() < 1)
			return -9999.99f;
		// we have one. grab first only (hopefully it's good)
		return contacts.get(0).worldA.y * granularity;
	}
	
	public void positionSolve() {
		for (Contact ct : contacts) {
			//ct.applyPseudoImpulse(baumgarte, slop);
			ct.applyBiasImpulse();
		}
	}
	
	public void reduceContacts() {
		// do not bother
		if (contacts.size() <= MAX_CONTACT_SIZE)
			return;
		
		// holds our most interesting shit
		Contact a = null,b=null,c=null,d=null;
		
		// 1. keep the deepest
		float deepest = 0;
		for (Contact ct : contacts) {
			if (ct.depth < deepest) {
				deepest = ct.depth;
				a = ct;
			}
		}
		if (a != null)
			contacts.remove(a);
		
		// if a is null, no further action possible (that means, all contacts are positive)
		if (a == null) {
			System.out.println("Persistent Manifold: no deep contact -> " + contacts.size());
			contacts.clear();
			return;
		}
		// ha! got em.
		// 2. keep the furthest from 1
		float furthest = 0;
		for (Contact ct : contacts) {
			float dist = Vector3.lengthSquared(a.worldA, ct.worldA);
			if (dist > furthest) {
				furthest = dist;
				b = ct;
			}
		}
		if (b != null)
			contacts.remove(b);
		
		// now we keep track of area
		float largest = 0;
		for (Contact ct : contacts) {
			float area = MathHelper.calcTriangleArea(a.worldA, b.worldA, ct.worldA);
			if (area > largest) {
				largest = area;
				c = ct;
			}
		}
		if (c != null)
			contacts.remove(c);
		
		// now we find the furthest from c
		furthest = 0;
		for (Contact ct : contacts) {
			float dist = Vector3.lengthSquared(c.worldA, ct.worldA);
			if (dist > furthest) {
				furthest = dist;
				d = ct;
			}
		}
		if (d != null)
			contacts.remove(d);
		// ok, there's a chance it might be inside abc, in which we remove it
		Vector3 bary = MathHelper.computeBarycentric(d.worldA, a.worldA, b.worldA, c.worldA);
		if (bary.x > Vector3.EPSILON && bary.y > Vector3.EPSILON && bary.z > Vector3.EPSILON) {
			d = null;
		}
		// clear all remaining contact
		contacts.clear();
		// add our most interesting contacts
		contacts.add(a);
		contacts.add(b);
		contacts.add(c);
		if (d != null)
			contacts.add(d);
		
	}
	
	public void debugDraw(GL2 gl, boolean drawNormal, boolean drawTangents) {
		for (Contact c : contacts)
			c.debugDraw(gl, drawNormal, drawTangents);
	}
}
