package com.bowie.javagl;

public interface ContactGenerator {
	int generateContact(PersistentManifold m, Shape sA, Vector3 posA, Quaternion rotA, Shape sB, Vector3 posB, Quaternion rotB, RigidBody bA, RigidBody bB);
}
