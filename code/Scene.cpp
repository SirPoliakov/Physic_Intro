//
//  Scene.cpp
//
#include "Scene.h"
#include "../Intersections.h"
#include "../Shape.h"


/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body1;
	body1.position = Vec3(0, 0, 8);
	body1.orientation = Quat(0, 0, 0, 1);
	body1.shape = new ShapeSphere(1.0f);
	body1.elasticity = 0.5f;
	body1.inverseMass = 1.0f;
	bodies.push_back(body1);
	Body body2;
	body2.position = Vec3(0, 10, 8);
	body2.orientation = Quat(0, 0, 0, 1);
	body2.shape = new ShapeSphere(1.0f);
	body2.elasticity = 0.5f;
	body2.inverseMass = 1.0f;
	bodies.push_back(body2);

	Body earth;
	earth.position = Vec3(0, 0, -1000);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(1000.0f);
	earth.elasticity = 1.0f;
	earth.inverseMass = 0.0f;
	bodies.push_back(earth);

	// TODO: Add code
}

/*
====================================================
Scene::Update
====================================================
*/

void Scene::Update( const float dt_sec ) {

	for (int i = 0; i < bodies.size(); ++i) {
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);

	}

	// Collision checks
	for (int i = 0; i < bodies.size(); ++i)
	{
		for (int j = i + 1; j < bodies.size(); ++j)
		{
			Body& bodyA = bodies[i];
			Body& bodyB = bodies[j];
			if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
				continue;
			Contact contact;
			if (Intersections::Intersect(bodyA, bodyB, contact))
			{
				Contact::ResolveContact(contact);
			}
		}
	}


	// Position update
	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i].position += bodies[i].linearVelocity * dt_sec;
	}

}