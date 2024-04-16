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
	body1.position = Vec3(0, 0.5, 5.5);
	body1.orientation = Quat(0, 0, 0, 1);
	body1.shape = new ShapeSphere(0.8f);
	body1.elasticity = 1.0f;
	body1.inverseMass = 10.0f;
	bodies.push_back(body1);
	Body body2;
	body2.position = Vec3(0, 0, 8);
	body2.orientation = Quat(0, 0, 0, 1);
	body2.shape = new ShapeSphere(0.9f);
	body2.elasticity = 0.5f;
	body2.inverseMass = 1.0f;
	bodies.push_back(body2);

	Body body3;
	body3.position = Vec3(0, -1.5, 4);
	body3.orientation = Quat(0, 0, 0, 1);
	body3.shape = new ShapeSphere(0.4f);
	body3.elasticity = 0.5f;
	body3.inverseMass = 1.0f;
	bodies.push_back(body3);

	Body body4;
	body4.position = Vec3(0, 1, 3);
	body4.orientation = Quat(0, 0, 0, 1);
	body4.shape = new ShapeSphere(0.3f);
	body4.elasticity = 0.5f;
	body4.inverseMass = 1.0f;
	bodies.push_back(body4);

	Body body9;
	body9.position = Vec3(0, -2, 3);
	body9.orientation = Quat(0, 0, 0, 1);
	body9.shape = new ShapeSphere(0.7f);
	body9.elasticity = 0.5f;
	body9.inverseMass = 1.0f;
	bodies.push_back(body9);

	Body body5;
	body5.position = Vec3(0, 3, 3);
	body5.orientation = Quat(0, 0, 0, 1);
	body5.shape = new ShapeSphere(0.8f);
	body5.elasticity = 0.5f;
	body5.inverseMass = 1.0f;
	bodies.push_back(body5);

	Body body6;
	body6.position = Vec3(0, -3, 4);
	body6.orientation = Quat(0, 0, 0, 1);
	body6.shape = new ShapeSphere(0.4f);
	body6.elasticity = 0.5f;
	body6.inverseMass = 1.0f;
	bodies.push_back(body6);

	Body body7;
	body7.position = Vec3(0, -2, 2);
	body7.orientation = Quat(0, 0, 0, 1);
	body7.shape = new ShapeSphere(0.2f);
	body7.elasticity = 0.5f;
	body7.inverseMass = 1.0f;
	bodies.push_back(body7);

	Body body8;
	body8.position = Vec3(0, -2, 8);
	body8.orientation = Quat(0, 0, 0, 1);
	body8.shape = new ShapeSphere(0.65f);
	body8.elasticity = 0.5f;
	body8.inverseMass = 1.0f;
	bodies.push_back(body8);

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