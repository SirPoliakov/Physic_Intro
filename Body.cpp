#include "Body.h"
#include "Shape.h"

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;
	// dv = J / m
	linearVelocity += impulse * inverseMass;
}