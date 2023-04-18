#include "Body.h"
#include "Shape.h"

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;
	// dv = J / m
	linearVelocity += impulse * inverseMass;
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	Mat3 inertiaTensor = shape->InertiaTensor();
	Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * inverseMass;
	return inverseInertiaTensor;
}
Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertiaTensor = shape->InertiaTensor();
	Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * inverseMass;
	Mat3 orient = orientation.ToMat3();
	inverseInertiaTensor = orient * inverseInertiaTensor
		* orient.Transpose();
	return inverseInertiaTensor;
}
