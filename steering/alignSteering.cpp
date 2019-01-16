#include <stdafx.h>
#include "alignSteering.h"


AlignSteering::AlignSteering(GameEntity& entity, float targetRotation) :
	mEntity(entity),
	mTargetRotation(targetRotation) {
	if (mTargetRotation < 0)
		mTargetRotation += 360;
}

// **************************************************

float AlignSteering::GetSteering() {
	float entityRotation = mEntity.GetRot();
	float angularArriveRadius = mEntity.GetParams().angular_arrive_radius;
	float angularDestRadius = mEntity.GetParams().angular_dest_radius;
	
	if (entityRotation < 0)
		entityRotation += 360;

	float angleToMove = entityRotation - mTargetRotation;

	if (angleToMove < 0)
		angleToMove += 360;

	float angularVelocity = mEntity.GetParams().max_angular_velocity;
	if (angleToMove > 180)
		angleToMove = 360 - angleToMove;
	else
		angularVelocity *= -1;

	if (abs(angleToMove) > 0) {
		if (abs(angleToMove) <= abs(angularDestRadius)) {
			angularVelocity = 0.f;
		}
		else if (abs(angleToMove) <= abs(angularArriveRadius)) {
			angularVelocity *= (angleToMove / angularArriveRadius);
		}
	}
	else {
		angularVelocity = 0.f;
	}

	float currentVelocity = mEntity.GetAngularVelocity();
	mAcceleration = angularVelocity - currentVelocity;

	float maxAcceleration = mEntity.GetParams().max_angular_acceleration;
	if ((maxAcceleration > 0 && mAcceleration < 0) || (maxAcceleration < 0 && mAcceleration > 0))
		mAcceleration = -maxAcceleration;
	else
		mAcceleration = maxAcceleration;

	return mAcceleration;
}

// **************************************************

void AlignSteering::SetTargetRotation(float targetRotation) {
	mTargetRotation = targetRotation;
}

// **************************************************

void AlignSteering::DrawDebug() {

}