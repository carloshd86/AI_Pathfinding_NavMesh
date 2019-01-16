#include <stdafx.h>
#include "arriveSteering.h"


ArriveSteering::ArriveSteering(GameEntity& entity, const USVec2D targetPosition) :
	mEntity         (entity),
	mTargetPosition (targetPosition),
	mAcceleration   (0.f, 0.f) {}

// **************************************************

USVec2D ArriveSteering::GetSteering() {
	USVec2D entityPosition = mEntity.GetLoc();
	float arriveRadius = mEntity.GetParams().arrive_radius;
	float destRadius = mEntity.GetParams().dest_radius;
	USVec2D desiredVelocity = mTargetPosition - entityPosition;
	float distanceSquareLength = desiredVelocity.LengthSquared();

	if (distanceSquareLength > arriveRadius * arriveRadius) {
		desiredVelocity.NormSafe();
		desiredVelocity *= mEntity.GetParams().max_velocity;
	} else if (distanceSquareLength <= destRadius * destRadius) {
		desiredVelocity.mX = 0.f;
		desiredVelocity.mY = 0.f;
	} else {
		desiredVelocity.NormSafe(); 
		desiredVelocity *= (distanceSquareLength / (arriveRadius * arriveRadius)) * mEntity.GetParams().max_velocity;
	}

	USVec2D entityVelocity = mEntity.GetLinearVelocity();

	mAcceleration = desiredVelocity - entityVelocity;
	mAcceleration.NormSafe();
	mAcceleration *= mEntity.GetParams().max_acceleration;

	return mAcceleration;
}

// **************************************************

void ArriveSteering::SetTargetPosition(USVec2D targetPosition) {
	mTargetPosition = targetPosition;
}

// **************************************************

void ArriveSteering::DrawDebug() {
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();

	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D linearVelocity    = mEntity.GetLinearVelocity();

	gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + linearVelocity);

	gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mAcceleration);
}