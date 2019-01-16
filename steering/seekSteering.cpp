#include <stdafx.h>
#include "seekSteering.h"


SeekSteering::SeekSteering(GameEntity& entity, USVec2D targetPosition) :
	mEntity         (entity),
	mTargetPosition (targetPosition),
	mAcceleration   (0.f, 0.f) {}

// **************************************************

USVec2D SeekSteering::GetSteering() {
	USVec2D entityPosition = mEntity.GetLoc();
	float destRadius = mEntity.GetParams().dest_radius;
	USVec2D desiredVelocity = mTargetPosition - entityPosition;
	float distanceSquareLength = desiredVelocity.LengthSquared();

	if (distanceSquareLength <= destRadius * destRadius) {
		desiredVelocity.mX = 0.f;
		desiredVelocity.mY = 0.f;
	} else {
		desiredVelocity.NormSafe();
		desiredVelocity *= mEntity.GetParams().max_velocity;
	}

	USVec2D entityVelocity = mEntity.GetLinearVelocity();

	mAcceleration = desiredVelocity - entityVelocity;
	mAcceleration.NormSafe();
	mAcceleration *= mEntity.GetParams().max_acceleration;

	return mAcceleration;
}

// **************************************************

void SeekSteering::SetTargetPosition(USVec2D targetPosition) {
	mTargetPosition = targetPosition;
}

// **************************************************

void SeekSteering::DrawDebug() {
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	
	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D linearVelocity    = mEntity.GetLinearVelocity();

	gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + linearVelocity);

	gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mAcceleration);
}