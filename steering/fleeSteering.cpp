#include <stdafx.h>
#include "fleeSteering.h"


FleeSteering::FleeSteering(GameEntity& entity, const USVec2D targetPosition) :
	mEntity         (entity),
	mTargetPosition (targetPosition),
	mAcceleration   (0.f, 0.f) {}

// **************************************************
//
// **************************************************

USVec2D FleeSteering::GetSteering() {
	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D desiredVelocity =  entityPosition - mTargetPosition;

	desiredVelocity.NormSafe();
	desiredVelocity *= mEntity.GetParams().max_velocity;

	USVec2D entityVelocity = mEntity.GetLinearVelocity();

	mAcceleration = desiredVelocity - entityVelocity;
	mAcceleration.NormSafe();
	mAcceleration *= mEntity.GetParams().max_acceleration;

	return mAcceleration;
}

// **************************************************

void FleeSteering::SetTargetPosition(USVec2D targetPosition) {
	mTargetPosition = targetPosition;
}

// **************************************************

void FleeSteering::DrawDebug() {
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();

	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D linearVelocity    = mEntity.GetLinearVelocity();

	gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + linearVelocity);

	gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mAcceleration);
}