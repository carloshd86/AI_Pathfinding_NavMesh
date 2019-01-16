#include <stdafx.h>
#include "pursueSteering.h"
#include "arriveSteering.h"


const float PursueSteering::MIN_SPEED_TO_PURSUE = 20.f;

PursueSteering::PursueSteering(GameEntity& pursuer, GameEntity& target) :
	mPursuer        (pursuer),
	mTarget         (target),
	mAcceleration   (0.f, 0.f),
	mPursuePosition (0.f, 0.f)
{
	mSteering = new ArriveSteering(pursuer, target.GetLoc());
}

// **************************************************

USVec2D PursueSteering::GetSteering() {

	USVec2D pursuerLocation = mPursuer.GetLoc();
	USVec2D targetLocation  = mTarget.GetLoc();
	USVec2D pursuerVelocity = mPursuer.GetLinearVelocity();
	USVec2D targetVelocityNorm  = mTarget.GetLinearVelocity();
	targetVelocityNorm.NormSafe();
	float pursuerMaxVelocity = mPursuer.GetParams().max_velocity;
	float targetMaxVelocity = mTarget.GetParams().max_velocity;

	float secondsToTarget = 0.f;
	if (pursuerVelocity.LengthSquared() > MIN_SPEED_TO_PURSUE * MIN_SPEED_TO_PURSUE) {
		secondsToTarget = pursuerLocation.Dist(targetLocation) / pursuerMaxVelocity;	
		mPursuePosition = targetLocation + (targetVelocityNorm * targetMaxVelocity * secondsToTarget);
	}
	else {
		mPursuePosition = targetLocation;
	}

	mSteering->SetTargetPosition(mPursuePosition);

	mAcceleration = mSteering->GetSteering();
	return mAcceleration;
}

// **************************************************

void PursueSteering::SetTargetPosition(USVec2D targetPosition) {
}

// **************************************************

void PursueSteering::DrawDebug() {
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();

	USVec2D pursuerPosition = mPursuer.GetLoc();
	USVec2D linearVelocity  = mPursuer.GetLinearVelocity();

	gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.5f);
	MOAIDraw::DrawPoint(mPursuePosition);
	MOAIDraw::DrawEllipseOutline(mPursuePosition.mX, mPursuePosition.mY, 5.f, 5.f, 50);

	gfxDevice.SetPenColor(0.0f, 1.0f, 1.0f, 0.5f);
	MOAIDraw::DrawLine(pursuerPosition, pursuerPosition + linearVelocity);

	gfxDevice.SetPenColor(0.0f, 0.0f, 1.0f, 0.5f);
	MOAIDraw::DrawLine(pursuerPosition, pursuerPosition + mAcceleration);
}