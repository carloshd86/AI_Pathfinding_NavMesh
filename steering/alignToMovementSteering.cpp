#include <stdafx.h>
#include "alignToMovementSteering.h"
#include "alignSteering.h"

#define RAD2DEG 57.2958f

AlignToMovementSteering::AlignToMovementSteering(GameEntity& entity) :
	mEntity(entity)
{
	mAlignSteering = new AlignSteering(entity, 0.f);
}

// **************************************************

float AlignToMovementSteering::GetSteering() {
	
	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D direction = mEntity.GetLinearVelocity();

	float angle = atan2(direction.mY, direction.mX) * RAD2DEG;

	mAlignSteering->SetTargetRotation(angle);
	return mAlignSteering->GetSteering();
}

// **************************************************

void AlignToMovementSteering::SetTargetRotation(float targetRotation) {
}

// **************************************************

void AlignToMovementSteering::DrawDebug() {
	mAlignSteering->DrawDebug();
}