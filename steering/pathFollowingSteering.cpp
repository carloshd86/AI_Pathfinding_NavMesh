#include <stdafx.h>
#include "pathFollowingSteering.h"
#include "seekSteering.h"


PathFollowingSteering::PathFollowingSteering(GameEntity& entity, USVec2D target) :
	mEntity          (entity),
	mTarget          (target),
	mAcceleration    (0.f, 0.f),
	mDesiredPosition (0.f, 0.f),
	mClosestPosition (0.f, 0.f),
	mClosestSegment  (0)
{
	mSteering = new SeekSteering(entity, target);
	mPathPoints = mEntity.GetPathPoints().points;
}

// **************************************************

USVec2D PathFollowingSteering::GetSteering() {

	USVec2D entityLocation = mEntity.GetLoc();
	USVec2D entityVelocity = mEntity.GetLinearVelocity();
	float entityMaxVelocity = mEntity.GetParams().max_velocity;

	if (mPathPoints.size()) {
		float timeAhead = mEntity.GetParams().time_ahead;
		float lookAhead = mEntity.GetParams().look_ahead;

		mClosestSegment = GetClosestSegment();
		mClosestPosition = GetClosestPoint(mClosestSegment);
		mDesiredPosition = GetNextPoint(mClosestPosition, mClosestSegment, lookAhead * lookAhead);

		mSteering->SetTargetPosition(mDesiredPosition);
		mAcceleration = mSteering->GetSteering();
	}

	return mAcceleration;
}

// **************************************************

void PathFollowingSteering::SetTargetPosition(USVec2D targetPosition) {
}

// **************************************************

void PathFollowingSteering::PathChanged() {
	mPathPoints = mEntity.GetPathPoints().points;
}

// **************************************************

size_t PathFollowingSteering::GetClosestSegment()
{
	USVec2D entityLocation = mEntity.GetLoc();
	size_t segments = mPathPoints.size() - 1;

	size_t closestSegment = 0;
	float closestDistance = -1;
	for (size_t i = 0; i < segments; ++i) {
		float distance = (entityLocation - GetClosestPoint(i)).LengthSquared();
		if (closestDistance >= distance || closestDistance < 0) {
			closestSegment = i;
			closestDistance = distance;
		}
	}

	return closestSegment;
}

// **************************************************

USVec2D PathFollowingSteering::GetClosestPoint(size_t segment)
{
	USVec2D closestPoint;
	USVec2D entityLocation = mEntity.GetLoc();
	
	USVec2D a(entityLocation.mX - mPathPoints[segment].mX, entityLocation.mY - mPathPoints[segment].mY);
	USVec2D b(entityLocation.mX - mPathPoints[segment + 1].mX, entityLocation.mY - mPathPoints[segment + 1].mY);
	USVec2D line(mPathPoints[segment + 1].mX - mPathPoints[segment].mX, mPathPoints[segment + 1].mY - mPathPoints[segment].mY);

	USVec2D aNorm = a;
	USVec2D bNorm = b;
	USVec2D lineNorm = line;
	aNorm.NormSafe();
	bNorm.NormSafe();
	lineNorm.NormSafe();
	float projA = aNorm.Dot(lineNorm);
	float projB = bNorm.Dot(lineNorm);

	if (projA < 0 && projB < 0) {
		// El punto mas cercano es el inicio del segmento
		closestPoint = mPathPoints[segment];
	} else if (projA >= 0 && projB < 0) {
		// El punto mas cercano está en el segmento
		closestPoint = mPathPoints[segment] + (lineNorm *  a.Length() * projA);
	} else {
		// El punto mas cercano es el final del segmento
		closestPoint = mPathPoints[segment + 1];
	}

	return closestPoint;
}

// **************************************************

USVec2D PathFollowingSteering::GetNextPoint(USVec2D closestPoint, size_t segment, float lookAheadSqrd)
{
	float distanceSqrdToNextSegment = (closestPoint).DistSqrd(mPathPoints[segment + 1]);
	if (lookAheadSqrd < distanceSqrdToNextSegment) {
		// DesiredPoint esta en el segmento actual
		USVec2D currentSegment = mPathPoints[segment + 1] - mPathPoints[segment];
		currentSegment.NormSafe();
		return closestPoint + (currentSegment * sqrtf(lookAheadSqrd));
	}
	else if (lookAheadSqrd == distanceSqrdToNextSegment) {
		// DesiredPoint es el final del segmento actual
		return  mPathPoints[segment + 1];
	}
	else if (lookAheadSqrd > distanceSqrdToNextSegment) {
		lookAheadSqrd -= distanceSqrdToNextSegment;
		// DesiredPoint esta en el siguiente segmento
		size_t pathSize = mPathPoints.size();
		if (segment + 1 == pathSize - 1) {
			return mPathPoints[segment + 1];
		}
		else {
			return GetNextPoint(mPathPoints[segment + 1], segment + 1, lookAheadSqrd);
		}
	}
}

// **************************************************

void PathFollowingSteering::DrawDebug() {
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();

	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D linearVelocity  = mEntity.GetLinearVelocity();

	gfxDevice.SetPenColor(0.8f, 0.8f, 0.8f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + linearVelocity);

	gfxDevice.SetPenColor(0.0f, 0.0f, 1.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mAcceleration);

	if (!mPathPoints.empty() && mClosestSegment < (mPathPoints.size() - 1)) {
		gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.5f);
		MOAIDraw::DrawLine(mPathPoints[mClosestSegment], mPathPoints[mClosestSegment + 1]);
	}

	gfxDevice.SetPenColor(0.0f, 1.0f, 0.0f, 0.5f);
	MOAIDraw::DrawEllipseFill(mClosestPosition.mX, mClosestPosition.mY, 5.f, 5.f, 50);

	gfxDevice.SetPenColor(1.0f, 0.0f, 1.0f, 0.5f);
	MOAIDraw::DrawEllipseFill(mDesiredPosition.mX, mDesiredPosition.mY, 5.f, 5.f, 50);
}