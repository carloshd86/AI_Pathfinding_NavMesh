#include <stdafx.h>
#include "obstacleAvoidanceSteering.h"


ObstacleAvoidanceSteering::ObstacleAvoidanceSteering(GameEntity& entity) :
	mEntity             (entity),
	mAcceleration       (0.f, 0.f),
	mIsNearVector       (0.f, 0.f),
	mDistanceToObstacle (0.f, 0.f),
	mCollisionCheck     (0.f, 0.f),
	mCollidingObstacle  (nullptr) {}

// **************************************************

USVec2D ObstacleAvoidanceSteering::GetSteering() {
	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D entityDirection = mEntity.GetLinearVelocity();
	entityDirection.NormSafe();
	mCollisionCheck = entityDirection * mEntity.GetParams().look_ahead;

	mAcceleration.mX = 0;
	mAcceleration.mY = 0;

	mDistanceToObstacle.mX = 0;
	mDistanceToObstacle.mY = 0;

	mIsNearVector.mX = 0;
	mIsNearVector.mY = 0;

	mCollidingObstacle = nullptr;

	const Obstacles& existingObstacles = mEntity.GetObstacles();
	size_t obstaclesSize = existingObstacles.obstacles.size();
	for (size_t i = 0; i < obstaclesSize; ++i) {
		USVec2D obstaclePosition(existingObstacles.obstacles[i].mX, existingObstacles.obstacles[i].mY);
		
		mDistanceToObstacle = obstaclePosition - entityPosition; 
		
		float proj = mDistanceToObstacle.Dot(entityDirection);
		
		if (proj > 0 && (proj*proj <= mCollisionCheck.LengthSquared())) {
			// Obstaculo en la direccion de movimiento y en rango de comprobacion

			mIsNearVector = entityPosition + (entityDirection * proj);
			USVec2D diff = mIsNearVector - obstaclePosition;
			float obstacleRadius = existingObstacles.obstacles[i].mZ + mEntity.GetParams().char_radius;

			if (diff.LengthSquared() <= obstacleRadius * obstacleRadius) {	
				// Existe colision.
				mCollidingObstacle = &existingObstacles.obstacles[i];
				mAcceleration = diff;
				mAcceleration.NormSafe();
				mAcceleration *= mEntity.GetParams().max_acceleration;
				break;
			}
		}
	}

	return mAcceleration;
}

// **************************************************

void ObstacleAvoidanceSteering::SetTargetPosition(USVec2D targetPosition) {
}

// **************************************************

void ObstacleAvoidanceSteering::DrawDebug() {
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();

	USVec2D entityPosition = mEntity.GetLoc();
	USVec2D linearVelocity = mEntity.GetLinearVelocity();

	gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mIsNearVector);

	gfxDevice.SetPenColor(0.0f, 1.0f, 1.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mAcceleration);

	gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mDistanceToObstacle);

	gfxDevice.SetPenColor(0.0f, 0.0f, 1.0f, 0.5f);
	MOAIDraw::DrawLine(entityPosition, entityPosition + mCollisionCheck);

	if (mCollidingObstacle) {
		gfxDevice.SetPenColor(1.f, 0.f, 0.f, 0.5f);
		MOAIDraw::DrawEllipseFill(mCollidingObstacle->mX, mCollidingObstacle->mY, mCollidingObstacle->mZ, mCollidingObstacle->mZ, 50);
	}

}