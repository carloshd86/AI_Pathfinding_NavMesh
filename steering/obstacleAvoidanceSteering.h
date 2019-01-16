#ifndef __OBSTACLEAVOIDANCE_STEERING_H__
#define __OBSTACLEAVOIDANCE_STEERING_H__


#include "iSteering.h"
#include <gameEntity.h>


class ObstacleAvoidanceSteering : public ISteering {

public:

	ObstacleAvoidanceSteering(GameEntity& entity);

	virtual USVec2D GetSteering       ();
	virtual void    SetTargetPosition (USVec2D targetPosition);
	virtual void    DrawDebug         ();

private:
	GameEntity&    mEntity;
	USVec2D        mAcceleration;
	USVec2D        mIsNearVector;
	USVec2D        mDistanceToObstacle;
	USVec2D        mCollisionCheck;
	const USVec3D* mCollidingObstacle;
};

#endif