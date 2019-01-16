#ifndef __ARRIVE_STEERING_H__
#define __ARRIVE_STEERING_H__


#include "iSteering.h"
#include <gameEntity.h>


class ArriveSteering : public ISteering {

public:

	ArriveSteering(GameEntity& entity, USVec2D targetPosition);

	virtual USVec2D GetSteering       ();
	virtual void    SetTargetPosition (USVec2D targetPosition);
	virtual void    DrawDebug         ();

private:
	GameEntity& mEntity;
	USVec2D     mTargetPosition;
	USVec2D     mAcceleration;
};

#endif