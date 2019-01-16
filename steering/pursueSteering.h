#ifndef __PURSUE_STEERING_H__
#define __PURSUE_STEERING_H__


#include <gameEntity.h>
#include <steering\iSteering.h>


class PursueSteering : public ISteering {

public:

	PursueSteering(GameEntity& pursuer, GameEntity& target);

	virtual USVec2D GetSteering       ();
	virtual void    DrawDebug         ();

	void SetTargetPosition(USVec2D targetPosition);

private:
	static const float MIN_SPEED_TO_PURSUE;

	GameEntity&  mPursuer;
	GameEntity&  mTarget;
	ISteering   *mSteering;
	USVec2D      mAcceleration;
	USVec2D      mPursuePosition;
};

#endif