#ifndef __ALIGN_STEERING_H__
#define __ALIGN_STEERING_H__


#include "iAlignSteering.h"
#include <gameEntity.h>


class AlignSteering : public IAlignSteering {

public:

	AlignSteering(GameEntity& entity, float targetRotation);

	virtual float GetSteering       ();
	virtual void  SetTargetRotation (float targetRotation);
	virtual void  DrawDebug         ();

private:
	GameEntity& mEntity;
	float       mTargetRotation;
	float       mAcceleration;
};

#endif