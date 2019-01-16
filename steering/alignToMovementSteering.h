#ifndef __ALIGN_TO_MOVEMENT_STEERING_H__
#define __ALIGN_TO_MOVEMENT_STEERING_H__


#include "iAlignSteering.h"
#include <gameEntity.h>


class AlignToMovementSteering : public IAlignSteering {

public:

	AlignToMovementSteering(GameEntity& entity);

	virtual float GetSteering       ();
	virtual void  SetTargetRotation (float targetRotation);
	virtual void  DrawDebug         ();

private:
	GameEntity&      mEntity;
	IAlignSteering * mAlignSteering;
};

#endif