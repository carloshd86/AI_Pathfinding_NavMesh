#ifndef __I_ALIGN_STEERING_H__
#define __I_ALIGN_STEERING_H__


class IAlignSteering {

public:

	virtual float GetSteering       () = 0;
	virtual void  SetTargetRotation (float targetRotation) = 0;
	virtual void  DrawDebug         () = 0;
};

#endif
