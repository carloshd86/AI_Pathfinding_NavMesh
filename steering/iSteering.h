#ifndef __I_STEERING_H__
#define __I_STEERING_H__


class ISteering {

public:

	virtual USVec2D GetSteering       () = 0;
	virtual void    SetTargetPosition (USVec2D targetPosition) = 0;
	virtual void    DrawDebug         () = 0;
};

#endif
