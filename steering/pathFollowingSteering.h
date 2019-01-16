#ifndef __PATH_FOLLOWING_STEERING_H__
#define __PATH_FOLLOWING_STEERING_H__


#include <gameEntity.h>
#include <steering\iSteering.h>


class PathFollowingSteering : public ISteering, public GameEntity::IPathSteeringListener {

public:
	PathFollowingSteering(GameEntity& entity, USVec2D target);

	virtual USVec2D GetSteering       ();
	virtual void    DrawDebug         ();

	void SetTargetPosition(USVec2D targetPosition);
	
	void PathChanged();

private:
	GameEntity&          mEntity;
	ISteering           *mSteering;
	USVec2D              mTarget;
	USVec2D              mAcceleration;
	std::vector<USVec2D> mPathPoints;
	USVec2D              mDesiredPosition;
	USVec2D              mClosestPosition;
	size_t               mClosestSegment;

	size_t GetClosestSegment();
	USVec2D GetClosestPoint(size_t segment);

	USVec2D GetNextPoint(USVec2D closestPoint, size_t segment, float lookAheadSqrd);
};

#endif