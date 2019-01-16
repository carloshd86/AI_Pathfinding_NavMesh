#ifndef __CHARACTER_H__
#define __CHARACTER_H__

#include <gameEntity.h>
#include "steering/iSteering.h"
#include "steering/iAlignSteering.h"
#include "pathPoints.h"
#include "obstacles.h"
#include "pathfinding/pathfinder.h"

class Character: public GameEntity, public Pathfinder::IPathListener
{
public:
	DECL_LUA_FACTORY(Character)
protected:
	virtual void OnStart();
	virtual void OnStop();
	virtual void OnUpdate(float step);

	struct SteeringWeight {
		SteeringWeight(ISteering* _steering, float _weight) : steering(_steering), weight(_weight) {};

		ISteering* steering;
		float      weight;
	};

public:
	virtual void DrawDebug();

	Character();
	~Character();

	void SetLinearVelocity(float x, float y) { mLinearVelocity.mX = x; mLinearVelocity.mY = y;}
	void SetAngularVelocity(float angle) { mAngularVelocity = angle;}
	void SetPathFinder(const Pathfinder* pathfinder);

	USVec2D GetLinearVelocity() const { return mLinearVelocity;}
	float GetAngularVelocity() const { return mAngularVelocity;}

	const Params& GetParams() const { return mParams; }
	const PathPoints& GetPathPoints() const { return mPathPoints; }
	const Obstacles& GetObstacles() const { return mObstacles; }

	void PathChanged();
	void RegisterPathSteeringListener(IPathSteeringListener& listener);

private:

	USVec2D mLinearVelocity;
	float mAngularVelocity;

	Params mParams;
	PathPoints mPathPoints;
	Obstacles mObstacles;
	const Pathfinder* mPathFinder;

	std::vector<SteeringWeight> mSteeringWeights;
	IAlignSteering             *mAlignSteering;

	std::vector<IPathSteeringListener*> mPathSteeringListeners;

	void AdjustAccelerationModule(USVec2D& acceleration);
	void UpdatePath();

	// Lua configuration
public:
	virtual void RegisterLuaFuncs(MOAILuaState& state);
private:
	static int _setLinearVel(lua_State* L);
	static int _setAngularVel(lua_State* L);
	static int _setPathFinder(lua_State* L);
};

#endif