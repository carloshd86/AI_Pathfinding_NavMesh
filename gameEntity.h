#ifndef __GAME_ENTITY_H__
#define __GAME_ENTITY_H__

#include <moaicore/MOAIEntity2D.h>
#include <params.h>
#include <pathPoints.h>
#include <obstacles.h>

class GameEntity: public MOAIEntity2D
{
protected:
	virtual void OnStart() = 0;
	virtual void OnStop() = 0;
	virtual void OnUpdate(float step) = 0;
public:
	class IPathSteeringListener {
		public:
			virtual void PathChanged() = 0;
	};

	virtual void DrawDebug() = 0;

	virtual void SetLinearVelocity(float x, float y) = 0;
	virtual void SetAngularVelocity(float angle) = 0;

	virtual USVec2D GetLinearVelocity() const = 0;
	virtual float GetAngularVelocity() const = 0;

	virtual const Params& GetParams() const = 0;
	virtual const PathPoints& GetPathPoints() const = 0;
	virtual const Obstacles& GetObstacles() const = 0;

	virtual void RegisterPathSteeringListener(IPathSteeringListener& listener) = 0;

	// Lua configuration
public:
	virtual void RegisterLuaFuncs(MOAILuaState& state) = 0;
};

#endif