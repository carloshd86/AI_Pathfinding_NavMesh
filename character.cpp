#include <stdafx.h>
#include "character.h"
#include <tinyxml.h>

#include <params.h>
#include <steering/obstacleAvoidanceSteering.h>
#include <steering/pathFollowingSteering.h>
#include <steering/alignToMovementSteering.h>


Character::Character() : 
	mLinearVelocity  (0.0f, 0.0f), 
	mAngularVelocity (0.0f),
	mAlignSteering   (nullptr),
	mPathFinder      (nullptr)
{
	RTTI_BEGIN
		RTTI_EXTEND (MOAIEntity2D)
		RTTI_END
}

Character::~Character()
{
	for (SteeringWeight& steeringWeight : mSteeringWeights) {
		delete steeringWeight.steering;
	}
	if (mAlignSteering) delete mAlignSteering;
}

void Character::OnStart()
{
	ReadParams("params.xml", mParams);
	//ReadPathPoints("path.xml", mPathPoints);
	//ReadObstacles("obstacles.xml", mObstacles);
	PathFollowingSteering* pfSteering = new PathFollowingSteering(*this, mParams.targetPosition);
	RegisterPathSteeringListener(*pfSteering);

	mSteeringWeights.push_back(SteeringWeight(pfSteering, 0.2f));
	mSteeringWeights.push_back(SteeringWeight(new ObstacleAvoidanceSteering(*this), 0.8f));

	mAlignSteering = new AlignToMovementSteering(*this);
}

void Character::OnStop()
{

}

void Character::OnUpdate(float step)
{
	USVec3D pos             = GetLoc();
	float   rot             = GetRot();

	pos.mX += mLinearVelocity.mX * step;
	pos.mY += mLinearVelocity.mY * step;

	rot += mAngularVelocity * step;

	SetLoc(pos);
	SetRot(rot);

	size_t numSteerings = mSteeringWeights.size();
	USVec2D linearAcceleration { 0.f, 0.f };
	for (size_t i = 0; i < numSteerings; ++i) {
		USVec2D loopSteering = mSteeringWeights[i].steering->GetSteering();
		loopSteering.NormSafe();
		linearAcceleration += loopSteering * mSteeringWeights[i].weight;
	}

	AdjustAccelerationModule(linearAcceleration);

	mLinearVelocity.mX += linearAcceleration.mX * step;
	mLinearVelocity.mY += linearAcceleration.mY * step;

	if (mAlignSteering) {
		float angularAcceleration = mAlignSteering->GetSteering();
		mAngularVelocity += angularAcceleration * step;
	}
}

void Character::DrawDebug()
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();

	gfxDevice.SetPenColor(0.75f, 0.75f, 1.0f, 0.5f);
	size_t pathParamsSize = mPathPoints.points.size();
	if (pathParamsSize > 1) {
		for (size_t i = 0; i < pathParamsSize - 1; ++i) {
			MOAIDraw::DrawLine(mPathPoints.points[i].mX, mPathPoints.points[i].mY, mPathPoints.points[i + 1].mX, mPathPoints.points[i + 1].mY);
		}
	}


	gfxDevice.SetPenColor(0.5f, 0.5f, 0.5f, 0.25f);
	size_t obstaclesSize = mObstacles.obstacles.size();
	if (obstaclesSize) {
		for (size_t i = 0; i < obstaclesSize; ++i) {
			MOAIDraw::DrawEllipseFill(mObstacles.obstacles[i].mX, mObstacles.obstacles[i].mY, mObstacles.obstacles[i].mZ, mObstacles.obstacles[i].mZ, 50);
		}
	}

	for (SteeringWeight& steeringWeight : mSteeringWeights) {
		if (steeringWeight.steering) steeringWeight.steering->DrawDebug();
	}
	if (mAlignSteering) mAlignSteering->DrawDebug();
}

void Character::SetPathFinder(const Pathfinder* pathfinder) {
	mPathFinder = pathfinder;
}




// Lua configuration

void Character::RegisterLuaFuncs(MOAILuaState& state)
{
	MOAIEntity2D::RegisterLuaFuncs(state);

	luaL_Reg regTable [] = {
	{ "setLinearVel",			_setLinearVel},
	{ "setAngularVel",			_setAngularVel},
	{ "setPathFinder",			_setPathFinder},
	{ NULL, NULL }
	};

	luaL_register(state, 0, regTable);
}

int Character::_setLinearVel(lua_State* L)
{
	MOAI_LUA_SETUP(Character, "U")

	float pX = state.GetValue<float>(2, 0.0f);
	float pY = state.GetValue<float>(3, 0.0f);
	self->SetLinearVelocity(pX, pY);
	return 0;	
}

int Character::_setAngularVel(lua_State* L)
{
	MOAI_LUA_SETUP(Character, "U")

	float angle = state.GetValue<float>(2, 0.0f);
	self->SetAngularVelocity(angle);

	return 0;
}

int Character::_setPathFinder(lua_State* L)
{
	MOAI_LUA_SETUP(Character, "U")

	Pathfinder* pathFinder = state.GetLuaObject<Pathfinder>(2, true);
	if (pathFinder) {
		self->SetPathFinder(pathFinder);
		pathFinder->RegisterListener(*self);
	}
	return 0;	
}

void Character::PathChanged() {
	UpdatePath();
}


void Character::UpdatePath() {
	mPathPoints.points.clear();
	if (mPathFinder) {
		mPathPoints = mPathFinder->GetPathPoints();
	}
	for (IPathSteeringListener* listener : mPathSteeringListeners) listener->PathChanged();
}


void Character::AdjustAccelerationModule(USVec2D& acceleration) {
	acceleration.NormSafe();
	acceleration *= mParams.max_acceleration;
}

void Character::RegisterPathSteeringListener(IPathSteeringListener& listener) {
	mPathSteeringListeners.push_back(&listener);
}