#ifndef __PATHFINDER_H__
#define __PATHFINDER_H__

#include <moaicore/MOAIEntity2D.h>
#include <navmesh.h>
#include "navNode.h"
#include "PathNode.h"

class Pathfinder: public virtual MOAIEntity2D
{
public:
	Pathfinder();
	~Pathfinder();

	virtual void DrawDebug();

	void SetStartPosition (float x, float y) { mStartPosition = USVec2D(x, y); mStartNode = GetNodeFromScreenPosition(mStartPosition); UpdatePath(); }
	void SetEndPosition   (float x, float y) { mEndPosition   = USVec2D(x, y); mEndNode   = GetNodeFromScreenPosition(mEndPosition);   UpdatePath(); }
	const USVec2D& GetStartPosition() const { return mStartPosition;}
	const USVec2D& GetEndPosition()   const { return mEndPosition;}

    bool PathfindStep();
private:
	void UpdatePath();
	void Astar();
	void GetNodeConnections(const PathNode& pathNode, std::vector<PathNode*>& connections);
	bool IsGridNodeValid(const NavNode& node) const;
	void BuildPath(const PathNode& lastNode);
	NavNode GetNodeFromScreenPosition(const USVec2D& screenPosition) const;
	int CalculateDistance(const NavNode& node) const;
	USVec2D GetStartNodePoint() const;
	USVec2D GetEndNodePoint() const;

	static bool PathNodeSort(PathNode* pathNode1, PathNode* pathNode2);

	std::vector<NavNode> mPath;

private:
	USVec2D mStartPosition;
	USVec2D mEndPosition;
	NavNode mStartNode;
	NavNode mEndNode;
	NavMesh mNavMesh;

	// Lua configuration
public:
	DECL_LUA_FACTORY(Pathfinder)
public:
	virtual void RegisterLuaFuncs(MOAILuaState& state);
private:
	static int _setStartPosition(lua_State* L);
	static int _setEndPosition(lua_State* L);
    static int _pathfindStep(lua_State* L);
};


#endif