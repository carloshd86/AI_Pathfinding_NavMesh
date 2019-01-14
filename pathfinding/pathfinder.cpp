#include <stdafx.h>

#include "pathfinder.h"
#include <algorithm>
#include <functional>

Pathfinder::Pathfinder() : MOAIEntity2D() {
	RTTI_BEGIN
	RTTI_EXTEND(MOAIEntity2D)
	RTTI_END

	ReadNavMesh("navmesh.xml", mNavMesh);
}

Pathfinder::~Pathfinder()
{

}

void Pathfinder::UpdatePath()
{
	mPath.clear();
	Astar();
}

void Pathfinder::Astar()
{
	if (IsGridNodeValid(mStartNode) && IsGridNodeValid(mEndNode) && !mStartNode.Compare(mEndNode)) {
		std::vector<PathNode*> openList;
		std::vector<PathNode*> closedList;

		PathNode* pathNode = new PathNode(mStartNode, 0, CalculateDistance(mStartNode));
		openList.push_back(pathNode);
		bool existPath = false;
		while (!openList.empty()) {
			std::sort(openList.begin(), openList.end(), &Pathfinder::PathNodeSort);

			pathNode = openList.front();
			openList.erase(openList.begin());
			closedList.push_back(pathNode);

			if (mEndNode.Compare(pathNode->node)) {
				// Node is the end node
				BuildPath(*pathNode);
				return;
			} else {
				// Node is not the end node
				std::vector<PathNode*> connections;
				GetNodeConnections(*pathNode, connections);
				for (PathNode* nextPathNode : connections) {
					auto& closedListNodeFound = std::find_if(closedList.begin(), closedList.end(), std::bind(&PathNode::CompareNodePointer, std::placeholders::_1, nextPathNode));
					if (closedList.end() != closedListNodeFound) {
						// Node already on closedList. Checking if cost is smaller to remove it and add to open list.
						if (nextPathNode->g + nextPathNode->h < (*closedListNodeFound)->g + (*closedListNodeFound)->h) {
							// Removing node from closedList
							closedList.erase(closedListNodeFound);
							// Adding the pathNode to openList
							openList.push_back(nextPathNode);
						}
						else {
							continue;
						}
					} else {

						auto& openListNodeFound = std::find_if(openList.begin(), openList.end(), std::bind(&PathNode::CompareNodePointer, std::placeholders::_1, nextPathNode));
						if (openList.end() != openListNodeFound) {
							// Change cost and parent if cost is smaller
							if (nextPathNode->g + nextPathNode->h < (*openListNodeFound)->g + (*openListNodeFound)->h) {
								(*openListNodeFound)->g = nextPathNode->g;
								(*openListNodeFound)->h = nextPathNode->h;
								(*openListNodeFound)->parent = nextPathNode->parent;
							}

							// Releasing the node memory as it is nor in openList neither in closedList
							delete nextPathNode;
						} else {
							// Adding the pathNode to openList
							openList.push_back(nextPathNode);
						}
					}
				}
			}
		}

		// Releasing memory for stored nodes
		for (PathNode* toDelete : openList) {
			delete toDelete;
		}
		for (PathNode* toDelete : closedList) {
			delete toDelete;
		}
	}
}

void Pathfinder::GetNodeConnections(const PathNode& pathNode, std::vector<PathNode*>& connections) {
	connections.clear();

	// Find in navmesh links where linkStart polygon or linkEnd polygon is the current navNode polygon
	for (Link& link : mNavMesh.links) {
		if (link.start.polygon == pathNode.node.polygon) {
			NavNode nextNode(link.end.polygon, link.end.edgestart, link.end.edgeend);
			if (IsGridNodeValid(nextNode)) {
				int cost = pathNode.g + 1;
				connections.push_back(new PathNode(nextNode, cost, CalculateDistance(nextNode), &pathNode));
			}
		} else if (link.end.polygon == pathNode.node.polygon) {
			NavNode nextNode(link.start.polygon, link.start.edgestart, link.start.edgeend);
			if (IsGridNodeValid(nextNode)) {
				int cost = pathNode.g + 1;
				connections.push_back(new PathNode(nextNode, cost, CalculateDistance(nextNode), &pathNode));
			}
		}
	}
}

bool Pathfinder::IsGridNodeValid(const NavNode& node) const {
	// Returns true if the node is within the limits of the grid and has a valid cost (reachable node)
	//return node.x >= 0 && node.y >= 0 && node.x < static_cast<int>(mGridCols) && node.y < static_cast<int>(mGridRows) && mGrid.end() != mGrid.find(node) && mGrid.at(node) >= 0;
	return true;
}

void Pathfinder::BuildPath(const PathNode& lastNode) {
	mPath.clear();
	mPath.push_back(lastNode.node);
	const PathNode* parentNode = lastNode.parent;
	while (parentNode) {
		mPath.push_back(parentNode->node);
		parentNode = parentNode->parent;
	}
	std::reverse(mPath.begin(), mPath.end());
}

NavNode Pathfinder::GetNodeFromScreenPosition(const USVec2D& screenPosition) const {
	NavNode result;
	bool found = false;

	int left = -512;
	int top = -384;

	size_t navMeshSize = mNavMesh.polygons.size();
	for (size_t i = 0; i < navMeshSize; ++i) {
		const Polygon& polygon = mNavMesh.polygons[i];

		// First check: look if point is inside the bounding rectangle
		float minX = -left * 4.f;
		float minY = -top * 4.f;
		float maxX = left * 4.f;
		float maxY = top * 4.f;
		for (const USVec2D& point : polygon.points) {
			if (minX > point.mX - left) minX = point.mX - left;
			if (minY > point.mY - top)  minY = point.mY - top;
			if (maxX < point.mX - left) maxX = point.mX - left;
			if (maxY < point.mY - top)  maxY = point.mY - top;
		}

		float fixedX = screenPosition.mX - left;
		float fixedY = screenPosition.mY - top;

		if (minX <= fixedX && minY <= fixedY && maxX >= fixedX && maxY >= fixedY) {
			// Point is inside the bounding rectangle
			// TODO Check if it is also inside de actual polygon
			result.polygon = i;
			result.edgeStart = 0;
			result.edgeEnd = 1;
			found = true;
		}
		if (found) break;
	}
	return result;
}

USVec2D Pathfinder::GetStartNodePoint() const {
	return mNavMesh.polygons[mStartNode.polygon].points[mStartNode.edgeStart];
}

USVec2D Pathfinder::GetEndNodePoint() const {
	return mNavMesh.polygons[mEndNode.polygon].points[mEndNode.edgeStart];
}

int Pathfinder::CalculateDistance(const NavNode& node) const {
	USVec2D endNodePoint = GetEndNodePoint();
	USVec2D nodePoint = mNavMesh.polygons[node.polygon].points[node.edgeStart];
	
	return static_cast<int>(endNodePoint.Dist(nodePoint));
}

bool Pathfinder::PathNodeSort(PathNode* pathNode1, PathNode* pathNode2) { 
	return *pathNode1 < *pathNode2; 
}

void Pathfinder::DrawDebug()
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	gfxDevice.SetPenColor(0.0f, 0.0f, 1.0f, 0.5f);

	for (auto& polygon : mNavMesh.polygons) {
		//Contorno con un color: MOAIDraw::DrawPolygon
		gfxDevice.SetPenColor(0.0f, 0.0f, 1.0f, 0.5f);
		MOAIDraw::DrawPolygon(polygon.points);
		//Relleno con otro: MOAIDraw::DrawPolygonFilled
		gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.5f);
		MOAIDraw::DrawPolygonFilled(polygon.points);
	}

	gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.75f);
	for (auto& link : mNavMesh.links) {
		Polygon& polygonStart = mNavMesh.polygons[link.start.polygon];
		USVec2D& edgePointStart1 = polygonStart.points[link.start.edgestart];
		USVec2D& edgePointStart2 = polygonStart.points[link.start.edgeend];
		Polygon& polygonEnd = mNavMesh.polygons[link.end.polygon];
		USVec2D& edgePointEnd1 = polygonEnd.points[link.end.edgestart];
		USVec2D& edgePointEnd2 = polygonEnd.points[link.end.edgeend];
		USVec2D edgePointStartMiddle = USVec2D((edgePointStart1.mX + edgePointStart2.mX) / 2.f, (edgePointStart1.mY + edgePointStart2.mY) / 2.f);
		USVec2D edgePointEndMiddle = USVec2D((edgePointEnd1.mX + edgePointEnd2.mX) / 2.f, (edgePointEnd1.mY + edgePointEnd2.mY) / 2.f);
		MOAIDraw::DrawEllipseFill(edgePointStartMiddle.mX, edgePointStartMiddle.mY, 5.f, 5.f, 15);
		MOAIDraw::DrawEllipseFill(edgePointEndMiddle.mX, edgePointEndMiddle.mY, 5.f, 5.f, 15);
		MOAIDraw::DrawLine(edgePointStartMiddle, edgePointEndMiddle);
	}

	if (!mPath.empty()) {
		for (NavNode& node : mPath) {
			
			//Contorno con un color: MOAIDraw::DrawPolygon
			gfxDevice.SetPenColor(0.0f, 0.0f, 0.0f, 1.0f);
			MOAIDraw::DrawPolygon(mNavMesh.polygons[node.polygon].points);
			//Relleno con otro: MOAIDraw::DrawPolygonFilled
			gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.75f);
			MOAIDraw::DrawPolygonFilled(mNavMesh.polygons[node.polygon].points);
		}
	}

	gfxDevice.SetPenColor(1.0f, 1.0f, 1.0f, 0.1f);
	MOAIDraw::DrawPolygonFilled(mNavMesh.polygons[mStartNode.polygon].points);

	gfxDevice.SetPenColor(0.0f, 0.75f, 0.0f, 0.1f);
	MOAIDraw::DrawPolygonFilled(mNavMesh.polygons[mEndNode.polygon].points);
}

bool Pathfinder::PathfindStep()
{
    // returns true if pathfinding process finished
    return true;
}















//lua configuration ----------------------------------------------------------------//
void Pathfinder::RegisterLuaFuncs(MOAILuaState& state)
{
	MOAIEntity::RegisterLuaFuncs(state);

	luaL_Reg regTable [] = {
		{ "setStartPosition",		_setStartPosition},
		{ "setEndPosition",			_setEndPosition},
        { "pathfindStep",           _pathfindStep},
		{ NULL, NULL }
	};

	luaL_register(state, 0, regTable);
}

int Pathfinder::_setStartPosition(lua_State* L)
{
	MOAI_LUA_SETUP(Pathfinder, "U")

	float pX = state.GetValue<float>(2, 0.0f);
	float pY = state.GetValue<float>(3, 0.0f);
	self->SetStartPosition(pX, pY);
	return 0;
}

int Pathfinder::_setEndPosition(lua_State* L)
{
	MOAI_LUA_SETUP(Pathfinder, "U")

	float pX = state.GetValue<float>(2, 0.0f);
	float pY = state.GetValue<float>(3, 0.0f);
	self->SetEndPosition(pX, pY);
	return 0;
}

int Pathfinder::_pathfindStep(lua_State* L)
{
    MOAI_LUA_SETUP(Pathfinder, "U")

    self->PathfindStep();
    return 0;
}