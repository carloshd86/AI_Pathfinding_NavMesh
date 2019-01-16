#include <stdafx.h>

#include "pathfinder.h"
#include <algorithm>
#include <functional>
#include "pathPoints.h"

const int Pathfinder::LEFT = -512;
const int Pathfinder::TOP  = -384;

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

	for(IPathListener* listener : mListeners) listener->PathChanged();
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

	size_t navMeshSize = mNavMesh.polygons.size();
	for (size_t i = 0; i < navMeshSize; ++i) {
		const Polygon& polygon = mNavMesh.polygons[i];

		// First check: look if point is inside the bounding rectangle
		float minX = -LEFT * 4.f;
		float minY = -TOP * 4.f;
		float maxX = LEFT * 4.f;
		float maxY = TOP * 4.f;
		for (const USVec2D& point : polygon.points) {
			if (minX > point.mX - LEFT) minX = point.mX - LEFT;
			if (minY > point.mY - TOP)  minY = point.mY - TOP;
			if (maxX < point.mX - LEFT) maxX = point.mX - LEFT;
			if (maxY < point.mY - TOP)  maxY = point.mY - TOP;
		}

		float fixedX = screenPosition.mX - LEFT;
		float fixedY = screenPosition.mY - TOP;

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

	gfxDevice.SetPenColor(1.0f, 1.0f, 0.0f, 0.25f);
	for (auto& link : mNavMesh.links) {
		USVec2D edgePointStartMiddle = GetNodeEdgeMiddlePosition(mNavMesh, link.start.polygon, link.start.edgestart, link.start.edgeend);
		USVec2D edgePointEndMiddle   = GetNodeEdgeMiddlePosition(mNavMesh, link.end.polygon, link.end.edgestart, link.end.edgeend);
		MOAIDraw::DrawEllipseFill(edgePointStartMiddle.mX, edgePointStartMiddle.mY, 5.f, 5.f, 15);
		MOAIDraw::DrawEllipseFill(edgePointEndMiddle.mX, edgePointEndMiddle.mY, 5.f, 5.f, 15);
		MOAIDraw::DrawLine(edgePointStartMiddle, edgePointEndMiddle);
	}

	if (!mPath.empty()) {

		// Drawing path polygons
		/*
		for (NavNode& node : mPath) {
			//Contorno con un color: MOAIDraw::DrawPolygon
			gfxDevice.SetPenColor(0.0f, 0.0f, 0.0f, 1.0f);
			MOAIDraw::DrawPolygon(mNavMesh.polygons[node.polygon].points);
			//Relleno con otro: MOAIDraw::DrawPolygonFilled
			gfxDevice.SetPenColor(1.0f, 0.0f, 0.0f, 0.75f);
			MOAIDraw::DrawPolygonFilled(mNavMesh.polygons[node.polygon].points);
		}
		*/

		// Drawing path lines + points
		size_t pathSize = mPath.size();
		if (pathSize > 1) {
			gfxDevice.SetPenColor(1.0f, 1.0f, 1.0f, 0.75f);
			for (size_t i = 0; i < pathSize - 1; ++i) {
				NavNode& startNode = mPath[i];
				NavNode& endNode = mPath[i + 1];

				USVec2D edgePointStartMiddle = GetNodeEdgeMiddlePosition(mNavMesh, startNode);
				USVec2D edgePointEndMiddle   = GetNodeEdgeMiddlePosition(mNavMesh, endNode);
				MOAIDraw::DrawEllipseFill(edgePointStartMiddle.mX, edgePointStartMiddle.mY, 5.f, 5.f, 15);
				MOAIDraw::DrawEllipseFill(edgePointEndMiddle.mX, edgePointEndMiddle.mY, 5.f, 5.f, 15);
				MOAIDraw::DrawLine(edgePointStartMiddle, edgePointEndMiddle);
			}
		}
	}

	// Drawing start + end polygons
	/*
	gfxDevice.SetPenColor(1.0f, 1.0f, 1.0f, 0.1f);
	MOAIDraw::DrawPolygonFilled(mNavMesh.polygons[mStartNode.polygon].points);

	gfxDevice.SetPenColor(0.0f, 0.75f, 0.0f, 0.1f);
	MOAIDraw::DrawPolygonFilled(mNavMesh.polygons[mEndNode.polygon].points);
	*/

	// Drawing start + end lines + points
	gfxDevice.SetPenColor(0.1f, 0.1f, 0.75f, 0.75f);
	USVec2D startNodeCenter = GetPolygonBoundingRectangleCenterPoint(mNavMesh, mStartNode.polygon);
	MOAIDraw::DrawEllipseFill(startNodeCenter.mX, startNodeCenter.mY, 15.f, 15.f, 15);

	gfxDevice.SetPenColor(0.1f, 0.75f, 0.1f, 0.75f);
	USVec2D endNodeCenter = GetPolygonBoundingRectangleCenterPoint(mNavMesh, mEndNode.polygon);
	MOAIDraw::DrawEllipseFill(endNodeCenter.mX, endNodeCenter.mY, 15.f, 15.f, 15);
}

bool Pathfinder::PathfindStep()
{
    // returns true if pathfinding process finished
    return true;
}

USVec2D Pathfinder::GetPolygonBoundingRectangleCenterPoint(const NavMesh& navMesh, int polygonIndex) {
	USVec2D result(0.f, 0.f);

	if (polygonIndex < static_cast<int>(navMesh.polygons.size())) {
		const Polygon& polygon = navMesh.polygons[polygonIndex];

		// First check: look if point is inside the bounding rectangle
		float minX = -LEFT * 4.f;
		float minY = -TOP * 4.f;
		float maxX = LEFT * 4.f;
		float maxY = TOP * 4.f;
		for (const USVec2D& point : polygon.points) {
			if (minX > point.mX - LEFT) minX = point.mX - LEFT;
			if (minY > point.mY - TOP)  minY = point.mY - TOP;
			if (maxX < point.mX - LEFT) maxX = point.mX - LEFT;
			if (maxY < point.mY - TOP)  maxY = point.mY - TOP;
		}

		minX += LEFT;
		minY += TOP;
		maxX += LEFT;
		maxY += TOP;

		result.mX = (minX + maxX) / 2.f;
		result.mY = (minY + maxY) / 2.f;
	}

	return result;
}

USVec2D Pathfinder::GetNodeEdgeMiddlePosition(const NavMesh& navMesh, const NavNode& navNode) {
	return GetNodeEdgeMiddlePosition(navMesh, navNode.polygon, navNode.edgeStart, navNode.edgeEnd);
}

USVec2D Pathfinder::GetNodeEdgeMiddlePosition(const NavMesh& navMesh, int polygonIndex, int edgeStartIndex, int edgeEndIndex)  {
	const Polygon& polygon = navMesh.polygons[polygonIndex];
	const USVec2D& edgePoint1 = polygon.points[edgeStartIndex];
	const USVec2D& edgePoint2 = polygon.points[edgeEndIndex];
	USVec2D edgePointMiddle((edgePoint1.mX + edgePoint2.mX) / 2.f, (edgePoint1.mY + edgePoint2.mY) / 2.f);

	return edgePointMiddle;
}


PathPoints Pathfinder::GetPathPoints() const {
	PathPoints path;

	size_t pathSize = mPath.size();
	if (pathSize) {
		for (const NavNode& node : mPath) {
			path.points.push_back(GetNodeEdgeMiddlePosition(mNavMesh, node));
		}
	}

	return path;
}

void Pathfinder::RegisterListener(IPathListener& listener) {
	mListeners.push_back(&listener);
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