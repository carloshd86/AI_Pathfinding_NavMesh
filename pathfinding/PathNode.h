#ifndef __PATHNODE_H__
#define __PATHNODE_H__

#include "navNode.h"

class PathNode {
public:
	PathNode(NavNode node, int g = 0, int h = 0, const PathNode* parent = nullptr) : node(node), g(g), h(h), parent(parent) {}

	static bool CompareNode(const PathNode& pathNode1, const PathNode& pathNode2);
	static bool CompareNodePointer(const PathNode* pathNode1, const PathNode* pathNode2);

	bool operator<(const PathNode& other) const;

	NavNode node;
	int g;
	int h;
	const PathNode* parent;
};

#endif