#include <stdafx.h>

#include "PathNode.h"

bool PathNode::CompareNode(const PathNode& pathNode1, const PathNode& pathNode2) {
	return pathNode1.node.Compare(pathNode2.node);
}

bool PathNode::CompareNodePointer(const PathNode* pathNode1, const PathNode* pathNode2) {
	return pathNode1 && pathNode2 && pathNode1->node.Compare(pathNode2->node);
}

bool PathNode::operator<(const PathNode& other) const {
	return g + h < other.g + other.h;
}