#include <stdafx.h>

#include "navNode.h"

bool NavNode::Compare(const NavNode& other) const {
	return other.polygon == polygon /*&& other.edgeStart == edgeStart && other.edgeEnd == edgeEnd*/;
}

bool NavNode::operator==(const NavNode& other) const {
	return Compare(other);
}

bool NavNode::operator<(const NavNode& other) const {
	return !Compare(other);
}