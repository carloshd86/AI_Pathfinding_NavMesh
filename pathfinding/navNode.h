#ifndef __GRIDNODE_H__
#define __GRIDNODE_H__

class NavNode {
public:
	NavNode(int polygon = 0, int edgeStart = 0, int edgeEnd = 1) : polygon(polygon), edgeStart(edgeStart), edgeEnd(edgeEnd) {}

	int polygon;
	int edgeStart;
	int edgeEnd;

	bool Compare(const NavNode& other) const;
	bool operator==(const NavNode& other) const;
	bool operator<(const NavNode& other) const;
};

#endif