#ifndef __NAVMESH_H__
#define __NAVMESH_H__


struct LinkPoint
{
	int polygon;
	int edgestart;
	int edgeend;
};

struct Link
{
	LinkPoint start;
	LinkPoint end;
};

struct Polygon
{
	std::vector<USVec2D> points;
};

struct NavMesh
{
	std::vector<Polygon> polygons;
	std::vector<Link> links;
};

bool ReadNavMesh(const char* filename, NavMesh& navmesh);

#endif