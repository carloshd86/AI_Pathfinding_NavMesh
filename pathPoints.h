#ifndef __PATH_POINTS_H__
#define __PATH_POINTS_H__

struct PathPoints
{
	std::vector<USVec2D> points;
};

bool ReadPathPoints(const char* filename, PathPoints& pathPoints);

#endif