#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__

struct Obstacles
{
	std::vector<USVec3D> obstacles;
};

bool ReadObstacles(const char* filename, Obstacles& obstacles);

#endif