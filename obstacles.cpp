#include <stdafx.h>
#include <tinyxml.h>
#include "obstacles.h"

bool ReadObstacles(const char* filename, Obstacles& obstacles)
{
    TiXmlDocument doc(filename);
    if (!doc.LoadFile())
    {
        fprintf(stderr, "Couldn't read params from %s", filename);
        return false;
    }

    TiXmlHandle hDoc(&doc);

    TiXmlElement* pElem;
    pElem = hDoc.FirstChildElement().Element();
    if (!pElem)
    {
        fprintf(stderr, "Invalid format for %s", filename);
        return false;
    }

    TiXmlHandle hRoot(pElem);
    TiXmlHandle hObstacles = hRoot.FirstChildElement("obstacles");

	// Points
	TiXmlElement* obstacleElem = hObstacles.FirstChildElement("obstacle").Element();
	for (obstacleElem; obstacleElem; obstacleElem = obstacleElem->NextSiblingElement("obstacle"))
	{
		USVec3D obstacle;
		obstacleElem->Attribute("x", &obstacle.mX);
		obstacleElem->Attribute("y", &obstacle.mY);
		obstacleElem->Attribute("r", &obstacle.mZ);
		obstacles.obstacles.push_back(obstacle);
	}

    return true;
}