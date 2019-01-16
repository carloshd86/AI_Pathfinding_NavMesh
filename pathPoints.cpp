#include <stdafx.h>
#include <tinyxml.h>
#include "pathPoints.h"

bool ReadPathPoints(const char* filename, PathPoints& pathPoints)
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
    TiXmlHandle hPoints = hRoot.FirstChildElement("points");

	// Points
	TiXmlElement* pointElem = hPoints.FirstChildElement("point").Element();
	for (pointElem; pointElem; pointElem = pointElem->NextSiblingElement("point"))
	{
		USVec2D point;
		pointElem->Attribute("x", &point.mX);
		pointElem->Attribute("y", &point.mY);
		pathPoints.points.push_back(point);
	}

    return true;
}