#include <stdafx.h>
#include <tinyxml.h>
#include "navmesh.h"

bool ReadNavMesh(const char* filename, NavMesh& navmesh)
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
    TiXmlHandle hPolygons = hRoot.FirstChildElement("polygons");

	// Polygons
	TiXmlElement* polygonElem = hPolygons.FirstChildElement("polygon").Element();
	for (polygonElem; polygonElem; polygonElem = polygonElem->NextSiblingElement("polygon"))
	{
		Polygon polygon;

		TiXmlElement* pointElem = polygonElem->FirstChildElement("point");
		for (pointElem; pointElem; pointElem = pointElem->NextSiblingElement("point"))
		{
			USVec2D point;

			pointElem->Attribute("x", &point.mX);
			pointElem->Attribute("y", &point.mY);

			polygon.points.push_back(point);
		}

		navmesh.polygons.push_back(polygon);
	}

	// Links
	TiXmlHandle hLinks = hRoot.FirstChildElement("links");

	TiXmlElement* linkElem = hLinks.FirstChildElement("link").Element();
	for (linkElem; linkElem; linkElem = linkElem->NextSiblingElement("link"))
	{
		Link link;

		TiXmlElement* startElem = linkElem->FirstChildElement("start");
		LinkPoint start;
		startElem->Attribute("polygon", &start.polygon);
		startElem->Attribute("edgestart", &start.edgestart);
		startElem->Attribute("edgeend", &start.edgeend);

		TiXmlElement* endElem = linkElem->FirstChildElement("end");
		LinkPoint end;
		endElem->Attribute("polygon", &end.polygon);
		endElem->Attribute("edgestart", &end.edgestart);
		endElem->Attribute("edgeend", &end.edgeend);

		link.start = start;
		link.end = end;

		navmesh.links.push_back(link);
	}

    return true;
}