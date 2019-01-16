#include <stdafx.h>
#include <tinyxml.h>
#include "params.h"

bool ReadParams(const char* filename, Params& params)
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
    TiXmlHandle hParams = hRoot.FirstChildElement("params");

    //TiXmlElement* paramElem = hParams.FirstChild().Element();
    //for (paramElem; paramElem; paramElem = paramElem->NextSiblingElement())
    //{
    //    const char* paramName = paramElem->Value();
    //    if (!strcmp(paramName, "max_velocity"))
    //    {
    //        const char* elemText = paramElem->GetText();
    //        params.max_velocity = static_cast<float>(atof(elemText));
    //    }
    //}

	// Position
    TiXmlElement* paramElem = hParams.FirstChildElement("max_velocity").Element();
    if (paramElem)
        paramElem->Attribute("value", &params.max_velocity);

    paramElem = hParams.FirstChildElement("max_acceleration").Element();
    if (paramElem)
        paramElem->Attribute("value", &params.max_acceleration);

    paramElem = hParams.FirstChildElement("dest_radius").Element();
    if (paramElem)
        paramElem->Attribute("value", &params.dest_radius);

	paramElem = hParams.FirstChildElement("arrive_radius").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.arrive_radius);

	paramElem = hParams.FirstChildElement("targetPosition").Element();
	if (paramElem)
	{
		paramElem->Attribute("x", &params.targetPosition.mX);
		paramElem->Attribute("y", &params.targetPosition.mY);
	}

	// Rotation
	paramElem = hParams.FirstChildElement("max_angular_velocity").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.max_angular_velocity);

	paramElem = hParams.FirstChildElement("max_angular_acceleration").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.max_angular_acceleration);

	paramElem = hParams.FirstChildElement("angular_dest_radius").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.angular_dest_radius);

	paramElem = hParams.FirstChildElement("angular_arrive_radius").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.angular_arrive_radius);

	paramElem = hParams.FirstChildElement("targetRotation").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.targetRotation);


	// Enemy
	paramElem = hParams.FirstChildElement("enemy_speed").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.enemy_speed);

	paramElem = hParams.FirstChildElement("enemy_minPosition").Element();
	if (paramElem)
	{
		paramElem->Attribute("x", &params.enemy_minPosition.mX);
		paramElem->Attribute("y", &params.enemy_minPosition.mY);
	}

	paramElem = hParams.FirstChildElement("enemy_maxPosition").Element();
	if (paramElem)
	{
		paramElem->Attribute("x", &params.enemy_maxPosition.mX);
		paramElem->Attribute("y", &params.enemy_maxPosition.mY);
	}


	// Path following
	paramElem = hParams.FirstChildElement("look_ahead").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.look_ahead);

	paramElem = hParams.FirstChildElement("time_ahead").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.time_ahead);

	// Obstacle avoiding
	paramElem = hParams.FirstChildElement("char_radius").Element();
	if (paramElem)
		paramElem->Attribute("value", &params.char_radius);

    return true;
}