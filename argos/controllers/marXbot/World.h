#ifndef __WORLD_H
#define __WORLD_H

#include "enki/Geometry.h"
#include "enki/PhysicalEngine2.h"
#include "Consts.h"

class Map;

// Simulated world, used to create map and run the simulated part of the TP
class World: public Enki::World
{
public:
	
	
public:
	World();
	void buildMaps(Map *map);
	
private:
	// walls in simulated world
	Enki::Polygone wallPolygone0;
	Enki::Polygone wallPolygone1;
	Enki::Polygone wallPolygone2;
	Enki::Polygone wallPolygone3;
};

#endif
