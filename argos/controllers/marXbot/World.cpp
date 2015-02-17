#include "World.h"
#include "enki/PhysicalEngine2.h"
//#include "Map.h"
#include <algorithm>

using Enki::Point;
using Enki::Segment;
using Enki::Polygone;
using Enki::PhysicalObject;
using Enki::Color;/**/
//using Enki::EPuck;

//World::World2():World2()
	//Enki::World()
//{};
World::World() :
	Enki::World(){};
	
	const Color wallColor(223. / 255., 210. / 255. , 162./255.);
	
	// add walls
/*	PhysicalObject* wall0 = new PhysicalObject;
	wall0->pos = Point(20, 14);
	wall0->setRectangular(14, 2, 10, -1);
	wall0->setColor(wallColor);
	addObject(wall0);
	
	PhysicalObject* wall1 = new PhysicalObject;
	wall1->pos = Point(20, 49);
	wall1->setRectangular(14, 2.1, 10, -1);
	wall1->setColor(wallColor);
	addObject(wall1);
	
	PhysicalObject* wall2 = new PhysicalObject;
	wall2->pos = Point(28, 31);
	wall2->setRectangular(14, 2, 10, -1);
	wall2->setColor(wallColor);
	addObject(wall2);
	
	PhysicalObject* wall3 = new PhysicalObject;
	wall3->pos = Point(34, 40);
	wall3->setRectangular(16, 20.1, 10, -1);
	wall3->setColor(wallColor);
	addObject(wall3);
	
	PhysicalObject* wall4 = new PhysicalObject;
	wall4->pos = Point(61, 60);
	wall4->setRectangular(6, 8, 10, -1);
	wall4->setColor(wallColor);
	addObject(wall4);
	
	PhysicalObject* wall5 = new PhysicalObject;
	wall5->pos = Point(26, 7);
	wall5->setRectangular(2, 14, 10, -1);
	wall5->setColor(wallColor);
	addObject(wall5);

}

void World::buildMaps(Map *map)
{
	// one interaction step to get true bounding surfaces
		
	// build localisation map
	for (int x = 0; x < MAP_COUNT; x++)
		for (int y = 0; y < MAP_COUNT; y++)
		{
			// get map case center coordinates
			Point p = Point(((double)x + 0.5) * MAP_RES + 0.001, ((double)y + 0.5) * MAP_RES + 0.001);
			
			// check if we are inside obstacle
			bool isInObstacle = false;
			bool isWall = false;
			for (Enki::World::ObjectsIterator i = objects.begin(); i != objects.end(); ++i)
			{
				PhysicalObject* o = *i;
				
				if (!o->isCylindric())
				{
					const Polygone bs = o->getHull()[0].getTransformedShape();
					
					// check obstacle
					if (bs.isPointInside(p))
					{
						isInObstacle = true;
						break;
					}
					
					// check wall
					for (size_t j = 0; j < bs.size(); j++)
					{
						Segment s(bs[j], bs[(j + 1) % bs.size()]);
						isWall |= s.doesIntersect(Segment(x*MAP_RES, y*MAP_RES, (x+1)*MAP_RES, y*MAP_RES));
						isWall |= s.doesIntersect(Segment((x+1)*MAP_RES, y*MAP_RES, (x+1)*MAP_RES, (y+1)*MAP_RES));
						isWall |= s.doesIntersect(Segment((x+1)*MAP_RES, (y+1)*MAP_RES, x*MAP_RES, (y+1)*MAP_RES));
						isWall |= s.doesIntersect(Segment(x*MAP_RES, (y+1)*MAP_RES, x*MAP_RES, y*MAP_RES));
					}
				}
			}
			
			// if so, mark as unaccessible and continue on next cell
			if (isInObstacle)
			{
				map->gradientMap[x][y] = Map::UNACCESSIBLE_CELL;
				continue;
			}
			else if (isWall)
			{
				map->gradientMap[x][y] = Map::CELL_WITH_WALL;
			}
			else
				map->gradientMap[x][y] = Map::SMALLEST_GRADIENT;
			
			// scan cell
			//epuck->pos = p;
			step(0.01);
			//GetEPuckScan(epuck, map->worldMap[x][y]);
			/*for (int i = 0; i < HIST_COUNT; i++)
			{
				int dist = 0;
				for (int j = 0; j < HIST_RES; j++)
					dist += (int)(epuck->scannerTurret.zbuffer[i*HIST_RES + j] / 8);
				map->worldMap[x][y][i] = dist;
			}*/
		//}
	
	// replace e-puck at the initial point of the arena
	//epuck->pos = Point(((double)map->mostProbX + 0.5) * MAP_RES + 0.001, ((double)map->mostProbY + 0.5) * MAP_RES + 0.001);
	//exit(1);
	
	// extend walls by robot size
	// build temporary map
	/*bool toBecomeWallMap[MAP_COUNT][MAP_COUNT];
	for (int x = 0; x < MAP_COUNT; x++)
		for (int y = 0; y < MAP_COUNT; y++)
			toBecomeWallMap[x][y] = false;
	// compute walls extension
	for (int x = 0; x < MAP_COUNT; x++)
		for (int y = 0; y < MAP_COUNT; y++)
		{
			if (map->gradientMap[x][y] == Map::UNACCESSIBLE_CELL)
				continue;
			
			bool isWall = map->gradientMap[x][y] == Map::CELL_WITH_WALL;
			
			// find if any obstacle around
			for (int i = 0; i < 8; i++)
			{
				int lookX = x + dirToDxDy8[i][0];
				int lookY = y + dirToDxDy8[i][1];
				if ((lookX >= 0) && (lookX < MAP_COUNT) && (lookY >= 0) && (lookY < MAP_COUNT))
					isWall |= map->gradientMap[lookX][lookY] <= Map::CELL_WITH_WALL;
			}
			
			toBecomeWallMap[x][y] = isWall;
		}
	// apply walls extension
	for (int x = 0; x < MAP_COUNT; x++)
		for (int y = 0; y < MAP_COUNT; y++)
			if (toBecomeWallMap[x][y])
				map->gradientMap[x][y] = std::min(map->gradientMap[x][y], (int)Map::CELL_WITH_WALL);
	
	// arena borders count as walls
	for (int x = 0; x < MAP_COUNT; x++)
	{
		map->gradientMap[x][0] = std::min(map->gradientMap[x][0], (int)Map::CELL_WITH_WALL);
		map->gradientMap[x][MAP_COUNT - 1] = std::min(map->gradientMap[x][MAP_COUNT - 1], (int)Map::CELL_WITH_WALL);
	}
	for (int y = 0; y < MAP_COUNT; y++)
	{
		map->gradientMap[0][y] = std::min(map->gradientMap[0][y], (int)Map::CELL_WITH_WALL);
		map->gradientMap[MAP_COUNT - 1][y] = std::min(map->gradientMap[MAP_COUNT - 1][y], (int)Map::CELL_WITH_WALL);
	}
}*/

