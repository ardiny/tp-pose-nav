#ifndef __MAP_H
#define __MAP_H
#include "Consts.h"

// Map, contain the representation of the environment from the perspective of the robot
class Map
{
public:
	// map of histograms for each cell
	int worldMap[MAP_COUNT][MAP_COUNT][HIST_COUNT];
	// gradient value for each cell
	int gradientMap[MAP_COUNT][MAP_COUNT];
	
	// probability map (computed for robot histogram and worldMap)
	int probMap[MAP_COUNT][MAP_COUNT];
	// maximum likelyhood estimator of position and orientation
	double mostProbX, mostProbY, mostProbDir;
	
	// special values of gradient to store in gradientMap
	enum GradientMapSpecialValues
	{
		UNACCESSIBLE_CELL = 0,
		CELL_WITH_WALL = 1,
		Obstacle=2,
		SMALLEST_GRADIENT = 3
	};
	
public:
	Map();
	int test3();
	void computeGradientFromDestination(int destX, int destY);
	private:
	void updateGradientCell(int cellX, int cellY);
};

#endif
