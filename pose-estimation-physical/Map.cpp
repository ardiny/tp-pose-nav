#include "Map.h"
#include <algorithm>
#include <iostream>
using namespace std;
//Map::Map() {};
int MAP_COUNTX=(int)(arenalon/arenares+1);
int MAP_COUNTY=(int)(arenawidth/arenares+1);
Map::Map() :
	mostProbX(10),
	mostProbY(2),
	mostProbDir(0)
{
	// init prob map
	for (int x = 1; x < MAP_COUNTX; x++)
		for (int y = 1; y < MAP_COUNTY; y++)
			probMap[x][y] = 0;
}/**/

void Map::computeGradientFromDestination(int destX, int destY)
{
	// clear map
//cout<<MAP_COUNTX<<"---"<<MAP_COUNTY;
	for (int x = 0; x < MAP_COUNTX; x++)
		for (int y = 0; y < MAP_COUNTY; y++)
			if (gradientMap[x][y] >= SMALLEST_GRADIENT){
				gradientMap[x][y] = SMALLEST_GRADIENT;
				}
	// destination is biggest gradient
	gradientMap[destX][destY] = 255;
	
	// update gradient
	for (int x = std::max(0, destX-1); x < std::min(destX+2, MAP_COUNTX); x++)
		for (int y = std::max(0, destY-1); y < std::min(destY+2, MAP_COUNTY); y++)
			if ((!((x == destX) && (y == destY))) &&(gradientMap[x][y] >= SMALLEST_GRADIENT)
){
				updateGradientCell(x, y);/**/
			
			}
}

void Map::updateGradientCell(int cellX, int cellY)
{
	int maxValue = 0;
	
	// scan around
	for (int i = 0; i < 4; i++)
	{
		int lookX = cellX + dirToDxDy4[i][0];
		int lookY = cellY + dirToDxDy4[i][1];
		if ((lookX >= 0) && (lookX < MAP_COUNTX) && (lookY >= 0) && (lookY < MAP_COUNTY))
			maxValue = std::max(maxValue, gradientMap[lookX][lookY]);
	}
	
	if (gradientMap[cellX][cellY] >= maxValue)
		return;
	
	maxValue--;
	gradientMap[cellX][cellY] = maxValue;
	
	// propagate gradient
	for (int i = 0; i < 4; i++)
	{
		int lookX = cellX + dirToDxDy4[i][0];
		int lookY = cellY + dirToDxDy4[i][1];
		if (((lookX >= 0) && (lookX < MAP_COUNTX) && (lookY >= 0) && (lookY < MAP_COUNTY)) &&
				(gradientMap[lookX][lookY] + 1 < maxValue) &&
				(gradientMap[lookX][lookY] >= SMALLEST_GRADIENT)
			)
			updateGradientCell(lookX, lookY);
	}
}

