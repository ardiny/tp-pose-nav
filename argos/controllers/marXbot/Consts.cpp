#include "Consts.h"

// Constants table to convert a direction to a pair of dx/dy
const int dirToDxDy4[4][2] =
{
	{  1,  0},
	{  0,  1},
	{ -1,  0},
	{  0, -1}
};

const int dirToDxDy8[8][2] =
{
	{  1,  0},
	{  1,  1},
	{  0,  1},
	{ -1,  1},
	{ -1,  0},
	{ -1, -1},
	{  0, -1},
	{  1, -1}
};

