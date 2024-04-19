/*
 * StablePlanarQuadGrid.cpp
 *
 *  Created on: Aug 4, 2011
 *      Author: papazov
 */

#include "StablePlanarQuadGrid.h"

using namespace btl1;

StablePlanarQuadGrid::StablePlanarQuadGrid()
{
}

StablePlanarQuadGrid::~StablePlanarQuadGrid()
{
}

//======================================================================================================================

void StablePlanarQuadGrid::generate(int numOfNodes[2], const double* bounds, EdgeStructure* out)
{
	if ( numOfNodes[0] <= 1 || numOfNodes[1] <= 1 )
		return;

	double step_x = (bounds[1]-bounds[0])/(double)(numOfNodes[0]-1);
	double step_y = (bounds[3]-bounds[2])/(double)(numOfNodes[1]-1);
	double p[3], *points;
	int i, j;

	// Initialize
	out->alloc(numOfNodes[0]*numOfNodes[1]);
	points = out->getPoints();
	p[2] = 0.0;

	// Setup the geometry and the neighborhood structure
	for ( i = 0, p[0] = bounds[0] ; i < numOfNodes[0] ; ++i, p[0] += step_x )
		for ( j = 0, p[1] = bounds[2] ; j < numOfNodes[1] ; ++j, p[1] += step_y )
		{
			vec_copy3(p, points);
			this->addToNeighborhood(i, j, numOfNodes[0], numOfNodes[1], out);
			// Go to the next point
			points += 3;
		}

	int w = numOfNodes[0], h = numOfNodes[1];
	printf("StablePlanarQuadGrid::%s(): there should be %i edge(s).\n", __func__,
			w*(2*h-3) + h*(2*w-3) + 2*(w-1)*(h-1));
}

//======================================================================================================================
