/*
 * StablePlanarQuadGrid.h
 *
 *  Created on: Aug 4, 2011
 *      Author: papazov
 */

#ifndef _STABLE_PLANAR_QUADGRID_H_
#define _STABLE_PLANAR_QUADGRID_H_

#include "../DataStructures/EdgeStructure.h"


class StablePlanarQuadGrid
{
public:
	StablePlanarQuadGrid();
	virtual ~StablePlanarQuadGrid();

	/** Not the most eficient implementation! It is meant to be used for small test cases anyway! */
	void generate(int numOfNodes[2], const double* bounds, EdgeStructure* out);

	inline void addToNeighborhood(int i, int j, int w, int h, EdgeStructure* out);
	inline bool isValid2dIndex(int i, int j, int w, int h);
	inline int toLinearId(int i, int j, int h);
};

//=== inline methods ===========================================================================================================

inline bool StablePlanarQuadGrid::isValid2dIndex(int i, int j, int w, int h)
{
	if ( 0 <= i && i < w && 0 <= j && j < h )
		return true;

	return false;
}

//==============================================================================================================================

inline int StablePlanarQuadGrid::toLinearId(int i, int j, int h)
{
	return i*h + j;
}

//==============================================================================================================================

inline void StablePlanarQuadGrid::addToNeighborhood(int i, int j, int w, int h, EdgeStructure* out)
{
	// The linear node index
	int k = this->toLinearId(i, j, h);

	// the inner part
	if ( (0 < i && i < w-1)  &&  (0 < j && j < h-1) )
	{
		out->addToNeighborhood(k, k - h - 1); // lower left
		out->addToNeighborhood(k, k - h); // left
		out->addToNeighborhood(k, k - h + 1); // upper left
		out->addToNeighborhood(k, k + 1); // up
		out->addToNeighborhood(k, k + h + 1); // upper right
		out->addToNeighborhood(k, k + h); // right
		out->addToNeighborhood(k, k + h - 1); // lower right
		out->addToNeighborhood(k, k - 1); // low
	}
	// the first column
	else if ( i == 0 && (0 < j && j < h-1) )
	{
		out->addToNeighborhood(k, k + 1); // up
		out->addToNeighborhood(k, k + h + 1); // upper right
		out->addToNeighborhood(k, k + h); // right
		out->addToNeighborhood(k, k + h - 1); // lower right
		out->addToNeighborhood(k, k - 1); // low
	}
	// the last column
	else if ( i == w-1 && (0 < j && j < h-1) )
	{
		out->addToNeighborhood(k, k - 1); // low
		out->addToNeighborhood(k, k - h - 1); // lower left
		out->addToNeighborhood(k, k - h); // left
		out->addToNeighborhood(k, k - h + 1); // upper left
		out->addToNeighborhood(k, k + 1); // up
	}
	// the first row
	else if ( (0 < i && i < w-1) && j == 0 )
	{
		out->addToNeighborhood(k, k - h); // left
		out->addToNeighborhood(k, k - h + 1); // upper left
		out->addToNeighborhood(k, k + 1); // up
		out->addToNeighborhood(k, k + h + 1); // upper right
		out->addToNeighborhood(k, k + h); // right
	}
	// the last row
	else if ( (0 < i && i < w-1) && j == h-1 )
	{
		out->addToNeighborhood(k, k - h); // left
		out->addToNeighborhood(k, k - h - 1); // lower left
		out->addToNeighborhood(k, k - 1); // low
		out->addToNeighborhood(k, k + h - 1); // lower right
		out->addToNeighborhood(k, k + h); // right
	}
	// upper left corner
	else if ( i == 0 && j == h-1 )
	{
		out->addToNeighborhood(k, k + h); // right
		out->addToNeighborhood(k, k + h - 1); // lower right
		out->addToNeighborhood(k, k - 1); // low
	}
	// upper right corner
	else if ( i == w-1 && j == h-1 )
	{
		out->addToNeighborhood(k, k - h); // left
		out->addToNeighborhood(k, k - h - 1); // lower left
		out->addToNeighborhood(k, k - 1); // low
	}
	// lower right corner
	else if ( i == w-1 && j == 0 )
	{
		out->addToNeighborhood(k, k - h); // left
		out->addToNeighborhood(k, k - h + 1); // upper left
		out->addToNeighborhood(k, k + 1); // up
	}
	// lower left corner
	else if ( i == 0 && j == 0 )
	{
		out->addToNeighborhood(k, k + 1); // up
		out->addToNeighborhood(k, k + h + 1); // upper right
		out->addToNeighborhood(k, k + h); // right
	}

//	return;

	// The secondary edges
	if ( this->isValid2dIndex(i-2, j,   w, h) ) out->addToNeighborhood(k, this->toLinearId(i-2, j,   h)); // left
	if ( this->isValid2dIndex(i,   j+2, w, h) ) out->addToNeighborhood(k, this->toLinearId(i,   j+2, h)); // up
	if ( this->isValid2dIndex(i+2, j,   w, h) ) out->addToNeighborhood(k, this->toLinearId(i+2, j,   h)); // right
	if ( this->isValid2dIndex(i,   j-2, w, h) ) out->addToNeighborhood(k, this->toLinearId(i,   j-2, h)); // down
}

//==============================================================================================================================

#endif /* _STABLE_PLANAR_QUADGRID_H_ */
