/*
 * Array3d2vtk.cpp
 *
 *  Created on: Jun 11, 2010
 *      Author: papazov
 */

#include "Array3d2vtk.h"
#include "Box2vtk.h"
#include <list>

using namespace std;

Array3d2vtk::Array3d2vtk()
{
}

Array3d2vtk::~Array3d2vtk()
{
}

//==============================================================================================================

void Array3d2vtk::getNonZerosAsPolygons(Array3d<char>* array, double begin[3], double step, vtkPolyData* out)
{
	double p[3]; int x, y, z;
	list<Box> boxes;

	for ( x = 0, p[0] = begin[0] ; x < array->width() ; ++x, p[0] += step )
		for ( y = 0, p[1] = begin[1]  ; y < array->height() ; ++y, p[1] += step )
			for ( z = 0, p[2] = begin[2]  ; z < array->depth() ; ++z, p[2] += step )
				if ( array->mData[x][y][z] != 0 )
				{
					Box box(3);
					  box.set(0, p[0], p[0] + step);
					  box.set(1, p[1], p[1] + step);
					  box.set(2, p[2], p[2] + step);
					  boxes.push_back(box);
				}

	Box2vtk box2vtk;
	  box2vtk.getOutputAsPolygons(boxes, out);
}

//==============================================================================================================
