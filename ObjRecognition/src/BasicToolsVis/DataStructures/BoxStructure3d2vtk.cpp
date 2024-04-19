/*
 * BoxStructure3d2vtk.cpp
 *
 *  Created on: Mar 31, 2010
 *      Author: papazov
 */

#include "BoxStructure3d2vtk.h"
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>

using namespace tum;

BoxStructure3d2vtk::BoxStructure3d2vtk()
{
}

BoxStructure3d2vtk::~BoxStructure3d2vtk()
{
}

//=========================================================================================================================

void BoxStructure3d2vtk::getAsWireframe(const BoxStructure3d<class T>* boxStruct, vtkPolyData* out)
{
	// Some variables
	vtkCellArray* lines = vtkCellArray::New();
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkIdType ids[2];
	int i, j, numOfBoxes[3];
	double x, y, z, boxLengths[3];
	Matrix bounds(3, 2);

	// Get some info
	boxStruct->getBoxLengths(boxLengths);
	boxStruct->getNumberOfBoxes(numOfBoxes);
	boxStruct->getBounds(bounds.m);
	ids[0] = 0;

	// Insert the lines parallel to the yz-plane
	x = bounds.m[0][0];
	for ( i = 0 ; i <= numOfBoxes[0] ; ++i, x += boxLengths[0] )
	{
		for ( j = 0, z = bounds.m[2][0] ; j <= numOfBoxes[2] ; ++j, z += boxLengths[2], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(x, bounds.m[1][0], z);
			points->InsertNextPoint(x, bounds.m[1][1], z);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}
	// Insert the lines parallel to the xz-plane
	y = bounds.m[1][0];
	for ( i = 0 ; i <= numOfBoxes[1] ; ++i, y += boxLengths[1] )
	{
		for ( j = 0, z = bounds.m[2][0] ; j <= numOfBoxes[2] ; ++j, z += boxLengths[2], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(bounds.m[0][0], y, z);
			points->InsertNextPoint(bounds.m[0][1], y, z);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}
	// Insert the planes parallel to the xy-plane
	x = bounds.m[0][0];
	for ( i = 0 ; i <= numOfBoxes[0] ; ++i, x += boxLengths[0] )
	{
		for ( j = 0, y = bounds.m[1][0] ; j <= numOfBoxes[1] ; ++j, y += boxLengths[1], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(x, y, bounds.m[2][0]);
			points->InsertNextPoint(x, y, bounds.m[2][1]);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}

	// Associate the points and lines with the vtk poly object
	out->SetPoints(points);
	out->SetLines(lines);

	// Clean up
	points->Delete();
	lines->Delete();
}

//=========================================================================================================================
