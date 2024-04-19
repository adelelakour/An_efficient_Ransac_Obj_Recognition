/*
 * VectorField32vtk.cpp
 *
 *  Created on: Sep 13, 2010
 *      Author: papazov
 */

#include "VectorField32vtk.h"
#include <BasicTools/LinearAlgebra/Vector.h>
#include <BasicTools/Stochastics/RandomGenerator.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

using namespace tum;

VectorField32vtk::VectorField32vtk()
{
}

VectorField32vtk::~VectorField32vtk()
{
}

//===========================================================================================================================

void VectorField32vtk::getVectorsAtRandom(VectorField3* field, int numOfVectors, const double box[6], vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* lines = vtkCellArray::New();
	vtkIdType ids[2] = {0, 1};
	RandomGenerator randgen;
	double x[3], y[3];

	for ( int i = 0 ; i < numOfVectors ; ++i, ids[0] += 2, ids[1] += 2 )
	{
		randgen.getRandom3dPointWithinBox(box, x);
		points->InsertNextPoint(x);
		field->F(x, y);
		Vector::add3(x, y);
		points->InsertNextPoint(x);
		lines->InsertNextCell(2, ids);
	}

	out->SetPoints(points);
	out->SetLines(lines);
	// Cleanup
	points->Delete();
	lines->Delete();
}

//===========================================================================================================================

void VectorField32vtk::getVectorsAtRandom(VectorField3* field, int numOfVectors,
		const double box[6], double maxVecLen, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* lines = vtkCellArray::New();
	vtkIdType ids[2] = {0, 1};
	RandomGenerator randgen;
	double x[3], y[3];

	for ( int i = 0 ; i < numOfVectors ; )
	{
		randgen.getRandom3dPointWithinBox(box, x);
		field->F(x, y);
		if ( Vector::length3(y) > maxVecLen )
			continue;
		points->InsertNextPoint(x);
		Vector::add3(x, y);
		points->InsertNextPoint(x);
		lines->InsertNextCell(2, ids);
		// For the next step
		++i; ids[0] += 2; ids[1] += 2;
	}

	out->SetPoints(points);
	out->SetLines(lines);
	// Cleanup
	points->Delete();
	lines->Delete();
}

//===========================================================================================================================
