/*
 * ScalarField2vtk.cpp
 *
 *  Created on: Mar 31, 2010
 *      Author: papazov
 */

#include "ScalarField2vtk.h"
#include <BasicTools/DataStructures/Box.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <cstdio>

ScalarField2vtk::ScalarField2vtk()
{
}

ScalarField2vtk::~ScalarField2vtk()
{
}

//===================================================================================================================

void ScalarField2vtk::get2dScalarFieldAsTriangularMesh(ScalarField* sf, const double** bounds,
		const double* spacing, vtkPolyData* out, double zfactor)
{
	double p[3];
	int i, j, dims[2];

	dims[0] = (int)((bounds[0][1] - bounds[0][0])/spacing[0] + 1.5);
	dims[1] = (int)((bounds[1][1] - bounds[1][0])/spacing[1] + 1.5);

	vtkPoints* points = vtkPoints::New();
	vtkCellArray* cells = vtkCellArray::New();

	// Compute surface points
	for ( j = 0, p[1] = bounds[1][0] ; j < dims[1] ; ++j, p[1] += spacing[1] )
	{
		for ( i = 0, p[0] = bounds[0][0] ; i < dims[0] ; ++i, p[0] += spacing[0] )
		{
			p[2] = zfactor*sf->f(p);
			points->InsertNextPoint(p);
		}
	}

	vtkIdType LL, LR, UL, ids[3];
	// Compute surface cells (polygons between the points)
	for ( j = 0 ; j < dims[1]-1 ; ++j )
	{
		for ( i = 0 ; i < dims[0]-1 ; ++i )
		{
			// First triangle
			ids[0] = LL = j*dims[0] + i; // Lower left corner
			ids[1] = LR = LL + 1; // Lower right
			ids[2] = UL = LL + dims[0]; // Upper left
			cells->InsertNextCell(3, ids);
			// Second triangle
			ids[0] = LR;
			ids[1] = UL + 1; // Upper right
			ids[2] = UL;
			cells->InsertNextCell(3, ids);
		}
	}

	out->SetPoints(points);
	out->SetPolys(cells);

	// Clean up
	points->Delete();
	cells->Delete();
}

//===================================================================================================================

void ScalarField2vtk::get2dScalarFieldAsWireframe(ScalarField* sf, const double** bounds,
		const double* spacing, vtkPolyData* out, double zfactor)
{
	double p[3];
	int i, j, dims[2];

	dims[0] = (int)((bounds[0][1] - bounds[0][0])/spacing[0] + 1.5);
	dims[1] = (int)((bounds[1][1] - bounds[1][0])/spacing[1] + 1.5);

	vtkPoints* points = vtkPoints::New();
	vtkCellArray* lines = vtkCellArray::New();

	// Compute surface points
	for ( j = 0, p[1] = bounds[1][0] ; j < dims[1] ; ++j, p[1] += spacing[1] )
	{
		for ( i = 0, p[0] = bounds[0][0] ; i < dims[0] ; ++i, p[0] += spacing[0] )
		{
			p[2] = zfactor*sf->f(p);
			points->InsertNextPoint(p);
		}
	}

	vtkIdType ids[2];
	// Compute lines between the points
	for ( j = 0 ; j < dims[1]-1 ; ++j )
	{
		for ( i = 0 ; i < dims[0]-1 ; ++i )
		{
			// Compose the lines which build the wireframe
			ids[0] = j*dims[0] + i;
			ids[1] = ids[0]+1;
			lines->InsertNextCell(2, ids);
			ids[1] = (j+1)*dims[0] + i;
			lines->InsertNextCell(2, ids);
		}
	}

	out->SetPoints(points);
	out->SetLines(lines);

	// Clean up
	points->Delete();
	lines->Delete();
}

//===================================================================================================================

void ScalarField2vtk::get2dFunctionAsWireframe(double (*func)(double x, double y), const double** bounds,
		const double* spacing, vtkPolyData* out, double zfactor)
{
	double p[3];
	int i, j, dims[2];

	dims[0] = (int)((bounds[0][1] - bounds[0][0])/spacing[0] + 0.5);
	dims[1] = (int)((bounds[1][1] - bounds[1][0])/spacing[1] + 0.5);

	vtkPoints* points = vtkPoints::New();
	vtkCellArray* lines = vtkCellArray::New();

	// Compute surface points
	for ( j = 0, p[1] = bounds[1][0] ; j < dims[1] ; ++j, p[1] += spacing[1] )
	{
		for ( i = 0, p[0] = bounds[0][0] ; i < dims[0] ; ++i, p[0] += spacing[0] )
		{
			p[2] = zfactor*func(p[0], p[1]);
			points->InsertNextPoint(p);
		}
	}

	vtkIdType ids[2];
	// Compute lines between the points
	for ( j = 0 ; j < dims[1]-1 ; ++j )
	{
		for ( i = 0 ; i < dims[0]-1 ; ++i )
		{
			// Compose the lines which build the wireframe
			ids[0] = j*dims[0] + i;
			ids[1] = ids[0]+1;
			lines->InsertNextCell(2, ids);
			ids[1] = (j+1)*dims[0] + i;
			lines->InsertNextCell(2, ids);
		}
	}

	out->SetPoints(points);
	out->SetLines(lines);

	// Clean up
	points->Delete();
	lines->Delete();
}

//===================================================================================================================

void ScalarField2vtk::get3dScalarFieldAs3dImage(ScalarField* f, const double* bounds, double* spacing,
		vtkImageData* out)
{
	Box b(3);
	  b.set(0, bounds[0], bounds[1]);
	  b.set(1, bounds[2], bounds[3]);
	  b.set(2, bounds[4], bounds[5]);
	this->get3dScalarFieldAs3dImage(f, b.getIntervals(), spacing, out);
}

//===================================================================================================================

void ScalarField2vtk::get3dScalarFieldAs3dImage(ScalarField* f, const double** bounds, double* spacing,
		vtkImageData* out)
{
	if ( f->getNumberOfVariables() != 3 )
	{
		fprintf(stderr, "ERROR in 'ScalarField2vtk::%s()': the scalar field is not 3d but %id.\n",
				__func__, f->getNumberOfVariables());
		fflush(stderr);
		return;
	}

	int i, j, k, offset, jOffset, kOffset, dims[3];
	double x[3], start[3];

	start[0] = bounds[0][0] + 0.5*spacing[0];
	start[1] = bounds[1][0] + 0.5*spacing[1];
	start[2] = bounds[2][0] + 0.5*spacing[2];

	// Set the dimensions of the data set
	dims[0] = (int)((bounds[0][1] - bounds[0][0])/spacing[0] + 0.5);
	dims[1] = (int)((bounds[1][1] - bounds[1][0])/spacing[1] + 0.5);
	dims[2] = (int)((bounds[2][1] - bounds[2][0])/spacing[2] + 0.5);

	vtkDoubleArray *scalars = vtkDoubleArray::New();
	  scalars->SetNumberOfTuples(dims[0]*dims[1]*dims[2]);
	  scalars->SetNumberOfComponents(1);

	printf("ScalarField2vtk::%s(): sampling %i x %i x %i points (in total %i points) ... ",
			__func__, dims[0], dims[1], dims[2], dims[0]*dims[1]*dims[2]); fflush(stdout);
	for ( k = 0, x[2] = start[2] ; k < dims[2] ; ++k, x[2] += spacing[2] )
	{
		kOffset = k*dims[0]*dims[1];
		for ( j = 0, x[1] = start[1] ; j < dims[1] ; ++j, x[1] += spacing[1] )
		{
			jOffset = j*dims[0];
			for ( i = 0, x[0] = start[0] ; i < dims[0] ; ++i, x[0] += spacing[0] )
			{
				offset = i + jOffset + kOffset;
				scalars->SetTuple1(offset, f->f(x));
			}
		}
	}
	printf("done.\n"); fflush(stdout);

	double origin[3] = {bounds[0][0], bounds[1][0], bounds[2][0]};

	out->SetScalarTypeToDouble();
	out->SetDimensions(dims);
	out->SetSpacing(spacing);
	out->SetOrigin(origin);
	out->GetPointData()->SetScalars(scalars);

	// Clean up
	scalars->Delete();
}

//===================================================================================================================
