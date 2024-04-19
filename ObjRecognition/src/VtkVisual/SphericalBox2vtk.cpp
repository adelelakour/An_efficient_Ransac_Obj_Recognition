/*
 * SphericalBox2vtk.cpp
 *
 *  Created on: Apr 9, 2010
 *      Author: papazov
 */

#include "SphericalBox2vtk.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>

SphericalBox2vtk::SphericalBox2vtk()
{
}

SphericalBox2vtk::~SphericalBox2vtk()
{
}

//==================================================================================================================================

void SphericalBox2vtk::getAsWireframe(const double** range, int resolution_phi, int resolution_psi, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();

	int i;
	vtkIdType ids[2];
	double phi, psi, r, step, p[3];
	double rSinPsi, rCosPhi, rSinPhi;

	// Phi step
	step = (range[0][1] - range[0][0])/(double)(resolution_phi-1);

	// Fix psi and r
	psi = range[1][0];
	r = range[2][0];
	// Compute other stuff
	rSinPsi = r*sin(psi);
	p[2] = r*cos(psi);
	// Insert the points
	for ( i = 0, phi = range[0][0] ; i < resolution_phi ; ++i, phi += step )
	{
		p[0] = cos(phi)*rSinPsi;
		p[1] = sin(phi)*rSinPsi;
		points->InsertNextPoint(p);
	}
	// Insert the lines
	ids[0] = 0; ids[1] = 1;
	for ( i = 1 ; i < resolution_phi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix psi and r
	psi = range[1][1];
	r = range[2][0];
	// Compute other stuff
	rSinPsi = r*sin(psi);
	p[2] = r*cos(psi);
	// Insert the points
	for ( i = 0, phi = range[0][0] ; i < resolution_phi ; ++i, phi += step )
	{
		p[0] = cos(phi)*rSinPsi;
		p[1] = sin(phi)*rSinPsi;
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_phi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix psi and r
	psi = range[1][0];
	r = range[2][1];
	// Compute other stuff
	rSinPsi = r*sin(psi);
	p[2] = r*cos(psi);
	// Insert the points
	for ( i = 0, phi = range[0][0] ; i < resolution_phi ; ++i, phi += step )
	{
		p[0] = cos(phi)*rSinPsi;
		p[1] = sin(phi)*rSinPsi;
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_phi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix psi and r
	psi = range[1][1];
	r = range[2][1];
	// Compute other stuff
	rSinPsi = r*sin(psi);
	p[2] = r*cos(psi);
	// Insert the points
	for ( i = 0, phi = range[0][0] ; i < resolution_phi ; ++i, phi += step )
	{
		p[0] = cos(phi)*rSinPsi;
		p[1] = sin(phi)*rSinPsi;
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_phi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Psi step
	step = (range[1][1] - range[1][0])/(double)(resolution_psi-1);

	// Fix phi and r
	phi = range[0][0];
	r = range[2][0];
	// Compute other stuff
	rCosPhi = r*cos(phi);
	rSinPhi = r*sin(phi);
	// Insert the points
	for ( i = 0, psi = range[1][0] ; i < resolution_psi ; ++i, psi += step )
	{
		p[0] = sin(psi)*rCosPhi;
		p[1] = sin(psi)*rSinPhi;
		p[2] = r*cos(psi);
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_psi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix phi and r
	phi = range[0][1];
	r = range[2][0];
	// Compute other stuff
	rCosPhi = r*cos(phi);
	rSinPhi = r*sin(phi);
	// Insert the points
	for ( i = 0, psi = range[1][0] ; i < resolution_psi ; ++i, psi += step )
	{
		p[0] = sin(psi)*rCosPhi;
		p[1] = sin(psi)*rSinPhi;
		p[2] = r*cos(psi);
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_psi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix phi and r
	phi = range[0][0];
	r = range[2][1];
	// Compute other stuff
	rCosPhi = r*cos(phi);
	rSinPhi = r*sin(phi);
	// Insert the points
	for ( i = 0, psi = range[1][0] ; i < resolution_psi ; ++i, psi += step )
	{
		p[0] = sin(psi)*rCosPhi;
		p[1] = sin(psi)*rSinPhi;
		p[2] = r*cos(psi);
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_psi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix phi and r
	phi = range[0][1];
	r = range[2][1];
	// Compute other stuff
	rCosPhi = r*cos(phi);
	rSinPhi = r*sin(phi);
	// Insert the points
	for ( i = 0, psi = range[1][0] ; i < resolution_psi ; ++i, psi += step )
	{
		p[0] = sin(psi)*rCosPhi;
		p[1] = sin(psi)*rSinPhi;
		p[2] = r*cos(psi);
		points->InsertNextPoint(p);
	}
	// Insert the lines
	++ids[0]; ++ids[1];
	for ( i = 1 ; i < resolution_psi ; ++i )
	{
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Fix phi and psi
	phi = range[0][0];
	psi = range[1][0];
	// Insert two points (Inserting points is a little bit redundant but easier to program)
	p[0] = range[2][0]*sin(psi)*cos(phi);
	p[1] = range[2][0]*sin(psi)*sin(phi);
	p[2] = range[2][0]*cos(psi);
	points->InsertNextPoint(p);
	p[0] = range[2][1]*sin(psi)*cos(phi);
	p[1] = range[2][1]*sin(psi)*sin(phi);
	p[2] = range[2][1]*cos(psi);
	points->InsertNextPoint(p);
	// Insert the lines
	++ids[0]; ++ids[1];
	lines->InsertNextCell(2, ids);

	// Fix phi and psi
	phi = range[0][1];
	psi = range[1][0];
	// Insert two points (Inserting points is a little bit redundant but easier to program)
	p[0] = range[2][0]*sin(psi)*cos(phi);
	p[1] = range[2][0]*sin(psi)*sin(phi);
	p[2] = range[2][0]*cos(psi);
	points->InsertNextPoint(p);
	p[0] = range[2][1]*sin(psi)*cos(phi);
	p[1] = range[2][1]*sin(psi)*sin(phi);
	p[2] = range[2][1]*cos(psi);
	points->InsertNextPoint(p);
	// Insert the lines
	ids[0] += 2; ids[1] += 2;
	lines->InsertNextCell(2, ids);

	// Fix phi and psi
	phi = range[0][0];
	psi = range[1][1];
	// Insert two points (Inserting points is a little bit redundant but easier to program)
	p[0] = range[2][0]*sin(psi)*cos(phi);
	p[1] = range[2][0]*sin(psi)*sin(phi);
	p[2] = range[2][0]*cos(psi);
	points->InsertNextPoint(p);
	p[0] = range[2][1]*sin(psi)*cos(phi);
	p[1] = range[2][1]*sin(psi)*sin(phi);
	p[2] = range[2][1]*cos(psi);
	points->InsertNextPoint(p);
	// Insert the lines
	ids[0] += 2; ids[1] += 2;
	lines->InsertNextCell(2, ids);

	// Fix phi and psi
	phi = range[0][1];
	psi = range[1][1];
	// Insert two points (Inserting points is a little bit redundant but easier to program)
	p[0] = range[2][0]*sin(psi)*cos(phi);
	p[1] = range[2][0]*sin(psi)*sin(phi);
	p[2] = range[2][0]*cos(psi);
	points->InsertNextPoint(p);
	p[0] = range[2][1]*sin(psi)*cos(phi);
	p[1] = range[2][1]*sin(psi)*sin(phi);
	p[2] = range[2][1]*cos(psi);
	points->InsertNextPoint(p);
	// Insert the lines
	ids[0] += 2; ids[1] += 2;
	lines->InsertNextCell(2, ids);


	// Save the point and the lines
	out->SetPoints(points);
	out->SetLines(lines);

	// Cleanup
	points->Delete();
	lines->Delete();
}

//==================================================================================================================================
