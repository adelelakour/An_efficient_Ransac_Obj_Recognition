#include "VtkNormals.h"
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <btl1/Vector.h>

using namespace btl1;

VtkNormals::VtkNormals()
{
}

VtkNormals::~VtkNormals()
{
}

//==================================================================================================================================

void VtkNormals::getPointNormals(vtkPolyData* input, vtkPolyData* out, double scaleFactor)
{
	vtkDataArray* normals = input->GetPointData()->GetNormals();

	if ( !normals )
		return;

	vtkCellArray *normalLines = vtkCellArray::New();
	vtkPoints *normalPoints = vtkPoints::New(VTK_DOUBLE);
	vtkIdType ids[2] = {0, 1};
	double p[3], n[3];

	for ( int i = 0 ; i < input->GetNumberOfPoints() ; ++i, ids[0] += 2, ids[1] += 2 )
	{
		// Get the normal
		normals->GetTuple(i, n);
		// Scale it
		n[0] *= scaleFactor;
		n[1] *= scaleFactor;
		n[2] *= scaleFactor;

		// Insert the first point (where the normal starts)
		input->GetPoint(i, p);
		normalPoints->InsertNextPoint(p);
		// Compute the second point (where the normal ends)
		p[0] += n[0];
		p[1] += n[1];
		p[2] += n[2];
		// Insert the second point (where the normal ends)
		normalPoints->InsertNextPoint(p);
		// Save the line
		normalLines->InsertNextCell(2, ids);
	}

	// Save the info
	out->SetPoints(normalPoints);
	out->SetLines(normalLines);
	// Cleanup
	normalPoints->Delete();
	normalLines->Delete();
}

//==================================================================================================================================

void VtkNormals::computeTriangleNormals(vtkPolyData* input, vtkPolyData* out, double scaleFactor)
{
	int i, num_of_cells = (int)input->GetNumberOfCells();
	vtkCell *vtk_tria;
	vtkPoints *vtk_points = input->GetPoints();
	double p0[3], p1[3], p2[3], p01[3], p02[3], n[3];

	vtkPoints* out_points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* out_lines = vtkCellArray::New();
	vtkIdType ids[2] = {0, 1};

	// Compute the normals for each triangle
	for ( i = 0 ; i < num_of_cells ; ++i )
	{
		// Get the current VTK triangle
		vtk_tria = input->GetCell(i);
		if ( vtk_tria->GetCellType() != VTK_TRIANGLE )
			continue;

		// Get the points of the cell
		vtk_points->GetPoint(vtk_tria->GetPointId(0), p0);
		vtk_points->GetPoint(vtk_tria->GetPointId(1), p1);
		vtk_points->GetPoint(vtk_tria->GetPointId(2), p2);

		// Compute the normal
		vec_diff3(p1, p0, p01);
		vec_diff3(p2, p0, p02);
		vec_cross3(p01, p02, n);
		vec_normalize3(n);
		vec_mult3(n, scaleFactor);

		// Compute the triangle's center of mass
		vec_add3(p0, p1);
		vec_add3(p0, p2);
		vec_mult3(p0, 1.0/3.0);
		out_points->InsertNextPoint(p0);
		// Compute and save the second point
		vec_add3(p0, n);
		out_points->InsertNextPoint(p0);

		// Insert the connecting line
		out_lines->InsertNextCell(2, ids);
		ids[0] += 2;
		ids[1] += 2;
	}

	// Save the result
	out->SetPoints(out_points);
	out->SetLines(out_lines);

	// Cleanup
	out_points->Delete();
	out_lines->Delete();
}

//==================================================================================================================================
