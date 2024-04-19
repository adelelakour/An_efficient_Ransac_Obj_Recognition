#include "NdBoxStructure2vtk.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>


NdBoxStructure2vtk::NdBoxStructure2vtk()
{
}

NdBoxStructure2vtk::~NdBoxStructure2vtk()
{
}

//==============================================================================================================================

void NdBoxStructure2vtk::getNdBoxStructureAsWireframe(NdBoxStructure<class T>* boxStruct, vtkPolyData* out)
{
	switch ( boxStruct->getDimension() )
	{
		case 3:
			this->get3dBoxStructureAsWireframe(boxStruct, out);
			break;

		default:
			fprintf(stderr, "ERROR in 'NdBoxStructure2vtk::%s()': dimension %i not implemented yet.\n",
					__func__, boxStruct->getDimension());
			break;
	}
}

//==============================================================================================================================

void NdBoxStructure2vtk::get3dBoxStructureAsWireframe(NdBoxStructure<class T>* boxStruct, vtkPolyData* out)
{
	vtkCellArray* lines = vtkCellArray::New();
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	vtkIdType ids[2];
	int i, j, numOfBoxes[3];
	double x, y, z, boxLengths[3], **bounds;

	// Alloc some memory
	bounds = new double*[3];
	bounds[0] = new double[2];
	bounds[1] = new double[2];
	bounds[2] = new double[2];

	boxStruct->getBoxLengths(boxLengths);
	boxStruct->getNumberOfBoxes(numOfBoxes);
	boxStruct->getStructureBounds(bounds);
	ids[0] = 0;

	// Insert the lines parallel to the yz-plane
	x = bounds[0][0];
	for ( i = 0 ; i <= numOfBoxes[0] ; ++i, x += boxLengths[0] )
	{
		for ( j = 0, z = bounds[2][0] ; j <= numOfBoxes[2] ; ++j, z += boxLengths[2], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(x, bounds[1][0], z);
			points->InsertNextPoint(x, bounds[1][1], z);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}
	// Insert the lines parallel to the xz-plane
	y = bounds[1][0];
	for ( i = 0 ; i <= numOfBoxes[1] ; ++i, y += boxLengths[1] )
	{
		for ( j = 0, z = bounds[2][0] ; j <= numOfBoxes[2] ; ++j, z += boxLengths[2], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(bounds[0][0], y, z);
			points->InsertNextPoint(bounds[0][1], y, z);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}
	// Insert the planes parallel to the xy-plane
	x = bounds[0][0];
	for ( i = 0 ; i <= numOfBoxes[0] ; ++i, x += boxLengths[0] )
	{
		for ( j = 0, y = bounds[1][0] ; j <= numOfBoxes[1] ; ++j, y += boxLengths[1], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(x, y, bounds[2][0]);
			points->InsertNextPoint(x, y, bounds[2][1]);
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
	delete[] bounds[0];
	delete[] bounds[1];
	delete[] bounds[2];
	delete[] bounds;
}

//==============================================================================================================================
