#include "Box2vtk.h"
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkType.h>


Box2vtk::Box2vtk()
{
}

Box2vtk::~Box2vtk()
{
}

//=======================================================================================================================

void Box2vtk::getBoxAsWireframe(const double box[6], vtkPolyData* out)
{
	vtkIdType ids[2];
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();

	// Insert the points
	points->InsertNextPoint(box[0], box[2], box[4]);
	points->InsertNextPoint(box[1], box[2], box[4]);
	points->InsertNextPoint(box[0], box[3], box[4]);
	points->InsertNextPoint(box[1], box[3], box[4]);
	points->InsertNextPoint(box[0], box[2], box[5]);
	points->InsertNextPoint(box[1], box[2], box[5]);
	points->InsertNextPoint(box[0], box[3], box[5]);
	points->InsertNextPoint(box[1], box[3], box[5]);
	// Insert the lines
	ids[0] = 0; ids[1] = 1;	lines->InsertNextCell(2, ids);
	ids[0] = 2; ids[1] = 3;	lines->InsertNextCell(2, ids);
	ids[0] = 4; ids[1] = 5;	lines->InsertNextCell(2, ids);
	ids[0] = 6; ids[1] = 7;	lines->InsertNextCell(2, ids);
	ids[0] = 0; ids[1] = 4;	lines->InsertNextCell(2, ids);
	ids[0] = 1; ids[1] = 5;	lines->InsertNextCell(2, ids);
	ids[0] = 2; ids[1] = 6;	lines->InsertNextCell(2, ids);
	ids[0] = 3; ids[1] = 7;	lines->InsertNextCell(2, ids);
	ids[0] = 0; ids[1] = 2;	lines->InsertNextCell(2, ids);
	ids[0] = 1; ids[1] = 3;	lines->InsertNextCell(2, ids);
	ids[0] = 4; ids[1] = 6;	lines->InsertNextCell(2, ids);
	ids[0] = 5; ids[1] = 7;	lines->InsertNextCell(2, ids);

	out->SetPoints(points);
	out->SetLines(lines);
	// Cleanup
	points->Delete();
	lines->Delete();
}

//=======================================================================================================================

void Box2vtk::getOutputAsLinesInR2(Box& box, vtkPolyData* out, double zvalue)
{
	list<Box> boxes;
	boxes.push_back(box);
	this->getOutputAsLinesInR2(boxes, out, zvalue);
	boxes.clear();
}

//=======================================================================================================================

void Box2vtk::getOutputAsLinesInR2(list<Box>& boxes, vtkPolyData* out, double zvalue)
{
	vtkIdType ids[2];
	int baseId;
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();
	list<Box>::iterator it;

	for ( baseId = 0, it = boxes.begin() ; it != boxes.end() ; ++it, baseId += 4 )
	{
		Box &box = *it;
		// Insert the 4 box points into the vtkPoints data structure
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][0], zvalue);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][0], zvalue);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][1], zvalue);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][1], zvalue);
		// Insert the box edges
		// Edges paralell to the x axis
		ids[0] = baseId;   ids[1] = baseId+1;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+1; ids[1] = baseId+2;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+2; ids[1] = baseId+3;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+3; ids[1] = baseId;
		lines->InsertNextCell(2, ids);
	}

	out->SetPoints(points);
	out->SetLines(lines);

	// Clean up
	points->Delete();
	lines->Delete();
}

//=======================================================================================================================

void Box2vtk::get3dBoxAsWireframe(Box& box, vtkPolyData* out)
{
	list<Box> boxes; boxes.push_back(box);
	this->getOutputAsLines(boxes, out);
}

//=======================================================================================================================

void Box2vtk::getOutputAsLines(list<Box>& boxes, vtkPolyData* out)
{
	vtkIdType ids[2];
	int b, baseId;
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();
	vtkFloatArray* scalars = vtkFloatArray::New();
	  scalars->SetNumberOfComponents(1);
	  scalars->SetNumberOfTuples(8*boxes.size());
	float value, step = 1.0f/(float)(boxes.size()-1);
	list<Box>::iterator it;

	for ( b = 0, value = 0.0f, it = boxes.begin() ; it != boxes.end() ; ++it, ++b, value += step )
	{
		baseId = 8*b; // Because we have 8 points per box
		Box &box = *it;
		// Insert the 8 box points into the vtkPoints data structure
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][0], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][0], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][1], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][1], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][0], box.mIntervals[2][1]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][0], box.mIntervals[2][1]);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][1], box.mIntervals[2][1]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][1], box.mIntervals[2][1]);
		// Insert the box edges
		// Edges paralell to the x axis
		ids[0] = baseId;   ids[1] = baseId+1;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+2; ids[1] = baseId+3;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+4; ids[1] = baseId+5;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+6; ids[1] = baseId+7;
		lines->InsertNextCell(2, ids);
		// Edges paralell to the y axis
		ids[0] = baseId;   ids[1] = baseId+2;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+1; ids[1] = baseId+3;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+4; ids[1] = baseId+6;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+5; ids[1] = baseId+7;
		lines->InsertNextCell(2, ids);
		// Edges paralell to the z axis
		ids[0] = baseId;   ids[1] = baseId+4;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+1; ids[1] = baseId+5;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+2; ids[1] = baseId+6;
		lines->InsertNextCell(2, ids);
		ids[0] = baseId+3; ids[1] = baseId+7;
		lines->InsertNextCell(2, ids);
		// Insert the scalar values
		scalars->SetTuple1(baseId,   value);
		scalars->SetTuple1(baseId+1, value);
		scalars->SetTuple1(baseId+2, value);
		scalars->SetTuple1(baseId+3, value);
		scalars->SetTuple1(baseId+4, value);
		scalars->SetTuple1(baseId+5, value);
		scalars->SetTuple1(baseId+6, value);
		scalars->SetTuple1(baseId+7, value);
	}

	out->SetPoints(points);
	out->SetLines(lines);
	out->GetPointData()->SetScalars(scalars);

	// Clean up
	points->Delete();
	lines->Delete();
	scalars->Delete();
}

//=======================================================================================================================

void Box2vtk::getOutputAsPolygons(list<Box>& boxes, vtkPolyData* out)
{
	vtkIdType ids[4];
	int b, basePointId, baseFaceId;
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* faces = vtkCellArray::New();
	vtkFloatArray* normals = vtkFloatArray::New();
	  normals->SetNumberOfComponents(3);
	  normals->SetNumberOfTuples(6*boxes.size());// We have six faces per box, so we need six normals per box.
	list<Box>::iterator it;

	for ( b = 0, it = boxes.begin() ; it != boxes.end() ; ++it, ++b )
	{
		basePointId = 8*b; // Because we have 8 points per box
		baseFaceId =  6*b; // Because we have 6 faces per box
		Box &box = *it;
		// Insert the 8 box points into the vtkPoints data structure
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][0], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][0], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][1], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][1], box.mIntervals[2][0]);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][0], box.mIntervals[2][1]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][0], box.mIntervals[2][1]);
		points->InsertNextPoint(box.mIntervals[0][0], box.mIntervals[1][1], box.mIntervals[2][1]);
		points->InsertNextPoint(box.mIntervals[0][1], box.mIntervals[1][1], box.mIntervals[2][1]);
		// Insert the box faces
		// Faces paralell to the xy plane
		ids[0] = basePointId+2; ids[1] = basePointId+3; ids[2] = basePointId+1; ids[3] = basePointId;
		faces->InsertNextCell(4, ids);
		ids[0] = basePointId+4; ids[1] = basePointId+5; ids[2] = basePointId+7; ids[3] = basePointId+6;
		faces->InsertNextCell(4, ids);
		// Faces paralell to the zx plane
		ids[0] = basePointId;   ids[1] = basePointId+1; ids[2] = basePointId+5; ids[3] = basePointId+4;
		faces->InsertNextCell(4, ids);
		ids[0] = basePointId+3; ids[1] = basePointId+2; ids[2] = basePointId+6; ids[3] = basePointId+7;
		faces->InsertNextCell(4, ids);
		// Faces paralell to the zy plane
		ids[0] = basePointId+2; ids[1] = basePointId;   ids[2] = basePointId+4; ids[3] = basePointId+6;
		faces->InsertNextCell(4, ids);
		ids[0] = basePointId+1; ids[1] = basePointId+3; ids[2] = basePointId+7; ids[3] = basePointId+5;
		faces->InsertNextCell(4, ids);
		// Insert the normals for each side
		normals->SetTuple3(baseFaceId,    0.0,  0.0, -1.0);
		normals->SetTuple3(baseFaceId+1,  0.0,  0.0,  1.0);
		normals->SetTuple3(baseFaceId+2,  0.0, -1.0,  0.0);
		normals->SetTuple3(baseFaceId+3,  0.0,  1.0,  0.0);
		normals->SetTuple3(baseFaceId+4, -1.0,  0.0,  0.0);
		normals->SetTuple3(baseFaceId+5,  1.0,  0.0,  0.0);
	}

	out->SetPoints(points);
	out->SetPolys(faces);
	out->GetCellData()->SetNormals(normals);

	// Clean up
	points->Delete();
	faces->Delete();
	normals->Delete();
}

//=======================================================================================================================
