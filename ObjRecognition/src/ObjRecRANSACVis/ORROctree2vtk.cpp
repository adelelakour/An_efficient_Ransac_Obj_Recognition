/*
 * ORROctree2vtk.cpp
 *
 *  Created on: Mar 12, 2011
 *      Author: papazov
 */

#include "ORROctree2vtk.h"
#include <ObjRecRANSAC/DataStructures/ORROctree/ORROctreeNodeData.h>
#include <btl1/Vector.h>
#include <vtkCellArray.h>
#include <vector>

using namespace std;

ORROctree2vtk::ORROctree2vtk()
{
}

ORROctree2vtk::~ORROctree2vtk()
{
}

//===================================================================================================================
#if 0
void ORROctree2vtk::getFullLeavesWithVisIdAsPoly(ORROctree* octree, int visId, vtkPolyData* out,
		list<const double*>* nodePoints, vtkPolyData* normals, double normal_scale)
{
	vtkPoints *points = vtkPoints::New(VTK_DOUBLE), *normal_points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray *polys = vtkCellArray::New(), *normal_lines = vtkCellArray::New();
	vtkIdType ids[4];
	int b; double min[3], max[3], n[3];
	const double *p;
	vector<OctreeNode*>& fullLeaves = octree->getFullLeafs();
	vtkIdType normal_ids[2] = {0, 1};

	// Build the boxes for the full leafs
	for ( int i = 0 ; i < (int)fullLeaves.size() ; ++i )
	{
		if ( ((ORROctreeNodeData*)fullLeaves[i]->getData())->mVisId != visId )
			continue;

		p = ((ORROctreeNodeData*)fullLeaves[i]->getData())->getPoint();
		if ( nodePoints )
			nodePoints->push_back(p);

		// Get the base id
		b = points->GetNumberOfPoints();
		// Get the boundaries of the leaf
		fullLeaves[i]->getBounds(min, max);

		// Add the points
		points->InsertNextPoint(min[0], min[1], min[2]);
		points->InsertNextPoint(min[0], min[1], max[2]);
		points->InsertNextPoint(min[0], max[1], min[2]);
		points->InsertNextPoint(min[0], max[1], max[2]);
		points->InsertNextPoint(max[0], min[1], min[2]);
		points->InsertNextPoint(max[0], min[1], max[2]);
		points->InsertNextPoint(max[0], max[1], min[2]);
		points->InsertNextPoint(max[0], max[1], max[2]);

		// Add the polys
		ids[0] = b  ; ids[1] = b+1; ids[2] = b+3; ids[3] = b+2;
		polys->InsertNextCell(4, ids);
		ids[0] = b+4; ids[1] = b+5; ids[2] = b+7; ids[3] = b+6;
		polys->InsertNextCell(4, ids);
		ids[0] = b  ; ids[1] = b+2; ids[2] = b+6; ids[3] = b+4;
		polys->InsertNextCell(4, ids);
		ids[0] = b+1; ids[1] = b+3; ids[2] = b+7; ids[3] = b+5;
		polys->InsertNextCell(4, ids);
		ids[0] = b  ; ids[1] = b+1; ids[2] = b+5; ids[3] = b+4;
		polys->InsertNextCell(4, ids);
		ids[0] = b+2; ids[1] = b+3; ids[2] = b+7; ids[3] = b+6;
		polys->InsertNextCell(4, ids);

		// Add the normal points
		normal_points->InsertNextPoint(p);
		((ORROctreeNodeData*)fullLeaves[i]->getData())->getNormal(n);
		vec_mult3(n, normal_scale);
		vec_add3(n, p);
		normal_points->InsertNextPoint(n);
		// Add the normal line
		normal_lines->InsertNextCell(2, normal_ids);
		normal_ids[0] += 2;
		normal_ids[1] += 2;
	}

	// Save the result
	out->SetPoints(points);
	out->SetPolys(polys);
	if ( normals )
	{
		normals->SetPoints(normal_points);
		normals->SetLines(normal_lines);
	}
	// Clean up
	points->Delete();
	polys->Delete();
	normal_points->Delete();
	normal_lines->Delete();
}
#endif
//===================================================================================================================
