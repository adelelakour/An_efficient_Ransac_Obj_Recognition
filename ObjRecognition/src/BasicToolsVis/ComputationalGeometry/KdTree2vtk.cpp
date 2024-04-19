#include "KdTree2vtk.h"
#include <cstdio>
#include <list>

using namespace std;


KdTree2vtk::KdTree2vtk()
{
}

KdTree2vtk::~KdTree2vtk()
{
}

//==================================================================================================================================

void KdTree2vtk::getFullLeafsAsPoly(KdTree* kdtree, vtkPolyData* polyData, double zvalue)
{
	switch ( kdtree->getDimension() )
	{
		case 2:
			this->get2dTreeAsPoly(kdtree, polyData, zvalue);
			break;

		default:
			fprintf(stderr, "WARNING in 'KdTree2vtk::%s()': "
					"no implementation for %i dimensional kd-trees.\n", __func__, kdtree->getDimension());
			break;
	}
}

//==================================================================================================================================

void KdTree2vtk::get2dTreeAsPoly(KdTree* kdtree, vtkPolyData* polyData, double zvalue)
{
	KdTreeNode* node;
	list<KdTreeNode*> nodes;
	  nodes.push_back(kdtree->getRoot());

	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray *polys = vtkCellArray::New();
	vtkIdType ids[4] = {0, 1, 2, 3};
	double p[3]; p[2] = zvalue;

	while ( nodes.size() )
	{
		node = nodes.back();
		nodes.pop_back();

		if ( node->hasChildren() )
		{
			// Push back the children for further processing
			nodes.push_back((KdTreeNode*)node->getChild(0));
			nodes.push_back((KdTreeNode*)node->getChild(1));
		}
		else if ( node->getData() )
		{
			// Get that leaf
			p[0] = node->getLowerBoxBound(0);
			p[1] = node->getLowerBoxBound(1);
			points->InsertNextPoint(p);
			p[0] = node->getUpperBoxBound(0);
			p[1] = node->getLowerBoxBound(1);
			points->InsertNextPoint(p);
			p[0] = node->getUpperBoxBound(0);
			p[1] = node->getUpperBoxBound(1);
			points->InsertNextPoint(p);
			p[0] = node->getLowerBoxBound(0);
			p[1] = node->getUpperBoxBound(1);
			points->InsertNextPoint(p);
			// Insert the polygon
			polys->InsertNextCell(4, ids);
			ids[0] += 4; ids[1] += 4; ids[2] += 4; ids[3] += 4;
		}
	}

	polyData->SetPoints(points);
	polyData->SetPolys(polys);

	points->Delete();
	polys->Delete();
}

//==================================================================================================================================

void KdTree2vtk::getKdTreeAsWireframe(KdTree* kdtree, vtkPolyData* polyData, double zvalue)
{
	switch ( kdtree->getDimension() )
	{
		case 2:
			this->get2dTreeAsWireframe(kdtree, polyData, zvalue);
			break;

		default:
			fprintf(stderr, "WARNING in 'KdTree2vtk::%s()': "
					"no implementation for %i dimensional kd-trees.\n", __func__, kdtree->getDimension());
			break;
	}
}

//==================================================================================================================================

void KdTree2vtk::get2dTreeAsWireframe(KdTree* kdtree, vtkPolyData* polyData, double zvalue)
{
	vtkCellArray *lines = vtkCellArray::New();
	vtkPoints *points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	// This is a recursive method call, i.e. get2dNodeLines() calls itself several times.
	this->get2dNodeLines(kdtree->getRoot(), lines, points, zvalue);

	// Add the last four lines (sorrounding the whole tree structure)
	double p[3]; p[2] = zvalue;
	int baseId = points->GetNumberOfPoints();
	Box &rootBox = kdtree->getRoot()->getSpace();
	// First insert the points
	p[0] = rootBox.mIntervals[0][0];
	p[1] = rootBox.mIntervals[1][0];
	points->InsertNextPoint(p);
	p[0] = rootBox.mIntervals[0][1];
	p[1] = rootBox.mIntervals[1][0];
	points->InsertNextPoint(p);
	p[0] = rootBox.mIntervals[0][1];
	p[1] = rootBox.mIntervals[1][1];
	points->InsertNextPoint(p);
	p[0] = rootBox.mIntervals[0][0];
	p[1] = rootBox.mIntervals[1][1];
	points->InsertNextPoint(p);
	// Now insert the lines
	vtkIdType ids[2] = {baseId, baseId+1};
	lines->InsertNextCell(2, ids);
	ids[0] = baseId+1; ids[1] = baseId+2;
	lines->InsertNextCell(2, ids);
	ids[0] = baseId+2; ids[1] = baseId+3;
	lines->InsertNextCell(2, ids);
	ids[0] = baseId+3; ids[1] = baseId;
	lines->InsertNextCell(2, ids);

	// Associate the lines and points with the poly data object
	polyData->SetPoints(points);
	polyData->SetLines(lines);

	// Clean up
	lines->Delete();
	points->Delete();
}

//==================================================================================================================================

void KdTree2vtk::get2dNodeLines(KdTreeNode* node, vtkCellArray* lines, vtkPoints* points, double zvalue)
{
	if ( node->hasChildren() )
	{
        double p[3]; p[2] = zvalue;
        vtkIdType ids[2];
        switch ( node->getAxis() )
		{
			case 0:
				// Compute the vertical line separating child 0 and child 1:
				//  _______
				// |   |   | 
				// | 0 | 1 |
				// |___|___|

				// Compute the ids
				ids[0] = points->GetNumberOfPoints();
				ids[1] = points->GetNumberOfPoints()+1;
				// Compute the points
				p[0] = node->getChild(0)->getSpace().mIntervals[0][1];
				p[1] = node->getChild(0)->getSpace().mIntervals[1][0];
				p[2] = zvalue;
				points->InsertNextPoint(p);
				p[0] = node->getChild(0)->getSpace().mIntervals[0][1];
				p[1] = node->getChild(0)->getSpace().mIntervals[1][1];
				p[2] = zvalue;
				points->InsertNextPoint(p);
				lines->InsertNextCell(2, ids);
				break;

			case 1:
				// Compute the horizontal line separating child 0 and child 1:
				//  _______
				// |   1   | 
				// |-------|
				// |___0___|

				// Compute the ids
				ids[0] = points->GetNumberOfPoints();
				ids[1] = points->GetNumberOfPoints()+1;
				// Compute the points
				p[0] = node->getChild(0)->getSpace().mIntervals[0][0];
				p[1] = node->getChild(0)->getSpace().mIntervals[1][1];
				p[2] = zvalue;
				points->InsertNextPoint(p);
				p[0] = node->getChild(0)->getSpace().mIntervals[0][1];
				p[1] = node->getChild(0)->getSpace().mIntervals[1][1];
				p[2] = zvalue;
				points->InsertNextPoint(p);
				lines->InsertNextCell(2, ids);
				break;

			default:
				fprintf(stderr, "ERROR in 'KdTree2vtkPolyData::%s()': illegal level %i for a two dimensional kd-tree.\n", 
						__func__, node->getAxis());
				return;
		}
		// Call the method for both children of the current node
		this->get2dNodeLines((KdTreeNode*)node->getChild(0), lines, points, zvalue);
		this->get2dNodeLines((KdTreeNode*)node->getChild(1), lines, points, zvalue);
	}

	// We do not need to process the leaves of the tree, so nodes with no children are
	// not interesting for us => do nothing.
}

//==================================================================================================================================
