#include "Octree2vtk.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>


Octree2vtk::Octree2vtk()
{
}

Octree2vtk::~Octree2vtk()
{
}

//==============================================================================================================================

void Octree2vtk::getOctreeAsWireframe(Octree* tree, vtkPolyData* out)
{
	// Do some stuff
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();

	// Build the boxes for the nodes
	this->addNodeBoxes(tree->getRoot(), points, lines);

	// Save the result
	out->SetPoints(points);
	out->SetLines(lines);
	// Clean up
	points->Delete();
	lines->Delete();
}

//==============================================================================================================================

void Octree2vtk::addNodeBoxes(OctreeNode* node, vtkPoints* points, vtkCellArray* lines)
{
	// Compute the lines seperating the children
	double *min = new double[3], *max = new double[3];
	node->getBounds(min, max);

	// Get the base id
	vtkIdType* ids = new vtkIdType[2];
	int b = points->GetNumberOfPoints();

	// Add the points
	points->InsertNextPoint(min[0], min[1], min[2]);
	points->InsertNextPoint(min[0], min[1], max[2]);
	points->InsertNextPoint(min[0], max[1], min[2]);
	points->InsertNextPoint(min[0], max[1], max[2]);
	points->InsertNextPoint(max[0], min[1], min[2]);
	points->InsertNextPoint(max[0], min[1], max[2]);
	points->InsertNextPoint(max[0], max[1], min[2]);
	points->InsertNextPoint(max[0], max[1], max[2]);

	// Add the lines
	ids[0] = b  ; ids[1] = b+1;
	lines->InsertNextCell(2, ids);
	ids[0] = b+2; ids[1] = b+3;
	lines->InsertNextCell(2, ids);
	ids[0] = b+4; ids[1] = b+5;
	lines->InsertNextCell(2, ids);
	ids[0] = b+6; ids[1] = b+7;
	lines->InsertNextCell(2, ids);

	ids[0] = b+1; ids[1] = b+3;
	lines->InsertNextCell(2, ids);
	ids[0] = b+5; ids[1] = b+7;
	lines->InsertNextCell(2, ids);
	ids[0] = b  ; ids[1] = b+2;
	lines->InsertNextCell(2, ids);
	ids[0] = b+4; ids[1] = b+6;
	lines->InsertNextCell(2, ids);

	ids[0] = b+3; ids[1] = b+7;
	lines->InsertNextCell(2, ids);
	ids[0] = b+1; ids[1] = b+5;
	lines->InsertNextCell(2, ids);
	ids[0] = b+2; ids[1] = b+6;
	lines->InsertNextCell(2, ids);
	ids[0] = b+0; ids[1] = b+4;
	lines->InsertNextCell(2, ids);

	// Clean up
	delete[] min;
	delete[] max;
	delete[] ids;

	if ( node->hasChildren() )
		for ( int id = 0 ; id < 8 ; ++id )
			this->addNodeBoxes(node->getChild(id), points, lines); // Recursive call for the other kids
}

//==============================================================================================================================

void Octree2vtk::getFullLeafsAsPolygons(Octree* tree, vtkPolyData* out)
{
	tree->buildFullLeafsVector();
	this->getNodesAsPolygon(tree->getFullLeafs(), out);
}

//==============================================================================================================================

void Octree2vtk::getFullLeafsAsWireframe(Octree* tree, vtkPolyData* out)
{
	tree->buildFullLeafsVector();
	this->getNodesAsWireframe(tree->getFullLeafs(), out);
}

//==============================================================================================================================

void Octree2vtk::getNodesAsWireframe(vector<OctreeNode*>& nodes, vtkPolyData* out)
{
	OctreeNode** array = new OctreeNode*[nodes.size()];

	for ( unsigned int i = 0 ; i < nodes.size() ; ++i )
		array[i] = nodes[i];

	// Call the other version of this method
	this->getNodesAsWireframe(array, nodes.size(), out);

	// Clean up
	delete[] array;
}

//==============================================================================================================================

void Octree2vtk::getNodesAsWireframe(OctreeNode** nodes, int numOfNodes, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();
	vtkIdType ids[2];
	int b;
	double min[3], max[3];

	// Build the boxes for the full leafs
	for ( int i = 0 ; i < numOfNodes ; ++i )
	{
		// Get the base id
		b = points->GetNumberOfPoints();
		// Get the boundaries of the leaf
		nodes[i]->getBounds(min, max);

		// Add the points
		points->InsertNextPoint(min[0], min[1], min[2]);
		points->InsertNextPoint(min[0], min[1], max[2]);
		points->InsertNextPoint(min[0], max[1], min[2]);
		points->InsertNextPoint(min[0], max[1], max[2]);
		points->InsertNextPoint(max[0], min[1], min[2]);
		points->InsertNextPoint(max[0], min[1], max[2]);
		points->InsertNextPoint(max[0], max[1], min[2]);
		points->InsertNextPoint(max[0], max[1], max[2]);

		// Add the lines
		ids[0] = b  ; ids[1] = b+1;
		lines->InsertNextCell(2, ids);
		ids[0] = b+2; ids[1] = b+3;
		lines->InsertNextCell(2, ids);
		ids[0] = b+4; ids[1] = b+5;
		lines->InsertNextCell(2, ids);
		ids[0] = b+6; ids[1] = b+7;
		lines->InsertNextCell(2, ids);

		ids[0] = b+1; ids[1] = b+3;
		lines->InsertNextCell(2, ids);
		ids[0] = b+5; ids[1] = b+7;
		lines->InsertNextCell(2, ids);
		ids[0] = b  ; ids[1] = b+2;
		lines->InsertNextCell(2, ids);
		ids[0] = b+4; ids[1] = b+6;
		lines->InsertNextCell(2, ids);

		ids[0] = b+3; ids[1] = b+7;
		lines->InsertNextCell(2, ids);
		ids[0] = b+1; ids[1] = b+5;
		lines->InsertNextCell(2, ids);
		ids[0] = b+2; ids[1] = b+6;
		lines->InsertNextCell(2, ids);
		ids[0] = b+0; ids[1] = b+4;
		lines->InsertNextCell(2, ids);
	}

	// Save the result
	out->SetPoints(points);
	out->SetLines(lines);
	// Clean up
	points->Delete();
	lines->Delete();
}

//==============================================================================================================================

void Octree2vtk::getNodesAsPolygon(vector<OctreeNode*>& nodes, vtkPolyData* out)
{
	OctreeNode** array = new OctreeNode*[nodes.size()];

	for ( unsigned int i = 0 ; i < nodes.size() ; ++i )
		array[i] = nodes[i];

	// Call the other version of this method
	this->getNodesAsPolygon(array, nodes.size(), out);

	// Clean up
	delete[] array;
}

//==============================================================================================================================

void Octree2vtk::getNodesAsPolygon(list<OctreeNode*>& nodes, vtkPolyData* out)
{
	OctreeNode** array = new OctreeNode*[nodes.size()];
	list<OctreeNode*>::iterator it = nodes.begin();

	for ( unsigned int i = 0 ; i < nodes.size() ; ++i, ++it )
		array[i] = *it;

	// Call the other version of this method
	this->getNodesAsPolygon(array, nodes.size(), out);

	// Clean up
	delete[] array;
}

//==============================================================================================================================

void Octree2vtk::getNodesAsPolygon(OctreeNode** nodes, int numOfNodes, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* polys = vtkCellArray::New();
	vtkIdType ids[4];
	int b;
	double min[3], max[3];

	// Build the boxes for the full leafs
	for ( int i = 0 ; i < numOfNodes ; ++i )
	{
		// Get the base id
		b = points->GetNumberOfPoints();
		// Get the boundaries of the leaf
		nodes[i]->getBounds(min, max);

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
	}

	// Save the result
	out->SetPoints(points);
	out->SetPolys(polys);
	// Clean up
	points->Delete();
	polys->Delete();
}

//==============================================================================================================================

void Octree2vtk::quads2vtk(list<OctreeNode**>& quads, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();
	vtkCellArray* lines = vtkCellArray::New();
	vtkIdType ids[2];

	list<OctreeNode**>::iterator it;
	for ( ids[0] = 0, ids[1] = 1, it = quads.begin() ; it != quads.end() ; ++it )
	{
		points->InsertNextPoint((*it)[0]->getCenter());
		points->InsertNextPoint((*it)[1]->getCenter());
		lines->InsertNextCell(2, ids);
		ids[0] += 2;
		ids[1] += 2;
		points->InsertNextPoint((*it)[2]->getCenter());
		points->InsertNextPoint((*it)[3]->getCenter());
		lines->InsertNextCell(2, ids);
		ids[0] += 2;
		ids[1] += 2;
	}

	out->SetPoints(points);
	out->SetLines(lines);
	points->Delete();
	lines->Delete();
}

//==============================================================================================================================
#ifdef NEVER
void Octree2vtk::getNormals(Octree* tree, vtkPolyData* out, double factor)
{
	tree->buildFullLeafsVector();
	vector<OctreeNode*>& leafs = tree->getFullLeafs();
	vtkCellArray* lines = vtkCellArray::New();
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	ORROctreeNodeData* leafdata;
	double p2[3];
	const double *p;
	vtkIdType ids[2] = {0, 1};
	int numOfFullLeafs = tree->getNumberOfFullLeafs();

	for ( int i = 0 ; i < numOfFullLeafs ; ++i )
	{
		leafdata = dynamic_cast<ORROctreeNodeData*>(leafs[i]->getData());
		if ( !leafdata ) continue;
		if ( !leafdata->getNormal() ) continue;

		p = leafdata->getPoint();
		p2[0] = p[0] + factor*leafdata->getNormal()[0];
		p2[1] = p[1] + factor*leafdata->getNormal()[1];
		p2[2] = p[2] + factor*leafdata->getNormal()[2];

		points->InsertNextPoint(p);
		points->InsertNextPoint(p2);
		lines->InsertNextCell(2, ids);

		ids[0] += 2;
		ids[1] += 2;
	}
	out->SetPoints(points);
	out->SetLines(lines);
	points->Delete();
	lines->Delete();
}

//==============================================================================================================================

void Octree2vtk::getNodeDataPoints(ORROctree* tree, vtkPoints* out)
{
	tree->buildFullLeafsVector();
	int i, numOfFullLeafs = tree->getNumberOfFullLeafs();
	vector<OctreeNode*>& fullLeafs = tree->getFullLeafs();

	for ( i = 0 ; i < numOfFullLeafs ; ++i )
		out->InsertNextPoint(((ORROctreeNodeData*)fullLeafs[i]->getData())->getPoint());
}
#endif
//==============================================================================================================================
