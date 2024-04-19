#ifndef OCTREE2VTK_H_
#define OCTREE2VTK_H_

#include <BasicTools/ComputationalGeometry/DataStructures/Octree/Octree.h>
#include <vtkPolyData.h>
#include <vector>
#include <set>

using namespace std;

class Octree2vtk
{
public:
	Octree2vtk();
	virtual ~Octree2vtk();

	void getOctreeAsWireframe(Octree* tree, vtkPolyData* out);
	void getFullLeafsAsPolygons(Octree* tree, vtkPolyData* out);
	void getFullLeafsAsWireframe(Octree* tree, vtkPolyData* out);

//	void getNormals(Octree* tree, vtkPolyData* out, double factor = 1.0);

	void getNodesAsWireframe(OctreeNode** nodes, int numOfNodes, vtkPolyData* out);
	void getNodesAsWireframe(vector<OctreeNode*>& nodes, vtkPolyData* out);

	void getNodesAsPolygon(OctreeNode** nodes, int numOfNodes, vtkPolyData* out);
	void getNodesAsPolygon(vector<OctreeNode*>& nodes, vtkPolyData* out);
	void getNodesAsPolygon(list<OctreeNode*>& nodes, vtkPolyData* out);

	void quads2vtk(list<OctreeNode**>& quads, vtkPolyData* out);

protected:
	void addNodeBoxes(OctreeNode* node, vtkPoints* points, vtkCellArray* lines);
};

#endif /*OCTREE2VTK_H_*/
