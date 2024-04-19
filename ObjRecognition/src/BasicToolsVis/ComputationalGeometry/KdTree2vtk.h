#ifndef _KDTREE2VTK_H_
#define _KDTREE2VTK_H_

#include <BasicTools/ComputationalGeometry/DataStructures/KdTree/KdTree.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>


class KdTree2vtk
{
public:
	KdTree2vtk();
	virtual ~KdTree2vtk();

	void getKdTreeAsWireframe(KdTree* kdtree, vtkPolyData* polyData, double zvalue);
	void getFullLeafsAsPoly(KdTree* kdtree, vtkPolyData* polyData, double zvalue);

protected:
	void get2dTreeAsWireframe(KdTree* kdtree, vtkPolyData* polyData, double zvalue);
	void get2dNodeLines(KdTreeNode* node, vtkCellArray* lines, vtkPoints* points, double zvalue);

	void get2dTreeAsPoly(KdTree* kdtree, vtkPolyData* polyData, double zvalue);
};

#endif /*_KDTREE2VTK_H_*/
