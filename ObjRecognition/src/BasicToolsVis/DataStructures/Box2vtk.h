#ifndef _BOX2VTK_H_
#define _BOX2VTK_H_

#include <BasicTools/DataStructures/Box.h>
#include <vtkPolyData.h>
#include <list>

using namespace std;


class Box2vtk
{
public:
	Box2vtk();
	virtual ~Box2vtk();

	/** For 3D boxes. */
	void getOutputAsLines(list<Box>& boxes, vtkPolyData* out);
	/** For 3D boxes. */
	void getOutputAsPolygons(list<Box>& boxes, vtkPolyData* out);

	void get3dBoxAsWireframe(Box& box, vtkPolyData* out);

	/** For 2D boxes. */
	void getOutputAsLinesInR2(list<Box>& boxes, vtkPolyData* out, double zvalue = 0.0);
	/** For 2D boxes. */
	void getOutputAsLinesInR2(Box& box, vtkPolyData* out, double zvalue = 0.0);

	void getBoxAsWireframe(const double box[6], vtkPolyData* out);
};

#endif /*_BOX2VTK_H_*/
