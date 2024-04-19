/*
 * VtkUnstructuredGrid2vtkPolyData.h
 *
 *  Created on: May 9, 2012
 *      Author: papazov
 */

#ifndef VTKUNSTRUCTUREDGRID2VTKPOLYDATA_H_
#define VTKUNSTRUCTUREDGRID2VTKPOLYDATA_H_

#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>

class VtkUnstructuredGrid2vtkPolyData
{
public:
	VtkUnstructuredGrid2vtkPolyData();
	virtual ~VtkUnstructuredGrid2vtkPolyData();

	void getTetrahedraWireframe(vtkUnstructuredGrid *grid, vtkPolyData *out);
};

#endif /* VTKUNSTRUCTUREDGRID2VTKPOLYDATA_H_ */
