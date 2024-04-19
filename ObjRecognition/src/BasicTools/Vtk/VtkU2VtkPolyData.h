/*
 * VtkU2VtkPolyData.h
 *
 *  Created on: Nov 8, 2011
 *      Author: papazov
 */

#ifndef _VTKU_2_VTK_POLY_DATA_H_
#define _VTKU_2_VTK_POLY_DATA_H_

#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>

class VtkU2VtkPolyData
{
public:
	VtkU2VtkPolyData();
	virtual ~VtkU2VtkPolyData();

	void getTetrahedraWireframe(vtkUnstructuredGrid *grid, vtkPolyData *out);
};

#endif /* _VTKU_2_VTK_POLY_DATA_H_ */
