/*
 * VectorField32vtk.h
 *
 *  Created on: Sep 13, 2010
 *      Author: papazov
 */

#ifndef VECTORFIELD32VTK_H_
#define VECTORFIELD32VTK_H_

#include <BasicTools/Functions/VectorField3.h>
#include <vtkPolyData.h>

class VectorField32vtk
{
public:
	VectorField32vtk();
	virtual ~VectorField32vtk();

	void getVectorsAtRandom(VectorField3* field, int numOfVectors, const double box[6], vtkPolyData* out);
	void getVectorsAtRandom(VectorField3* field, int numOfVectors, const double box[6], double maxVecLen, vtkPolyData* out);
};

#endif /* VECTORFIELD32VTK_H_ */
