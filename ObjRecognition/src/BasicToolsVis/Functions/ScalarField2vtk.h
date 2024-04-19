/*
 * ScalarField2vtk.h
 *
 *  Created on: Mar 31, 2010
 *      Author: papazov
 */

#ifndef SCALARFIELD2VTK_H_
#define SCALARFIELD2VTK_H_

#include <BasicTools/Functions/ScalarField.h>
#include <vtkImageData.h>
#include <vtkPolyData.h>

class ScalarField2vtk
{
public:
	ScalarField2vtk();
	virtual ~ScalarField2vtk();

	void get2dScalarFieldAsTriangularMesh(ScalarField* sf, const double** bounds,
			const double* spacing, vtkPolyData* out, double zfactor = 1.0);

	void get2dScalarFieldAsWireframe(ScalarField* sf, const double** bounds,
			const double* spacing, vtkPolyData* out, double zfactor = 1.0);

	void get2dFunctionAsWireframe(double (*func)(double x, double y), const double** bounds,
			const double* spacing, vtkPolyData* out, double zfactor = 1.0);

	void get3dScalarFieldAs3dImage(ScalarField* f, const double** bounds, double* spacing, vtkImageData* out);
	void get3dScalarFieldAs3dImage(ScalarField* f, const double* bounds, double* spacing, vtkImageData* out);
};

#endif /* SCALARFIELD2VTK_H_ */
