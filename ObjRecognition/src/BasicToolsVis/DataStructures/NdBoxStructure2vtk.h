#ifndef NDBOXSTRUCTURE2VTK_H_
#define NDBOXSTRUCTURE2VTK_H_

#include <BasicTools/DataStructures/NdBoxStructure.h>
#include <vtkPolyData.h>


class NdBoxStructure2vtk
{
public:
	NdBoxStructure2vtk();
	virtual ~NdBoxStructure2vtk();

	void getNdBoxStructureAsWireframe(NdBoxStructure<class T>* boxStruct, vtkPolyData* out);

protected:
	void get3dBoxStructureAsWireframe(NdBoxStructure<class T>* boxStruct, vtkPolyData* out);
};

#endif /*NDBOXSTRUCTURE2VTK_H_*/
