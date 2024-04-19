/*
 * HashTableBoxStruct32vtk.h
 *
 *  Created on: Mar 12, 2011
 *      Author: papazov
 */

#ifndef HASHTABLEBOXSTRUCT32VTK_H_
#define HASHTABLEBOXSTRUCT32VTK_H_

//#include <ObjRecRANSAC/DataStructures/HashTableBoxStruct/HashTableBoxStruct3.h>

class HashTableBoxStruct32vtk
{
public:
	HashTableBoxStruct32vtk();
	virtual ~HashTableBoxStruct32vtk();
#if 0
	/** WARNING: table could contain many keys! */
	void getKeysAsPoints(const HashTableBoxStruct3* hash, vtkPoints* out);

	void getTriples(const HashTableCell3* cell, vtkPolyData* out, vtkPolyData* normals = NULL, double normal_scale = 1.0);
#endif
};

#endif /* HASHTABLEBOXSTRUCT32VTK_H_ */
