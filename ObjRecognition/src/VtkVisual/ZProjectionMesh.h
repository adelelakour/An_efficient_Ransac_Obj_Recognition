/*
 * ZProjectionMesh.h
 *
 *  Created on: Nov 1, 2010
 *      Author: papazov
 */

#ifndef ZPROJECTIONMESH_H_
#define ZPROJECTIONMESH_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>

class ZPMPixel
{
public:
	ZPMPixel(){ p[0] = p[1] = p[2] = 0.0; numOfPts = 0;}
	virtual ~ZPMPixel(){}
public:
	double p[3];
	int id, numOfPts;
};

class ZProjectionMesh
{
public:
	ZProjectionMesh();
	virtual ~ZProjectionMesh();

	void create_mesh(vtkPoints* input, double voxelsize, vtkPolyData* out, double max_z_diff = -1.0);

	inline void comp_xy_bounds(vtkPoints* input, double b[4]);
	inline bool check_z_diff(ZPMPixel* a, ZPMPixel* b, ZPMPixel* c, double max_z_diff);
	inline bool check_z_diff(ZPMPixel* a, ZPMPixel* b, double max_z_diff);

protected:
	double mBorderFrac;
};

//=== inline methods ===================================================================================================

inline void ZProjectionMesh::comp_xy_bounds(vtkPoints* input, double b[4])
{
	double p[3];
	input->GetPoint(0, p);
	b[0] = b[1] = p[0];
	b[2] = b[3] = p[1];
//	b[4] = b[5] = p[2];

	for ( int i = 1 ; i < input->GetNumberOfPoints() ; ++i )
	{
		input->GetPoint(i, p);
			 if ( p[0] < b[0] ) b[0] = p[0];
		else if ( p[0] > b[1] ) b[1] = p[0];
			 if ( p[1] < b[2] ) b[2] = p[1];
		else if ( p[1] > b[3] ) b[3] = p[1];
//			 if ( p[2] < b[4] ) b[4] = p[2];
//		else if ( p[2] > b[5] ) b[5] = p[2];
	}
}

//======================================================================================================================

inline bool ZProjectionMesh::check_z_diff(ZPMPixel* a, ZPMPixel* b, ZPMPixel* c, double max_z_diff)
{
	if ( fabs(a->p[2] - b->p[2]) > max_z_diff ) return false;
	if ( fabs(a->p[2] - c->p[2]) > max_z_diff ) return false;
	if ( fabs(b->p[2] - c->p[2]) > max_z_diff ) return false;

	return true;
}

//======================================================================================================================

inline bool ZProjectionMesh::check_z_diff(ZPMPixel* a, ZPMPixel* b, double max_z_diff)
{
	if ( fabs(a->p[2] - b->p[2]) > max_z_diff )
		return false;

	return true;
}

//======================================================================================================================

#endif /* ZPROJECTIONMESH_H_ */
