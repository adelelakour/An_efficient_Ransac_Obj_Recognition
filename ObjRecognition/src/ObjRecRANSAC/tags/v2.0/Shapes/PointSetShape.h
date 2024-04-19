#ifndef _POINT_SET_SHAPE_H_
#define _POINT_SET_SHAPE_H_

#include <BasicToolsL1/Vector.h>
#include <BasicToolsL1/Matrix.h>
#include "../UserData.h"
#include <vtkPolyData.h>
#include <list>

using namespace tum;
using namespace std;

class PointSetShape
{
public:
	inline PointSetShape(UserData* userdata, vtkPolyData* polydata, const double* rigid_transform);
	virtual ~PointSetShape(){}

	UserData* getUserData(){ return mUserData;}
	vtkPolyData* getPolyData(){ return mPolyData;}

	/** Copies the rigid transform of this shape to %mat4x4. %mat4x4 has to be a 4x4 matrix. */
	inline void getHomogeneousRigidTransform(double** mat4x4){ mat_copy_rigid_transform_to4x4(mRigidTransform, mat4x4);}
	/** Returns the rigid transform for this shape as a 12-vector: the first 9 elements are the elements of a rotation matrix
	  * (first the first row then the second and finally the third row) and the last three elements are the x, y, z coordinates
	  * of the translation vector. */
	const double* getRigidTransform(){ return mRigidTransform;}
	/** This list contains the scene points which belong to this shape. */
	list<int>& getScenePointIds(){ return mScenePointIds;}

	double getConfidence()const{ return mConfidence;}
	void setConfidence(double value){ mConfidence = value;}

protected:
	double mRigidTransform[12];
	UserData* mUserData;
	vtkPolyData *mPolyData;
	list<int> mScenePointIds;
	double mConfidence;
};

//=== inline methods ===========================================================================================================

inline PointSetShape::PointSetShape(UserData* userdata, vtkPolyData* polydata, const double* rigid_transform)
{
	mUserData = userdata;
	mPolyData = polydata;
	vec_copy12<double>(rigid_transform, mRigidTransform);
	mConfidence = -1.0;
}

//==============================================================================================================================

#endif /*_POINT_SET_SHAPE_H_*/
