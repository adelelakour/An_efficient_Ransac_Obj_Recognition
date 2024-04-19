#ifndef _POINT_SET_SHAPE_H_
#define _POINT_SET_SHAPE_H_

#include <BasicTools/LinearAlgebra/Matrix.h>
#include "../UserData.h"
#include <vtkPolyData.h>
#include <list>

using namespace tum;
using namespace std;

class PointSetShape
{
public:
	inline PointSetShape(UserData* userdata, vtkPolyData* polydata, const double** mat4x4);
	virtual ~PointSetShape(){}

	UserData* getUserData(){ return mUserData;}
	vtkPolyData* getPolyData(){ return mPolyData;}

	/** Copies 'mat4x4' to the homogeneous matrix of this object. 'mat4x4' has to be a 4x4 matrix. */
	inline void setHomogeneousRigidTransform(const double** mat4x4);
	/** Copies the homogeneous matrix of this object to 'mat4x4'. 'mat4x4' has to be a 4x4 matrix. */
	inline void getHomogeneousRigidTransform(double** mat4x4);
	/** Returns the homogeneous matrix of this object. */
	const double** getHomogeneousRigidTransform(){ return (const double**)mHomRigidTransform.m;}
	/** This list contains the scene points which belong to this shape. */
	list<int>& getScenePointIds(){ return mScenePointIds;}

	double getConfidence()const{ return mConfidence;}
	void setConfidence(double value){ mConfidence = value;}

protected:
	Matrix mHomRigidTransform;
	UserData* mUserData;
	vtkPolyData *mPolyData;
	list<int> mScenePointIds;
	double mConfidence;
};

//=== inline methods ===========================================================================================================

inline PointSetShape::PointSetShape(UserData* userdata, vtkPolyData* polydata, const double** mat4x4)
: mHomRigidTransform(4, 4)
{
	mUserData = userdata;
	mPolyData = polydata;
	mHomRigidTransform.copyfrom(mat4x4);
	mConfidence = -1.0;
}

//==============================================================================================================================

inline void PointSetShape::setHomogeneousRigidTransform(const double** mat4x4)
{
	mHomRigidTransform.copyfrom(mat4x4);
}

//==============================================================================================================================

inline void PointSetShape::getHomogeneousRigidTransform(double** mat4x4)
{
	mHomRigidTransform.copyto(mat4x4);
}

//==============================================================================================================================

#endif /*_POINT_SET_SHAPE_H_*/
