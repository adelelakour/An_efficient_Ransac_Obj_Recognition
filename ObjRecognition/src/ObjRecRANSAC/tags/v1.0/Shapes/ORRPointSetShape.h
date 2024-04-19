#ifndef _GEOMETRIC_SHAPE_H_
#define _GEOMETRIC_SHAPE_H_

#include "PointSetShape.h"
#include "../PixelVector.h"
#include "../DataStructures/RangeImage/ORRRangeImage2.h"
#include <BasicToolsL1/Vector.h>
#include <BasicToolsL1/Matrix.h>
#include <BasicTools/DataStructures/LayeredPointSet.h>
#include <vtkPolyData.h>
#include <algorithm>
#include <vector>
#include <list>

using namespace std;

class ORRPointSetShape: public PointSetShape
{
public:
	inline ORRPointSetShape(UserData* userdata, vtkPolyData* polydata, const LayeredPointSet* layeredModel, const double** mat4x4, int instanceNumber);
	virtual ~ORRPointSetShape(){}

	enum SceneState {SCENE_FREE = 0, SCENE_ON, SCENE_OFF};

	int getInstanceNumber(){ return mInstanceNumber;}
	const char* getLabel(){ return mLabel;}
	SceneState getSceneState(){ return mSceneState;}
	void setSceneState(SceneState state){ mSceneState = state;}
	void setSceneStateOff(){ mSceneState = SCENE_OFF;}
	void setSceneStateOn(){ mSceneState = SCENE_ON;}
	bool isSceneStateOff(){ return mSceneState == SCENE_OFF;}
	bool isSceneStateOn(){ return mSceneState == SCENE_ON;}

	inline void getPoint1(double* p)const{ vec_copy3(m_p1, p);}
	inline void getPoint2(double* p)const{ vec_copy3(m_p2, p);}
	inline void getPoint3(double* p)const{ vec_copy3(m_p3, p);}
	inline void getNormal1(double* n)const{ vec_copy3(m_n1, n);}
	inline void getNormal2(double* n)const{ vec_copy3(m_n2, n);}
	inline void getNormal3(double* n)const{ vec_copy3(m_n3, n);}

	const double* getPoint1(){ return m_p1;}
	const double* getPoint2(){ return m_p2;}
	const double* getPoint3(){ return m_p3;}
	const double* getNormal1(){ return m_n1;}
	const double* getNormal2(){ return m_n2;}
	const double* getNormal3(){ return m_n3;}

	inline void setPoint1(const double* p){ vec_copy3(p, m_p1);}
	inline void setPoint2(const double* p){ vec_copy3(p, m_p2);}
	inline void setPoint3(const double* p){ vec_copy3(p, m_p3);}
	inline void setNormal1(const double* n){ vec_copy3(n, m_n1);}
	inline void setNormal2(const double* n){ vec_copy3(n, m_n2);}
	inline void setNormal3(const double* n){ vec_copy3(n, m_n3);}

	int getNumberOfOccupiedScenePixels(){ return mNumOfOccupiedScenePixels;}
	inline void setOccupiedScenePixels(ORRRangeImage2* image, PixelVector* pixels);
	list<OctreeNode*>& getOctreeSceneNodes(){ return mSceneOctreeNodes;}

	inline void sortLinearPixelIds();
	const vector<int>& getLinearPixelIds(){ return mLinearPixelIds;}

	const LayeredPointSet* getLayeredPointSet(){ return mLayeredModel;}
	double* getRigidTransform12(){ return mRigidTransform12;}
	inline void copyRigidTransformFrom12To4x4();
	inline void setRigidTransform12(const double* rigreg);

	void setShapeId(int id){ mShapeId = id;}
	int getShapeId()const{ return mShapeId;}

protected:
	int mInstanceNumber;
	char mLabel[2048];
	SceneState mSceneState;
	double m_p1[3], m_p2[3], m_p3[3], m_n1[3], m_n2[3], m_n3[3], mRigidTransform12[12];
	int mNumOfOccupiedScenePixels, mShapeId;
	list<OctreeNode*> mSceneOctreeNodes;
	vector<int> mLinearPixelIds;
	const LayeredPointSet* mLayeredModel;
};

//=== inline methods ==================================================================================================================

inline ORRPointSetShape::ORRPointSetShape(UserData* userdata, vtkPolyData* polydata, const LayeredPointSet* layeredModel,
		const double** mat4x4, int instanceNumber): PointSetShape(userdata, polydata, mat4x4)
{
	mInstanceNumber = instanceNumber;
	mSceneState = ORRPointSetShape::SCENE_FREE;
	mLayeredModel = layeredModel;
	mShapeId = -1;

	if ( mUserData )
		sprintf(mLabel, "%s_%i", mUserData->getLabel(), mInstanceNumber);
	else
		sprintf(mLabel, "point set shape %i", mInstanceNumber);

	mNumOfOccupiedScenePixels = 0;
}

//=====================================================================================================================================

inline void ORRPointSetShape::setOccupiedScenePixels(ORRRangeImage2* image, PixelVector* pixels)
{
	mNumOfOccupiedScenePixels = pixels->getNumberOfOccupiedPixels();
	const int_2* pixels_coords = pixels->getPixelCoords();
	int i, x, y;
	const list<OctreeNode*>* nodes;
	list<OctreeNode*>::const_iterator node;

	mSceneOctreeNodes.clear();
	mLinearPixelIds.resize(mNumOfOccupiedScenePixels);

	list<int>::const_iterator linId = pixels->getLinearIds().begin();

	// Just save the occupied octree leaves to this shape
	for ( i = 0 ; linId != pixels->getLinearIds().end() ; ++i, ++linId )
	{
		x = pixels_coords[i].x;
		y = pixels_coords[i].y;
		nodes = image->getOctreeNodes()[x][y];
		if ( !nodes )
		{
			fprintf(stderr, "WARNING in 'ORRPointSetShape::%s()': there should be a list of nodes at pixel [%i, %i].\n", __func__, x, y);
			fflush(stderr);
			continue;
		}
		// Save the linear id
		mLinearPixelIds[i] = *linId;

		for ( node = nodes->begin() ; node != nodes->end() ; ++node )
			mSceneOctreeNodes.push_back(*node);
	}
}

//=====================================================================================================================================

inline void ORRPointSetShape::setRigidTransform12(const double* rigreg)
{
	mRigidTransform12[0]  = rigreg[0];
	mRigidTransform12[1]  = rigreg[1];
	mRigidTransform12[2]  = rigreg[2];
	mRigidTransform12[3]  = rigreg[3];
	mRigidTransform12[4]  = rigreg[4];
	mRigidTransform12[5]  = rigreg[5];
	mRigidTransform12[6]  = rigreg[6];
	mRigidTransform12[7]  = rigreg[7];
	mRigidTransform12[8]  = rigreg[8];
	mRigidTransform12[9]  = rigreg[9];
	mRigidTransform12[10] = rigreg[10];
	mRigidTransform12[11] = rigreg[11];
}

//=====================================================================================================================================

inline void ORRPointSetShape::copyRigidTransformFrom12To4x4()
{
	mat_copy_rigid_transform_to4x4(this->mRigidTransform12, this->mHomRigidTransform.m);
	this->mHomRigidTransform.m[3][0] = this->mHomRigidTransform.m[3][1] = this->mHomRigidTransform.m[3][2] = 0.0;
	this->mHomRigidTransform.m[3][3] = 1.0;
}

//=====================================================================================================================================

inline bool compareIntFunc(int i,int j) { return (i<j); }

//=====================================================================================================================================

inline void ORRPointSetShape::sortLinearPixelIds()
{
	sort(mLinearPixelIds.begin(), mLinearPixelIds.end(), compareIntFunc);
}

//=====================================================================================================================================

#endif /*_GEOMETRIC_SHAPE_H_*/
