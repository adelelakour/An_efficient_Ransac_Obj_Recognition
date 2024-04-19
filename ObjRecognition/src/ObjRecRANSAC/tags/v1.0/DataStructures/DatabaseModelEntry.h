#ifndef _DATABASE_MODEL_ENTRY_H_
#define _DATABASE_MODEL_ENTRY_H_

#include "../UserData.h"
#include "../ORRBasicTypes.h"
#include "ORROctree/ORROctree.h"
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <BasicTools/DataStructures/LayeredPointSet.h>
#include <vtkPolyData.h>

using namespace tum;

class DatabaseModelEntry
{
public:
	DatabaseModelEntry();
	virtual ~DatabaseModelEntry();

	bool init(UserData* userData, int numberOfPointsPerLayer, int id);

	UserData* getUserData()const{ return mUserData;}
	vtkPolyData* getOwnModel(){ return mOwnModel;}
	const LayeredPointSet* getLayeredModel()const{ return &mLayeredModel;}
	ORROctree* getModelOctree(){ return &mModelOctree;}
	int getId()const{ return mId;}

	int getInstanceCounter()const{ return mInstanceCounter;}
	void incrementInstanceCounter(){ ++mInstanceCounter;}
	void resetInstanceCounter(){ mInstanceCounter = 0;}

protected:
	UserData* mUserData;
	vtkPolyData *mOwnModel;
	LayeredPointSet mLayeredModel;
	int mInstanceCounter, mId;
	ORROctree mModelOctree;
};

//=== inline methods ==========================================================================================

#endif /*_DATABASE_MODEL_ENTRY_H_*/
