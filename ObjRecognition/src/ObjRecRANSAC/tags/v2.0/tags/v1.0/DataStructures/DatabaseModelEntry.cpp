#include "DatabaseModelEntry.h"
#include "../ORRDefines.h"
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <map>

using namespace std;


DatabaseModelEntry::DatabaseModelEntry()
{
	// Create own model (will be used by other classes)
	mOwnModel = vtkPolyData::New();
	this->resetInstanceCounter();
}

DatabaseModelEntry::~DatabaseModelEntry()
{
	mOwnModel->Delete();
}

//=============================================================================================================================

bool DatabaseModelEntry::init(UserData* userData, int numberOfPointsPerLayer, int id)
{
	if ( mOwnModel->GetNumberOfPoints() <= 0 )
	{
		fprintf(stderr, "ERROR in 'DatabaseModelEntry::%s()': the own model is empty - initialize it first!\n", __func__);
		return false;
	}

	// Save some info
	mUserData = userData;
	mId = id;

#ifdef OBJ_REC_RANSAC_VERBOSE_2
	printf("DatabaseModelEntry::%s(): building layered model ... ", __func__); fflush(stdout);
#endif

	// Build the layered model point set: the result is a set of disjoint point sets, which contains
	// all points of the model. This will be used for transform hypothesis verification.
	mLayeredModel.buildPointSet(numberOfPointsPerLayer, mOwnModel->GetPoints());

#ifdef OBJ_REC_RANSAC_VERBOSE_2
	printf("OK.\n"); fflush(stdout);
#endif

	return true;
}

//=============================================================================================================================
