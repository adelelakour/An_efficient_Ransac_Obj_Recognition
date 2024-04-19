#include "DatabaseModelEntry.h"
#include "../ORRDefines.h"
#include <BasicTools/DataStructures/PointSetAdapter.h>
#include <BasicTools/Vtk/VtkMeshSampler.h>
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

bool DatabaseModelEntry::init(vtkPolyData* mesh, UserData* userData, int numberOfPointsPerLayer, int id)
{
	if ( mOwnModel->GetNumberOfPoints() <= 0 )
	{
		fprintf(stderr, "ERROR in 'DatabaseModelEntry::%s()': the own model is empty - initialize it first!\n", __func__);
		return false;
	}

	// Create the own point set
	vtkPoints* tmp = vtkPoints::New(VTK_DOUBLE);
	VtkMeshSampler sampler;
	  sampler.sample(mesh, tmp, ORR_NUM_OF_OWN_MODEL_POINTS); // Sample from the mesh surface

	// Copy to the own model
	PointSetAdapter psa;
	psa.vtk2PointSet(tmp, &mOwnPointSet);
	tmp->Delete();

	// Save some info
	mUserData = userData;
	mId = id;

	return true;
}

//=============================================================================================================================
