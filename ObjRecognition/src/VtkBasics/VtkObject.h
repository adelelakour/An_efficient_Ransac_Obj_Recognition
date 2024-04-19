#ifndef _VTK_OBJECT_H_
#define _VTK_OBJECT_H_

#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>

#define VTK_OBJ_MAX_NAME_LEN	1024


class VtkObject
{
public:
	VtkObject();
	virtual ~VtkObject();

	virtual vtkPolyData* getPolyData() = 0;
	virtual void update() = 0;

	void setName(const char *name);
	void getName(char *name){ strcpy(name, mName);}
	int getNameLength(){ return strlen(mName);}
	void setColor(double r, double g, double b){ this->getActor()->GetProperty()->SetColor(r, g, b);}
	void getColor(double& r, double& g, double& b){ this->getActor()->GetProperty()->GetColor(r, g, b);}
	double* getColor(){ return this->getActor()->GetProperty()->GetColor();}
	void setColor(double* rgb){ this->getActor()->GetProperty()->SetColor(rgb);}
	void backfaceCullingOn(){ this->getActor()->GetProperty()->BackfaceCullingOn();}
	void backfaceCullingOff(){ this->getActor()->GetProperty()->BackfaceCullingOff();}

	void setOpacity(double s){ mActor->GetProperty()->SetOpacity(s);}

	vtkActor* getActor(){ return mActor;}
	bool isVisible(){ return (bool)mActor->GetVisibility();}
	void visibilityOff(){ mActor->VisibilityOff();}
	void visibilityOn(){ mActor->VisibilityOn();}
	void setVisibility(int vis){ mActor->SetVisibility(vis);}
	int getVisibility(){ return mActor->GetVisibility();}
	void switchVisibility(){ mActor->SetVisibility(!mActor->GetVisibility());}

	void computeBounds();
	void getBounds(double min[3], double max[3]);

	void getCenterOfMass(double com[3]);

protected:
	char mName[VTK_OBJ_MAX_NAME_LEN];
	double mMinBound[3], mMaxBound[3];
	bool mBoundsAreComputed;

	vtkActor *mActor;
};

#endif /*_VTK_OBJECT_H_*/
