/*
 * TextIdsAtPoints.h
 *
 *  Created on: Nov 16, 2011
 *      Author: papazov
 */

#ifndef TEXTIDSATPOINTS_H_
#define TEXTIDSATPOINTS_H_

#include <vtkPolyData.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkPoints.h>

class TextIdsAtPoints
{
public:
	TextIdsAtPoints();
	virtual ~TextIdsAtPoints();

	void init(vtkCamera *cam, vtkRenderer *renderer, double scale){ mCamera = cam; mRenderer = renderer; mTextScale = scale;}

	void setCamera(vtkCamera *cam){ mCamera = cam;}
	void setRenderer(vtkRenderer *renderer){ mRenderer = renderer;}
	void setTextScale(double scale){ mTextScale = scale;}

	void showPointIds(vtkPoints *points);

protected:
	vtkCamera *mCamera;
	vtkRenderer *mRenderer;
	double mTextScale;
};

#endif /* TEXTIDSATPOINTS_H_ */
