/*
 * TextIdsAtPoints.cpp
 *
 *  Created on: Nov 16, 2011
 *      Author: papazov
 */

#include "TextIdsAtPoints.h"
#include <vtkPolyDataMapper.h>
#include <vtkVectorText.h>
#include <vtkFollower.h>
#include <vtkProperty.h>

TextIdsAtPoints::TextIdsAtPoints()
{
}

TextIdsAtPoints::~TextIdsAtPoints()
{
}

//===========================================================================================================================

void TextIdsAtPoints::showPointIds(vtkPoints* points)
{
	double p[3];
	char text[16];

	for ( int i = 0 ; i < points->GetNumberOfPoints() ; ++i )
	{
		sprintf(text, "%i", i);
		vtkVectorText *vtk_text = vtkVectorText::New();
		  vtk_text->SetText(text);
		vtkPolyDataMapper *vtk_text_mapper = vtkPolyDataMapper::New();
		  vtk_text_mapper->SetInputConnection(vtk_text->GetOutputPort());
		vtkFollower *vtk_follower = vtkFollower::New();
		  vtk_follower->SetMapper(vtk_text_mapper);
		  vtk_follower->SetScale(mTextScale, mTextScale, mTextScale);
		  points->GetPoint(i, p);
		  vtk_follower->SetPosition(p);
		  vtk_follower->GetProperty()->SetColor(0.0, 0.0, 0.0);

		// Add to the renderer and camera
		vtk_follower->SetCamera(mCamera);
		mRenderer->AddActor(vtk_follower);

		// Cleanup
		vtk_text->Delete();
		vtk_text_mapper->Delete();
		vtk_follower->Delete();
	}
}

//===========================================================================================================================
