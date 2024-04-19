/*
 * ORRRangeImage2vtk.cpp
 *
 *  Created on: Jun 9, 2010
 *      Author: papazov
 */

#include "ORRRangeImage2vtk.h"
#include <BasicToolsVis/DataStructures/Box2vtk.h>
#include <BasicTools/DataStructures/Box.h>

ORRRangeImage2vtk::ORRRangeImage2vtk()
{
}

ORRRangeImage2vtk::~ORRRangeImage2vtk()
{
}

//=================================================================================================================

void ORRRangeImage2vtk::getAsPolygons(const ORRRangeImage2* image, vtkPolyData* out)
{
	int i, j, width = image->width(), height = image->height();
	const double_2*** pixels = image->getPixels();
	double pixelBounds[6];
	list<Box> boxes;
	Box2vtk box2vtk;

	for ( i = 0 ; i < width ; ++i )
	{
		for ( j = 0 ; j < height ; ++j )
		{
			if ( pixels[i][j] == NULL )
				continue;

			image->getPixelBounds(pixelBounds, i, j);

			Box box(3);
			  box.set(0, pixelBounds[0], pixelBounds[1]);
			  box.set(1, pixelBounds[2], pixelBounds[3]);
			  box.set(2, pixelBounds[4], pixelBounds[5]);
			  boxes.push_back(box);
		}
	}

	box2vtk.getOutputAsPolygons(boxes, out);
}

//=================================================================================================================

void ORRRangeImage2vtk::getPoints(const ORRRangeImage2* image, vtkPoints* out)
{
	const list<ORRRangeImage2PixelSet*>& full_sets = image->getFullGridSets();
	list<ORRRangeImage2PixelSet*>::const_iterator it;

	for ( it = full_sets.begin() ; it != full_sets.end() ; ++it )
		out->InsertNextPoint((*it)->p());
}

//=================================================================================================================
