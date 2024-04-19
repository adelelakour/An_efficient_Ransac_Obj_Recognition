/*
 * FK3dImage.cpp
 *
 *  Created on: Nov 30, 2010
 *      Author: papazov
 */

// Copyright (C) 2010 Chavdar Papazov

// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along with
// this program. If not, see http://www.gnu.org/licenses/.

#include "../include/FK3dImage.h"

FK3dImage::FK3dImage(int width, int height)
{
	mPixels = NULL;
	this->alloc(width, height);
}

//==========================================================================================================================

void FK3dImage::alloc(int width, int height)
{
	this->clear();

	mWidth = width;
	mHeight = height;

	mPixels = new FK3dPixel*[width];
	for ( int i = 0 ; i < width ; ++i )
		mPixels[i] = new FK3dPixel[height];
}

//==========================================================================================================================

void FK3dImage::clear()
{
	if ( mPixels )
	{
		for ( int i = 0 ; i < mWidth ; ++i )
			delete[] mPixels[i];

		delete[] mPixels;
		mPixels = NULL;
		mWidth = mHeight = 0;
	}
}

//==========================================================================================================================

FK3dImage::~FK3dImage()
{
	this->clear();
}

//==========================================================================================================================
