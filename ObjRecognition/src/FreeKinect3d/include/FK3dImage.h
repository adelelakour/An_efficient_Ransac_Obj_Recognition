/*
 * FK3dImage.h
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

#ifndef _FK3D_IMAGE_H_
#define _FK3D_IMAGE_H_

#include "FK3dPixel.h"
#include <libfreenect.h>
#include <cstdio>

#define FREENECT_FRAME_W         640
#define FREENECT_FRAME_H         480
#define FREENECT_FRAME_PIX       FREENECT_FRAME_W*FREENECT_FRAME_H
#define FREENECT_VIDEO_RGB_SIZE  3*FREENECT_FRAME_PIX

class FK3dImage
{
public:
	FK3dImage(int width = FREENECT_FRAME_W, int height = FREENECT_FRAME_H);
	virtual ~FK3dImage();

	void alloc(int width, int height);

	FK3dPixel** getPixels(){ return mPixels;}
	const FK3dPixel** getPixels_const()const{ return (const FK3dPixel**)mPixels;}

	void setPixel(int i, int j, const double* p){ mPixels[i][j].set(p);}
	void setInvalid(int i, int j){ mPixels[i][j].setInvalid();}

	int computeLinearId(int x, int y){ return y*mWidth + x;}

	int width()const{ return mWidth;}
	int height()const{ return mHeight;}

protected:
	void clear();

protected:
	FK3dPixel** mPixels;
	int mWidth, mHeight;
};

//=== inline methods =================================================================================================

#endif /* _FK3D_IMAGE_H_ */
