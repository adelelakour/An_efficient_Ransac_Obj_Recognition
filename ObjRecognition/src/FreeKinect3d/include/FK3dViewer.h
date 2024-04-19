/*
 * FK3dViewer.h
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

#ifndef _FK3D_VIEWER_H_
#define _FK3D_VIEWER_H_

#include "FreeKinect3d.h"
#include "FK3dImage.h"
#include "GL3dDrawer.h"
#include <libfreenect.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <pthread.h>
#include <cstring>
#include <list>

using namespace std;


class FK3dViewer
{
public:
	static void renderMeshOn(){ mRenderMesh = true;}
	static void renderMeshOff(){ mRenderMesh = false;}
	static void renderPointOn(){ mRenderPoints = true;}
	static void renderPointOff(){ mRenderPoints = false;}
	/** In millimeter. */
	static void setMaxZDiscontinuity(double value){ mMaxZDiscontinuity = value;}
	/** Computes a 3d image based on the depth saved in 'input_depth', which is a double-array and not an uint_16-array. This makes
	  * sense if you call 'this->getDepth()' K times and build a mean depth-array over the K arrays in order to get better accuracy.
	  * In this case you need a floating point type of array. Make sure that 'result' is a valid pointer! */
	static inline void compute3dImage(double input_depth[FREENECT_FRAME_PIX], FK3dImage* result);
	/** The method 'drawer->draw3d()' will be called every time in the 3d display callback (before the 3d coordinate frame is drawn). */
	static void addGL3dDrawer(GL3dDrawer* drawer){ mGL3dDrawers.push_back(drawer);}

protected:
	FK3dViewer();
	virtual ~FK3dViewer();

	static void start(FreeKinect3d* fk3d);

	static void initGL();

	// glut callbacks
	static void keyPressed(unsigned char key, int x, int y);
	static void idleFunc();
	static void draw2d();
	static void draw3d();
	static void resize2dWin(int width, int height);
	static void resize3dWin(int width, int height);
	static void motion3dWin(int x, int y);
	static void mouse3dWin(int button, int state, int x, int y);
	static void specialFunc(int key, int x, int y);

	friend class FreeKinect3d;

protected:
	static FreeKinect3d* mFreeKinect3d;

	/** If this object is not NULL its drawing method will be called before the 3d coordinate frame is drawn in 'draw3d()'. */
	static list<GL3dDrawer*> mGL3dDrawers;
	/** This image is used for mesh visualization. */
	static FK3dImage m3dImage;
	/** This array is used only for the depth-visualization by OpenGL. */
	static uint8_t mGLDepth[FREENECT_VIDEO_RGB_SIZE];
	/** This is the real Kinect depth. */
	static freenect_depth mDepth[FREENECT_FRAME_PIX];
	/** This is used for RGB-visualization by OpenGL. */
	static uint8_t mGLRGB[FREENECT_VIDEO_RGB_SIZE];
	/** Only for depth-visualization. */
	static uint16_t mGamma[2048];

	static bool mDragLeftBtn, mDragRightBtn, mDragMiddleBtn, mRenderMesh, mRenderPoints, mRenderLookAtPoint;
	static double mCamPos[3], mCamLookAt[3], mCamLookup[3], mMaxZDiscontinuity;
	static int mMouseX, mMouseY, m2dWindow, m3dWindow, mKinectCamAngle;
};

//=== inline methods =================================================================================================

#endif /* _FK3D_VIEWER_H_ */
