/*
 * FreeKinect3d.h
 *
 *  Created on: Nov 25, 2010
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

#ifndef _FREE_KINECT_3D_H_
#define _FREE_KINECT_3D_H_

#include "FK3dImage.h"
#include "GL3dDrawer.h"
#include <libfreenect.h>
#include <cstdio>
#include <cstring>
#include <pthread.h>

#define FK3_INVALID_DEPTH  2047

typedef uint8_t freenect_pixel;
typedef uint16_t freenect_depth;


class FreeKinect3d
{
protected:
	FreeKinect3d();
	virtual ~FreeKinect3d();

public:
	inline static FreeKinect3d* Get();
	inline static void Destroy();

	/** Call this method within the main thread. */
	void start();
	void stop(){ mGoOn = false;}
	const pthread_t getFreenectThread(){ return mFreenectThread;}

	inline void setTiltDegrees(int& degrees);
	inline void get3dImage(FK3dImage* dst);
	inline void getDepth(freenect_depth depth[FREENECT_FRAME_PIX]);
	inline void getRGB(uint8_t rgb[FREENECT_VIDEO_RGB_SIZE]);

	static inline void comp3dPoint(double x, double y, double z, double *out);

	/** 'drawer->draw3d()' will be called within the OpenGL display callback for a window. */
	void addGL3dDrawer(GL3dDrawer* drawer);

protected:
	bool init();
	static void *freenectThreadFunc(void *arg);

	// freenect callbacks
	static void depthCallback(freenect_device *dev, void *v_depth, uint32_t timestamp);
	static void rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp);

protected:
	const static double mMat3d[4][4];
	static FreeKinect3d* mKinect3d;

	static pthread_mutex_t mMutexDepth, mMutexRGB, mMutex3dImage;
	static pthread_cond_t mCond3dImage;

	static uint8_t mRGB[FREENECT_VIDEO_RGB_SIZE];
	static freenect_depth mDepth[FREENECT_FRAME_PIX];

	static FK3dImage* m3dImageDst;
	pthread_t mFreenectThread;
	freenect_context *mFreenectContext;
	freenect_device *mFreenectDevice;
	volatile bool mGoOn;
};

//=== inline methods =================================================================================================

inline FreeKinect3d* FreeKinect3d::Get()
{
	if ( mKinect3d )
		return mKinect3d;

	mKinect3d = new FreeKinect3d();
	return mKinect3d;
}

//====================================================================================================================

inline void FreeKinect3d::Destroy()
{
	if ( mKinect3d )
	{
		delete mKinect3d;
		mKinect3d = NULL;
	}
}

//====================================================================================================================

inline void FreeKinect3d::setTiltDegrees(int& degrees)
{
		 if ( degrees >  30 ) degrees =  30;
	else if ( degrees < -30 ) degrees = -30;

	freenect_set_tilt_degs(mFreenectDevice, degrees);
}

//====================================================================================================================

inline void FreeKinect3d::getDepth(freenect_depth depth[FREENECT_FRAME_PIX])
{
	pthread_mutex_lock(&mMutexDepth);
	memcpy(depth, mDepth, FREENECT_FRAME_PIX*sizeof(freenect_depth));
	pthread_mutex_unlock(&mMutexDepth);
}

//====================================================================================================================

inline void FreeKinect3d::getRGB(uint8_t rgb[FREENECT_VIDEO_RGB_SIZE])
{
	pthread_mutex_lock(&mMutexRGB);
	memcpy(rgb, mRGB, FREENECT_VIDEO_RGB_SIZE);
	pthread_mutex_unlock(&mMutexRGB);
}

//====================================================================================================================

inline void FreeKinect3d::get3dImage(FK3dImage* dst)
{
	pthread_mutex_lock(&mMutex3dImage);
	m3dImageDst = dst;
	pthread_cond_wait(&mCond3dImage, &mMutex3dImage);
	pthread_mutex_unlock(&mMutex3dImage);
}

//====================================================================================================================

inline void FreeKinect3d::comp3dPoint(double x, double y, double z, double *out)
{
	double hx = x*mMat3d[0][0] + y*mMat3d[0][1] + z*mMat3d[0][2] + mMat3d[0][3];
	double hy = x*mMat3d[1][0] + y*mMat3d[1][1] + z*mMat3d[1][2] + mMat3d[1][3];
	double hz = x*mMat3d[2][0] + y*mMat3d[2][1] + z*mMat3d[2][2] + mMat3d[2][3];
	double hw = x*mMat3d[3][0] + y*mMat3d[3][1] + z*mMat3d[3][2] + mMat3d[3][3];

	out[0] =  hx/hw;
	out[1] =  hy/hw;
	out[2] = -hz/hw;
}

//====================================================================================================================
#if 0
inline void FreeKinect3d::compute3dImage(double input_depth[FREENECT_FRAME_PIX], FK3dImage* result)
{
	double p[3];
	for ( int j = 0, k = 0 ; j < FREENECT_FRAME_H ; ++j )
		for ( int i = 0 ; i < FREENECT_FRAME_W ; ++i, ++k )
			if ( input_depth[k] >= 0.0 )
			{
				FreeKinect3d::comp3dPoint((double)i + 0.5, (double)j + 0.5, input_depth[k], p);
				result->setPixel(i, j, p);
			}
			else
				result->setInvalid(i, j);
}
#endif
//====================================================================================================================

#endif /* _FREE_KINECT_3D_H_ */
