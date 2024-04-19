/*
 * FreeKinect3d.cpp
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

#include "../include/FreeKinect3d.h"
#include "../include/FK3dViewer.h"

const double FreeKinect3d::mMat3d[4][4] =
{{0.017857142857142856585267454150312005368,  0.0000000000000000000000000000000000000000, 0.0000000000000000000000000000000000000000, -5.714285714285713968507707249955274164677},
 {0.0000000000000000000000000000000000000000, 0.017857142857142856585267454150312005368,  0.0000000000000000000000000000000000000000, -4.285714285714285476380780437466455623507},
 {0.0000000000000000000000000000000000000000, 0.0000000000000000000000000000000000000000, 0.0000000000000000000000000000000000000000, -10.000000000000000000000000000000000000000},
 {0.0000000000000000000000000000000000000000, 0.0000000000000000000000000000000000000000,-0.0000290697674418604667457340789971098616,  0.0316860465116279091390971700548107037321}};

FreeKinect3d* FreeKinect3d::mKinect3d = NULL;
FK3dImage* FreeKinect3d::m3dImageDst = NULL;

pthread_mutex_t FreeKinect3d::mMutexDepth = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FreeKinect3d::mMutexRGB = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FreeKinect3d::mMutex3dImage = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t FreeKinect3d::mCond3dImage = PTHREAD_COND_INITIALIZER;

uint8_t FreeKinect3d::mRGB[FREENECT_VIDEO_RGB_SIZE];
freenect_depth FreeKinect3d::mDepth[FREENECT_FRAME_PIX];

FreeKinect3d::FreeKinect3d()
{
	mFreenectContext = NULL;
	mFreenectDevice = NULL;
	mGoOn = true;
}

FreeKinect3d::~FreeKinect3d()
{
}

//===============================================================================================================

void FreeKinect3d::addGL3dDrawer(GL3dDrawer* drawer)
{
	FK3dViewer::addGL3dDrawer(drawer);
}

//===============================================================================================================

void FreeKinect3d::depthCallback(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	pthread_mutex_lock(&mMutexDepth);
	memcpy(mDepth, v_depth, FREENECT_FRAME_PIX*sizeof(freenect_depth));
	pthread_mutex_unlock(&mMutexDepth);

	pthread_mutex_lock(&mMutex3dImage);
	if ( m3dImageDst ) // Someone wants to have a 3d image
	{
		freenect_depth *depth = (freenect_depth*)v_depth;
		double p[3];
		// Copy the depth data to a Kinect3dImage object
		for ( int j = 0, k = 0 ; j < FREENECT_FRAME_H ; ++j )
			for ( int i = 0 ; i < FREENECT_FRAME_W ; ++i, ++k )
				if ( depth[k] != FK3_INVALID_DEPTH )
				{
					FreeKinect3d::comp3dPoint((double)i + 0.5, (double)j + 0.5, (double)depth[k], p);
					m3dImageDst->setPixel(i, j, p);
				}
				else
					m3dImageDst->setInvalid(i, j);

		// Ready with the copy process -> set to NULL again
		m3dImageDst = NULL;
		pthread_cond_signal(&mCond3dImage);
	}
	pthread_mutex_unlock(&mMutex3dImage);
}

//===============================================================================================================

void FreeKinect3d::rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&mMutexRGB);
	memcpy(mRGB, rgb, FREENECT_VIDEO_RGB_SIZE);
	pthread_mutex_unlock(&mMutexRGB);
}

//===============================================================================================================

void *FreeKinect3d::freenectThreadFunc(void *arg)
{
	FreeKinect3d* kinect = FreeKinect3d::Get();

	freenect_set_led(kinect->mFreenectDevice, LED_RED);
	freenect_set_depth_callback(kinect->mFreenectDevice, FreeKinect3d::depthCallback);
	freenect_set_video_callback(kinect->mFreenectDevice, FreeKinect3d::rgbCallback);
	// Instead of freenect_set_video_format(kinect->mFreenectDevice, FREENECT_VIDEO_RGB);
	if ( freenect_set_video_mode(kinect->mFreenectDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB)) < 0 )
	{
		fprintf(stderr, "ERROR in 'FreeKinect3d::%s()': can not set the right video mode.\n", __func__);
		return NULL;
	}
	//Instead of freenect_set_depth_format(kinect->mFreenectDevice, FREENECT_DEPTH_11BIT);
	if ( freenect_set_depth_mode(kinect->mFreenectDevice, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT)) < 0 )
	{
		fprintf(stderr, "ERROR in 'FreeKinect3d::%s()': can not set the right depth mode.\n", __func__);
		return NULL;
	}

	freenect_start_depth(kinect->mFreenectDevice);
	freenect_start_video(kinect->mFreenectDevice);

	printf("Starting the Kinect loop ...\n");

	while( kinect->mGoOn && freenect_process_events(kinect->mFreenectContext) >= 0 )
		;// do nothing

	printf("\nShutting down streams ... "); fflush(stdout);

	freenect_stop_depth(kinect->mFreenectDevice);
	freenect_stop_video(kinect->mFreenectDevice);

	freenect_close_device(kinect->mFreenectDevice);
	freenect_shutdown(kinect->mFreenectContext);

	printf("done.\n");
	return NULL;
}

//===============================================================================================================

bool FreeKinect3d::init()
{
	if ( freenect_init(&mFreenectContext, NULL) < 0 )
	{
		fprintf(stderr, "ERROR: 'freenect_init()' failed.\n"); fflush(stderr);
		return false;
	}

	freenect_set_log_level(mFreenectContext, FREENECT_LOG_FATAL);
	if ( freenect_num_devices(mFreenectContext) < 1 )
	{
		fprintf(stderr, "ERROR: No devices found.\n"); fflush(stderr);
		return false;
	}

	int user_device_number = 0;
	if ( freenect_open_device(mFreenectContext, &mFreenectDevice, user_device_number) < 0 )
	{
		fprintf(stderr, "ERROR: Could not open device.\n"); fflush(stderr);
		return false;
	}

	printf("\nKinect device: 'UP'-tilt up, 'LEFT'/'RIGHT'-level, 'DOWN'-tilt down.\n"); fflush(stdout);

	int thread_res = pthread_create(&mFreenectThread, NULL, FreeKinect3d::freenectThreadFunc, NULL);
	if ( thread_res )
	{
		fprintf(stderr, "ERROR: 'pthread_create()' failed.\n"); fflush(stderr);
		return false;
	}

	return true;
}

//===============================================================================================================

void FreeKinect3d::start()
{
	if ( !this->init() )
	{
		fprintf(stderr, "ERROR in 'FreeKinect3d::%s()': Could not initialize Kinect.\n", __func__);
		fflush(stderr);
		return;
	}

	FK3dViewer::start(FreeKinect3d::Get());
}

//===============================================================================================================
