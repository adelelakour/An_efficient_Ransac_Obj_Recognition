/*
 * FK3dViewer.cpp
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

#include "../include/FK3dViewer.h"
#include "../include/FreeKinect3d.h"
#include "../include/FK3dImage.h"
#include "../include/FK3dUtils.h"
#include <GL/freeglut.h>
#include <cmath>
#include <list>

using namespace std;

FreeKinect3d* FK3dViewer::mFreeKinect3d = NULL;
list<GL3dDrawer*> FK3dViewer::mGL3dDrawers;

FK3dImage FK3dViewer::m3dImage(FREENECT_FRAME_W, FREENECT_FRAME_H);

int FK3dViewer::m2dWindow;
int FK3dViewer::m3dWindow;
int FK3dViewer::mKinectCamAngle;

uint8_t FK3dViewer::mGLDepth[FREENECT_VIDEO_RGB_SIZE];
uint8_t FK3dViewer::mGLRGB[FREENECT_VIDEO_RGB_SIZE];
freenect_depth FK3dViewer::mDepth[FREENECT_FRAME_PIX];
uint16_t FK3dViewer::mGamma[2048];

bool FK3dViewer::mDragLeftBtn = false;
bool FK3dViewer::mDragRightBtn = false;
bool FK3dViewer::mDragMiddleBtn = false;

bool FK3dViewer::mRenderPoints = true;
bool FK3dViewer::mRenderMesh = false;

bool FK3dViewer::mRenderLookAtPoint = false;

double FK3dViewer::mCamPos[3] = {-50.0, -100.0, -1500.0};
double FK3dViewer::mCamLookAt[3] = {0.0, 0.0, 0.0};
double FK3dViewer::mCamLookup[3] = {0.0, -1.0, 0.0};
double FK3dViewer::mMaxZDiscontinuity = 10.0;
int FK3dViewer::mMouseX;
int FK3dViewer::mMouseY;

//================================================================================================================================

FK3dViewer::FK3dViewer()
{
}

FK3dViewer::~FK3dViewer()
{
}

//================================================================================================================================

void FK3dViewer::draw2d()
{
	mFreeKinect3d->getDepth(mDepth);
	mFreeKinect3d->getRGB(mGLRGB);

	for ( int i = 0 ; i < FREENECT_FRAME_PIX ; ++i )
	{
		int pval = mGamma[mDepth[i]];
		int lb = pval & 0xff;
		switch ( pval >> 8 )
		{
			case 0:
				mGLDepth[3*i+0] = 255;
				mGLDepth[3*i+1] = 255-lb;
				mGLDepth[3*i+2] = 255-lb;
				break;

			case 1:
				mGLDepth[3*i+0] = 255;
				mGLDepth[3*i+1] = lb;
				mGLDepth[3*i+2] = 0;
				break;

			case 2:
				mGLDepth[3*i+0] = 255-lb;
				mGLDepth[3*i+1] = 255;
				mGLDepth[3*i+2] = 0;
				break;

			case 3:
				mGLDepth[3*i+0] = 0;
				mGLDepth[3*i+1] = 255;
				mGLDepth[3*i+2] = lb;
				break;

			case 4:
				mGLDepth[3*i+0] = 0;
				mGLDepth[3*i+1] = 255-lb;
				mGLDepth[3*i+2] = 255;
				break;

			case 5:
				mGLDepth[3*i+0] = 0;
				mGLDepth[3*i+1] = 0;
				mGLDepth[3*i+2] = 255-lb;
				break;

			default:
				mGLDepth[3*i+0] = 0;
				mGLDepth[3*i+1] = 0;
				mGLDepth[3*i+2] = 0;
				break;
		}
	}

	// Local buffers to draw
	uint8_t depth_flipped[3*FREENECT_FRAME_PIX], rgb_flipped[3*FREENECT_FRAME_PIX];
	// Some pointers
	uint8_t *rgb_front = rgb_flipped, *rgb_back = mGLRGB + 3*FREENECT_FRAME_H*FREENECT_FRAME_W - 3*FREENECT_FRAME_W;
	uint8_t *depth_front = depth_flipped, *depth_back = mGLDepth + 3*FREENECT_FRAME_H*FREENECT_FRAME_W - 3*FREENECT_FRAME_W;
	int line_size = FREENECT_FRAME_W*3*sizeof(uint8_t);

	// Flip the images
	for ( int i = 0 ; i < FREENECT_FRAME_H ; ++i )
	{
		memcpy(rgb_front, rgb_back, line_size);
		memcpy(depth_front, depth_back, line_size);

		// Go to the next line
		rgb_front += 3*FREENECT_FRAME_W;
		rgb_back -= 3*FREENECT_FRAME_W;
		depth_front += 3*FREENECT_FRAME_W;
		depth_back -= 3*FREENECT_FRAME_W;
	}

	glutSetWindow(m2dWindow);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDrawBuffer(GL_BACK);

	// Draw the depth image
	glRasterPos2i(0, 0);
	glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, depth_flipped);
	// Draw the rgb image
	glRasterPos2i(641, 0);
	glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, rgb_flipped);

	glutSwapBuffers();
	glFlush();
}

//================================================================================================================================

void FK3dViewer::draw3d()
{
	// Do visualization stuff (can take longer since OpenGL thread)
	glutSetWindow(FK3dViewer::m3dWindow);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(mCamPos[0], mCamPos[1], mCamPos[2], mCamLookAt[0], mCamLookAt[1], mCamLookAt[2], mCamLookup[0], mCamLookup[1], mCamLookup[2]);
	glEnable(GL_DEPTH_TEST);

	if ( mRenderMesh )
	{
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		// Get the 3d information
		mFreeKinect3d->get3dImage(&m3dImage);
		FK3dPixel** pixels = m3dImage.getPixels();
		double n[3], u[3], v[3];

		// Compute a mesh (very simple procedure: just connect neighboring points which have similar z-coordinates)
		glBegin(GL_TRIANGLES);
		for ( int j = 0 ; j < FREENECT_FRAME_H-1 ; ++j )
			for ( int i = 0 ; i < FREENECT_FRAME_W-1 ; ++i )
				if ( pixels[i][j].isValid() )
				{
					// First triangle and first edges
					if ( pixels[i][j+1].isValid() && pixels[i+1][j+1].isValid() )
					{
						if ( fabs(pixels[i][j  ].p()[2]-pixels[i  ][j+1].p()[2]) <= mMaxZDiscontinuity &&
							 fabs(pixels[i][j  ].p()[2]-pixels[i+1][j+1].p()[2]) <= mMaxZDiscontinuity &&
							 fabs(pixels[i][j+1].p()[2]-pixels[i+1][j+1].p()[2]) <= mMaxZDiscontinuity )
						{
							// Compute the normal for the triangle
							utils_subtract3(pixels[i][j+1].p(), pixels[i][j].p(), u);
							utils_subtract3(pixels[i+1][j+1].p(), pixels[i][j].p(), v);
							utils_cross_prod3(u, v, n);
							utils_normalize3(n);
							// Use the normal and define the points
							glNormal3f(n[0], n[1], n[2]);
							glVertex3f(pixels[i  ][j  ].p()[0], pixels[i  ][j  ].p()[1], pixels[i  ][j  ].p()[2]);
							glVertex3f(pixels[i  ][j+1].p()[0], pixels[i  ][j+1].p()[1], pixels[i  ][j+1].p()[2]);
							glVertex3f(pixels[i+1][j+1].p()[0], pixels[i+1][j+1].p()[1], pixels[i+1][j+1].p()[2]);
						}
					}
					// Second triangle and second group of edges
					if ( pixels[i+1][j].isValid() && pixels[i+1][j+1].isValid() )
					{
						if ( fabs(pixels[i  ][j].p()[2]-pixels[i+1][j  ].p()[2]) < mMaxZDiscontinuity &&
							 fabs(pixels[i  ][j].p()[2]-pixels[i+1][j+1].p()[2]) < mMaxZDiscontinuity &&
							 fabs(pixels[i+1][j].p()[2]-pixels[i+1][j+1].p()[2]) < mMaxZDiscontinuity )
						{
							// Compute the normal for the triangle
							utils_subtract3(pixels[i+1][j+1].p(), pixels[i][j].p(), u);
							utils_subtract3(pixels[i+1][j].p(), pixels[i][j].p(), v);
							utils_cross_prod3(u, v, n);
							utils_normalize3(n);
							// Use the normal and define the points
							glNormal3f(n[0], n[1], n[2]);
							glVertex3f(pixels[i  ][j  ].p()[0], pixels[i  ][j  ].p()[1], pixels[i  ][j  ].p()[2]);
							glVertex3f(pixels[i+1][j  ].p()[0], pixels[i+1][j  ].p()[1], pixels[i+1][j  ].p()[2]);
							glVertex3f(pixels[i+1][j+1].p()[0], pixels[i+1][j+1].p()[1], pixels[i+1][j+1].p()[2]);
						}
					}
				}
		glEnd();

		glDisable(GL_LIGHTING);
		glDisable(GL_LIGHT0);
	}

	if ( mRenderPoints )
	{
		glColor3f(0.2, 0.3, 0.4);
		glPointSize(1);

		if ( mRenderMesh ) // Use the 3d image acquired for the mesh rendering
		{
			FK3dPixel** pixels = m3dImage.getPixels();
			glBegin(GL_POINTS);
			for ( int j = 0 ; j < FREENECT_FRAME_H ; ++j )
				for ( int i = 0 ; i < FREENECT_FRAME_W ; ++i )
					if ( pixels[i][j].isValid() )
						glVertex3f(pixels[i][j].p()[0], pixels[i][j].p()[1], pixels[i][j].p()[2]);
			glEnd();
		}
		else
		{
			// Get the depth and compute the 3d points
			mFreeKinect3d->getDepth(mDepth);
			double p[3];

			glBegin(GL_POINTS);
			for ( int j = 0, k = 0 ; j < FREENECT_FRAME_H ; ++j )
				for ( int i = 0 ; i < FREENECT_FRAME_W ; ++i, ++k )
					if ( mDepth[k] != FK3_INVALID_DEPTH )
					{
						FreeKinect3d::comp3dPoint((double)i + 0.5, (double)j + 0.5, (double)mDepth[k], p);
						glVertex3f(p[0], p[1], p[2]);
					}
			glEnd();
		}
	}

	for ( list<GL3dDrawer*>::iterator it = mGL3dDrawers.begin() ; it != mGL3dDrawers.end() ; ++it )
		(*it)->draw3d();

	if ( mRenderLookAtPoint )
	{
		glPointSize(5);
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_POINTS);
		  glVertex3f(mCamLookAt[0], mCamLookAt[1], mCamLookAt[2]);
		glEnd();
	}

	glDisable(GL_DEPTH_TEST);

	// Draw the coordinate system
	double ale = 300.0;
	glBegin(GL_LINES);
	  // x-axis
	  glColor3f(1.0, 0.0, 0.0);
	  glVertex3f(0.0, 0.0, 0.0);
	  glVertex3f(ale, 0.0, 0.0);
	  // y-axis
	  glColor3f(0.0, 1.0, 0.0);
	  glVertex3f(0.0, 0.0, 0.0);
	  glVertex3f(0.0, ale, 0.0);
	  // z-axis
	  glColor3f(0.0, 0.0, 1.0);
	  glVertex3f(0.0, 0.0, 0.0);
	  glVertex3f(0.0, 0.0, ale);
	glEnd();

	glutSwapBuffers();
	glFlush();
}

//================================================================================================================================

void FK3dViewer::resize2dWin(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 1280, 0, 480, -1.0f, 1.0f);
}

//================================================================================================================================

void FK3dViewer::resize3dWin(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0, (double)width/(double)height, 20.0, 8000.0);
}

//================================================================================================================================

void FK3dViewer::mouse3dWin(int button, int state, int x, int y)
{
	mDragLeftBtn = mDragRightBtn = mDragMiddleBtn = false;
	mMouseX = x;
	mMouseY = y;

	if ( state == GLUT_UP )
	{
		double zoom_factor = 90.0;

		if ( button == 3 ) // zoom in (wheel event)
		{
			const double old_view_dir[3] = {mCamLookAt[0] - mCamPos[0], mCamLookAt[1] - mCamPos[1], mCamLookAt[2] - mCamPos[2]};
			double move_dir[3];
			utils_normalize3(old_view_dir, move_dir);
			utils_mult3(move_dir, zoom_factor);
			double new_cam_pos[3]; utils_sum3(mCamPos, move_dir, new_cam_pos);
			double new_view_dir[3]; utils_subtract3(mCamLookAt, new_cam_pos, new_view_dir);
			// If we would NOT overshoot -> move to the new position
			if ( utils_dot_prod3(new_view_dir, old_view_dir) > 0.0 )
				utils_add3(mCamPos, move_dir);
			glutPostRedisplay();
		}
		if ( button == 4 ) // zoom out (wheel event)
		{
			const double old_view_dir[3] = {mCamLookAt[0] - mCamPos[0], mCamLookAt[1] - mCamPos[1], mCamLookAt[2] - mCamPos[2]};
			double move_dir[3];
			utils_normalize3(old_view_dir, move_dir);
			utils_mult3(move_dir, -zoom_factor);
			double new_cam_pos[3]; utils_sum3(mCamPos, move_dir, new_cam_pos);
			double new_view_dir[3]; utils_subtract3(mCamLookAt, new_cam_pos, new_view_dir);
			utils_add3(mCamPos, move_dir);
			glutPostRedisplay();
		}
		return;
	}

	switch ( button )
	{
		case GLUT_LEFT_BUTTON:
			mDragLeftBtn = true;
			break;

		case GLUT_RIGHT_BUTTON:
			mDragRightBtn = true;
			break;

		case GLUT_MIDDLE_BUTTON:
			mDragMiddleBtn = true;
			break;
	}
}

//================================================================================================================================

void FK3dViewer::motion3dWin(int x, int y)
{
	if ( mDragLeftBtn ) // rotate
	{
		// Use the x difference to rotate about the y axis
		utils_rot_about_y_axis(((double)(mMouseX-x))*0.005, mCamPos, mCamLookAt);

		// Use the y difference to rotate
		double axis[3], view_dir[3] = {mCamLookAt[0] - mCamPos[0], mCamLookAt[1] - mCamPos[1], mCamLookAt[2] - mCamPos[2]};
		utils_cross_prod3(view_dir, mCamLookup, axis);
		utils_normalize3(axis);
		double new_cam_pos[3] = {mCamPos[0], mCamPos[1], mCamPos[2]};
		utils_rot_about_axis(((double)(mMouseY-y))*0.005, axis, mCamLookAt, new_cam_pos);

		double new_view_dir[3];
		utils_subtract3(mCamLookAt, new_cam_pos, new_view_dir);
		utils_normalize3(new_view_dir);

		if ( fabs(utils_dot_prod3(new_view_dir, mCamLookup)) <= 0.999 )
			utils_copy3(mCamPos, new_cam_pos);
	}

	if ( mDragRightBtn ) // zoom and move the lookAt point
	{
		const double old_view_dir[3] = {mCamLookAt[0] - mCamPos[0], mCamLookAt[1] - mCamPos[1], mCamLookAt[2] - mCamPos[2]};
		double move_dir[3];
		utils_normalize3(old_view_dir, move_dir);
		utils_mult3(move_dir, (double)(mMouseY-y)*5.0);

		utils_add3(mCamPos, move_dir);
		utils_add3(mCamLookAt, move_dir);
	}

	if ( mDragMiddleBtn ) // pan
	{
		const double view_dir[3] = {mCamLookAt[0] - mCamPos[0], mCamLookAt[1] - mCamPos[1], mCamLookAt[2] - mCamPos[2]};
		double pan_dir_x[3], pan_dir_y[3];
		utils_cross_prod3(mCamLookup, view_dir, pan_dir_x);
		utils_cross_prod3(view_dir, pan_dir_x, pan_dir_y);
		// Normalize the pan directions
		utils_normalize3(pan_dir_x);
		utils_normalize3(pan_dir_y);
		// Determine to pan to the left or to the right
		utils_mult3(pan_dir_x, (double)(x-mMouseX)*1.6);
		utils_mult3(pan_dir_y, (double)(y-mMouseY)*1.6);
		// Move
		utils_add3(mCamPos, pan_dir_x);
		utils_add3(mCamPos, pan_dir_y);
		utils_add3(mCamLookAt, pan_dir_x);
		utils_add3(mCamLookAt, pan_dir_y);
	}

	mMouseX = x;
	mMouseY = y;

	glutPostRedisplay();
}

//================================================================================================================================

void FK3dViewer::idleFunc()
{
	FK3dViewer::draw2d();
	FK3dViewer::draw3d();
}

//================================================================================================================================

void FK3dViewer::keyPressed(unsigned char key, int x, int y)
{
	switch ( key )
	{
		case 27:
			FreeKinect3d::Get()->stop();
			pthread_join(FreeKinect3d::Get()->getFreenectThread(), NULL);
			glutDestroyWindow(FK3dViewer::m2dWindow);
			glutDestroyWindow(FK3dViewer::m3dWindow);
		break;

		case 'm':
		case 'M':
			mRenderMesh = !mRenderMesh;
			glutPostRedisplay();
			break;

		case 'p':
		case 'P':
			mRenderPoints = !mRenderPoints;
			glutPostRedisplay();
			break;

		case 'l':
		case 'L':
			mRenderLookAtPoint = !mRenderLookAtPoint;
			glutPostRedisplay();
			break;
	}
}

//================================================================================================================================

void FK3dViewer::specialFunc(int key, int x, int y)
{
	switch ( key )
	{
		case GLUT_KEY_UP:
			++mKinectCamAngle;
			FreeKinect3d::Get()->setTiltDegrees(mKinectCamAngle);
		break;

		case GLUT_KEY_DOWN:
			--mKinectCamAngle;
			FreeKinect3d::Get()->setTiltDegrees(mKinectCamAngle);
		break;

		case GLUT_KEY_RIGHT:
		case GLUT_KEY_LEFT:
			mKinectCamAngle = 0;
			FreeKinect3d::Get()->setTiltDegrees(mKinectCamAngle);
		break;
	}
}

//================================================================================================================================

void FK3dViewer::initGL()
{
	printf("Initializing GL ... "); fflush(stdout);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glShadeModel(GL_FLAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	GLfloat ambient[4] = {0.2, 0.25, 0.3, 1.0};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

	printf("done.\n");
}

//================================================================================================================================

void FK3dViewer::start(FreeKinect3d* fk3d)
{
	mFreeKinect3d = fk3d;
	mKinectCamAngle = 0;

	printf("\n3D scene:\n"
			"\t'Esc' - quit\n"
			"\t'm' - switch on/off mesh rendering\n"
			"\t'p' - switch on/off point rendering\n"
			"\t'l' - shows/hides camera look-at-point\n"
			"\trotate: left mouse button\n"
			"\tpan: middle mouse button\n"
			"\tzoom: wheel (does NOT move the camera look-at-point)\n"
			"\tzoom: right mouse button (moves the camera look-at-point)\n");

	// For the depth-to-color mapping
	for ( int i = 0; i < 2048 ; ++i )
	{
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		mGamma[i] = v*6*256;
	}

	// Some dummy variables
	int argc = 1; char *argv = new char[256]; sprintf(argv, "FreeKinect3D");
	glutInit(&argc, &argv);
	// This one crashes (at least on fedora 16)
//	FK3dViewer::initGL();

	// The 2d window
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(1280, 480);
	glutInitWindowPosition(0, 0);
	m2dWindow = glutCreateWindow("FreeKinect3D Depth and RGB");
	glutDisplayFunc(&FK3dViewer::draw2d);
	glutReshapeFunc(&FK3dViewer::resize2dWin);
	glutKeyboardFunc(&FK3dViewer::keyPressed);
	glutSpecialFunc(&FK3dViewer::specialFunc);

	// The 3d window
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(1000, 800);
	m3dWindow = glutCreateWindow("FreeKinect3D (powered by TUM)");
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glutPositionWindow(10, 10);
	glutDisplayFunc(&FK3dViewer::draw3d);
	glutReshapeFunc(&FK3dViewer::resize3dWin);
	glutMouseFunc(&FK3dViewer::mouse3dWin);
	glutMotionFunc(&FK3dViewer::motion3dWin);
	glutKeyboardFunc(&FK3dViewer::keyPressed);
	glutSpecialFunc(&FK3dViewer::specialFunc);

	// For all windows
	glutIdleFunc(&idleFunc);

	printf("Entering the GLUT main loop ...\n");

	glutMainLoop();

	// Cleanup
	delete[] argv;
}

//================================================================================================================================
