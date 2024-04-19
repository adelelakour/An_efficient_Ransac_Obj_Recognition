# Makefile for the FreeKinect3d
# Author:  Chavdar Papazov
# Created: Dec 7, 2010

# Set the path to the libfreenect here:
FREENECT_LIB     = -L/usr/local/lib -lfreenect
FREENECT_INCLUDE = -I/usr/local/include/libfreenect

# If your GL and glut installations are not at the default locations complete the following two lines
INCLUDES = 
LIBS     = -L./lib -lFreeKinect3d -lGL -lGLU -lglut -lpthread


# The rest (not the best way, will be improved ...)
CC       = g++
CFLAGS   = -O3 -Wall
AFLAGS   = -r
ARCHIVER = ar

all: cpp FreeKinect3d.a 
	$(CC) $(CFLAGS) $(FREENECT_INCLUDE) example/main.cpp $(FREENECT_LIB) $(INCLUDES) $(LIBS) -o bin/FreeKinect3d

cpp:
	$(CC) -c $(CFLAGS) $(FREENECT_INCLUDE) $(INCLUDES) src/FreeKinect3d.cpp -o ./obj/FreeKinect3d.o
	$(CC) -c $(CFLAGS) $(FREENECT_INCLUDE) $(INCLUDES) src/FK3dViewer.cpp -o ./obj/FK3dViewer.o
	$(CC) -c $(CFLAGS) $(FREENECT_INCLUDE) $(INCLUDES) src/FK3dImage.cpp -o ./obj/FK3dImage.o

FreeKinect3d.a:
	$(ARCHIVER) $(AFLAGS) ./lib/libFreeKinect3d.a ./obj/FreeKinect3d.o ./obj/FK3dViewer.o ./obj/FK3dImage.o

clean:
	rm -f bin/* lib/* obj/*
