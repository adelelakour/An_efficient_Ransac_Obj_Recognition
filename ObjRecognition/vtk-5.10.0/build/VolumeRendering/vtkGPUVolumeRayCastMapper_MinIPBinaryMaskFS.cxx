/* DO NOT EDIT.
 * Generated by ../bin/vtkEncodeString
 * 
 * Define the vtkGPUVolumeRayCastMapper_MinIPBinaryMaskFS string.
 *
 * Generated from file: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/VolumeRendering/vtkGPUVolumeRayCastMapper_MinIPBinaryMaskFS.glsl
 */
#include "vtkGPUVolumeRayCastMapper_MinIPBinaryMaskFS.h"
const char *vtkGPUVolumeRayCastMapper_MinIPBinaryMaskFS =
"/*=========================================================================\n"
"\n"
"  Program:   Visualization Toolkit\n"
"  Module:    vtkGPUVolumeRayCastMapper_MinIPBinaryMaskFS.glsl\n"
"\n"
"  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen\n"
"  All rights reserved.\n"
"  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.\n"
"\n"
"     This software is distributed WITHOUT ANY WARRANTY; without even\n"
"     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR\n"
"     PURPOSE.  See the above copyright notice for more information.\n"
"\n"
"=========================================================================*/\n"
"\n"
"// Fragment program with ray cast and Minimum Intensity Projection (MinIP)\n"
"// method, in the presence of an auxiliary mask. Only regions in the\n"
"// mask texture contribute. Regions outside are as good as completely\n"
"// transparent.\n"
"//\n"
"// Compilation: header part and the projection part are inserted first.\n"
"// pos is defined and initialized in header\n"
"// rayDir is defined in header and initialized in the projection part\n"
"// initMinValue() and writeColorAndMinScalar are defined in some specific\n"
"// file depending on cropping flag being on or off.\n"
"\n"
"#version 110\n"
"\n"
"uniform sampler3D dataSetTexture;\n"
"uniform sampler3D maskTexture;\n"
"uniform sampler1D colorTexture;\n"
"uniform sampler1D opacityTexture;\n"
"\n"
"uniform vec3 lowBounds;\n"
"uniform vec3 highBounds;\n"
"\n"
"// Entry position (global scope)\n"
"vec3 pos;\n"
"// Incremental vector in texture space (global scope)\n"
"vec3 rayDir;\n"
"\n"
"float tMax;\n"
"\n"
"// Sub-functions, depending on cropping mode\n"
"float initialMinValue();\n"
"void writeColorAndMinScalar(vec4 sample,\n"
"                            vec4 opacity,\n"
"                            float minValue);\n"
"\n"
"void trace(void)\n"
"{\n"
"  // Max intensity is the lowest value.\n"
"  float minValue=initialMinValue();\n"
"  bool inside=true;\n"
"  vec4 sample;\n"
"  vec4 maskValue;\n"
"\n"
"  float t=0.0;\n"
"  // We NEED two nested while loops. It is trick to work around hardware\n"
"  // limitation about the maximum number of loops.\n"
"  while(inside)\n"
"    {\n"
"    while(inside)\n"
"      {\n"
"      // get the mask value at this location\n"
"      maskValue=texture3D(maskTexture,pos);\n"
"\n"
"      if(maskValue.a!=0.0)\n"
"        {\n"
"        sample=texture3D(dataSetTexture,pos);\n"
"        minValue=min(minValue,sample.r);\n"
"        }\n"
"\n"
"      pos=pos+rayDir;\n"
"      t+=1.0;\n"
"      inside=t<tMax && all(greaterThanEqual(pos,lowBounds))\n"
"        && all(lessThanEqual(pos,highBounds));\n"
"\n"
"      // yes, t<tMax && all(greaterThanEqual(pos,lowBounds))\n"
"      // && all(lessThanEqual(pos,highBounds));\n"
"      // looks better but the latest nVidia 177.80 has a bug...\n"
"      inside=t<tMax && pos.x>=lowBounds.x && pos.y>=lowBounds.y\n"
"        && pos.z>=lowBounds.z && pos.x<=highBounds.x && pos.y<=highBounds.y\n"
"        && pos.z<=highBounds.z;\n"
"\n"
"      }\n"
"    }\n"
"\n"
"  sample=texture1D(colorTexture,minValue);\n"
"  vec4 opacity=texture1D(opacityTexture,minValue);\n"
"\n"
"  writeColorAndMinScalar(sample,opacity,minValue);\n"
"}\n"
"\n";

