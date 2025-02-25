/* DO NOT EDIT.
 * Generated by ../bin/vtkEncodeString
 * 
 * Define the vtkVolumeTextureMapper3D_FourDependentShadeFP string.
 *
 * Generated from file: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/VolumeRendering/vtkVolumeTextureMapper3D_FourDependentShadeFP.asm
 */
#include "vtkVolumeTextureMapper3D_FourDependentShadeFP.h"
const char *vtkVolumeTextureMapper3D_FourDependentShadeFP =
"!!ARBfp1.0\n"
"	\n"
"# This is the fragment program for two\n"
"# component dependent data with shading	\n"
"\n"
"# We need some temporary variables		\n"
"TEMP index1, index2, normal, finalColor;\n"
"TEMP temp1, temp2, temp3; \n"
"TEMP sampleColor, sampleOpacity;\n"
"TEMP ndotl, ndoth, ndotv; \n"
"TEMP lightInfo, lightResult;\n"
"	 \n"
"# We are going to use the first \n"
"# texture coordinate		\n"
"ATTRIB tex0 = fragment.texcoord[0];\n"
"\n"
"# This is the lighting information\n"
"PARAM lightDirection = program.local[0];\n"
"PARAM halfwayVector  = program.local[1];\n"
"PARAM coefficient    = program.local[2];\n"
"PARAM lightDiffColor = program.local[3]; \n"
"PARAM lightSpecColor = program.local[4]; \n"
"PARAM viewVector     = program.local[5];\n"
"PARAM constants      = program.local[6];\n"
"	\n"
"# This is our output color\n"
"OUTPUT out = result.color;\n"
"\n"
"# Look up color in the first volume\n"
"TEX sampleColor, tex0, texture[0], 3D;\n"
"	\n"
"# Look up the fourth scalar value / gradient\n"
"# magnitude in the second volume	\n"
"TEX temp1, tex0, texture[1], 3D;\n"
"\n"
"# Look up the gradient direction\n"
"# in the third volume	\n"
"TEX temp2, tex0, texture[2], 3D;\n"
"\n"
"# This normal is stored 0 to 1, change to -1 to 1\n"
"# by multiplying by 2.0 then adding -1.0.	\n"
"MAD normal, temp2, constants.x, constants.y;\n"
"	\n"
"# Swizzle this to use (a,r) as texture\n"
"# coordinates for opacity\n"
"SWZ index1, temp1, a, r, 1, 1;\n"
"\n"
"# Use this coordinate to look up a \n"
"# final opacity in the fourth texture\n"
"TEX sampleOpacity, index1, texture[3], 2D;\n"
"\n"
"# Take the dot product of the light\n"
"# direction and the normal\n"
"DP3 ndotl, normal, lightDirection;\n"
"\n"
"# Take the dot product of the halfway\n"
"# vector and the normal		\n"
"DP3 ndoth, normal, halfwayVector;\n"
"\n"
"DP3 ndotv, normal, viewVector;\n"
"	 \n"
"# flip if necessary for two sided lighting\n"
"MUL temp3, ndotl, constants.y; \n"
"CMP ndotl, ndotv, ndotl, temp3;\n"
"MUL temp3, ndoth, constants.y; \n"
"CMP ndoth, ndotv, ndoth, temp3;\n"
"	 \n"
"# put the pieces together for a LIT operation		\n"
"MOV lightInfo.x, ndotl.x; \n"
"MOV lightInfo.y, ndoth.x; \n"
"MOV lightInfo.w, coefficient.w; \n"
"\n"
"# compute the lighting	\n"
"LIT lightResult, lightInfo;\n"
"\n"
"# This is the ambient contribution	\n"
"MUL finalColor, coefficient.x, sampleColor;\n"
"\n"
"# This is the diffuse contribution	\n"
"MUL temp3, lightDiffColor, sampleColor;\n"
"MUL temp3, temp3, lightResult.y;\n"
"ADD finalColor, finalColor, temp3;\n"
"\n"
"# This is th specular contribution	\n"
"MUL temp3, lightSpecColor, lightResult.z; \n"
"	\n"
"# Add specular into result so far, and replace\n"
"# with the original alpha.	\n"
"ADD out, finalColor, temp3;\n"
"MOV out.w, sampleOpacity.w;\n"
"	 \n"
"END\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"	\n";

