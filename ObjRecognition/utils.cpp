/*
 * utils.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: papazov
 */

#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <ObjRecRANSAC/Shapes/PointSetShape.h>
#include <ObjRecRANSACVis/ORRRangeImage2vtk.h>
#include <VtkBasics/VtkPolyData.h>
#include <VtkBasics/VtkWindow.h>
#include <VtkBasics/VtkPoints.h>
#include <BasicTools/Vtk/VtkTransform.h>
#include <BasicToolsVis/ComputationalGeometry/Octree2vtk.h>
#include <vtkPoints.h>
#include <opencv2/opencv.hpp>
#include <list>
#include "CustomTypes.h"
#include <vtkSmartPointer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>
#include <vtkVertexGlyphFilter.h>


using namespace std;
using namespace Eigen;


typedef vector<vector<Eigen::Vector4d>> ContactList;





void utils_convert_to_vtk(cv::Mat &img, vtkPoints *pts, double depthThreshold) {
    int NegXY{0};
    double p[3];
    for (int j = 0; j < img.rows; ++j) {
        for (int i = 0; i < img.cols; ++i) {
            // depth threshold
            if (img.at<cv::Vec3f>(j, i)[2] < depthThreshold) {
                // convert to mm and shift by 1 meter in all directions
                p[0] = (img.at<cv::Vec3f>(j, i)[0]) * 1000.0,
                p[1] = (img.at<cv::Vec3f>(j, i)[1]) * 1000.0,
                p[2] = (img.at<cv::Vec3f>(j, i)[2]) * 1000.0,
                pts->InsertNextPoint(p);
            }
        }
    }

    double x_min {0};
    double y_min {0};

    for (int i = 0; i < pts->GetNumberOfPoints(); ++i)
    {
        double PointInThisVTKcloud [3];
        pts->GetPoint(i, PointInThisVTKcloud);
        if (PointInThisVTKcloud[0] < x_min) { x_min = PointInThisVTKcloud[0];}
        if (PointInThisVTKcloud[1] < y_min) { y_min = PointInThisVTKcloud[1];}
    }

    double PointInThisVTKcloud2 [3];
    for (int ii = 0; ii < pts->GetNumberOfPoints(); ++ii)
    {
        pts->GetPoint(ii, PointInThisVTKcloud2);
        PointInThisVTKcloud2[0]+= x_min;
        PointInThisVTKcloud2[1]+= y_min;
    }

    double PointInThisVTKcloud3 [3];
    for (int iii = 0; iii < pts->GetNumberOfPoints(); ++iii)
    {
        pts->GetPoint(iii, PointInThisVTKcloud2);
        if (PointInThisVTKcloud3[0] < 0 || PointInThisVTKcloud3[1] < 0)
        {
            NegXY++;
        }
    }

    cout << "min x-value is : " << x_min << endl;
    cout << "min y-value is : " << y_min << endl;
    cout << "number of negative values in X or Y : " << NegXY << endl;

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(pts);

    vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New();
    writer->SetFileName("Exported_vtk_cloud.vtk");
    writer->SetInput(polydata); // Use SetInput instead of SetInputData
    writer->Write();

}


//============================================================
Eigen::Matrix4d convertToEigenMatrix(const double* matrixPtr) {
    Eigen::Matrix4d result;

    // Copy the values from the pointer to the Eigen matrix
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result(i, j) = matrixPtr[i * 4 + j];
        }
    }

    return result;
}
//============================================================
#include <Eigen/Dense>

array<long double, 3> transformPoint(const Eigen::Matrix4d& transformationMatrix, const array<long double, 3> inputPoint, array<long double, 3> outputPoint) {
    Eigen::Vector4d inputVec(inputPoint[0], inputPoint[1], inputPoint[2], 1.0);
    Eigen::Vector4d outputVec = transformationMatrix * inputVec;

    // Copy the transformed coordinates to the output array
    outputPoint[0] = outputVec[0];
    outputPoint[1] = outputVec[1];
    outputPoint[2] = outputVec[2];

    return outputPoint;
}
//============================================================







void utils_vis(ObjRecRANSAC *objrec, vtkPoints *scene, list<PointSetShape *> &shapes, unordered_map<string, GRASPS_vector_for_one_model> retrieved_grasps, bool visualizeSceneRange = false,
               bool visualizeOctree = false, bool visualizeSampledPointPairs = false, vector<vector<Eigen::Vector4f>> List_of_contact_points = {})
               {
    printf("visualizing ...\n");
    fflush(stdout);

    VtkWindow vtkwin(1281, 0, 1000, 800);
    // vtkwin.parallelProjectionOn();
    vtkwin.showAxes();
    vtkwin.setAxesLength(100.0);
    vtkwin.setAxesRadius(0.5);
    // Position and orientation stuff
    vtkwin.setCameraPosition(-92.381155, -150.855469, -765.737139);
    vtkwin.setCameraFocalPoint(22.547971, -22.272753, 1057.570109);
    vtkwin.setCameraViewUp(-0.051830, -0.995947, 0.073503);
    vtkwin.setCameraViewAngle(30.000000);
    vtkwin.setWindowSize(1000, 800);

    // Visualize the scene
    VtkPoints myScene(scene);
    myScene.setPointRadius(1.5);
    myScene.setResolution(3, 3);
    myScene.setColor(0.5, 0.7, 1.0);
    vtkwin.addToRenderer(myScene.getActor());

    // Visualize the scene range image
    if (visualizeSceneRange) {
        VtkPolyData mySceneRangeImage;
        ORRRangeImage2vtk rimg2vtk;
        rimg2vtk.getAsPolygons(objrec->getSceneRangeImage(), mySceneRangeImage.getPolyData());
        mySceneRangeImage.setColor(0.2, 0.3, 0.5);
        vtkwin.addToRenderer(mySceneRangeImage.getActor());
    }

    // Visualize the scene octree
    if (visualizeOctree) {
        Octree2vtk octree2vtk;
        VtkPolyData mySceneOctree;
        mySceneOctree.setColor(1.0, 0.0, 0.0);
        octree2vtk.getFullLeafsAsWireframe(objrec->getSceneOctree(), mySceneOctree.getPolyData());
        vtkwin.addToRenderer(mySceneOctree.getActor());
    }

    list<VtkPolyData *> my_shapes_list;

    for (auto shape: shapes) {
        // Which object do we have
        if (shape->getUserData()) {
            printf("\t%s\n", shape->getUserData()->getLabel());
        }

        // Get the transformation matrix
        double **mat = mat_alloc(4, 4);
        shape->getHomogeneousRigidTransform(mat);

        // Copy the high-res mesh
        vtkPolyData *vtk_shape = vtkPolyData::New();
        vtk_shape->DeepCopy(shape->getHighResModel());

        // Transform the mesh
        vtkTransformPolyDataFilter *transformer = vtkTransformPolyDataFilter::New();
        transformer->SetInput(vtk_shape);
        VtkTransform::mat4x4ToTransformer((const double **) mat, transformer);

        // Visualize the mesh
        auto *my_shape = new VtkPolyData(transformer->GetOutput());
        my_shape->setColor(1.0, 0.7, 0.1);
        vtkwin.addToRenderer(my_shape->getActor());
        // Save the pointer in order to delete it later
        my_shapes_list.push_back(my_shape);

        cout << "Gggggggggggggggggggggggggg" << endl;
//********************************
    for (auto C : List_of_contact_points)
    {
        cout << "xxxxxxxxxxxxx" << endl;
        Eigen::Vector3d PointA = {C[0][0], C[0][1], C[0][2]};
        Eigen::Vector3d PointB = {C[1][0], C[1][1], C[1][2]};

        cout << "PointA is : " << PointA << endl;
        cout << "PointB is : " << PointB << endl;

        vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtk_points->InsertNextPoint(PointA.data());
        vtk_points->InsertNextPoint(PointB.data());

        // Create VTK actors for PointA and PointB
        vtkSmartPointer<vtkPolyData> vtk_polydata = vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtkSmartPointer<vtkGlyph3D> glyphFilter = vtkSmartPointer<vtkGlyph3D>::New();
        glyphFilter->SetInput(vtk_polydata);
        glyphFilter->SetScaleModeToDataScalingOff();
        glyphFilter->SetScaleFactor(1); // Set the scale factor as needed

        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetRadius(5); // Set the radius of the spheres representing points

        glyphFilter->SetSourceConnection(sphereSource->GetOutputPort());

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(glyphFilter->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Set color to red for both points

        // Add actors for PointA and PointB to the renderer
        vtkwin.addToRenderer(actor);
    }



//*****************
        // Cleanup
        vtk_shape->Delete();
        transformer->Delete();
        mat_dealloc(mat, 3);
    }

    // Visualize the sampled point pairs
    if (visualizeSampledPointPairs) {
        list<ObjRecRANSAC::OrientedPair> &point_pairs = objrec->getSampledPairs();
        std::cout << "We sampled " << point_pairs.size() << " pairs from the scene " << std::endl;
        vtkPoints *vtk_sampled_pairs = vtkPoints::New(VTK_DOUBLE);
        // Get all sampled point pairs
        for (auto &point_pair: point_pairs) {
            vtk_sampled_pairs->InsertNextPoint(point_pair.p1);
            vtk_sampled_pairs->InsertNextPoint(point_pair.p2);
        }
        VtkPoints my_sampled_pairs(vtk_sampled_pairs);
        my_sampled_pairs.setPointRadius(1.9);
        my_sampled_pairs.setColor(1.0, 0.0, 0.0);
        vtkwin.addToRenderer(my_sampled_pairs.getActor());
    }

    vtkwin.getRenderer()->ResetCameraClippingRange();
    vtkwin.render();
    vtkwin.vtkMainLoop();

    // Cleanup
    for (auto &it : my_shapes_list) {
        delete it;
    }
}
