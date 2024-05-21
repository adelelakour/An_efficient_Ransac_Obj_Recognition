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

#include <nlohmann/json.hpp>

#include "globals.h"

using json = nlohmann::json;


using namespace std;
using namespace Eigen;

typedef vector<vector<Eigen::Vector4d>> ContactList;

//***********************************
Eigen::Matrix4d estimatedPOSEinEigen_util(const double* rigidTransform) {

    // Extract rotation matrix elements
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << rigidTransform[0], rigidTransform[1], rigidTransform[2],
            rigidTransform[3], rigidTransform[4], rigidTransform[5],
            rigidTransform[6], rigidTransform[7], rigidTransform[8];

    Eigen::Vector3d translationVector(rigidTransform[9], rigidTransform[10], rigidTransform[11]);

    Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();

    transformMatrix.block<3, 3>(0, 0) = rotationMatrix;

    transformMatrix.block<3, 1>(0, 3) = translationVector;

    return transformMatrix;
}


//*****************************************
void utils_convert_to_vtk(cv::Mat &img, vtkPoints *pts, double depthThreshold) {
    double p[3];
    for (int j = 0; j < img.rows; ++j) {
        for (int i = 0; i < img.cols; ++i) {
            // depth threshold
            if (img.at<cv::Vec3f>(j, i)[2] < depthThreshold) {
                // convert to mm
                p[0] = img.at<cv::Vec3f>(j, i)[0] * 1000.0,
                p[1] = -img.at<cv::Vec3f>(j, i)[1] * 1000.0,
                p[2] = img.at<cv::Vec3f>(j, i)[2] * 1000.0;
                pts->InsertNextPoint(p);
            }
        }
    }
}

void utils_vis(ObjRecRANSAC *objrec,
               vtkPoints *scene,
               list<PointSetShape *> &shapes,
               bool visualizeSceneRange,
               bool visualizeOctree,
               bool visualizeSampledPointPairs)

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

    json Off_Data;
    ifstream INPUT (Offline_generated_grasps);
    INPUT >> Off_Data;
    INPUT.close();

    for (auto shape : shapes) {
        // Retrieve the transformation matrix for the shape
        double **mat = mat_alloc(4, 4);
        shape->getHomogeneousRigidTransform(mat);

        // Copy the high-res mesh
        vtkPolyData *vtk_shape = vtkPolyData::New();
        vtk_shape->DeepCopy(shape->getHighResModel());

        // Transform the mesh
        vtkTransformPolyDataFilter *transformer = vtkTransformPolyDataFilter::New();
        transformer->SetInput(vtk_shape);
        VtkTransform::mat4x4ToTransformer((const double **)mat, transformer);

        // Visualize the mesh
        auto *my_shape = new VtkPolyData(transformer->GetOutput());
        my_shape->setColor(1.0, 0.7, 0.1);
        vtkwin.addToRenderer(my_shape->getActor());

        // Transform and visualize contact points
        array<double, 3> PNT1_array = Off_Data[RobotHand][shape->getUserData()->getLabel()][GraspNumber]["CNT_PNT"][0];
        PNT1_array [0] *= 1000;
        PNT1_array [1] *= 1000;
        PNT1_array [2] *= 1000;

        array<double, 3> PNT2_array = Off_Data[RobotHand][shape->getUserData()->getLabel()][GraspNumber]["CNT_PNT"][1];
        PNT2_array [0] *= 1000;
        PNT2_array [1] *= 1000;
        PNT2_array [2] *= 1000;

        std::cout << "Original PNT1_eigen: " << PNT1_array[0] << ", " << PNT1_array[1] << ", " << PNT1_array[2] << std::endl;
        std::cout << "Original PNT2_eigen: " << PNT2_array[0] << ", " << PNT2_array[1] << ", " << PNT2_array[2] << std::endl;

        // Contact points in Eigen
        Eigen::Vector4d PNT1_eigen(PNT1_array[0], PNT1_array[1], PNT1_array[2], 1.0);
        Eigen::Vector4d PNT2_eigen(PNT2_array[0], PNT2_array[1], PNT2_array[2], 1.0);

        Eigen::Matrix4d shape_pose_in_eigen;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                shape_pose_in_eigen(i, j) = mat[i][j];
            }
        }


        cout << shape->getUserData()->getLabel() <<" pose_in_eigen is : \n" << shape_pose_in_eigen << endl;

        Eigen::Vector4d PNT1_transformed = shape_pose_in_eigen * PNT1_eigen;
        Eigen::Vector4d PNT2_transformed = shape_pose_in_eigen * PNT2_eigen;

        cout << "PNT1_transformed \n " << PNT1_transformed << endl;
        cout << "PNT2_transformed \n " << PNT2_transformed << endl;

        // Visualize red spheres at transformed contact points
        // Note: Adjust radius and color as needed
        vtkSmartPointer<vtkSphereSource> sphereSource1 = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource1->SetCenter(PNT1_transformed[0], PNT1_transformed[1], PNT1_transformed[2]);
        sphereSource1->SetRadius(5.0);
        vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper1->SetInputConnection(sphereSource1->GetOutputPort());
        vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
        actor1->SetMapper(mapper1);
        actor1->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red color
        vtkwin.addToRenderer(actor1);

        vtkSmartPointer<vtkSphereSource> sphereSource2 = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource2->SetCenter(PNT2_transformed[0], PNT2_transformed[1], PNT2_transformed[2]);
        sphereSource2->SetRadius(5.0);
        vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper2->SetInputConnection(sphereSource2->GetOutputPort());
        vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
        actor2->SetMapper(mapper2);
        actor2->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red color
        vtkwin.addToRenderer(actor2);


        // Cleanup
        vtk_shape->Delete();
        transformer->Delete();
        mat_dealloc(mat, 3);
    }




    // Visualize the sampled point pairs
    if (visualizeSampledPointPairs) {
        list<ObjRecRANSAC::OrientedPair> &point_pairs = objrec->getSampledPairs();
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
