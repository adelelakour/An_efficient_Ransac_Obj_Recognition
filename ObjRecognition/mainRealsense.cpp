#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <ObjRecRANSAC/Shapes/PointSetShape.h>
#include <opencv2/opencv.hpp>
#include <vtkPolyDataReader.h>
#include "config.h"

// comment 

//Standard
#include <list>
#include <atomic>
#include <thread>
#include <random>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <array>

// RealSense
#include <librealsense2/rs.hpp>
#include "example.hpp"

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <nlohmann/json.hpp>
#include "CustomTypes.h"
#include "globals.h"


using namespace std;
using namespace cv;
using json = nlohmann::json;

namespace fs = std::filesystem;



std::string Offline_generated_grasps = "../files/Database_22_highQualityGrasps.json";
std::string to_panda_json = "../files/Grasping_Data_to_Panda.json";
std::string RobotHand = "Panda Gripper";
std::string GraspNumber = "Grasp0";


//********** declarations
void utils_convert_to_vtk(cv::Mat &img, vtkPoints *pts, double depthThreshold = 4.0);

void utils_vis(ObjRecRANSAC *objrec, vtkPoints *scene, list<PointSetShape *> &shapes,
               bool visualizeSceneRange = false,
               bool visualizeOctree = false,
               bool visualizeSampledPointPairs = false);


void loadModels(ObjRecRANSAC &objrec, list<UserData *> &userDataList, list<vtkPolyDataReader *> &readers,
                std::string const &modelDirectory, bool loadStandardModels = true, bool loadDLRModels = false);

vtkPoints *preprocessScene(vtkPoints *scene_in, double depthThreshold, bool removePlane);

//######################################################################################################################

Eigen::Matrix4d estimatedPOSEinEigen(const double* rigidTransform) {

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

//**********************************
Eigen::Vector4f arrayToEigenVector(const double * array) {
    Eigen::Vector4f vector;
    vector << array[0], array[1], array[2], 1.0f; // Set the last element to 1
    return vector;
}

//**********************************
void saveGraspingDataForPanda(const std::string& label, std::array<double, 16>  estimatedPose, std::array<double, 3> Mean_gripper, std::array<double, 3> AppDir) {

    json oldData;
    std::ifstream inputFile(to_panda_json);
    if (inputFile.is_open()) {
        inputFile >> oldData;
    } else {
        std::cerr << "Failed to open Grasping_Data_to_Panda.json" << std::endl;
        return;
    }

    // Fill the JSON object with your data
    json newData;
    newData["estimatedPose"] = estimatedPose;
    newData["Mean_gripper"] = {Mean_gripper[0], Mean_gripper[1], Mean_gripper[2]};
    newData["AppDir"] = {AppDir[0], AppDir[1], AppDir[2]};

    if (oldData.contains(label))
    {
        // If the label already exists, you can update the data for that label
        oldData[label] = newData;
    } else
    {
        // If the label doesn't exist, simply add the new data
        oldData[label] = newData;
    }

    std::ofstream outputFile(to_panda_json);
    if (outputFile.is_open()) {
        outputFile << oldData.dump(4) << std::endl;
        outputFile.close();
    } else {
        std::cerr << "Unable to open file for writing!" << std::endl;
    }
}


//**********************************
void objrec_func(vtkPoints* IN_VTKscene, std::atomic<bool> &exitFlag, std::atomic<bool> &performRecognition,
                       bool visualizeAllScenePoints, std::string const &modelDir) {

    ObjRecRANSAC objrec(50.0/*pair width*/, 4.5/*voxel size*/, 0.5/*pairs to save*/);
    objrec.setVisibility(0.28); // 0.28
    objrec.setRelativeNumberOfIllegalPoints(0.02);
    objrec.setRelativeObjectSize(0.06);
    objrec.setIntersectionFraction(0.1); // 0.08
    objrec.setNumberOfThreads(1);

    // Load the models
    list<UserData *> userDataList; // Look inside the next function to see how to use this list
    list<vtkPolyDataReader *> readers; // This is just to delete the readers at the end

    loadModels(objrec, userDataList, readers, modelDir); // Look inside this function - it is important

    // The list where the recognized shapes will be saved
    list<PointSetShape *> shapes;

    int negative{0};
    unordered_map<string, GRASPS_vector_for_one_model> retrieved_grasps;
    // The scene
    while (!exitFlag.load()) {

        std::cout << "recognizing" << std::endl;

        // Scene preprocessing
        bool removePlane = false;
        double maxz = 1050.0;  // in mm

        vtkPoints *scene_out = preprocessScene(IN_VTKscene, maxz, removePlane); //

        std::vector<std::pair<string, Eigen::Matrix4f>> Object_Pose;
        std::vector<std::pair<std::string, double>> objectConfidences;

        objrec.doRecognition(scene_out, 0.995, shapes);


        std::ifstream file(Offline_generated_grasps);
        if (!file.is_open()) {
            std::cerr << "Failed to open file." << std::endl;
        }

        json JsonData;
        file >> JsonData;

        unordered_map<string, array<array<double, 3>, 2>> obj_vs_points;

        for (auto& detected_shape : shapes) {

            string Object_Label = detected_shape->getUserData()->getLabel();

            cout << "I detected " << Object_Label << endl;

            auto Pose_of_detected_shape = detected_shape->getRigidTransform();

            Eigen::Matrix4d Pose_of_detected_shape_IN_Eigen = estimatedPOSEinEigen(Pose_of_detected_shape);

            array <double, 16> Pose_of_detected_shape_in_16;
             for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        Pose_of_detected_shape_in_16[i * 4 + j] = static_cast<double>(Pose_of_detected_shape_IN_Eigen(i, j));
                    }
                }

             cout << "Pose_of_detected_shape_in_16 is .. \n" << endl;

             for (auto d : Pose_of_detected_shape_in_16)
             {
                 cout << d << endl;
             }

            Pose_of_detected_shape_in_16[3] /= 1000;
            Pose_of_detected_shape_in_16[7] /= 1000;
            Pose_of_detected_shape_in_16[11] /= 1000;


            // I retrieve the two contact points for this shape from the json file
            vector<array<double, 3>> TWO_CONT_POINTS;

            auto Grasp_of_detected_shape = JsonData[RobotHand][Object_Label][GraspNumber]["CNT_PNT"];
            array<double,3> CNT_PNT_1 = Grasp_of_detected_shape[0];
            array<double,3> CNT_PNT_2 = Grasp_of_detected_shape[1];

            cout << "CNT_PNT_1 is .. " << CNT_PNT_1 [0] << ", " << CNT_PNT_1[1] << ", " << CNT_PNT_1[2] << endl;
            cout << "CNT_PNT_2 is .. " << CNT_PNT_2 [0] << ", " << CNT_PNT_2[1] << ", " << CNT_PNT_2[2] << endl;


            auto Grasp_Approach = JsonData[RobotHand][Object_Label][GraspNumber]["APP_DIR"];
            std::array<double, 3> Grasp_Approach_Direction = Grasp_Approach;


            std::array<double, 3> Mean_point_to_Panda;
            for (int i = 0; i < 3; i++)
            {
                Mean_point_to_Panda [i] = (CNT_PNT_1[i] + CNT_PNT_2[i])/2;
            }

            // so far, I've the mean, the pose of object w.r.t camera frame, the grasping approach

            saveGraspingDataForPanda(Object_Label,
                                     Pose_of_detected_shape_in_16,
                                     Mean_point_to_Panda,
                                     Grasp_Approach_Direction);
            }


        utils_vis(&objrec, scene_out, shapes,false , false , false);

        // Cleanup
        for (auto &shape: shapes) {
            delete shape;
        }
        shapes.clear();
        IN_VTKscene->Delete();
        scene_out->Delete();
        performRecognition.store(false);
        obj_vs_points.clear();
    }

    // Destroy the 'UserData' objects
    for (auto &it: userDataList) {
        delete it;
    }
    // Destroy the readers
    for (auto &reader : readers) {
        reader->Delete();
        reader->Delete();
    }

    cout << "I reached the end of this function  ..........." << endl;

}

//**********************************
vtkPoints* readVTKPointCloud(const std::string& filePath) {
    vtkPolyDataReader* reader = vtkPolyDataReader::New();
    reader->SetFileName(filePath.c_str());
    reader->Update();

    vtkPoints* points = reader->GetOutput()->GetPoints()->NewInstance();
    points->DeepCopy(reader->GetOutput()->GetPoints());

    reader->Delete();

    return points;
}

//**********************************
void loadModels(ObjRecRANSAC &objrec, list<UserData *> &userDataList, list<vtkPolyDataReader *> &readers,
                std::string const &modelDirectory, bool loadStandardModels, bool loadDLRModels) {

    // Derive the class 'UserData' if you want to save some specific information about each model.
    // When you load a model in the library you have to pass a 'UserData'-object to the method 'addModel()'.
    // If the corresponding model is detected in the scene, you can use
    // the 'UserData'-object which is returned by the detection method, in order to know which model
    // has been detected.

    std::vector<std::string> modelLabels;
    std::vector<std::string> modelPaths;
    std::string PATH_of_vtk_models = "../models/vtk";

    if (loadStandardModels) {
        printf("Loading standard models ...\n");
        fflush(stdout);       //The function fflush() is used to clear stdout buffer


        for (auto const &item : fs::directory_iterator(PATH_of_vtk_models))
        {
            std::string model_name = item.path().stem().string();
            std::string model_path = item.path().string();
            modelLabels.emplace_back(model_name);
            modelPaths.emplace_back(model_path);

        }

    }

    if (loadDLRModels) {
        printf("Loading DLR models ...\n");
        fflush(stdout);
        modelLabels.emplace_back("Cube");
        modelPaths.emplace_back(modelDirectory + "JHU_Models/Cube25k.vtk");
        modelLabels.emplace_back("Rod");
        modelPaths.emplace_back(modelDirectory + "JHU_Models/Rod50k.vtk");
    }
    if (modelLabels.empty()) {
        printf("No models loaded... Exiting!\n");
        exit(-1);
    }

    for (int i = 0; i < modelLabels.size(); ++i) {
        auto &modelLabel = modelLabels[i];
        auto &modelPath = modelPaths[i];

        // I think he created an object of type (UserData) to each model
        auto userData = new UserData();
        userData->setLabel(modelLabel.c_str()); // This time we have a salt cylinder


        // Load the model
        vtkPolyDataReader *reader = vtkPolyDataReader::New();  // dynamic allocation
        reader->SetFileName(modelPath.c_str());
        reader->Update();

        // Add the model to the model library
        objrec.addModel(reader->GetOutput(), userData);
        // Save the user data and the reader in order to delete them later (outside this function)
        userDataList.push_back(userData);
        readers.push_back(reader);
    }
}


//**********************************

vtkPoints *preprocessScene(vtkPoints *in, double depthThreshold, bool removePlane) {
    vtkPoints *nearScene, *out = vtkPoints::New(VTK_DOUBLE);

    bool withLimitDepthThreshold = depthThreshold > 0.0;
    if (withLimitDepthThreshold) {
        double p[3];
        nearScene = vtkPoints::New(VTK_DOUBLE);

        for (int i = 0; i < in->GetNumberOfPoints(); ++i) {
            in->GetPoint(i, p);
            if (p[2] <= depthThreshold ) {
                nearScene->InsertNextPoint(p);
            }
        }
    } else {
        nearScene = in;
    }

    if (removePlane) {
        double p[3];
        RANSACPlaneDetector planeDetector;
        // Detect the plane in the scene
        planeDetector.detectPlane(nearScene, 0.2, 10.0);
        // Check whether the plane normal is pointing to the right direction
        if (planeDetector.getPlaneNormal()[2] > 0.0) {
            planeDetector.flipPlaneNormal();
        }
        // Get the points above the plane (i.e., on the table)
        list<int> abovePlanePointIds, belowPlanePointIds;
        planeDetector.getPointsAbovePlane(abovePlanePointIds, belowPlanePointIds);
        // The scene above the plane
        out->SetNumberOfPoints(abovePlanePointIds.size());

        // Get the points above the plane
        int i = 0;
        for (int &abovePlanePointId : abovePlanePointIds) {
            nearScene->GetPoint(abovePlanePointId, p);
            out->SetPoint(i++, p);
        }
    } else {
        double p[3];
        out->SetNumberOfPoints(nearScene->GetNumberOfPoints());

        // Just copy the points from 'nearScene' to 'out'
        for (int i = 0; i < nearScene->GetNumberOfPoints(); ++i) {
            nearScene->GetPoint(i, p);
            out->SetPoint(i, p);
        }
    }

    if (withLimitDepthThreshold) {
        nearScene->Delete();
    }


    for (int i = 0; i < out->GetNumberOfPoints(); ++i) {
        double p[3];
        out->GetPoint(i, p);
        if (p[0] < -500)
        {
            out->SetPoint(i, p);
        }

    }






    return out;
}

//**********************************

int main() {

    uint16_t mGamma[2048];
    srand(2);

    std::string SAVED_IN_SCENE = "../files/Apple_Soda_Drill.vtk";
    vtkPoints* IN_vtk_pointCloud = readVTKPointCloud(SAVED_IN_SCENE);

    std::atomic<bool> exitFlag(false);
    std::atomic<bool> performRecognition(false);

    std::string modelDir = MODEL_DIR;

    objrec_func (IN_vtk_pointCloud, std::ref(exitFlag), std::ref(performRecognition), true, modelDir);

    for (int i = 0; i < 2048; ++i) {
        auto v = float(i / 2048.0);
        v = powf(v, 3) * 6;
        mGamma[i] = v * 6 * 256;
    }

    return 0;
}
