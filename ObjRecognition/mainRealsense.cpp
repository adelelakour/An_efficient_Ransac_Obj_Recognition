
#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <ObjRecRANSAC/Shapes/PointSetShape.h>
#include <opencv2/opencv.hpp>
#include <vtkPolyDataReader.h>
#include <list>
#include <atomic>
#include <thread>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "example.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <unordered_map>

#include "CustomTypes.h"
#include <random>
#include <ctime>
#include <cstdlib>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace std;
using namespace cv;
using json = nlohmann::json;

namespace fs = std::filesystem;


string Offline_generated_grasps = "../files/Database16.json";
string RobotHand = "Panda Gripper";

//********** declarations
void utils_convert_to_vtk(cv::Mat &img, vtkPoints *pts, double depthThreshold = 4.0);

void utils_vis(ObjRecRANSAC *objrec, vtkPoints *scene, list<PointSetShape *> &shapes, unordered_map<string, GRASPS_vector_for_one_model> retrieved_grasps, bool visualizeSceneRange = false,
               bool visualizeOctree = false, bool visualizeSampledPointPairs = false, vector<vector<Eigen::Vector4f>> List_of_contact_points = {});

void loadModels(ObjRecRANSAC &objrec, list<UserData *> &userDataList, list<vtkPolyDataReader *> &readers,
                std::string const &modelDirectory, bool loadStandardModels = true, bool loadDLRModels = false);

vtkPoints *preprocessScene(vtkPoints *scene_in, double depthThreshold, bool removePlane);


//==========
void saveObjectContactToJson(const std::string& label, const double * estimatedPose, const Eigen::Matrix<float, 4, 1> & CP1, const Eigen::Matrix<float, 4, 1> & CP2) {

    json objContact;
    objContact[label] = *estimatedPose;
    objContact[label]["CP1"] = {CP1[0], CP1[1], CP1[2], CP1[3]};
    objContact[label]["CP2"] = {CP2[0], CP2[1], CP2[2], CP2[3]};

    std::ofstream outputFile("Object_Contact.json");
    if (outputFile.is_open()) {
        outputFile << std::setw(4) << objContact << std::endl;
        outputFile.close();
        std::cout << "Object contact information saved to Object_Contact.json" << std::endl;
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

//************************** Eigen::Matrix4f pose estimation
Eigen::Matrix4f estimatedPOSEinEigen(const double* rigidTransform) {
    // Extract rotation matrix elements
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << rigidTransform[0], rigidTransform[1], rigidTransform[2],
            rigidTransform[3], rigidTransform[4], rigidTransform[5],
            rigidTransform[6], rigidTransform[7], rigidTransform[8];

    Eigen::Vector3f translationVector(rigidTransform[9], rigidTransform[10], rigidTransform[11]);

    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

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

//********************************
vector<vector<double>> getContactPoints(const string& RobotHand, const string& Object, const string& GraspNum) {
    ifstream file("Offline_Generated_Grasps.json");
    if (!file.is_open()) {
        cerr << "Error: Unable to open file." << endl;
        return {};
    }

    json root;
    file >> root;

    if (root.find(RobotHand) == root.end()) {
        cerr << "Error: Robot hand not found." << endl;
        return {};
    }

    if (root[RobotHand].find(Object) == root[RobotHand].end()) {
        cerr << "Error: Object not found for the given robot hand." << endl;
        return {};
    }

    if (root[RobotHand][Object].find(GraspNum) == root[RobotHand][Object].end()) {
        cerr << "Error: Grasp number not found for the given object and robot hand." << endl;
        return {};
    }

    vector<vector<double>> contactPoints;

    for (const auto& coord : root[RobotHand][Object][GraspNum]) {
        vector<double> point;
        for (const auto& val : coord) {
            point.push_back(val);
        }
        contactPoints.push_back(point);
    }

    return contactPoints;
}

//====================================================================================================================================

void objrec_func(void *v_pipe, std::atomic<bool> &exitFlag, std::atomic<bool> &performRecognition,
                 bool visualizeAllScenePoints, std::string const &modelDir) {

    //auto *kinect = reinterpret_cast<cv::VideoCapture *>(v_kinect);
    auto *RealSense_pipeline = reinterpret_cast<rs2::pipeline *>(v_pipe);

    ObjRecRANSAC objrec(40.0/*pair width*/, 4.5/*voxel size*/, 0.5/*pairs to save*/);
    objrec.setVisibility(0.28); // 0.28
    objrec.setRelativeNumberOfIllegalPoints(0.02);
    objrec.setRelativeObjectSize(0.06);
    objrec.setIntersectionFraction(0.1); // 0.08
    objrec.setNumberOfThreads(8);

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

        //RealSense_pipeline->start();
        rs2::frameset frames;
        rs2::frame color_frame, depth_frame;

        // Get color and depth frames
        frames = RealSense_pipeline->wait_for_frames();
        color_frame = frames.get_color_frame();
        depth_frame = frames.get_depth_frame();
        // Get width and height of the frames
        int w = color_frame.as<rs2::video_frame>().get_width();
        int h = color_frame.as<rs2::video_frame>().get_height();

        // Create cv::Mat for color and depth frames
        cv::Mat color_image(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(w, h), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        int width = depth_frame.as<rs2::video_frame>().get_width();
        int height = depth_frame.as<rs2::video_frame>().get_height();

        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth_frame);

        cv::Mat pointCloudMat(cv::Size(width, height), CV_32FC3, (void*)points.get_vertices(), cv::Mat::AUTO_STEP);

        // Convert to vtkPoints
        vtkPoints *scene_in = vtkPoints::New(VTK_DOUBLE);
        utils_convert_to_vtk(pointCloudMat, scene_in, 100);

        // Scene preprocessing
        bool removePlane = true;
        double maxz = 1050.0;  // in mm
        vtkPoints *scene_out = preprocessScene(scene_in, maxz, removePlane); //

        std::vector<std::pair<string, Eigen::Matrix4f>> Object_Pose;
        std::vector<std::pair<std::string, double>> objectConfidences;

        objrec.doRecognition(scene_out, 0.995, shapes);

        std::ifstream file(Offline_generated_grasps);
        if (!file.is_open()) {
            std::cerr << "Failed to open file." << std::endl;
            return;
        }

        json data;
        file >> data;

        vector<vector<Eigen::Vector4f>> List_of_contact_points;

        for (auto& detected_shape : shapes) {

            auto label_of_detected_shape = detected_shape->getUserData()->getLabel();
            cout << "I detected " << label_of_detected_shape << endl;

            const double* Pose_of_detected_shape = detected_shape->getRigidTransform();
            Eigen::Matrix4f Pose_of_detected_shape_IN_Eigen = estimatedPOSEinEigen(Pose_of_detected_shape);

            cout << " Estimated pose in Eigen is : \n" << Pose_of_detected_shape_IN_Eigen << endl;

            auto Grasp_of_detected_shape = data[RobotHand][label_of_detected_shape]["grasp1"];

            auto CONT_A = Grasp_of_detected_shape[0];
            auto CONT_B = Grasp_of_detected_shape[1];

            cout << "CONTACT_POINT_1_in_OBJ_FRAME is : " << CONT_A[0] << " , "  << CONT_A[1] << " , " << CONT_A[2]<< " "<< endl;
            cout << "CONTACT_POINT_2_in_OBJ_FRAME is : " << CONT_B[0] << " , "  << CONT_B[1] << " , " << CONT_B[2]<< " "<< endl;


            Eigen::Vector4f CONT_A_in_Eigen;
            CONT_A_in_Eigen[0] = CONT_A[0];
            CONT_A_in_Eigen[1] = CONT_A[1];
            CONT_A_in_Eigen[2] = CONT_A[2];
            CONT_A_in_Eigen[3] = 1.0;

            Eigen::Vector4f CONT_B_in_Eigen;
            CONT_B_in_Eigen[0] = CONT_B[0];
            CONT_B_in_Eigen[1] = CONT_B[1];
            CONT_B_in_Eigen[2] = CONT_B[2];
            CONT_B_in_Eigen[3] = 1.0;

            cout << "A in Obj Frame  in Egi " << CONT_A_in_Eigen << endl;
            cout << "B in Obj Frame  in Egi " << CONT_B_in_Eigen << endl;

            Eigen::Vector4f CONTACT_POINT_A_in_CAM_FRAME = Pose_of_detected_shape_IN_Eigen * CONT_A_in_Eigen;
            Eigen::Vector4f CONTACT_POINT_B_in_CAM_FRAME = Pose_of_detected_shape_IN_Eigen * CONT_B_in_Eigen;


            cout << "CONTACT_POINT_A_in_CAM_FRAME : " << CONTACT_POINT_A_in_CAM_FRAME << endl;
            cout << "CONTACT_POINT_B_in_CAM_FRAME : " << CONTACT_POINT_B_in_CAM_FRAME << endl;

            vector<Eigen::Vector4f > ContactPair;
            ContactPair.push_back(CONTACT_POINT_A_in_CAM_FRAME);
            ContactPair.push_back(CONTACT_POINT_B_in_CAM_FRAME);
            List_of_contact_points.push_back(ContactPair);

            saveObjectContactToJson(label_of_detected_shape, Pose_of_detected_shape, CONTACT_POINT_A_in_CAM_FRAME, CONTACT_POINT_B_in_CAM_FRAME);
        }

        for (auto Item : Object_Pose)
        {
            cout << Item.first << " pose is : \n " << Item.second << endl;
        }

        utils_vis(&objrec, scene_out, shapes, retrieved_grasps,false , false , false, List_of_contact_points);


        // Cleanup
        for (auto &shape: shapes) {
            delete shape;
        }
        shapes.clear();
        scene_in->Delete();
        scene_out->Delete();
        performRecognition.store(false);
        List_of_contact_points.clear();
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


int main() {

    uint16_t mGamma[2048];
    srand(1000);

    rs2::pipeline pipe;
    pipe.start();

    std::atomic<bool> exitFlag(false);
    std::atomic<bool> performRecognition(false);

    // Start a thread for the object recognition
    std::string modelDir = "/home/adelelakour/CLionProjects/objectmeshrecognition/models/vtk/";
    //std::thread thread_objrec(objrec_func, &pipe, std::ref(exitFlag), std::ref(performRecognition), true, modelDir);
    objrec_func (&pipe, std::ref(exitFlag), std::ref(performRecognition), true, modelDir);

    // I did not understand what this for loop does
    for (int i = 0; i < 2048; ++i) {
        auto v = float(i / 2048.0);
        v = powf(v, 3) * 6;
        mGamma[i] = v * 6 * 256;
    }

    return 0;
}

//====================================================================================================================================

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


//====================================================================================================================================

vtkPoints *preprocessScene(vtkPoints *in, double depthThreshold, bool removePlane) {
    vtkPoints *nearScene, *out = vtkPoints::New(VTK_DOUBLE);
    bool withLimitDepthThreshold = depthThreshold > 0.0;
    if (withLimitDepthThreshold) {
        double p[3];
        nearScene = vtkPoints::New(VTK_DOUBLE);

        for (int i = 0; i < in->GetNumberOfPoints(); ++i) {
            in->GetPoint(i, p);
            if (p[2] <= depthThreshold) {
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
    return out;
}

