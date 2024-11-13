#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include "GlobalVariables.h"

#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>

using std::cout;
using std::endl;
using namespace VirtualRobot;

namespace fs = std::filesystem; // Alias for the filesystem namespace

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspPlannerWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Grasp Planner");
    std::cout << " --- START --- " << std::endl;

    std::string robot("../simox/VirtualRobot/data/robots/FRANKA/FrankaParallelGripper.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);
    std::string eef("Panda Gripper");
    std::string preshape("");


    std::string Path_to_YCB_models = "../simox/VirtualRobot/data/objects/YCB";
    std::vector <std::string> OBJECTS;



    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
    VirtualRobot::RuntimeEnvironment::considerKey("preshape");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

    if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
    {
        robot = robFile;
    }

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");

    if (!eefname.empty())
    {
        eef = eefname;
    }


    for (const auto& entry : fs::directory_iterator(Path_to_YCB_models)) {
        if (entry.is_regular_file() && entry.path().extension() == ".xml") {
            std::string xmlFile = entry.path().string();
            OBJECTS.push_back(xmlFile);
        }
    }

    std::string ps = VirtualRobot::RuntimeEnvironment::getValue("preshape");

    if (!ps.empty())
    {
        preshape = ps;
    }


    std::cout << "Using robot from " << robot << std::endl;
    std::cout << "End effector:" << eef << ", preshape:" << preshape << std::endl;

    for ( auto object : OBJECTS)
    {
        VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);
        objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

        if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
        {
            object = objFile;
        }

        std::cout << "Using object from " << object << std::endl;

        GraspPlannerWindow rw(robot, eef, preshape, object);

        rw.main();

        object = "";
        objFile = "";

    }

    return 0;
}
