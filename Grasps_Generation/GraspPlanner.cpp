#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>
#include <filesystem> // Include the filesystem library

using std::cout;
using std::endl;
using namespace VirtualRobot;
namespace fs = std::filesystem; // Alias for the filesystem namespace

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspPlannerWindow.h"
#include "myGraspPlannerWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Grasp Planner");
    std::cout << " --- START --- " << std::endl;
    std::cout << endl << "Hint: You can start this demo for different hands:" << std::endl;

    std::string robot("robots/FRANKA/FEE.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);
    std::string eef("Panda Gripper");

    std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

    if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
    {
        robot = robFile;
    }

    std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");

    if (!eefname.empty())
    {
        eef = eefname;
    }

    std::string xmlDirectory = "Simox/VirtualRobot/data/objects/YCB";

    std::vector <std::string> OBJECTS;

    for (const auto& entry : fs::directory_iterator(xmlDirectory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".xml") {
            std::string xmlFile = entry.path().string();
            OBJECTS.push_back(xmlFile);
        }
    }

    std::string preshape("");
    std::string object = "";
    std::string objFile = "";
    std::string ps;


    ps = VirtualRobot::RuntimeEnvironment::getValue("preshape");

    if (!ps.empty())
    {
        preshape = ps;
    }



    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
    VirtualRobot::RuntimeEnvironment::considerKey("preshape");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::cout << "Using robot from " << robot << std::endl;
    std::cout << "End effector:" << eef << ", preshape:" << preshape << std::endl;
    std::cout << "Using object from " << object << std::endl;


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

