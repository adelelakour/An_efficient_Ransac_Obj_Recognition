
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Obstacle.h>

#include "GraspPlanning/GraspStudio.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "GraspPlanning/GraspPlanner/GenericGraspPlanner.h"
#include "GraspPlanning/ApproachMovementSurfaceNormal.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h>


#include <vector>
#include <Eigen/Dense>

#include "ui_GraspPlanner.h"


using Vector3f = Eigen::Vector3f;


class GraspPlannerWindow : public QMainWindow
{
    Q_OBJECT
public:
    GraspPlannerWindow(std::string& robotFile, std::string& eefName, std::string& preshape, std::string& objFile);
    ~GraspPlannerWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void selectRobotObject(int n);

    void closeEEF();
    void openEEF();
    void colModel();
    void frictionConeVisu();
    void showGrasps();

    void buildVisu();

    void plan();
    void save();

    void save_to_json(std::string objName, std::tuple<std::array<Vector3f, 4>, std::string, std::array <double,3>> ContactData);

    void planObjectBatch();
protected:
    bool evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspStudio::GraspEvaluationPoseUncertainty::PoseEvalResults& results);

    void loadRobot();
    void loadObject(const std::string& objFile);

    void setupUI();

    static void timerCB(void* data, SoSensor* sensor);
    Ui::GraspPlanner UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* frictionConeSep;
    SoSeparator* graspsSep;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr eefCloned;
    VirtualRobot::RobotPtr robotObject;
    VirtualRobot::GraspableSensorizedObjectPtr object;
    VirtualRobot::EndEffectorPtr eef;

    VirtualRobot::GraspSetPtr grasps;


    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string eefName;
    std::string preshape;
    std::string objectFile;

    std::string objectName;
    std::string robotName;
    int graspNumber = 0;
    std::array<Vector3f, 4>  ContactPoints;
    std::array <double,3 > Approah_direction;
    std::tuple<std::array<Vector3f, 4>, std::string, std::array <double, 3>> ContactData;


    SoSeparator* eefVisu;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
    GraspStudio::ApproachMovementSurfaceNormalPtr approach;
    GraspStudio::GenericGraspPlannerPtr planner;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;
private slots:
    void on_pushButtonLoadObject_clicked();
};

