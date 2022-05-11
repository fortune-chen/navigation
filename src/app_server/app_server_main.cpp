#include <unistd.h>
#include <iostream>
#include <functional>
#include <unordered_map>
#include "flslam_type.h"

#include "mapping.h"
#include "navigation.h"
#include "trimmer.h"

using namespace flslam;

static std::shared_ptr<flslam::Mapping> kMapping;
static std::shared_ptr<flslam::Navigation> kNavigation;
static std::shared_ptr<flslam::Trimmer> kTrimmer;

int room_id;

static std::unordered_map<std::string, std::function<void(void)>> functionsMap;

class UserPathListener : public flslam::PathListener {
  public:
    UserPathListener() {}

    void onGlobalPathReceived(const std::vector<flslam::Pose>& path) override {
        printf("global path size : %zu\n", path.size());
        int index = 0;
        for (const auto& p : path) {
            printf("%d (%lf, %lf)\n", ++index, p.x, p.y);
        }
    }
};


static const std::string helpMessage = 
    "+----------------------------------------------------------------------+\n"
    "|                                  Options:                            |\n"
    "|       Press '0' to stop mapping                                      |\n"
    "|       Press '1' to start mapping                                     |\n"
    "|       Press '2' to continue mapping                                  |\n"
    "|       Press '3' to save map                                          |\n"
    "|       Press '4' to start localization                                |\n"
    "|       Press '5' to mapping->save->localization                       |\n"
    "|       Press '6' to set navigation goal                               |\n"
    "|       Press '7' to register global planning path listener            |\n"
    "|       Press '8' to open teleop key                                   |\n"
	"|       Press '9' to do full coverage path planning                    |\n"
    "| Map_Optimize                                                         |\n"
    "|       Press 's0' to map optimized                                    |\n"
    "|       Press 's1' to room split                                       |\n"
    "|       Press 's2' to visual wall                                      |\n"
    "|       Press 's3' to forbid area                                      |\n"
    "|       Press 's4' to room merge                                       |\n"
    "|       Press 's5' to room info                                        |\n"
    "|       Press 's6' to seletc Map                                       |\n"    
    "| Quit:                                                                |\n"
    "|       Press 'q' to quit the application.                             |\n"
    "+----------------------------------------------------------------------+\n";

static const std::string teleopKeyHelpMessage =
    "Control Your Robot!        \n"
    "---------------------------\n"
    "Moving around:             \n"
    "        w                  \n"
    "   a    s    d             \n"
    "        x                  \n";

void startMapping() {
    std::cout << "start mapping "<< std::endl;
    kMapping->start();
}

void continueMapping() {
    std::cout << "continue mapping "<< std::endl;
    std::string mapId;
    std::cout << "please input map id:\n";
    std::cin >> mapId;
    std::cout << "mapId: " << mapId << std::endl;
    kMapping->continueMapping(atoi(mapId.c_str()));
}

void stopMapping() {
    std::cout << "stop mapping "<< std::endl;
    kMapping->stop();
}

void saveMap() {
    std::cout << "save map "<< std::endl;
    auto mapId = kMapping->saveMap();
    std::cout << "map id: " << mapId << std::endl;
}

void localization() {
    std::cout << "statrt localization "<< std::endl;
    std::string mapId;
    std::cout << "please input map id:\n";
    std::cin >> mapId;
    std::cout << "mapId: " << mapId << std::endl;
    kMapping->localization(atoi(mapId.c_str()));
}

void mappingStressTest() {
    int count = 0;
    std::cout << "statrt mapping Test "<< std::endl;
    while (1) {
        startMapping();
        sleep(20);
        auto mapId = kMapping->saveMap();
        std::cout << "save map id: " << mapId << std::endl;
        sleep(2);
        std::cout << "statrt localization count: " << count++ << std::endl;
        kMapping->localization(mapId);
        sleep(20);
    }
}

void setNavigationGoal() {
    static int seq = 0;
    std::cout << "Input pose of the goal: " << std::endl;
    std::string x_str, y_str, orientation_str;
    std::cout << "X coordinate of the goal" << std::endl;
    std::cin >> x_str;
    std::cout << "Y coordinate of the goal" << std::endl;
    std::cin >> y_str;
    std::cout << "Orientation of the goal" << std::endl;
    std::cin >> orientation_str;

    flslam::Pose pose;
    pose.x = atof(x_str.c_str());
    pose.y = atof(y_str.c_str());
    pose.orientation = atof(orientation_str.c_str());
    kNavigation->setGoal(pose);
}

void registerGlobalPlanningPathListener() {
    std::shared_ptr<flslam::PathListener> listener = std::make_shared<UserPathListener>();
    kNavigation->registerPathListener(listener);
}

void teleopKey() {
    std::string key;
    while (true) {
        std::cout << teleopKeyHelpMessage;
        std::cin >> key;
        if (key == "w") {
            kNavigation->controlMotion(Direction::kUp);
        } else if (key == "x") {
            kNavigation->controlMotion(Direction::kDown);
        } else if (key == "a") {
            kNavigation->controlMotion(Direction::kLeft);
        } else if (key == "d") {
            kNavigation->controlMotion(Direction::kRight);
        } else if (key == "s") {
            kNavigation->controlMotion(Direction::kStop);
        } else if (key == "q") {
            return;
        }
    }
}

void startOptimizeMap() {
    std::cout << "start Map_Optimize "<< std::endl;
    kTrimmer->optimizeMap();
}

void startSplitRoom() {
    std::cout << "start Room_Split "<< std::endl;
    Image pngmap = kTrimmer->splitRoom(30, 1.4);
}

void startSetVisualWall() {
    std::cout << "start Visual_Wall "<< std::endl;
    kTrimmer->setVisualWall(50, 50, 100, 50);
}

void startSetForbidArea() {
    std::cout << "start Forbid_Area "<< std::endl;
    kTrimmer->setForbidArea(150, 100, 50, 100);
}

void startMergeRoom() {
    std::cout << "start Room_Merge "<< std::endl;
    kTrimmer->mergeRoom(0, 1);          //para1 : room1;    parar2:room2;
}

void startGetRoomInfo() {
    std::cout << "start Room_Info "<< std::endl;
    kTrimmer->RoomInfo();
}

void startPubLocalMap() {
    std::cout << "start seletc_Map "<< std::endl;
    kTrimmer->setCurrentMap(7);  // para1 : mapId;   para2 : format;
}

void startSaveMap() {
    std::cout << "start save_map "<< std::endl;
    room_id = kTrimmer->saveMap();  
    std::cout << "mapid = "<< room_id << std::endl;
}

void startRemoveMap() {
    std::cout << "start removeMap "<< std::endl;
    kTrimmer->removeMap(204); 
}

bool fullClean() {
    Pose startPose;
    startPose.x = 90;
    startPose.y = 100;
    startPose.orientation = 0;
    kTrimmer->setCurrentMap(5);
    kNavigation->doFullCoveragePathPlan(startPose);
}

void doFullCoveragePathPlan() {
    static int seq = 0;
    std::cout << "Input pose of the initial point: " << std::endl;
    std::string x_str, y_str, orientation_str;
    std::cout << "X coordinate of the initial point" << std::endl;
    std::cin >> x_str;
    std::cout << "Y coordinate of the initial point" << std::endl;
    std::cin >> y_str;
    std::cout << "Orientation of the initial point" << std::endl;
    std::cin >> orientation_str;

    flslam::Pose initialPoint;
    initialPoint.x = atof(x_str.c_str());
    initialPoint.y = atof(y_str.c_str());
    initialPoint.orientation = atof(orientation_str.c_str());
    kNavigation->doFullCoveragePathPlan(initialPoint);
}
 
int32_t main(int argc, char **argv) {
    kMapping = std::make_shared<flslam::Mapping>();
	kNavigation = std::make_shared<flslam::Navigation>();
    kTrimmer = std::make_shared<flslam::Trimmer>();

    functionsMap.emplace("0", stopMapping);
    functionsMap.emplace("1", startMapping);
    functionsMap.emplace("2", continueMapping);
    functionsMap.emplace("3", saveMap);
    functionsMap.emplace("4", localization);
    functionsMap.emplace("5", mappingStressTest);
    functionsMap.emplace("6", setNavigationGoal);
    functionsMap.emplace("7", registerGlobalPlanningPathListener);
    functionsMap.emplace("8", teleopKey);
	functionsMap.emplace("9", doFullCoveragePathPlan);

    functionsMap.emplace("s0", startSaveMap);
    functionsMap.emplace("s1", startSplitRoom);
    functionsMap.emplace("s2", startSetVisualWall);
    functionsMap.emplace("s3", startSetForbidArea);
    functionsMap.emplace("s4", startMergeRoom);
    functionsMap.emplace("s5", startGetRoomInfo);
    functionsMap.emplace("s6", startPubLocalMap);
    functionsMap.emplace("s7", startRemoveMap);
    functionsMap.emplace("s8", fullClean);

    std::string cmd;
    while (true) {
        std::cout << helpMessage;
        std::cin >> cmd;

        auto function = functionsMap.find(cmd);
        if (functionsMap.end() != function) {
            function->second();
        }

        if ("q" == cmd) {
            break;
        }
    }
    return 0;
}
