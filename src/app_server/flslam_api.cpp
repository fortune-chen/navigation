#include "flslam_api.h"
#include<math.h>
#include "trimmer.h"
#include "navigation.h"
#include "mapping.h"
using namespace flslam;

int mapid;
Trimmer kTrimmer;
Mapping KMapping;
Navigation kNvaigation;
std::vector<Room> room_info;

FlslamApi::FlslamApi() {

}

bool FlslamApi::startWallFollow() {
    KMapping.start();
}

bool FlslamApi::stopWallFollow() {
    KMapping.stop();
    mapid = KMapping.saveMap();
}

bool FlslamApi::continueMapping(int mapId) {
    kTrimmer.setCurrentMap(mapId);      
    KMapping.continueMapping(mapId);
}

int FlslamApi::saveMap() {
    mapid = kTrimmer.saveMap();
    return mapid;
}

bool FlslamApi::removeMap(int mapId) {      
    kTrimmer.removeMap(mapId);
}

bool FlslamApi::setCurrentMap(int mapId) {
    kTrimmer.setCurrentMap(mapId);
}

Image FlslamApi::setVisualWall(Wall& wall) {
    Image pngmap = kTrimmer.setVisualWall(wall.start_x, wall.start_y, wall.end_x, wall.end_y);
    return pngmap;
}

Image FlslamApi::setForbidArea(Zone& zone) {
    float width_x = zone.p1_x - zone.p2_x;
    float width_y = zone.p1_y - zone.p2_y;
    float width = sqrt(width_x * width_x + width_y * width_y);
    float height_x = zone.p1_x - zone.p4_x;
    float height_y = zone.p1_y - zone.p4_y;
    float height = sqrt(height_x * height_x + height_y * height_y);;
    Image pngmap = kTrimmer.setForbidArea(zone.p1_x, zone.p1_y, width, height);
    return pngmap;
}

Image FlslamApi::splitRoom() {
    Image pngmap = kTrimmer.splitRoom(30, 1.7);
    return pngmap;
}

std::vector<Room> FlslamApi::getRoomInfo() {
    room_info = kTrimmer.RoomInfo();
    return room_info;
}

Image FlslamApi::mergeRoom(int roomid1, int roomid2) {
    int32_t room_num1 = 0;
    int32_t room_num2 = 0;
    if(room_info.size() >= 0)
    {
        std::cout << "not executed split room;" << std::endl;
        Image pngmap;
        return pngmap;
    }    
    for (int i = 0; i < room_info.size(); i++) {
        if(roomid1 == room_info.at(i).room_id) {
            room_num1 = i;
        } else if(roomid1 == room_info.at(i).room_id) {
            room_num2 = i;
        }
    }
    Image pngmap = kTrimmer.mergeRoom(room_num1, room_num2);
    return pngmap;
}

bool FlslamApi::fullClean(Point startPoint) {
    Pose startPose;
    startPose.x = startPoint.x;
    startPose.y = startPoint.y;
    startPose.orientation = 0;
    // kNvaigation.setGoal(startPose);
    kTrimmer.setCurrentMap(mapid);
    kNvaigation.doFullCoveragePathPlan(startPose);
}

bool FlslamApi::areaClean(Zone& zone) {
    Pose startPose;
    startPose.x = zone.p1_x + 10;
    startPose.y = zone.p1_y + 10;
    startPose.orientation = 0;
    // kNvaigation.setGoal(startPose);
    kTrimmer.setCurrentMap(mapid);
    kNvaigation.doFullCoveragePathPlan(startPose);
}

