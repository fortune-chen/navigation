#ifndef FLSLAM_SRC_TRIMMER_SRC_TRIMMER_MAP_MGR_H
#define FLSLAM_SRC_TRIMMER_SRC_TRIMMER_MAP_MGR_H

#include <map>
#include "trimmer_dds_client.h"

class MapMgr {
public:
    cv::Mat nva_map;
    cv::Mat map_Optimize;
    nav_msgs::OccupancyGrid deal_map;
    RoomPublish room_information;
    uint32_t map_width,map_height;   		
    int raw_map_flag = 1;           
    int app_server_flag = 2;        
    cv::Mat map_optimize;
    cv::Mat room_Split;
    cv::Mat virtul_wall;
    cv::Mat set_Forbidden;
    cv::Mat room_Merge;
    cv::Mat nva_room_Merge;
    cv::Mat room_Info;
    cv::Mat color_segmented_map;

    std::vector<Room> rooms;
    const FunctionOrder* app_command; 

    std::vector<std::vector<std::vector<cv::Point>>> all_contour_points;

    MapMgr(std::shared_ptr<TrimmerDdsClient> dds_client);

    ~MapMgr();

    void registerHandler();

    void gridMapUpdateHandler(const nav_msgs::OccupancyGrid* map);

    void mapConfigureHandler(const FunctionOrder* command);

private:
    void optimizeMap();

    void splitRoom(int max_iterations, double min_critical_point_distance_factor);

    void setVirtualWall(float start_x, float start_y, float end_x, float end_y);

    void setForbidden(float point_x, float point_y, float width, float height);

    void mergeRoom(int room1, int room2);

    void pubRoomInfo();

    void pubMapModify();

    void MatToOccupancyGrid( );

    std::shared_ptr<TrimmerDdsClient> m_ddsClient;
    std::map<int, std::string> m_gridMaps;
    std::map<int, std::string> m_pngMaps;
    std::map<int, std::string> m_contourMaps;

     int saveMap();

     void saveGridmap(int mapId, cv::Mat nva_map);

     void saveContourmap(int mapId, cv::Mat nva_map);

     void savePngmap(int mapId, cv::Mat app_map);

     void RemoveMap(int mapId);
};
#endif  // FLSLAM_SRC_TRIMMER_SRC_TRIMMER_MAP_MGR_H
