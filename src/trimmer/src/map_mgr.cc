#include "map_mgr.h"
#include <dirent.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/highgui.h>
#include "nav_msgs/OccupancyGrid.h"

#include "RoomPublish.h"
#include "RoomPublishPubSubTypes.h"
#include <sensor_msgs/Image.h>

int mapId;
constexpr int kMaxStoredGripMap = 5;
constexpr char kStoredGripMapPath[] = "/var/tmp/flslam/map/";
std::string path = kStoredGripMapPath;

MapMgr::MapMgr(std::shared_ptr<TrimmerDdsClient> dds_client) {
    m_ddsClient = dds_client;
    struct dirent *ptr;    
    DIR *dir;
    dir = opendir(kStoredGripMapPath);
    if (dir == NULL) {
        std::string command;
        command = "mkdir -p " + path;  
        system(command.c_str());
    }
    while((ptr=readdir(dir))!=NULL) {
        if(ptr->d_name[0] == '.') {
            continue;
        }
        std::string mapPath = path + ptr->d_name;
        std::size_t grid = mapPath.find(".gridmap");
        std::size_t png = mapPath.find(".pngmap");
        std::size_t contour = mapPath.find(".contourmap");
        if(grid != std::string::npos) {
            int fileName = atoi(mapPath.substr(20, grid - 20).c_str());  
            m_gridMaps.emplace(fileName, mapPath);
        } else if(png != std::string::npos) {
            int fileName = atoi(mapPath.substr(20, png - 20).c_str());  
            m_pngMaps.emplace(fileName, mapPath);
        }
        else if(contour != std::string::npos) {
            int fileName = atoi(mapPath.substr(20, contour - 20).c_str());  
            m_contourMaps.emplace(fileName, mapPath);
        }
    }
    closedir(dir);
}

MapMgr::~MapMgr() {
}

void MapMgr::registerHandler() {
  m_ddsClient->registerGridMapUpdateHandler([this](const nav_msgs::OccupancyGrid* msg){
    this->gridMapUpdateHandler(msg);}); 

  m_ddsClient->registerMapConfigureHandler([this](const FunctionOrder* command){
    this->mapConfigureHandler(command);});
}

void MapMgr::gridMapUpdateHandler(const nav_msgs::OccupancyGrid* map) {
    const nav_msgs::OccupancyGrid* raw_map = map; 

    nva_map.create(raw_map->info.height, raw_map->info.width, CV_8UC1);
    for(int i=0; i< raw_map->info.height * raw_map->info.width; i++) {
        nva_map.data[i] = raw_map->data[i];
    }
    MapMgr::optimizeMap();
}

void MapMgr::mapConfigureHandler(const FunctionOrder* command) {
    app_command = command; 

    // if(app_command->set_optimize().flag() == true) {
    //     MapMgr::optimizeMap();
    // }
    // 接收一个保存地图指令；
    if(app_command->set_savemap().flag() == true) {  
        std::cout << "set_savemap " << std::endl;
        MapMgr::pubMapModify();
    }
    if(app_command->set_removemap().flag() == true) {  
        std::cout << "RemoveMap " << std::endl;
        MapMgr::RemoveMap(app_command->set_removemap().mapid());
    }
    if(app_command->set_split().flag() == true) {   
        MapMgr::splitRoom(app_command->set_split().split_param().param4(), app_command->set_split().split_param().param5());
    }
    if(app_command->set_wall().flag() == true) {
        MapMgr::setVirtualWall(app_command->set_wall().point().at(0).x(), app_command->set_wall().point().at(0).y(), app_command->set_wall().point().at(1).x(), app_command->set_wall().point().at(1).y());
    }
    if(app_command->set_area().flag() == true) {
        MapMgr::setForbidden(app_command->set_area().point().x(), app_command->set_area().point().y(), app_command->set_area().width(), app_command->set_area().height());
    }
    if(app_command->set_merge().flag() == true) {
        MapMgr::mergeRoom(app_command->set_merge().room_num1(), app_command->set_merge().room_num2());
    }
    if(app_command->set_info().flag() == true) {
        MapMgr::pubRoomInfo();
    }
    #ifdef DISPLAY_IMSHOW
        cv::waitKey(0);       
    #endif
}

void MapMgr::optimizeMap() {
    map_Optimize = nva_map.clone();
    if(map_Optimize.empty()) {
        std::cout << "The gridmap is empty;"  << std::endl;
    }
    cv::Mat dst_erode;
    cv::Mat structElement = cv::getStructuringElement( 0, cv::Size(3,3), cv::Point(-1,-1));
    erode(nva_map, dst_erode, structElement);
    cv::Mat dst_medianBlur;
    medianBlur(dst_erode, dst_medianBlur, (5,5));
    cv::Mat dst_dilate;
    structElement = cv::getStructuringElement(0, cv::Size(3,3), cv::Point(-1,-1));
    dilate(dst_medianBlur, dst_dilate, structElement, cv::Point(-1,-1), 1);
    nva_map = dst_dilate; 
    #ifdef DISPLAY_IMSHOW
        cv::imshow("dst_dilate", nva_map);
    #endif
    // MapMgr::saveMap();
    MapMgr::MatToOccupancyGrid();

    sensor_msgs::Image Png_image;
    Png_image.header.frame_id = std::to_string(mapId);
    Png_image.width = nva_map.cols;
    Png_image.height = nva_map.rows;
    if(0 == nva_map.type()) {
        Png_image.encoding = "8UC1";
    } else if(16 == nva_map.type()) {
        Png_image.encoding = "8UC3";
    }
    Png_image.is_bigendian = 0;
    Png_image.step = nva_map.channels() * nva_map.cols;
    for (int row = 0; row <  Png_image.height; row++) {
        for(int col = 0; col < Png_image.step; col++) {
            if(1 == nva_map.channels()) {
                uint8_t com = nva_map.at<uchar>(row,col) ;
                Png_image.data.push_back(com);
            } else if(3 == nva_map.channels()) {
                uint8_t com0 = nva_map.at<cv::Vec3b>(row, col)[0];
                Png_image.data.push_back(com0);
                uint8_t com1 = nva_map.at<cv::Vec3b>(row, col)[1];
                Png_image.data.push_back(com1);
                uint8_t com2 = nva_map.at<cv::Vec3b>(row, col)[2];
                Png_image.data.push_back(com2);
            }
        }
    }
    m_ddsClient->pubPngMap(Png_image); 
}

int MapMgr::saveMap() {
    std::cout << "save map function;" << std::endl;    
    int mapOld, mapNew;
    mapOld = m_gridMaps.begin()->first;
    mapNew = m_gridMaps.begin()->first;

    for (auto iter : m_gridMaps) {
        if (mapOld > iter.first) {
            mapOld = iter.first;        
        }
        if (mapNew < iter.first) {
            mapNew = iter.first;        
        }
    }
    if (m_gridMaps.size() >= kMaxStoredGripMap) {
        std::string command = "rm -f " + m_gridMaps[mapOld] + "&& rm -f " + m_pngMaps[mapOld] + "&& rm -f " + m_contourMaps[mapOld];  
        system(command.c_str());
        m_gridMaps.erase(mapOld);
        m_pngMaps.erase(mapOld);        
        m_contourMaps.erase(mapOld);        
    } 
    mapId = mapNew + 1;    
    saveGridmap(mapId, map_Optimize);
    saveContourmap(mapId, map_Optimize);
    savePngmap(mapId, map_Optimize);
    return mapId;
}

void MapMgr::pubMapModify() {
    int map_Id = MapMgr::saveMap();
    MapModify mapOrder; 
    mapOrder.mapid(map_Id);
    m_ddsClient->pubMapModify(mapOrder);
    std::cout << "pubMapModify = " << map_Id << std::endl;
}


void MapMgr::RemoveMap(int map_Id) {
    std::string command = "rm -f " + m_gridMaps[map_Id] + "&& rm -f " + m_pngMaps[map_Id] + "&& rm -f " + m_contourMaps[map_Id];  
    system(command.c_str());
    m_gridMaps.erase(map_Id);
    m_pngMaps.erase(map_Id);        
    m_contourMaps.erase(map_Id);        
    std::cout << "pubMapModify = " << map_Id << std::endl;
}

void MapMgr::saveGridmap(int map_Id, cv::Mat nva_map) {
    std::FILE *gridmap;
    std::string gridname = path + std::to_string(map_Id) + ".gridmap";
    gridmap = std::fopen(gridname.c_str(), "wb");
    fprintf(gridmap,"P5\n");
    fprintf(gridmap,"%d %d\n",nva_map.cols,nva_map.rows);
    fprintf(gridmap,"%d\n",255);
    uchar *receive = nva_map.data;
    int32_t map_data[nva_map.cols * nva_map.rows];
    for (int32_t i = 0; i <nva_map.cols * nva_map.rows; ++i) {
        uint8_t com = receive[i];
        fputc(com, gridmap);
    }
    fclose(gridmap);
}
void MapMgr::saveContourmap(int map_Id, cv::Mat nva_map) {
    std::FILE *contourmap;
    std::string contourname = path + std::to_string(map_Id) + ".contourmap";
    contourmap = std::fopen(contourname.c_str(), "wb");
    fprintf(contourmap,"P5\n");
    fprintf(contourmap,"%d %d\n",nva_map.cols,nva_map.rows);
    fprintf(contourmap,"%d\n",255);
    uchar *receive = nva_map.data;
    int32_t map_data[nva_map.cols * nva_map.rows];
    for (int32_t i = 0; i <nva_map.cols * nva_map.rows; ++i) {
        uint8_t com = receive[i];
        fputc(com, contourmap);
    }
    fclose(contourmap);
}

void MapMgr::savePngmap(int map_Id, cv::Mat app_map) {
    cv::imwrite("raw_map.png", app_map);
    std::fstream file;
    file.open("./raw_map.png");
    std::istreambuf_iterator<char>beg(file), end;
    std::string data(beg, end);
    file.close();
    std::fstream pngmap;
    std::string pngname = path + std::to_string(map_Id) + ".pngmap";
    pngmap.open(pngname, std::ios::out);
    pngmap << data;
    pngmap.close();
}

void MapMgr::splitRoom(int max_iterations, double min_critical_point_distance_factor) {
    room_Split = nva_map.clone();
    if(room_Split.empty()) {
        std::cout << "The grid map is empty;"  << std::endl;
    }
    cv::threshold(room_Split, room_Split, 225, 255, CV_THRESH_BINARY);
    const cv::Mat& original_img = room_Split;
    cv::Mat segmented_map;
    segmented_map.create(room_Split.size(), room_Split.type());
    const float map_resolution = 0.05;
    double room_upper_limit_voronoi_ = 7.0;                
    double room_lower_limit_voronoi_ = 1.6;                
    int voronoi_neighborhood_index_ = 120;                 
    int max_iterations_ = max_iterations;             
    double min_critical_point_distance_factor_ = min_critical_point_distance_factor;     
    double max_area_for_merging_ = 1.0;              
    bool publish_segmented_map = false;
    bool display_segmented_map_ = false;
    static bool DEBUG_DISPLAYS=false;

    VoronoiSegmentation voronoi_segmentation; 
    voronoi_segmentation.segmentMap(original_img, segmented_map, map_resolution, room_lower_limit_voronoi_, room_upper_limit_voronoi_,
        voronoi_neighborhood_index_, max_iterations_, min_critical_point_distance_factor_, max_area_for_merging_, (display_segmented_map_&&DEBUG_DISPLAYS));
    
    #ifdef DISPLAY_IMSHOW
        cv::imshow("segmented_map", segmented_map);
    #endif
      
	std::map<int, size_t> label_vector_index_codebook; 
	size_t vector_index = 0;
	for (int v = 0; v < segmented_map.rows; ++v) {
		for (int u = 0; u < segmented_map.cols; ++u) {
			const int label = segmented_map.at<int>(v, u);                     
			if (label > 0 && label < 65280) {
				if (label_vector_index_codebook.find(label) == label_vector_index_codebook.end()) {
					label_vector_index_codebook[label] = vector_index;
					vector_index++;
				}
			}
		}
	}
				
	std::vector<int> min_x_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_x_value_of_the_room(label_vector_index_codebook.size(), 0);
	std::vector<int> min_y_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_y_value_of_the_room(label_vector_index_codebook.size(), 0);
	std::vector<int> room_centers_x_values(label_vector_index_codebook.size(), -1);
	std::vector<int> room_centers_y_values(label_vector_index_codebook.size(), -1);

    cv::Mat indexed_map = segmented_map.clone();
	for (int y = 0; y < segmented_map.rows; ++y) {
		for (int x = 0; x < segmented_map.cols; ++x) {
			const int label = segmented_map.at<int>(y,x);
			if (label > 0 && label < 65280) {
				indexed_map.at<int>(y,x) = label_vector_index_codebook[label]+1;
            }
		}
	}
    std::vector<cv::Vec3b> room_color;
    color_segmented_map = indexed_map.clone();
    color_segmented_map.convertTo(color_segmented_map, CV_8U);
    cv::cvtColor(color_segmented_map, color_segmented_map, CV_GRAY2BGR);
	for(size_t i = 1; i <= room_centers_x_values.size(); ++i) {
        const cv::Vec3b color((rand() % 230) + 1, (rand() % 230) + 1, (rand() % 230) + 1);
        for(size_t v = 0; v < indexed_map.rows; ++v) {
            for(size_t u = 0; u < indexed_map.cols; ++u) {
                if(indexed_map.at<int>(v,u) == i) {
                    color_segmented_map.at<cv::Vec3b>(v,u) = color;
                }
            }   
        }
        room_color.push_back(color);
	}

    const cv::Vec3b ignore_zone(200, 200, 200);
    for(size_t v = 0; v < indexed_map.rows; ++v) {
        for(size_t u = 0; u < indexed_map.cols; ++u) {
            if(indexed_map.at<int>(v,u) == 0 || indexed_map.at<int>(v,u) >= 250) {
                color_segmented_map.at<cv::Vec3b>(v,u) = ignore_zone;
            }
        }   
    }
    #ifdef DISPLAY_IMSHOW
        cv::imshow("color_segmented_map", color_segmented_map);
    #endif

    sensor_msgs::Image Png_image;
    Png_image.header.frame_id = std::to_string(mapId);
    Png_image.width = color_segmented_map.cols;
    Png_image.height = color_segmented_map.rows;
    if(0 == color_segmented_map.type()) {
        Png_image.encoding = "8UC1";
    } else if(16 == color_segmented_map.type()) {
        Png_image.encoding = "bgr8";
    }
    Png_image.is_bigendian = 0;
    Png_image.step = color_segmented_map.channels() * color_segmented_map.cols;
    for (int row = 0; row <  Png_image.height; row++) {    
        for(int col = 0; col < Png_image.width; col++) {    
            if(1 == color_segmented_map.channels()) {   
                uint8_t com = color_segmented_map.at<uchar>(row,col) ;
                Png_image.data.push_back(com);
            } else if(3 == color_segmented_map.channels()) {
                uint8_t com0 = color_segmented_map.at<cv::Vec3b>(row, col)[0];
                Png_image.data.push_back(com0);
                uint8_t com1 = color_segmented_map.at<cv::Vec3b>(row, col)[1];
                Png_image.data.push_back(com1);
                uint8_t com2 = color_segmented_map.at<cv::Vec3b>(row, col)[2];
                Png_image.data.push_back(com2);
            }
        }
    }
    m_ddsClient->pubPngMap(Png_image); 
    savePngmap(mapId, color_segmented_map);

    extern std::vector<Room> rooms;
    cv::Mat nav_split = nva_map.clone();
    for (int i = 0; i < rooms.size() ; ++i) {
        cv::Mat black(nav_split.rows, nav_split.cols, CV_8UC1,cv::Scalar(0));               
        const std::vector<cv::Point>& contour_point = rooms[i].getMembers();         
        for (double m = 0; m < contour_point.size(); m++) {
            uchar* p = black.ptr<uchar>(contour_point[m].y);                        
            p[contour_point[m].x] = 255; 
        }
        std::vector<std::vector<cv::Point>> contours;
        findContours(black, contours, 3, 1);
        drawContours(nav_split, contours, 0, cv::Scalar(0), 1); 
        all_contour_points.push_back(contours);
    }

    nva_map = nav_split;
    #ifdef DISPLAY_IMSHOW
        cv::imshow("nva_segmented_map", nav_split);
    #endif
    saveGridmap(mapId, nav_split);
    saveContourmap(mapId, nav_split);
    MapMgr::MatToOccupancyGrid();
}

void MapMgr::setVirtualWall(float start_x = 0, float start_y = 0, float end_x = 0, float end_y = 0) {
    virtul_wall = nva_map.clone();
    cv::Point start_p = cv::Point(start_x,start_y);
    cv::Point end_p = cv::Point(end_x,end_y);
    cv::line(virtul_wall, start_p, end_p, cv::Scalar(0, 0, 0), 2, 8);
    #ifdef DISPLAY_IMSHOW
        cv::imshow("virtul_wall", virtul_wall);
    #endif      
    nva_map = virtul_wall;
    saveGridmap(mapId, virtul_wall);
    MapMgr::MatToOccupancyGrid();

    virtul_wall = color_segmented_map.clone();
    cv::Point start_p1 = cv::Point(start_x,start_y);
    cv::Point end_p1 = cv::Point(end_x,end_y);
    cv::line(virtul_wall, start_p1, end_p1, cv::Scalar(0, 0, 0), 2, 8);
    #ifdef DISPLAY_IMSHOW
        cv::imshow("color_virtul_wall", virtul_wall);
    #endif 
    color_segmented_map = virtul_wall;

    sensor_msgs::Image Png_image;
    Png_image.header.frame_id = std::to_string(mapId);
    Png_image.width = color_segmented_map.cols;
    Png_image.height = color_segmented_map.rows;
    if(0 == color_segmented_map.type()) {
        Png_image.encoding = "8UC1";
    } else if(16 == color_segmented_map.type()) {
        Png_image.encoding = "bgr8";
    }
    Png_image.is_bigendian = 0;
    Png_image.step = color_segmented_map.channels() * color_segmented_map.cols;
    for (int row = 0; row <  Png_image.height; row++) {    
        for(int col = 0; col < Png_image.width; col++) {    
            if(1 == color_segmented_map.channels()) {   
                uint8_t com = color_segmented_map.at<uchar>(row,col) ;
                Png_image.data.push_back(com);
            } else if(3 == color_segmented_map.channels()) {
                uint8_t com0 = color_segmented_map.at<cv::Vec3b>(row, col)[0];
                Png_image.data.push_back(com0);
                uint8_t com1 = color_segmented_map.at<cv::Vec3b>(row, col)[1];
                Png_image.data.push_back(com1);
                uint8_t com2 = color_segmented_map.at<cv::Vec3b>(row, col)[2];
                Png_image.data.push_back(com2);
            }
        }
    }
    m_ddsClient->pubPngMap(Png_image);  
    savePngmap(mapId, color_segmented_map);
}

void MapMgr::setForbidden(float point_x = 0, float point_y = 0, float width = 0, float height = 0) {
    set_Forbidden = nva_map.clone();
    cv::Rect rect = cv::Rect(point_x,point_y,width,height);
    cv::Scalar color = cv::Scalar(0, 0, 0);
    rectangle(set_Forbidden,rect,color,2, 8 );
    #ifdef DISPLAY_IMSHOW
        cv::imshow("set_Forbidden", set_Forbidden);
    #endif 
    nva_map = set_Forbidden;
    saveGridmap(mapId, set_Forbidden);
    MapMgr::MatToOccupancyGrid();

    set_Forbidden = color_segmented_map.clone();
    cv::Rect rect1 = cv::Rect(point_x,point_y,width,height);
    cv::Scalar color1 = cv::Scalar(0, 0, 0);
    rectangle(set_Forbidden,rect1,color1,2, 8 );
    #ifdef DISPLAY_IMSHOW
        cv::imshow("color_set_Forbidden", set_Forbidden);
    #endif 
    color_segmented_map = set_Forbidden;
    sensor_msgs::Image Png_image;
    Png_image.header.frame_id = std::to_string(mapId);
    Png_image.width = color_segmented_map.cols;
    Png_image.height = color_segmented_map.rows;
    if(0 == color_segmented_map.type()) {
        Png_image.encoding = "8UC1";
    } else if(16 == color_segmented_map.type()) {
        Png_image.encoding = "bgr8";
    }
    Png_image.is_bigendian = 0;
    Png_image.step = color_segmented_map.channels() * color_segmented_map.cols;
    for (int row = 0; row <  Png_image.height; row++) {    
        for(int col = 0; col < Png_image.width; col++) {    
            if(1 == color_segmented_map.channels()) {   
                uint8_t com = color_segmented_map.at<uchar>(row,col) ;
                Png_image.data.push_back(com);
            } else if(3 == color_segmented_map.channels()) {
                uint8_t com0 = color_segmented_map.at<cv::Vec3b>(row, col)[0];
                Png_image.data.push_back(com0);
                uint8_t com1 = color_segmented_map.at<cv::Vec3b>(row, col)[1];
                Png_image.data.push_back(com1);
                uint8_t com2 = color_segmented_map.at<cv::Vec3b>(row, col)[2];
                Png_image.data.push_back(com2);
            }
        }
    }
    m_ddsClient->pubPngMap(Png_image); 
    savePngmap(mapId, color_segmented_map);
}

void MapMgr::mergeRoom(int room1, int room2) {
    extern std::vector<Room> rooms;
    if(rooms.size() <= 0) {
        std::cout << "Without room split, rooms cannot be merge;" << std::endl;
        return ;
    }
    bool merge_flag = false;
    nva_room_Merge = nva_map.clone();
    for(int i=0; i< all_contour_points.at(room1).at(0).size(); i++) {
        for(int j=0; j< all_contour_points.at(1).at(0).size(); j++) {
            if(pow(all_contour_points.at(room1).at(0).at(i).x - all_contour_points.at(room2).at(0).at(j).x, 2) +
                pow(all_contour_points.at(room1).at(0).at(i).y - all_contour_points.at(room2).at(0).at(j).y, 2) == 1) {
                uchar* p = nva_room_Merge.ptr<uchar>(all_contour_points.at(room1).at(0).at(i).y);                          //获取第i行的首地址；
                p[all_contour_points.at(room1).at(0).at(i).x] = 255; 
                uchar* q = nva_room_Merge.ptr<uchar>(all_contour_points.at(room2).at(0).at(j).y);                          //获取第i行的首地址；
                q[all_contour_points.at(room2).at(0).at(j).x] = 255; 
                merge_flag = true;
            }
        }
    }
    if(merge_flag == false) {
        std::cout << "The room is not in the adjacent area and cannot be merged;;" << std::endl;
        return ;
    }
    nva_map = nva_room_Merge;
    #ifdef DISPLAY_IMSHOW
        cv::imshow("room_Merge", nva_room_Merge);
    #endif 
    saveGridmap(mapId, nva_room_Merge);
    MapMgr::MatToOccupancyGrid();
    
    room_Merge = color_segmented_map.clone();
    int32_t point1_x = rooms[room1].getCenter().x;
    int32_t point1_y = rooms[room1].getCenter().y;
    int point1_b = room_Merge.at<cv::Vec3b>(point1_y, point1_x)[0];
    int point1_g = room_Merge.at<cv::Vec3b>(point1_y, point1_x)[1];
    int point1_r = room_Merge.at<cv::Vec3b>(point1_y, point1_x)[2];

    int32_t point2_x = rooms[room2].getCenter().x;
    int32_t point2_y = rooms[room2].getCenter().y;
    int point2_b = room_Merge.at<cv::Vec3b>(point2_y, point2_x)[0];
    int point2_g = room_Merge.at<cv::Vec3b>(point2_y, point2_x)[1];
    int point2_r = room_Merge.at<cv::Vec3b>(point2_y, point2_x)[2];

    for(size_t v = 0; v < room_Merge.rows; ++v) {
        for(size_t u = 0; u < room_Merge.cols; ++u) {
            if(room_Merge.at<cv::Vec3b>(v, u)[0] == point2_b && room_Merge.at<cv::Vec3b>(v, u)[1] == point2_g && room_Merge.at<cv::Vec3b>(v, u)[2] == point2_r) {
                room_Merge.at<cv::Vec3b>(v, u)[0] = point1_b; 
                room_Merge.at<cv::Vec3b>(v, u)[1] = point1_g;
                room_Merge.at<cv::Vec3b>(v, u)[2] = point1_r;
            }
        }   
    }
    #ifdef DISPLAY_IMSHOW
        cv::imshow("color_room_Merge", room_Merge);
    #endif
    color_segmented_map = room_Merge;
    sensor_msgs::Image Png_image;
    Png_image.header.frame_id = std::to_string(mapId);
    Png_image.width = color_segmented_map.cols;
    Png_image.height = color_segmented_map.rows;
    if(0 == color_segmented_map.type()) {
        Png_image.encoding = "8UC1";
    } else if(16 == color_segmented_map.type()) {
        Png_image.encoding = "bgr8";
    }
    Png_image.is_bigendian = 0;
    Png_image.step = color_segmented_map.channels() * color_segmented_map.cols;
    for (int row = 0; row <  Png_image.height; row++) {    
        for(int col = 0; col < Png_image.width; col++) {    
            if(1 == color_segmented_map.channels()) {   
                uint8_t com = color_segmented_map.at<uchar>(row,col) ;
                Png_image.data.push_back(com);
            } else if(3 == color_segmented_map.channels()) {
                uint8_t com0 = color_segmented_map.at<cv::Vec3b>(row, col)[0];
                Png_image.data.push_back(com0);
                uint8_t com1 = color_segmented_map.at<cv::Vec3b>(row, col)[1];
                Png_image.data.push_back(com1);
                uint8_t com2 = color_segmented_map.at<cv::Vec3b>(row, col)[2];
                Png_image.data.push_back(com2);
            }
        }
    }
    m_ddsClient->pubPngMap(Png_image);  
    savePngmap(mapId, color_segmented_map);
}

void MapMgr::pubRoomInfo() {
    room_Info =  color_segmented_map.clone();
    extern std::vector<Room> rooms;
    if(rooms.size() < 0) {
        std::cout << "The room information cannot be solved without room segmentation;" << std::endl;
        return ;
    }
    OneRoom one_room;
    cv::Point centre_point;
    Position getcenter;
    room_information.room_num(rooms.size());
    for (int i = 0; i < rooms.size() ; ++i) {	
        one_room.room_id(rooms[i].getID());
        centre_point = rooms[i].getCenter();
        getcenter.x(centre_point.x);
        getcenter.y(centre_point.y);
        one_room.room_centre(getcenter);
        one_room.room_area(rooms[i].getArea());
        one_room.clean_num(0);
        one_room.forbid(false);
        
        std::vector<std::vector<cv::Point2f>> contour_points_all;
        std::vector<cv::Point2f> room_32_point;
        cv::Mat contour(room_Info.rows, room_Info.cols, CV_8UC1,cv::Scalar(0));
        const std::vector<cv::Point>& contour_point = rooms[i].getMembers();
        int contour_num = 32;
        Contour room_conts;
        for (double m = 0; m < contour_point.size(); m++) {
            uchar* p = contour.ptr<uchar>(contour_point[m].y);
            p[contour_point[m].x] = 255; 
        }	
        goodFeaturesToTrack( contour, room_32_point, contour_num, 0.01, 1, cv::Mat(), 2, false, 0.04);
        for( size_t i = 0; i < room_32_point.size(); i++ ) {
            room_conts.contour_point().at(i).x(room_32_point[i].x);
            room_conts.contour_point().at(i).y(room_32_point[i].y);
        }
        one_room.room_points(room_conts);
        room_information.room_info().emplace_back(one_room);
    }
    m_ddsClient->pubRoomInfo(room_information);
    std::cout << "pubRoomInfo = " << rooms.size() << std::endl;
}

void MapMgr::MatToOccupancyGrid( ) {
    cv::Mat MatToOccupancyGrid = nva_map;
    map_width = MatToOccupancyGrid.cols;
    map_height = MatToOccupancyGrid.rows;
    int32_t map_dat[map_width * map_height];
    std::vector<int8_t> pub_data;
	for (int32_t i = 0; i < map_width * map_height; ++i) {
        uint8_t com = MatToOccupancyGrid.data[i];
		map_dat[i] = com;
		pub_data.push_back(map_dat[i]);	
	}
    static uint32_t count = 0;
    deal_map.header.frame_id = "deal_map";
    deal_map.header.seq = count;
    deal_map.header.stamp.sec = 20;
    deal_map.header.stamp.nsec = 30;
    deal_map.info.resolution = 0.05;
    deal_map.info.width = map_width;
    deal_map.info.height = map_height;
    deal_map.info.origin.position.x = 0;
    deal_map.info.origin.position.y = 0;
    deal_map.info.origin.position.z = 0;
    deal_map.info.origin.orientation.x = 0;
    deal_map.info.origin.orientation.y = 0;
    deal_map.info.origin.orientation.z = 0;
    deal_map.info.origin.orientation.w = 0.4;
    deal_map.data = pub_data;
    count ++;

    m_ddsClient->pubGridMap(deal_map);
}