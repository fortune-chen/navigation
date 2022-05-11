#include "trimmer.h"
#include <fstream>
#include <ctime>  
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "RosAdapterContent.h"
#include "RosAdapterContentPubSubTypes.h"
#include "RoomPublish.h"
#include "RoomPublishPubSubTypes.h"
#include <sensor_msgs/Image.h>

constexpr char kStoredGripMapPath[] = "/var/tmp/flslam/map/";
std::string path = kStoredGripMapPath;

using namespace flslam;

static std::shared_ptr<DDSPublisher> app_server_command;
static std::vector<std::shared_ptr<DDSSubscriber>> received_map;
static std::vector<std::shared_ptr<DDSSubscriber>> received_mapInfo;
static std::vector<std::shared_ptr<DDSSubscriber>> received_mapModify;
RoomPublish* received_room_info;
MapModify* received_map_modify;
sensor_msgs::Image* msgImage;
Image  image;

bool flag_room_info = false;
bool flag_png = false;
bool flag_map_modify = false;
class app_server_Listener : public TopicListener {
public:
    app_server_Listener(flslam::Trimmer* trimmer, const std::string& topic) : TopicListener((void*)&m_data) {
        trimmer_sub = trimmer;
        Topic = topic;
    }
    void onTopicReceived(void* msg) override {
        msgImage = (sensor_msgs::Image*)msg;       // T = sensor_msgs::Image;
        image.mapid = atoi(msgImage->header.frame_id.c_str()); 
        image.width = msgImage->width;
        image.height = msgImage->height;
        image.encoding = msgImage->encoding;
        image.step = msgImage->step;
        image.is_bigendian = msgImage->is_bigendian;
        for (int i = 0; i < msgImage->data.size(); i++) {
            image.data.push_back(msgImage->data[i]);
        }
        flag_png = true;
    };
    sensor_msgs::Image m_data;
    std::string Topic;
    flslam::Trimmer* trimmer_sub;
};

class RoomInfoListener : public TopicListener {
public:
    RoomInfoListener(flslam::Trimmer* trimmer, const std::string& topic) : TopicListener((void*)&m_map) {
        trimmer_sub = trimmer;
        Topic = topic;
    }
    void onTopicReceived(void* msg) override {
        received_room_info = (RoomPublish*) msg;
        flag_room_info = true;
    };
    RoomPublish m_map;
    std::string Topic;
    flslam::Trimmer* trimmer_sub;
};

class MapModifyListener : public TopicListener {
public:
    MapModifyListener(flslam::Trimmer* trimmer, const std::string& topic) : TopicListener((void*)&m_map) {
        trimmer_sub = trimmer;
        Topic = topic;
    }
    void onTopicReceived(void* msg) override {
        received_map_modify = (MapModify*) msg;
        flag_map_modify = true;
    };
    MapModify m_map;
    std::string Topic;
    flslam::Trimmer* trimmer_sub;
};

Trimmer::Trimmer() {
    std::shared_ptr<FunctionOrderPubSubType> map_type(new FunctionOrderPubSubType);
    app_server_command = std::make_shared<DDSPublisher>("fl_app_server_command", map_type);
                
    std::shared_ptr<NavMapContentPubSubType> ros_type(new NavMapContentPubSubType);
    received_map.push_back(std::make_shared<DDSSubscriber>("fl_app_server_map", ros_type, std::make_shared<app_server_Listener>(this, "fl_app_server_map")));

    std::shared_ptr<RoomPublishPubSubType> room_type(new RoomPublishPubSubType);
    received_mapInfo.push_back(std::make_shared<DDSSubscriber>("fl_room_info", room_type, std::make_shared<RoomInfoListener>(this, "fl_room_info")));
    
    std::shared_ptr<MapModifyPubSubType> room_type1(new MapModifyPubSubType);
    received_mapModify.push_back(std::make_shared<DDSSubscriber>("fl_map_modify", room_type1, std::make_shared<MapModifyListener>(this, "fl_map_modify")));
}

Trimmer::~Trimmer() {
}


void Trimmer::optimizeMap() {
    FunctionOrder command;
    command.set_optimize().flag(true);
    app_server_command->publish<FunctionOrder>(command);
}

Image Trimmer::splitRoom(int max_iterations = 30, double min_critical_point_distance_factor = 1.7) {
    FunctionOrder command;
    command.set_split().flag(true);
    command.set_split().split_param().param1(10);                                       //房间面积的最大约束值
    command.set_split().split_param().param2(1);                                        // 房间面积的最小约束值    
    command.set_split().split_param().param3(120);                                      // 相邻系数：系数越大房间越少；
    command.set_split().split_param().param4(max_iterations);                           // 迭代次数；
    command.set_split().split_param().param5(min_critical_point_distance_factor);       // 最小临界点距离因子为：1.7
    command.set_split().split_param().param6(1.0);                                      // 融合系数，小于这个值的房间会被合并到其他房间；
    app_server_command->publish<FunctionOrder>(command);
	std::cout << "\n app_command publisher " << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    time_t now = time(0); 
    while (1) {
        if(flag_png == true ) {   
            flag_png = false;
            break;
        } else if(now + 2 < time(0) && flag_png == false) {
	        std::cout << "fl_app_server_map received fail;" << std::endl;
            break;
        } 
    }
    return image;
}

Image Trimmer::setVisualWall(float start_x = 50, float start_y = 50, float end_x = 100, float end_y = 50) {
    FunctionOrder command;
    command.set_wall().flag(true);
    command.set_wall().point().at(0).x(start_x);
    command.set_wall().point().at(0).y(start_y);
    command.set_wall().point().at(1).x(end_x);
    command.set_wall().point().at(1).y(end_y);
    app_server_command->publish<FunctionOrder>(command);
	std::cout << "\n app_command publisher " << std::endl;
    time_t now = time(0); 
    while (1) {
        if(flag_png == true ) {   
            flag_png = false;
            break;
        } else if(now + 2 < time(0) && flag_png == false) {
	        std::cout << "fl_app_server_map received fail;" << std::endl;
            break;
        } 
    }
    return image;
}

Image Trimmer::setForbidArea(float point_x = 150, float point_y = 100, float width = 50, float height = 100) {
    FunctionOrder command;
    command.set_area().flag(true);
    command.set_area().point().x(point_x);
    command.set_area().point().y(point_y);
    command.set_area().width(width);
    command.set_area().height(height);
    app_server_command->publish<FunctionOrder>(command);
	std::cout << "\n app_command publisher " << std::endl;
    time_t now = time(0); 
    while (1) {
        if(flag_png == true ) {   
            flag_png = false;
            break;
        } else if(now + 2 < time(0) && flag_png == false) {
	        std::cout << "fl_app_server_map received fail;" << std::endl;
            break;
        } 
    }
    return image;
}

Image Trimmer::mergeRoom(int room_num1 = 0, int room_num2 = 1) {
    FunctionOrder command;
    command.set_merge().flag(true);
    command.set_merge().room_num1(room_num1);
    command.set_merge().room_num2(room_num2);
    app_server_command->publish<FunctionOrder>(command);
	std::cout << "\n app_command publisher " << std::endl;
    time_t now = time(0); 
    while (1) {
        if(flag_png == true ) {   
            flag_png = false;
            break;
        } else if(now + 2 < time(0) && flag_png == false) {
	        std::cout << "fl_app_server_map received fail;" << std::endl;
            break;
        } 
    }
    return image;
}

std::vector<Room> Trimmer::RoomInfo() {
    FunctionOrder command;
    command.set_info().flag(true);
    app_server_command->publish<FunctionOrder>(command);
    std::vector<Room> Rooms_Info;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    while (1) {
        if(flag_room_info == true) {
           for (int i = 0; i < received_room_info->room_num(); i++) {       
                Room one_room;
                one_room.room_id = received_room_info->room_info().at(i).room_id();
                // one_room.color = "";
                // one_room.positon;
                Rooms_Info.push_back(one_room);
            } 
        }
        break;
    }
    return Rooms_Info;
}


//***********************Split binary string********************
std::vector<std::string> split(const std::string &str, const char pattern) {
	std::vector<std::string> res;
	std::stringstream input(str);        
	std::string temp;
	while(getline(input, temp, pattern)) {
		res.push_back(temp);
	}
	return res;
}

void Trimmer::setCurrentMap(int mapId) {
    // 1、读取mapId对应的地图gridmap；
    std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
	std::string map_topic = "fl_global_map";
	DDSPublisher publisher(map_topic, map_type);
    std::string data_type;
    std::string map_data_string;
    uint32_t map_width,map_height;
    uint32_t map_max_value;
    std::vector<int8_t> pub_data;

    std::fstream file;
    std::string filePath = path + std::to_string(mapId) + ".gridmap";
    file.open(filePath);
    std::istreambuf_iterator<char>beg(file), end;
    std::string data(beg, end);
    file.close();
	std::vector<std::string> data_split = split(data, '\n');
    if( data_split[1][0] != '#') {
        data_type = data_split[0];
        std::vector<std::string> width_heith_split = split(data_split[1], ' ');
        map_width = std::atoi(width_heith_split[0].c_str());
        map_height = std::atoi(width_heith_split[1].c_str());
        map_max_value = std::atoi(data_split[2].c_str());
        int data_split_size = data_split.size();
        for(int i=3; i< data_split_size; i++) {
            map_data_string = map_data_string + data_split[i];
            if(i<data_split_size-1) {
                map_data_string = map_data_string + '\n';
            }
        }
    } else {
        data_type = data_split[0];
        std::vector<std::string> width_heith_split = split(data_split[2], ' ');
        map_width = std::atoi(width_heith_split[0].c_str());
        map_height = std::atoi(width_heith_split[1].c_str());
        map_max_value = std::atoi(data_split[3].c_str());
        int data_split_size = data_split.size();
        for(int i=4; i< data_split_size; i++) {
            map_data_string = map_data_string + data_split[i];
            if(i<data_split_size-1) {
                map_data_string = map_data_string + '\n';
            }
        }
    }
    int32_t map_data[map_width * map_height];
    for (int32_t i = 0; i < map_width * map_height; ++i) {
        uint8_t com = map_data_string[i];
        map_data[i] = com;
        pub_data.push_back(map_data[i]);
    }

    static uint32_t count = 0;
    nav_msgs::OccupancyGrid map;
    map.header.seq = count;
    map.header.stamp.sec = 0;
    map.header.stamp.nsec = 0;
    map.header.frame_id = std::to_string(mapId);
    map.info.resolution = 0.05;
    map.info.width = map_width;
    map.info.height = map_height;
    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.data = pub_data;
    count ++;
    publisher.publishMap<nav_msgs::OccupancyGrid>(map);
    std::cout << "gridmap publisher successful;" << std::endl;
}

int Trimmer::saveMap() {
    FunctionOrder command;
    command.set_savemap().flag(true);
    app_server_command->publish<FunctionOrder>(command);
	std::cout << "\n app_command publisher " << std::endl;
    int mapid = 0;
    time_t now = time(0); 
    while (1) {
        std::cout << "fl_map_mod  while;"  << mapid << std::endl;
        if(flag_map_modify == true ) {   
            flag_map_modify = false;
            mapid = received_map_modify->mapid();
            std::cout << "fl_map_modify received successful;"  << mapid << std::endl;
            break;
        } else if(now + 2 < time(0) && flag_map_modify == false) {
	        std::cout << "fl_map_modify received fail;"  << mapid << std::endl;
            break;
        } 
    }
    std::cout << "\n app_command sleep_for " << std::endl;
    return mapid;
}

void Trimmer::removeMap(int mapid) {
    FunctionOrder command;
    command.set_removemap().flag(true);
    command.set_removemap().mapid(mapid);
    app_server_command->publish<FunctionOrder>(command);
	std::cout << "\n app_command publisher " << std::endl;
}
