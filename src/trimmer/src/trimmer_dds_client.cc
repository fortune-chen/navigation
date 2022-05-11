#include "trimmer_dds_client.h"
#include <iostream>
#include <mutex>
#include <fstream>

bool flag = false;
nav_msgs::OccupancyGrid deal_map;

class MapConfigCommandListener : public TopicListener {
public:
    MapConfigCommandListener(TrimmerDdsClient* client, const std::string& topic) : TopicListener((void*)& app_data) {
        app_client = client;
        app_topic = topic;
    }
    void onTopicReceived(void* msg) override {
        FunctionOrder* command = (FunctionOrder*)msg;
        app_client->m_mapConfigureHandler(command);
    };
    FunctionOrder app_data;
    TrimmerDdsClient* app_client;
    std::string app_topic;
};

class MapUpdateListener : public TopicListener {
public:
    MapUpdateListener(TrimmerDdsClient* client, const std::string& topic) : TopicListener((void*)&m_data) {
      m_client = client;
      m_topic = topic;
    }
    void onTopicReceived(void* msg) override {
        nav_msgs::OccupancyGrid* map = (nav_msgs::OccupancyGrid*)msg;
        m_client->m_gridMapUpdateHandler(map);
    };
    nav_msgs::OccupancyGrid m_data;
    TrimmerDdsClient* m_client;
    std::string m_topic;
};

TrimmerDdsClient::TrimmerDdsClient(const std::string& local_ip, const std::string& bridge_ip) {
    std::shared_ptr<RosAdapterContentPubSubType> ros_type(new RosAdapterContentPubSubType);
    std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
    std::shared_ptr<RoomPublishPubSubType> Room_type(new RoomPublishPubSubType);
    std::shared_ptr<MapModifyPubSubType> Room_type1(new MapModifyPubSubType);
    std::shared_ptr<MessagePubSubType> type(new MessagePubSubType);
    std::shared_ptr<FunctionOrderPubSubType> command_type(new FunctionOrderPubSubType); 

    if (bridge_ip.empty()) {
        // kGlobalMapTopicName = "fl_global_map";                          
        m_subscribers.push_back(std::make_shared<DDSSubscriber>(kGlobalMapTopicName, map_type, std::make_shared<MapUpdateListener>(this, kGlobalMapTopicName)));

        std::string kAppServerCommandTopicName = "fl_app_server_command";            
        m_subscriber_command.push_back(std::make_shared<DDSSubscriber>(kAppServerCommandTopicName, command_type, std::make_shared<MapConfigCommandListener>(this, kAppServerCommandTopicName)));

        std::string kOptimizedMapTopicName = "fl_optimized_map";                                     
        m_gridMapPublisher = std::make_shared<DDSPublisher>(kOptimizedMapTopicName, map_type);

        std::string kRoomInfoTopicName = "fl_room_info";                                          
        m_roomPublisher = std::make_shared<DDSPublisher>(kRoomInfoTopicName, Room_type); 
        
        std::string kMapModifyTopicName = "fl_map_modify";                                          
        m_mapModifyPublisher = std::make_shared<DDSPublisher>(kMapModifyTopicName, Room_type1);
        
        std::string kAppServerMapTopicName = "fl_app_server_map";                                         
        m_pngMapPublisher = std::make_shared<DDSPublisher>(kAppServerMapTopicName, map_type);
    } else {
        // TCP implement create two subscribers, mapUpdate and mapConfig
        // crreate three publishers, m_pngMapPublisher,m_gridMapPublisher,m_roomPublisher 
    }
}

TrimmerDdsClient::~TrimmerDdsClient() {}
     
void TrimmerDdsClient::pubGridMap(const nav_msgs::OccupancyGrid grid_map) {
    m_gridMapPublisher->publishMap<nav_msgs::OccupancyGrid>(grid_map);
    std::cout << "gridmap publisher;" << std::endl;
}

void TrimmerDdsClient::pubPngMap(sensor_msgs::Image Png_image) {
    m_pngMapPublisher->publishMap<sensor_msgs::Image>(Png_image);
    std::cout << "pngmap publisher"  << std::endl;	
}

void TrimmerDdsClient::pubRoomInfo(RoomPublish room_information) {
    m_roomPublisher->publish<RoomPublish>(room_information);
    std::cout << "RoomInfo publisher"  << std::endl;	
}

void TrimmerDdsClient::pubMapModify(MapModify map_modify) {
    m_mapModifyPublisher->publish<MapModify>(map_modify);
    std::cout << "map_modify publisher"  << map_modify.mapid() << std::endl;	
}

void TrimmerDdsClient::registerGridMapUpdateHandler(const GridMapUpdateHandler& cb) {
  m_gridMapUpdateHandler = cb;
}

void TrimmerDdsClient::registerMapConfigureHandler(const MapConfigureHandler& cb) {
    m_mapConfigureHandler = cb;
}
