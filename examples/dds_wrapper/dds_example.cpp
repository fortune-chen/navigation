#include <unistd.h>

#include "DDSExample.h"
#include "DDSExamplePubSubTypes.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "RosAdapterContentPubSubTypes.h"

#include "nav_msgs/OccupancyGrid.h"

class DDSExampleTopicListener : public TopicListener {
public:
    DDSExampleTopicListener() : TopicListener((void*)&m_example) {
    }
    void onTopicReceived(void* msg) override {
        DDSExample* example = (DDSExample*)msg;
        std::cout << "Message " << example->message() << " " << example->index() << " RECEIVED" << std::endl;
    };
    DDSExample m_example;
};

class DDSROSMAPTopicListener : public TopicListener {
public:
    DDSROSMAPTopicListener() : TopicListener((void*)&m_map) {
    }
    void onTopicReceived(void* msg) override {
        nav_msgs::OccupancyGrid* map = (nav_msgs::OccupancyGrid*)msg;
        std::cout << "Message-> info: " << "width: " << map->info.width << " height: " << map->info.height << std::endl;
        std::string data;
        data.assign(map->data.begin(), map->data.end());
        std::cout << "Message-> data size: " << data.size() << " :" << data << std::endl;
    };
    nav_msgs::OccupancyGrid m_map;
};

int main(int argc, char** argv) {
    int subOrpub = 1;
    std::string ip;
    std::shared_ptr<DDSExamplePubSubType> type(new DDSExamplePubSubType);
    std::string topic = "HelloWorldTopic";
    
    if(argc > 1) {
        if(strcmp(argv[1],"publisher") == 0) {
            subOrpub = 1;
        } else if(strcmp(argv[1],"subscriber") == 0) {
            subOrpub = 2;
        } else if (strcmp(argv[1],"publisher_tcp") == 0) {
            subOrpub = 3;
        } else if (strcmp(argv[1],"subscriber_tcp") == 0) {
            subOrpub = 4;
        } else if (strcmp(argv[1],"publisher_map") == 0) {
            subOrpub = 5;
        } else if (strcmp(argv[1],"subscriber_map") == 0) {
            subOrpub = 6;
        }
        if (argc > 2) {
            ip = argv[2];
        }
    } else {
        std::cout << "publisher OR subscriber argument needed" << std::endl;
        return 0;
    }

    switch(subOrpub) {
        case 1: {
                DDSPublisher publisher(topic, type);
                std::thread thread([&publisher]() {
                    while (1) {
                        static uint32_t count = 0; 
                        DDSExample hello;
                        hello.index(count);
                        hello.message("a a a");
                        std::cout << "index: " << count++ << std::endl;
                        publisher.publish<DDSExample>(hello);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                });
                thread.join();
                break;
            }
        case 2: {
                auto listener =  std::make_shared<DDSExampleTopicListener>();
                DDSSubscriber subscriber(topic, type, listener);
                std::thread thread([&subscriber]() {
                    while (1) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                });
                thread.join();
                break;
            }
        case 3: {
                unsigned short port = 5100;
                DDSPublisher publisher(ip, port, topic, type);
                std::thread thread([&publisher]() {
                    while (1) {
                        static uint32_t count = 0; 
                        DDSExample hello;
                        hello.index(count);
                        std::cout << "index: " << count++ << std::endl;
                        publisher.publish<DDSExample>(hello);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                });
                thread.join();
                break;
            }
        case 4: {
                if (ip.empty()) {
                    std::cout << "tcp subscriber, please input pubisher ip example as 'dds_example subscriber_tcp 10.10.60.50'\n";
                }
                unsigned short port = 5100;
                auto listener =  std::make_shared<DDSExampleTopicListener>();
                DDSSubscriber subscriber(ip, port, topic, type, listener);
                std::thread thread([&subscriber]() {
                    while (1) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                });
                thread.join();
                break;
            }
        case 5: {
                unsigned short port = 5100;
                std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
                std::string map_topic = "MAP";
                DDSPublisher publisher(ip, port, map_topic, map_type);
                std::thread thread([&publisher]() {
                    while (1) {
                        static uint32_t count = 0;
                        count ++;
                        nav_msgs::OccupancyGrid map;
                        map.header.seq = 10;
                        map.header.stamp.sec = 20;
                        map.header.stamp.nsec = 30;
                        map.header.frame_id = "aaaaaaa";
                        map.info.width = count * 2;
                        map.info.height = count;
                        std::string data = "12345678";
                        map.data.assign(data.begin(), data.end());
                        std::cout << "width: " << map.info.width << " height: " << map.info.height << std::endl;
                        publisher.publishMap<nav_msgs::OccupancyGrid>(map);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                });
                thread.join();
                break;
            }
        case 6: {
                unsigned short port = 5100;
                std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
                std::string map_topic = "MAP";
                auto listener =  std::make_shared<DDSROSMAPTopicListener>();
                DDSSubscriber subscriber(ip, port,map_topic, map_type, listener);
                std::thread thread([&subscriber]() {
                    while (1) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                });
                thread.join();
                break;
            }
    }
    pause();
}
