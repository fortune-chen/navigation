// ############################################
// ## map_publisher.cpp
// #############################################
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/highgui.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>
#include "DDSExample.h"
#include "DDSExamplePubSubTypes.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "RosAdapterContentPubSubTypes.h"
#include "nav_msgs/OccupancyGrid.h"

#include "RoomPublish.h"
#include "RoomPublishPubSubTypes.h"

#include <sensor_msgs/Image.h>

std::string data_type;
std::string map_data_string;            //数据格式：
bool flag = false;
static nav_msgs::OccupancyGrid* publisher_map;
sensor_msgs::Image* server_map;

uint32_t map_width,map_height;   		//图片宽高
uint32_t map_max_value; 				// 图片数据中存在的最大值；
std::vector<int8_t> pub_data;
char file_buffer;			            //读取文件数据；
long file_size;
void fl_deal_map();

//***********************Split binary string********************
std::vector<std::string> split(const std::string &str, const char pattern)
{
	std::vector<std::string> res;
	std::stringstream input(str);        //读取str到字符串流中
	std::string temp;
	//使用getline函数从字符串流中读取,遇到分隔符时停止,和从cin中读取类似
	//注意,getline默认是可以读取空格的
	while(getline(input, temp, pattern))
	{
		res.push_back(temp);
	}
	return res;
}

class RoomPublishTopicListener : public TopicListener {
public:
    RoomPublishTopicListener() : TopicListener((void*)&m_map) {
    }
    void onTopicReceived(void* msg) override {
        RoomPublish* map = (RoomPublish*)msg;
        std::cout << "room_num = " << m_map.room_num() << std::endl;
        for(int i=0; i<m_map.room_num(); i++)
        {
            std::cout << "room_id[" << i << "] = " << m_map.room_info().at(i).room_id() << std::endl;
            std::cout << "room_centre = (" << m_map.room_info().at(i).room_centre().x() << " , "  << m_map.room_info().at(i).room_centre().y() << ")" <<std::endl;
            std::cout << "room_area = " << m_map.room_info().at(i).room_area() <<std::endl;
            std::cout << "clean_num = " << m_map.room_info().at(i).clean_num() <<std::endl;
            std::cout << "forbid = " << m_map.room_info().at(i).forbid() <<std::endl;
            std::cout << "Contour room_points = ";
            for(int j=0; j<32; j++)
            {
                std::cout << " [" << m_map.room_info().at(i).room_points().contour_point().at(j).x() <<" , " << m_map.room_info().at(i).room_points().contour_point().at(j).y() <<"] ";
            }
            std::cout <<std::endl;
        }
    };
    RoomPublish m_map;
};

//***********************subscribe the nav_msgs::OccupancyGrid message ********************
class MapUpdateListener : public TopicListener {
public:
    MapUpdateListener() : TopicListener((void*)&m_data) {
    }
    void onTopicReceived(void* msg) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        publisher_map = (nav_msgs::OccupancyGrid*)msg;
        std::cout << "onTopicRece ived" << std::endl;
        flag = true;
    };
    nav_msgs::OccupancyGrid m_data;
};

class MapServerListener : public TopicListener {
public:
    MapServerListener() : TopicListener((void*)&m_data) {
    }
    void onTopicReceived(void* msg) override {
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        server_map = (sensor_msgs::Image*)msg;
        std::cout << "server_map->rows = " << server_map->height << std::endl;
        std::cout << "server_map->cols = " << server_map->width << std::endl;
        std::cout << "server_map->encoding = " << server_map->encoding << std::endl;
        std::cout << "server_map->is_bigendian = " << (int)server_map->is_bigendian << std::endl;
        std::cout << "server_map->step = " << server_map->step << std::endl;

        if("8UC1" == server_map->encoding) {
            cv::Mat img(server_map->height, server_map->width, CV_8UC1, cv::Scalar(255));
            for (int row = 0; row <  server_map->height * server_map->step; row++) {
                uint8_t com = server_map->data[row];
                img.data[row] = com;
            }
            cv::imshow("pngfile", img);
        } else if("bgr8" == server_map->encoding) {
            cv::Mat img(server_map->height, server_map->width, CV_8UC3, cv::Scalar(255, 255, 255));
            for (int row = 0; row <  server_map->height * server_map->step; row++) {
                uint8_t com = server_map->data[row];
                img.data[row] = com;
            }
            cv::imshow("pngfile", img);
        }
        cv::waitKey(0);

        flag = true;
    };
    sensor_msgs::Image m_data;
};


int main(int argc, char** argv) 
{
	int subOrpub = 1;
    if(argc > 1) 
	{
        if(strcmp(argv[1],"publisher") == 0) {
            subOrpub = 1;
        } else if(strcmp(argv[1],"subscriber_room_info") == 0) {
            subOrpub = 2;
        } else if(strcmp(argv[1],"subscriber_map") == 0) {
            subOrpub = 3;
        } else if(strcmp(argv[1],"publisher_command") == 0) {
            subOrpub = 4;
        } else if(strcmp(argv[1],"subscriber_server_map") == 0) {
            subOrpub = 5;
        } 
    } 
	else 
	{
        std::cout << "publisher OR subscriber argument needed" << std::endl;
        return 0;
    }

    switch(subOrpub)
	{
        case 1: 
		{
			//***********************publish the nav_msgs::OccupancyGrid message********************
			std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
			std::string map_topic = "fl_global_map";
			DDSPublisher publisher(map_topic, map_type);
			std::cout << "publisher OccupancyGrid map :" << std::endl;
			std::fstream file;
			file.open("./robot.pgm");//,std::ios::binary);
			std::istreambuf_iterator<char>beg(file), end;           //一次读取文件的wf全部内容
			std::string data(beg, end);
			file.close();
			std::cout << "data.size() = " << data.size() << std::endl; 
			//C++使用strtok函数实现字符串分割：char * strtok (char *str, char * delim);
			std::vector<std::string> data_split = split(data, '\n');
			//***********************Determine whether the picture data has an interpretation statement********************
			//判断图片数据是否有解释语句
			if( data_split[1][0] != '#')
			{
				std::cout << "No Eixst explanation statement" << std::endl;
				//数据格式：
				data_type = data_split[0];
				std::cout << "data_type = " << data_type << std::endl;
				//图片宽高
				std::vector<std::string> width_heith_split = split(data_split[1], ' ');
				map_width = std::atoi(width_heith_split[0].c_str());
				map_height = std::atoi(width_heith_split[1].c_str());
				std::cout << "map_width = " << map_width << std::endl; 
				std::cout << "map_height = " << map_height << std::endl; 
				// 图片数据中存在的最大值；
				map_max_value = std::atoi(data_split[2].c_str());
				std::cout << "map_max_value = " << map_max_value << std::endl; 
				//图片数据处理；
				int data_split_size = data_split.size();
				for(int i=3; i< data_split_size; i++)
				{
					map_data_string = map_data_string + data_split[i];
					if(i<data_split_size-1)
					{
					map_data_string = map_data_string + '\n';
					}
				}
			}
			else
			{
				std::cout << "Eixst explanation statement" << std::endl;
				//数据格式：
				data_type = data_split[0];
				std::cout << "data_type = " << data_type << std::endl;
				// 解释数据：
				std::cout << "map explanation date = " << data_split[1] << std::endl;
				//图片宽高
				std::vector<std::string> width_heith_split = split(data_split[2], ' ');
				map_width = std::atoi(width_heith_split[0].c_str());
				map_height = std::atoi(width_heith_split[1].c_str());
				// map_height = std::atoi((char *)width_heith_split[1]);
				std::cout << "map_width = " << map_width << std::endl; 
				std::cout << "map_height = " << map_height << std::endl; 
				// 图片数据中存在的最大值；
				map_max_value = std::atoi(data_split[3].c_str());
				std::cout << "map_max_value = " << map_max_value << std::endl; 
				//图片数据处理；
				int data_split_size = data_split.size();
				std::cout << "data_split_size = " << data_split_size << std::endl; 
				for(int i=4; i< data_split_size; i++)
				{
					map_data_string = map_data_string + data_split[i];
					if(i<data_split_size-1)
					{
						map_data_string = map_data_string + '\n';
					}
				}
				std::cout << "map_data[4] = " << map_data_string.size() << std::endl;
			}

			//***********************map_data********************
			//图片数据处理；
			int32_t map_data[map_width * map_height];
			for (int32_t i = 0; i < map_width * map_height; ++i)
			{
				uint8_t com = map_data_string[i];
				map_data[i] = com;
				pub_data.push_back(map_data[i]);

				if(i< 172600 && i> 172500)
					printf("map_data[] = %d  \n",com);
			}

			std::thread thread([&publisher]() 
			{
				while (1) 
				{
					static uint32_t count = 0;
					count ++;
					nav_msgs::OccupancyGrid map;
					map.header.seq = count;
					map.header.stamp.sec = 20;
					map.header.stamp.nsec = 30;
					map.header.frame_id = "map";
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
					//map.data.assign(pub_data.begin(), pub_data.end());
                    publisher.publishMap<nav_msgs::OccupancyGrid>(map);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    std::cout << "\n map.data publisher " << std::endl;
				}
			});
			thread.join();    
        }
        case 2: 
		{
			//***********************subscriber the nav_msgs::OccupancyGrid message********************
			std::shared_ptr<RoomPublishPubSubType> types(new RoomPublishPubSubType);
			auto listener =  std::make_shared<RoomPublishTopicListener>();
			DDSSubscriber subscriber("fl_room_info", types, listener);
           	std::cout << "publisher OR subscriber argument needed" << std::endl;
			std::thread thread_sub([&subscriber]() 
			{
				while (1) 
				{
					//std::cout << "subscriber map_successful; :" << std::endl;
				}
			});
			thread_sub.join(); 
        }
		case 3: 
		{
            std::shared_ptr<NavMapContentPubSubType> ros_type(new NavMapContentPubSubType);
            std::string map_topic = "fl_optimized_map";
            auto listener =  std::make_shared<MapUpdateListener>();
            DDSSubscriber subscriber_map(map_topic, ros_type, listener);
			std::thread thread([&subscriber_map]()
			{
				while (1)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if(flag == true)
					{
                        std::cout << "flag = " << flag << std::endl;
						fl_deal_map();
                        flag = false;
                    }
                }
            });
            thread.join();  
        }
        case 4: 
		{
			//***********************subscriber the nav_msgs::OccupancyGrid message********************
			//std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
			std::shared_ptr<FunctionOrderPubSubType> map_type(new FunctionOrderPubSubType); 

			DDSPublisher publisher("fl_app_server_command", map_type);	
            
			std::thread thread([&publisher]() 
			{
                FunctionOrder app_command;
                while (1)
                {   
                    app_command.set_optimize().flag(true);
                    
                    app_command.set_split().flag(true);

                    app_command.set_wall().flag(true);
                    app_command.set_wall().point().at(0).x(0);
                    app_command.set_wall().point().at(0).y(0);
                    app_command.set_wall().point().at(1).x(10);
                    app_command.set_wall().point().at(1).y(10);

                    app_command.set_area().flag(true);

                    app_command.set_merge().flag(true);

                    app_command.set_info().flag(true);
                    //m_roomPublisher->publish<RoomPublish>(room_information);
                    publisher.publish<FunctionOrder>(app_command);     //自定义数据格式的发送失败；
                    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					std::cout << "\n app_command publisher " << std::endl;
                }
			});
			thread.join();   
        }
        case 5: 
		{
            std::shared_ptr<NavMapContentPubSubType> ros_type(new NavMapContentPubSubType);
            std::string map_topic = "fl_app_server_map";
            auto listener =  std::make_shared<MapServerListener>();
            DDSSubscriber subscriber_server_map(map_topic, ros_type, listener);
			std::thread thread([&subscriber_server_map]()
			{ 
				while (1)
				{

                }
            });
            thread.join();  
        }
    }
	pause();
}

//***********************subscribe map and  Optimize the map********************
void fl_deal_map()
{
    std::cout << "trimmer_node_successful; " << std::endl;
    //地图数据订阅成功 -- 把地图数据写成一个图片.pgm
    std::FILE *pgm_file;
    pgm_file = std::fopen("./fl_deal_map.pgm","wb");
    fprintf(pgm_file,"P5\n");
    fprintf(pgm_file,"%d %d\n",publisher_map->info.width,publisher_map->info.height);
    fprintf(pgm_file,"%d\n",255);
    std::vector<int8_t> receive = publisher_map->data;
    int32_t map_data[publisher_map->info.width * publisher_map->info.height];
    // 原始地图用于添加虚拟墙，禁区的图片；
    for (int32_t i = 0; i <publisher_map->info.width * publisher_map->info.height; ++i)
    {
        uint8_t com = receive[i];
        fputc(com, pgm_file);
    }
    fclose(pgm_file);
    cv::Mat row_map = cv::imread("./fl_deal_map.pgm",1);
    cv::imshow("fl_deal_map", row_map);
	cv::waitKey(0);
}

// 将png图片的数据提取出来使用MapServer的数据格式发送；
// MapServer PngMap;
// MapColor  PngPoint;
// PngMap.channels() = color_segmented_map.channels();
// PngMap.rows() = color_segmented_map.rows;
// PngMap.cols() = color_segmented_map.cols;
// for(int i=0; i< color_segmented_map.rows; i++)
// {
//     for(int j=0; j< color_segmented_map.cols; j++)
//     {
//         PngPoint.color_b() = room_Merge.at<cv::Vec3b>(i, j)[0];
//         PngPoint.color_g() = room_Merge.at<cv::Vec3b>(i, j)[1];
//         PngPoint.color_r() = room_Merge.at<cv::Vec3b>(i, j)[2];
//         PngMap.point_value().emplace_back(PngPoint);
//     }
// }


   // // sensor_msgs::ImageMat_Image(cv::Mat image);

    // sensor_msgs::Image Png_image;
    // {
    //     Png_image.width = color_segmented_map.cols;
    //     Png_image.height = color_segmented_map.rows;
    //     Png_image.step = color_segmented_map.channels() * color_segmented_map.cols;
    //     for (int i = 0; i < Png_image.step * Png_image.height; i++)
    //     {
    //         Png_image.data.push_back(color_segmented_map.data[i] );
    //     }
    // }
    // std::cout << "color_segmented_map.data.size() = " << color_segmented_map.data().size() << std::endl;