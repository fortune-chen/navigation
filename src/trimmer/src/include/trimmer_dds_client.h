#ifndef FLSLAM_SRC_TRIMMER_SRC_TRIMMER_DDS_CLIENT_H
#define FLSLAM_SRC_TRIMMER_SRC_TRIMMER_DDS_CLIENT_H

#include <vector>
#include <memory>
#include <functional>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/highgui.h>

#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "nav_msgs/OccupancyGrid.h"

#include "DDSExample.h"
#include "DDSExamplePubSubTypes.h"
#include "RosAdapterContent.h"
#include "RosAdapterContentPubSubTypes.h"
#include "RoomPublish.h"
#include "RoomPublishPubSubTypes.h"
#include <ipa_room_segmentation/voronoi_segmentation.h>

using GridMapUpdateHandler = std::function<void(const nav_msgs::OccupancyGrid* map)>;
using MapConfigureHandler = std::function<void(const FunctionOrder* command)>;

class TrimmerDdsClient {
 public:

    TrimmerDdsClient(const std::string& local_ip, const std::string& bridge_ip);

    ~TrimmerDdsClient();

    TrimmerDdsClient(const TrimmerDdsClient&) = delete;

    TrimmerDdsClient& operator=(const TrimmerDdsClient&) = delete;

    void pubGridMap(const nav_msgs::OccupancyGrid map);

    void pubPngMap(sensor_msgs::Image Png_image);

    void pubRoomInfo(RoomPublish room_information);

    void pubMapModify(MapModify map_modify);

    void registerGridMapUpdateHandler(const GridMapUpdateHandler& cb);

    void registerMapConfigureHandler(const MapConfigureHandler& cb);

    GridMapUpdateHandler m_gridMapUpdateHandler;

    MapConfigureHandler m_mapConfigureHandler;

private:
    std::vector<std::shared_ptr<DDSSubscriber>> m_subscribers;

    std::vector<std::shared_ptr<DDSSubscriber>> m_subscriber_command;

    std::shared_ptr<DDSPublisher> m_pngMapPublisher;

    std::shared_ptr<DDSPublisher> m_gridMapPublisher;

    std::shared_ptr<DDSPublisher> m_roomPublisher;

    std::shared_ptr<DDSPublisher> m_mapModifyPublisher;
};
#endif  // FLSLAM_SRC_TRIMMER_SRC_TRIMMER_DDS_CLIENT_H
