
#include "memory"

#include "collector_dds_client.h"
#include "imu/imu_node.h"

#define IMU_RATE_HZ                1000
#define LASER_RATE_HZ              10
#define ODOM_RATE_HZ               10
#define GNSS_RATE_HZ               10
#define MILLISECONDS_PER_SECOND    1000

using namespace collector;

int main(int argc, char** argv) {

    std::shared_ptr<CollectorDDSClient> ddsClient =  std::make_shared<CollectorDDSClient>();
    auto imuNode = std::make_shared<ImuNode>(ddsClient, (MILLISECONDS_PER_SECOND / IMU_RATE_HZ));

    pause();
}