#include "imu_node.h"


using namespace collector;

ImuNode::ImuNode(std::shared_ptr<CollectorDDSClient> dds_client, int delay /*ms*/) {
  m_ddsClient = dds_client;    
  m_read_thread = new boost::thread(boost::bind(&ImuNode::readThread, this));
}

ImuNode::~ImuNode() {
  m_read_thread->interrupt();
  m_read_thread->join();
  delete m_read_thread;
  m_imu.close_device();
}

bool ImuNode::is_opened(void) {
  return (m_imu.fd_ >= 0);
}

bool ImuNode::open(void) {
  if (m_imu.open_device(m_device) < 0) {
    std::cout << "Failed to open device: " << m_device.c_str() << std::endl;
  }
  // Wait 10ms for SPI ready
  usleep(10000);
  unsigned char pid = 0;
  m_imu.get_product_id(pid);
  std::cout << "Product ID: " << pid << std::endl;
}

int ImuNode::publish_imu_data() {
  sensor_msgs::Imu data;
  data.header.frame_id = m_frame_id;
  data.header.stamp = ros::Time::now();

  // Linear acceleration
  data.linear_acceleration.x = m_imu.accl[0];
  data.linear_acceleration.y = m_imu.accl[1];
  data.linear_acceleration.z = m_imu.accl[2];

  // Orientation (quarternion)
  data.orientation.x = 0;
  data.orientation.y = 0;
  data.orientation.z = 0;
  data.orientation.w = 0;

  m_ddsClient->postImu(data);
}
void ImuNode::readThread() {
  while (true) {
    if (m_imu.update() == 0) {
      publish_imu_data();
    } else {
      std::cout << "Cannot update\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}