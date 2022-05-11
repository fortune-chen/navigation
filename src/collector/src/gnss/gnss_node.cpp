#include "gnss_node.h"


using namespace collector;

GnssNode::GnssNode(std::shared_ptr<CollectorDDSClient> dds_client, int delay /*ms*/) {
  m_ddsClient = dds_client;    
  m_read_thread = new boost::thread(boost::bind(&GnssNode::readThread, this));
}

GnssNode::~GnssNode() {
  m_read_thread->interrupt();
  m_read_thread->join();
  delete m_read_thread;
}

bool GnssNode::is_opened(void) {
  //TODO
  return true;
}

bool GnssNode::open(void) {
  //TODO
}

int GnssNode::publish_gnss_data() {
  sensor_msgs::NavSatFix data;
  //TODO
  m_ddsClient->postGnss(data);
}
void GnssNode::readThread() {
  while (true) {
    publish_gnss_data();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}