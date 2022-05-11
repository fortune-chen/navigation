#include "map_mgr.h"
#include "trimmer_dds_client.h"

int main(int argc, char** argv) {
    std::string local_ip;
    std::string ros_bridge_ip;

    if (argc > 2) {
        local_ip  = argv[1];
        ros_bridge_ip = argv[2];
    }                
    std::shared_ptr<TrimmerDdsClient> ddsClient =  std::make_shared<TrimmerDdsClient>(local_ip, ros_bridge_ip);  // 
    MapMgr mapMgr(ddsClient);
    mapMgr.registerHandler();
    pause();
}
