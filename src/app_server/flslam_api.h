
#ifndef FLSLAM_SRC_APP_SERVER_FLSLAM_API_H
#define FLSLAM_SRC_APP_SERVER_FLSLAM_API_H

#include <vector>
#include <string>
#include "flslam_type.h"

namespace flslam {

class FlslamApi {
  public:
    static FlslamApi& getInstance() {
      static FlslamApi m_flslamApi;
      return m_flslamApi;
    }

    bool startWallFollow();

    bool stopWallFollow();

    bool continueMapping(int mapId);

    int saveMap();              

    bool removeMap(int mapId);

    bool setCurrentMap(int mapId);

    Image setVisualWall(Wall& wall);

    Image setForbidArea(Zone& zone);

    Image splitRoom();

    std::vector<Room> getRoomInfo();
    
    Image mergeRoom(int roomid1, int roomid2);

    bool fullClean(Point startPoint);

    bool areaClean(Zone& zone);

  private:
    FlslamApi();

    FlslamApi(const FlslamApi&) = delete;

    FlslamApi& operator=(const FlslamApi&) = delete;
};
}

#endif  // FLSLAM_SRC_APP_SERVER_FLSLAM_API_H
