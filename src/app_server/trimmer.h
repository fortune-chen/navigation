#ifndef FLSLAM_SRC_APP_SERVER_TRIMMER_H
#define FLSLAM_SRC_APP_SERVER_TRIMMER_H
#include <memory>
#include <mutex>
#include <vector>
#include <functional>
#include "flslam_type.h"

namespace flslam {

class Trimmer {
  public:
    Trimmer();

    ~Trimmer();

    void optimizeMap();

    Image splitRoom(int max_iterations, double min_critical_point_distance_factor);

    Image setVisualWall(float start_x, float start_y, float end_x, float end_y);

    Image setForbidArea(float point_x, float point_y, float width, float height);      
                    
    Image mergeRoom(int room_num1, int room_num2);

    std::vector<Room> RoomInfo();

	void setCurrentMap(int mapId);

	int saveMap();		//保存gridmap 和 pngmap;

	void removeMap(int mapid);

  };
}
#endif  // FLSLAM_SRC_APP_SERVER_TRIMMER_H

