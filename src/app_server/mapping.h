
#ifndef FLSLAM_SRC_APP_SERVER_MAPPING_H
#define FLSLAM_SRC_APP_SERVER_MAPPING_H

#include <string>
#include <map>

namespace flslam {

class Mapping {
  public:
    Mapping();

    ~Mapping();

    void start();

    void continueMapping(int id);

    void stop();

    int saveMap();

    void localization(int id);

  private:
    int getMapStorePath(std::string& path);

    std::map<int, std::string> m_gridmaps;
    std::map<int, std::string> m_pngMaps;
};

}

#endif  // FLSLAM_SRC_APP_SERVER_MAPPING_H
