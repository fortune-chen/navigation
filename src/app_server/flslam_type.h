#ifndef FLSLAM_SRC_APP_SERVER_FLSLAM_TYPE_H
#define FLSLAM_SRC_APP_SERVER_FLSLAM_TYPE_H
#include <iostream>
#include <vector>
typedef struct {
    float                      x;
    float                      y;
} Point;

typedef struct {
    float                      start_x;
    float                      start_y;
    float                      end_x;
    float                      end_y;
} Wall;

typedef struct {
    float                       p1_x;
    float                       p1_y;
    float                       p2_x;
    float                       p2_y;
    float                       p3_x;
    float                       p3_y;
    float                       p4_x;
    float                       p4_y;
} Zone;

typedef struct {
    int32_t                     room_id;
    int8_t                      color;
    std::vector<Point>          positions;
} Room;


typedef struct {
	uint32_t                    mapid;
	uint32_t                    height;
	uint32_t                    width;
	std::string                 encoding;       // 8UC1(GIMP) or bgr8;
	uint8_t                     is_bigendian;
	uint32_t                    step;
	std::vector<uint8_t>        data;
} Image;


#endif  // FLSLAM_SRC_APP_SERVER_FLSLAM_TYPE_H