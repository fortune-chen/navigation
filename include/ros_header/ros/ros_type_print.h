#ifndef NAV_DEBUG_H
#define NAV_DEBUG_H

#include <iostream>
#include <iomanip>

#define PRINT_TRACE                 printf("\n-------------------- [%s][%s][%d]\n", __FILE__, __FUNCTION__, __LINE__)
#define PRINT_TIME(VAR)             printf("\n-------------------- [%d]\n", VAR.header.stamp.toSec())

#define PRINT_POSE_STAMPED(NAME, VAR)               {\
    PRINT_TRACE;\
    std::cout << NAME << ".header.frame_id         : " << VAR.header.frame_id << std::endl;\
    std::cout << NAME << ".header.seq              : " << VAR.header.seq << std::endl;\
    std::cout << NAME << ".header.stamp.toSec()     : " << std::setprecision(10) << VAR.header.stamp.toSec() << std::endl;\
    std::cout << NAME << ".pose.position.x;        : " << VAR.pose.position.x << std::endl;\
    std::cout << NAME << ".pose.position.y;        : " << VAR.pose.position.y << std::endl;\
    std::cout << NAME << ".pose.position.z;        : " << VAR.pose.position.z << std::endl;\
    std::cout << NAME << ".pose.orientation.x;     : " << VAR.pose.orientation.x << std::endl;\
    std::cout << NAME << ".pose.orientation.y;     : " << VAR.pose.orientation.y << std::endl;\
    std::cout << NAME << ".pose.orientation.z;     : " << VAR.pose.orientation.z << std::endl;\
    std::cout << NAME << ".pose.orientation.w;     : " << VAR.pose.orientation.w << std::endl;\
}

#define PRINT_TRANSFORM_STAMPED(NAME, VAR)              {\
    PRINT_TRACE;\
    std::cout << NAME << ".header.frame_id          : " << VAR.header.frame_id << std::endl;\
    std::cout << NAME << ".header.seq               : " << VAR.header.seq << std::endl;\
    std::cout << NAME << ".header.stamp.toSec()     : " << std::setprecision(10) << VAR.header.stamp.toSec() << std::endl;\
    std::cout << NAME << ".child_frame_id           : " << VAR.child_frame_id << std::endl;\
    std::cout << NAME << ".transform.translation.x  : " << VAR.transform.translation.x << std::endl;\
    std::cout << NAME << ".transform.translation.y  : " << VAR.transform.translation.y << std::endl;\
    std::cout << NAME << ".transform.translation.z  : " << VAR.transform.translation.z << std::endl;\
    std::cout << NAME << ".transform.rotation.x     : " << VAR.transform.rotation.x << std::endl;\
    std::cout << NAME << ".transform.rotation.y     : " << VAR.transform.rotation.y << std::endl;\
    std::cout << NAME << ".transform.rotation.z     : " << VAR.transform.rotation.z << std::endl;\
    std::cout << NAME << ".transform.rotation.w     : " << VAR.transform.rotation.w << std::endl;\
}


// void printPoseStamped(const std::string& name, const geometry_msgs::PoseStamped& pose) {
//     std::cout << name << ".header.frame_id         : " << pose.header.frame_id << std::endl;
//     std::cout << name << ".header.seq              : " << pose.header.seq << std::endl;
//     std::cout << name << ".header.stamp.toSec()    : " << pose.header.stamp.toSec() << std::endl;
//     std::cout << name << ".pose.position.x;        : " << pose.pose.position.x << std::endl;
//     std::cout << name << ".pose.position.y;        : " << pose.pose.position.y << std::endl;
//     std::cout << name << ".pose.position.z;        : " << pose.pose.position.z << std::endl;
//     std::cout << name << ".pose.orientation.x;     : " << pose.pose.orientation.x << std::endl;
//     std::cout << name << ".pose.orientation.y;     : " << pose.pose.orientation.y << std::endl;
//     std::cout << name << ".pose.orientation.z;     : " << pose.pose.orientation.z << std::endl;
//     std::cout << name << ".pose.orientation.w;     : " << pose.pose.orientation.w << std::endl;
// }

// void printTransformStamped(const std::string& name, const geometry_msgs::TransformStamped& transform) {
//     std::cout << name << ".header.frame_id          : " << transform.header.frame_id << std::endl;
//     std::cout << name << ".header.frame_id          : " << transform.header.seq << std::endl;
//     std::cout << name << ".header.frame_id          : " << transform.header.stamp.toSec() << std::endl;
//     std::cout << name << ".child_frame_id           : " << transform.child_frame_id << std::endl;
//     std::cout << name << ".transform.translation.x  : " << transform.transform.translation.x << std::endl;
//     std::cout << name << ".transform.translation.y  : " << transform.transform.translation.y << std::endl;
//     std::cout << name << ".transform.translation.z  : " << transform.transform.translation.z << std::endl;
//     std::cout << name << ".transform.rotation.x     : " << transform.transform.rotation.x << std::endl;
//     std::cout << name << ".transform.rotation.y     : " << transform.transform.rotation.y << std::endl;
//     std::cout << name << ".transform.rotation.z     : " << transform.transform.rotation.z << std::endl;
//     std::cout << name << ".transform.rotation.w     : " << transform.transform.rotation.w << std::endl;
    
// }

// #define PRINT_POSE_STAMPED(NAME, VAR)              {\
//     printf("-----------------------------------------------------------------\n");\
//     PRINT_TRACE;\
//     printPoseStamped(NAME, VAR);\
//     printf("-----------------------------------------------------------------\n");\
// }

// #define PRINT_TRANSFORM_STAMPED(NAME, VAR)              {\
//     printf("-----------------------------------------------------------------\n");\
//     PRINT_TRACE;\
//     printTransformStamped(NAME, VAR);\
//     printf("-----------------------------------------------------------------\n");\
// }

#endif