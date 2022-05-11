

#ifndef ROS_LOG_TRANSFER_H
#define ROS_LOG_TRANSFER_H

#include <stdio.h>
#include <assert.h>

#define ROS_DEBUG       //printf

#define ROS_INFO(...)        fprintf(stdout, __VA_ARGS__);\
                             printf("\n");
#define ROS_WARN(...)        fprintf(stdout, __VA_ARGS__);\
                             printf("\n");
#define ROS_ERROR(...)       fprintf(stdout, __VA_ARGS__);\
                             printf("\n");
#define ROS_FATAL(...)       fprintf(stdout, __VA_ARGS__);\
                             printf("\n");

#define ROS_ASSERT(cond) \
  do { \
    if (!(cond)) { \
      ROS_FATAL("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
      assert(cond); \
    } \
  } while (false)

#endif
