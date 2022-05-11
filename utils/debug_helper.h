#ifndef DEBUG_HELPER_H
#define DEBUG_HELPER_H

#include <mutex>
#include <string>

namespace utils {

class DebugHelper {
 public:

  static DebugHelper* getInstance();

  /** 
   * @brief Used to print the backtrace directly when an exception occurs in a program
   */
  void init();

  /** 
   * @brief Used to generate backtrace files when an exception occurs in a program
   * @param name The name for this backtrace files and path
   */
  void init(const std::string& name);

 private:
  
  DebugHelper();

  ~DebugHelper();

 private:
  static DebugHelper *mInstance;
  static std::mutex mInstanceMutex;
  bool mInited;

};
  
}

#endif