// #include <string>
// #include <unistd.h>

// #include "tf2_ros/buffer.h"
// #include "tf2_bridge/tf2_bridge_client.h"
// #include "tf2_ros/tf2_bridge_server.h"


// static const std::string HELP_MESSAGE =
//     "+----------------------------------------------------------------------------+\n"
//     "|                                  Options:                                  |\n"
//     "|       Press '1' to start tf2 bridge client                                 |\n"
//     "|       Press '2' to start tf2 bridge server                                 |\n"
//     "+----------------------------------------------------------------------------+\n";

// int main(int argc, char** argv) {
//     std::string cmd;
//     while (true) {
//         std::cout << HELP_MESSAGE;
//         std::cin >> cmd;
//         if (cmd == "1") {
//             // tf2_ros::TF2BridgeClient tf2Client;
//             std::string name;
//             std::string country;
//             std::cout << "Please input your name :\n";
//             std::cin >> name;
//             std::cout << "Please input your country :\n";
//             std::cin >> country;

//             std::thread([&]() {
//                 TF2Request req;
//                 int count = 0;
//                 while (1) {
//                     count++;
//                     if (count > 10000) {
//                         count = 1;
//                     }
//                     req.request_id(std::to_string(count));
//                     req.message(name);
//                     auto ret = tf2_ros::TF2BridgeClient::getInstance()->TF2Call(req);
//                     printf("[TF2Response][%s] %s\n", ret.response_id().c_str(), ret.message().c_str());
//                     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//                 }
//             }).detach();

//             std::thread([&]() {
//                 TF2Request req;
//                 int count = 10000;
//                 while (1) {
//                     count++;
//                     if (count > 20000) {
//                         count = 10001;
//                     }
//                     req.request_id(std::to_string(count));
//                     req.message(country);
//                     auto ret = tf2_ros::TF2BridgeClient::getInstance()->TF2Call(req);
//                     printf("[TF2Response][%s] %s\n", ret.response_id().c_str(), ret.message().c_str());
//                     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//                 }
//             }).join();
//         } else if (cmd == "2") {
//             tf2_ros::TF2BridgeServer tf2Server(tf2_ros::Buffer::getInstance());
//             pause();
//         } else {
//             std::cout << "Input error!!!\n";
//         }
//     }
//     pause();
//     return 0;
// }
