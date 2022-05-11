#include <cmath>
#include <Eigen/Eigen>
#include "navigation.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "RosAdapterContentPubSubTypes.h"

using namespace flslam;

static std::shared_ptr<RosAdapterContentPubSubType> kContentType;
static std::shared_ptr<NavMapContentPubSubType> kHugeContentType;
static std::shared_ptr<DDSPublisher> kGoalPublisher;
static std::shared_ptr<DDSPublisher> kVelocityPublisher;
static std::shared_ptr<DDSSubscriber> kPathSubscriber;
static std::shared_ptr<DDSPublisher> kFullCoveragePublisher;

template <typename T>
class NavigationListener : public TopicListener {
public:
    NavigationListener(flslam::Navigation* nav, const std::string& topic) : TopicListener((void*)&m_data) {
        mNavigation = nav;
        mTopic = topic;
    }

    void onTopicReceived(void* msg) override {
        T* map = (T*)msg;
    };

    T m_data;
    flslam::Navigation* mNavigation;
    std::string mTopic;
};
template<>
void NavigationListener<nav_msgs::Path>::onTopicReceived(void* msg) {
    nav_msgs::Path *rosPath = (nav_msgs::Path*)msg;
    std::vector<Pose> path;
    Pose pose;
    for (const auto& p : rosPath->poses) {
        pose.x = p.pose.position.x;
        pose.y = p.pose.position.y;
        path.push_back(pose);
    }
    if (mTopic == kGlobalPlanTopicName) {
        std::lock_guard<std::mutex> lock(mNavigation->mMutex);
        for (auto listener : mNavigation->mPathListeners) {
            listener->onGlobalPathReceived(path);
        }
    }
}

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll * M_PI / 180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw * M_PI / 180, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    std::cout << "Euler2Quaternion result is:" << std::endl;
    std::cout << "x = " << q.x() << std::endl;
    std::cout << "y = " << q.y() << std::endl;
    std::cout << "z = " << q.z() << std::endl;
    std::cout << "w = " << q.w() << std::endl;
    return q;
}

Navigation::Navigation() : mLinearVelocity(0.0), mAngularVelocity(0.0) {
    kContentType = std::make_shared<RosAdapterContentPubSubType>();
    kGoalPublisher = std::make_shared<DDSPublisher>(kGoalPoseStampedTopicName, kContentType);
    kVelocityPublisher = std::make_shared<DDSPublisher>(kTeleopKeyTopicName, kContentType);
    kHugeContentType = std::make_shared<NavMapContentPubSubType>();
    kPathSubscriber = std::make_shared<DDSSubscriber>(kGlobalPlanTopicName, kHugeContentType,
                        std::make_shared<NavigationListener<nav_msgs::Path>>(this, kGlobalPlanTopicName));
    kFullCoveragePublisher = std::make_shared<DDSPublisher>(kCoveragePoseStampedTopicName, kContentType);
}

Navigation::~Navigation() {

}

void Navigation::setGoal(const Pose& pose) {
    static uint32_t seq = 0;
    auto quaternion = euler2Quaternion(0.0, 0.0, pose.orientation);
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.seq = seq++;
    goal.header.stamp.sec = 0;
    goal.header.stamp.nsec = 0;
    goal.pose.orientation.x = quaternion.x();
    goal.pose.orientation.y = quaternion.y();
    goal.pose.orientation.z = quaternion.z();
    goal.pose.orientation.w = quaternion.w();
    goal.pose.position.x = pose.x;
    goal.pose.position.y = pose.y;
    goal.pose.position.z = 0.0;

    printf("setGoal (%lf, %lf)\n", pose.x, pose.y);
    kGoalPublisher->publishRos<geometry_msgs::PoseStamped>(goal);
}

void Navigation::doFullCoveragePathPlan(const Pose& initialPoint) {
    static uint32_t seq = 0;
    auto quaternion = euler2Quaternion(0.0, 0.0, initialPoint.orientation);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.seq = seq++;
    p.header.stamp.sec = 0;
    p.header.stamp.nsec = 0;
    p.pose.orientation.x = quaternion.x();
    p.pose.orientation.y = quaternion.y();
    p.pose.orientation.z = quaternion.z();
    p.pose.orientation.w = quaternion.w();
    p.pose.position.x = initialPoint.x;
    p.pose.position.y = initialPoint.y;
    p.pose.position.z = 0.0;

    printf("doFullCoverage (%lf, %lf)\n", initialPoint.x, initialPoint.y);
    kFullCoveragePublisher->publishRos<geometry_msgs::PoseStamped>(p);
}

void Navigation::controlMotion(Direction direction) {
    // Different models have different parameters
    // Burger : linear velocity (-0.22 ~ 0.22), angular velocity (-2.84 ~ 2.84)
    double linearStep = 0.01;
    double angularStep = 0.1;
    double linearMax = 0.22;
    double angularMax = 2.8;

    geometry_msgs::Twist cmd_vel;
    {
        std::lock_guard<std::mutex> lock(mVelocityMutex);
        switch (direction) {
            case kUp:
                if (mLinearVelocity < linearMax) {
                    mLinearVelocity += linearStep;
                }
                break;

            case kDown:
                if (mLinearVelocity > -linearMax) {
                    mLinearVelocity -= linearStep;
                }
                break;

            case kLeft:
                if (mAngularVelocity < angularMax) {
                    mAngularVelocity += angularStep;
                }
                break;

            case kRight:
                if (mAngularVelocity > -angularMax) {
                    mAngularVelocity -= angularStep;
                }
                break;

            case kStop:
                mLinearVelocity = 0.0;
                mAngularVelocity = 0.0;
                break;

            default:
                break;
        }

        cmd_vel.linear.x = mLinearVelocity;
        cmd_vel.angular.z = mAngularVelocity;
    }
    printf("currently:	linear vel %lf	 angular vel %lf\n", cmd_vel.linear.x, cmd_vel.angular.z);
    kVelocityPublisher->publishRos<geometry_msgs::Twist>(cmd_vel);
}

void Navigation::registerPathListener(const std::shared_ptr<PathListener>& listener) {
    std::lock_guard<std::mutex> lock(mMutex);
    mPathListeners.push_back(listener);
}

void Navigation::unregisterPathListener(const std::shared_ptr<PathListener>& listener) {
    std::lock_guard<std::mutex> lock(mMutex);
    for (auto iter = mPathListeners.begin(); iter != mPathListeners.end();) {
        if (listener == *iter) {
            iter = mPathListeners.erase(iter);
        } else {
            ++iter;
        }
    }
}