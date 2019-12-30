#ifndef PURSUER_DECISION_MAKING_H
#define PURSUER_DECISION_MAKING_H

#include "behavior_decision/behavior_decision_base.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <vector>
#include <string>
#include <algorithm>
#include <pursuer_behavior_decision/KalmanFilter.h>
#include "behavior_decision/evaderState.h"

namespace behavior_decision{
    class PursuerBehaviorDecision : public BehaviorDecision {
    public:
        enum state{
            SEARCHING,
            KEEPING,
            TRACKING,
            FOLLOWING
        };
        PursuerBehaviorDecision();
        ~PursuerBehaviorDecision();

        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
        void resetFrequency(double controller_frequency);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path);

        void getState(int &state);

        void test();
        // bool selectOptimalPolicy(Policy& policy);
    private:
        void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
        
        std::string ns, ns_tf2;
        std::string fixed_frame_;
        double controller_frequency_;
        int id, pnum, state_, target_id;
        
        ros::Subscriber detection_sub;
        ros::Publisher evaderState_pub;
        tf::TransformListener* tf_;
        
        KalmanFilter* kf;
        std::map<int,Eigen::Matrix<double,4,1>> detected;
        };
}
#endif