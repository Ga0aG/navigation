#ifndef PURSUER_DECISION_MAKING_H
#define PURSUER_DECISION_MAKING_H

#include "behavior_decision/behavior_decision_base.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <vector>
// #include <apriltag_ros/common_functions.h>

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

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path);

        void getState(int &state);

        void test();
        // bool selectOptimalPolicy(Policy& policy);
    private:
        void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
        
        int id,pnum,state_;
        ros::Subscriber sub;
        };
}
#endif