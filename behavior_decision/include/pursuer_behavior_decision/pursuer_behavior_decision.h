#ifndef PURSUER_DECISION_MAKING_H
#define PURSUER_DECISION_MAKING_H

#include <mutex>
#include <vector>
#include <string>
#include <algorithm>
#include "behavior_decision/behavior_decision_base.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
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
        void evaderStateCallback(const behavior_decision::evaderState::ConstPtr& msg, int id_);
        void targetAssignment();
        //TODO : Lost target recovery;

        std::string ns, ns_tf2, gns;
        std::string fixed_frame_;
        int id, pnum, state_, target_id;
        double controller_frequency_;
        double lost_time_threshold_; // Check for targetState's availity
        tf::TransformListener* tf_;
        
        ros::Subscriber detection_sub;
        ros::Publisher evaderState_pub;
        std::vector<ros::Subscriber> evaderState_subs; // Subscribe evaderState from other pursuers;
        std::recursive_mutex mutex_tags, mutex_evaderStates;
        apriltag_ros::AprilTagDetectionArray tags;
        std::map<int,behavior_decision::evaderState> detected;
        
        KalmanFilter* kf;//### why it has to be pointer, or --> undefined symbol
        };
}
#endif