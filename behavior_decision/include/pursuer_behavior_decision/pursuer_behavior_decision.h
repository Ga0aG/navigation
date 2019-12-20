#ifndef PURSUER_DECISION_MAKING_H
#define PURSUER_DECISION_MAKING_H

#include <behavior_decision/behavior_decision_base.h>

namespace behavior_decision{
    class PursuerBehaviorDecision : public BehaviorDecision {
    public:
        PursuerBehaviorDecision();
        ~PursuerBehaviorDecision();

        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path);

        // bool selectOptimalPolicy(Policy& policy);
        };
}
#endif