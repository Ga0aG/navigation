#ifndef PURSUER_DECISION_MAKING_H
#define PURSUER_DECISION_MAKING_H

#include "behavior_decision/behavior_decision_base.h"
#include <vector>

namespace behavior_decision{
    class Pursuer{
        Pursuer();
        ~Pursuer();
    };
    class PursuerBehaviorDecision : public BehaviorDecision {
    public:
        PursuerBehaviorDecision();
        ~PursuerBehaviorDecision();

        void initialize(std::string name, tf::TransformListener* tf);

        // BUG
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path);

        void test();
        // bool selectOptimalPolicy(Policy& policy);
        };
}
#endif