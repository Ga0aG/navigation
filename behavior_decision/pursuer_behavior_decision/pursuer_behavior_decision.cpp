#include "pursuer_behavior_decision/pursuer_behavior_decision.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(behavior_decision::PursuerBehaviorDecision, behavior_decision::BehaviorDecision)

using namespace behavior_decision;

PursuerBehaviorDecision::PursuerBehaviorDecision()
{

}

PursuerBehaviorDecision::~PursuerBehaviorDecision()
{

}
void PursuerBehaviorDecision::initialize(std::string name, tf::TransformListener* tf){
    tf::TransformListener* m_p_tf;
    m_p_tf = tf;
    ROS_INFO("###############################costmap2");
}
bool PursuerBehaviorDecision::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path){
    ROS_INFO("###############################setPlan");
}
void PursuerBehaviorDecision::test(){
    ROS_INFO("==================================Working");
    ROS_INFO("==================================Working");
    ROS_INFO("==================================Working");
    ROS_INFO("==================================Working");
    ROS_INFO("==================================Working");
}