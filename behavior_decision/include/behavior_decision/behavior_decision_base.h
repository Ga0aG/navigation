#ifndef BEHAVIOR_DECISION_H
#define BEHAVIOR_DECISION_H

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

namespace behavior_decision {
class BehaviorDecision
{
public:
    BehaviorDecision(){}
    virtual ~BehaviorDecision(){}

    virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;
    
    virtual void resetFrequency(double controller_frequency)=0;
    
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path) = 0;

    virtual void getState(int &state)=0;

    virtual void test()=0;
};
}


#endif 