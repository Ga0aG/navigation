#ifndef CCBSO_LOCAL_PLANNER_H_
#define CCBSO_LOCAL_PLANNER_H_

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include "ccbso_config.h"
#include <dynamic_reconfigure/server.h>

namespace ccbso_local_planner{
  class CCBSOPlanner : public nav_core::BaseLocalPlanner
  {
  public:
    enum state{
          SEARCHING,
          KEEPING,
          TRACKING,
          FOLLOWING
    };
    CCBSOPlanner();
    ~CCBSOPlanner();
    bool computeVelocityCommands(geometry_msgs::Twist&);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool isGoalReached();
    bool setState(int &state);
  
  private:
    int state_;
    int id, pnum;

    double max_vel_x;
    double max_rot_vel;
    double acc_lim_x;
    double acc_lim_theta;

    boost::shared_ptr<dynamic_reconfigure::Server<CCBSOPlannerConfig>> dsrv_;


    void reconfigureCB(CCBSOPlannerConfig &config, uint32_t level);

    std::string states[4] = {"SEARCHING", "KEEPING", "TRACKING", "FOLLOWING"};
  };
};
#endif
