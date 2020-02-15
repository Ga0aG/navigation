#ifndef CCBSO_LOCAL_PLANNER_H_
#define CCBSO_LOCAL_PLANNER_H_

#include <set>
#include <map>
#include <mutex>
#include <string>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <ccbso_local_planner/check_fitness.h>

#include "ccbso_config.h"
#include <dynamic_reconfigure/server.h>
#include <behavior_decision/evaderState.h>

const float fmaxi = 1000.0;
//const float pheromone = 10;
const float PI = 3.14159265;

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
    void resetFrequency(double frequency);
  
  private:
    int state_;
    int id, pnum;
    double robot_radius;
    double controller_frequency_;
    bool received_scan;

    std::string ns, ns_tf2, gns, tf_prefix, fixed_frame_;

    tf::TransformListener* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    boost::shared_ptr<dynamic_reconfigure::Server<CCBSOPlannerConfig>> dsrv_;
    std::string states[4] = {"SEARCHING", "KEEPING", "TRACKING", "FOLLOWING"};

    ros::Subscriber sub_laser;
    ros::Subscriber sub_path;
    ros::Publisher pub_pheromoneTrail; 
    ros::Publisher pub_pheromoneTrail_;
    ros::Publisher pub_lookAheadPoint;
    ros::Publisher pub_checkPoint;
    ros::Publisher pub_state;
    ros::Publisher pub_evaderState;
    std::vector<ros::Subscriber> sub_trails;
    std::vector<ros::Subscriber> sub_evaderStates;
    ros::ServiceServer srv_check_fitness;

    std::recursive_mutex mutex_scan, mutex_trails, mutex_pursuers, mutex_path, mutex_evaders;
    sensor_msgs::LaserScan scan;
    nav_msgs::Path trackingPath;
    std::map<int,int> targets;
    std::map<int,geometry_msgs::Pose2D> pursuer_poses;
    std::map<int,nav_msgs::Path> trails;
    std::map<int,behavior_decision::evaderState> evaderStates;
    // std::map<int,geometry_msgs::PoseArray> trails;

    void reconfigureCB(CCBSOPlannerConfig &config, uint32_t level);
    void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void trailCallback(const nav_msgs::PathConstPtr &msg, int i);
    void pathCallback(const nav_msgs::PathConstPtr &msg);
    void evaderCallback(const behavior_decision::evaderStateConstPtr &msg);
    void publishLookAheadPoint(std::pair<float,float>& pos, bool switch_ = false);
    bool checkFitnessService(check_fitness::Request &req,
                             check_fitness::Response &res);
    void publishState();

    void bsoSelection(std::pair<float,float>& nextPos);
    double calFitness(std::pair<float,float>& pos);
    float Fobstacle(std::pair<float,float>& pos); 
    bool isNearToObstacle(std::pair<float,float>& pos);
    float Fpheromone(std::pair<float,float>& pos);
    // float Fcruise(std::pair<float,float>& pos);
    float FangularAcc(std::pair<float,float>& pos);
    float FpotentialCollision(std::pair<float,float>& pos);
    float Fkeeping(std::pair<float,float>& pos);
    float FdisToTarget(std::pair<float,float>& pos);

    inline void getNthElement(std::multiset<std::pair<double,int>>::iterator it, int n, std::pair<double,int>& individual){
      std::advance(it,n);
      individual = *it;
    };
    // Lase: angle_min = 0, angle_max = 2*pi
    inline float theta2pi(float angle){
      if(angle<0){
        return angle + 2*PI;
      }
      else{
        angle = angle >= 2*PI ? angle - 2*PI: angle;
        return angle;
      }
    };

    inline float diffDis(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2){
      return hypot(pose2.pose.position.y - pose1.pose.position.y, pose2.pose.position.x - pose1.pose.position.x);
    };
    // diff = q2 * inverse(q1)
    inline float diffAngle(geometry_msgs::PoseStamped& msg1, geometry_msgs::PoseStamped& msg2){
      tf::Pose pose1, pose2;
      tf::poseMsgToTF(msg1.pose, pose1);
      tf::poseMsgToTF(msg2.pose, pose2);
      tf::Quaternion diff = pose2.getRotation ()*pose1.getRotation().inverse();
      return tf::getYaw(diff);
    };
    
  };
};
#endif
