#ifndef CCBSO_CONFIG_H_
#define CCBSO_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include "ccbso_local_planner/CCBSOPlannerConfig.h" //cfg的名字加上Config

class CCBSOConfig
{
public:

    double max_vel_x;
    double max_rot_vel;
    double acc_lim_x;
    double acc_lim_theta;

    static CCBSOConfig& getInstance();
    #define CCBSOCONFIG CCBSOConfig::getInstance()


    void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);
    void reconfigure(ccbso_local_planner::CCBSOPlannerConfig& cfg);
    void checkParameters() const;
    void checkDeprecated(const ros::NodeHandle& nh) const;
    boost::mutex& configMutex() {return config_mutex_;}
  
private:
    boost::mutex config_mutex_; //!< Mutex for config accesses and changes

    CCBSOConfig()
    {
        max_vel_x = 0.22;
        max_rot_vel = 2.75;
        acc_lim_x = 2.5;
        acc_lim_theta = 3.2;
    }
    static CCBSOConfig *instance;
};

CCBSOConfig* CCBSOConfig::instance = NULL;

//从Parameter Server中加载参数， 如果参数不存在， 则使用SfmConfig构造函数中初始化的值。
void CCBSOConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
    nh.param("max_vel_x", max_vel_x, max_vel_x);
    nh.param("max_rot_vel", max_rot_vel, max_rot_vel);
    nh.param("acc_lim_x", acc_lim_x, acc_lim_x);
    nh.param("acc_lim_theta", acc_lim_theta, acc_lim_theta);
    checkParameters();
    checkDeprecated(nh);
}

CCBSOConfig& CCBSOConfig::getInstance() {
    if (instance == NULL) instance = new CCBSOConfig();
    return *instance;
}

void CCBSOConfig::reconfigure(ccbso_local_planner::CCBSOPlannerConfig& cfg)
{ 
    boost::mutex::scoped_lock l(config_mutex_);
    max_vel_x = cfg.max_vel_x;
    max_rot_vel = cfg.max_rot_vel;
    acc_lim_x = cfg.acc_lim_x;
    acc_lim_theta = cfg.acc_lim_theta;
    checkParameters();
}
    
    
void CCBSOConfig::checkParameters() const
{
  
}    

void CCBSOConfig::checkDeprecated(const ros::NodeHandle& nh) const
{
  
}

#endif