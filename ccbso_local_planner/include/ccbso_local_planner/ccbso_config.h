#ifndef CCBSO_CONFIG_H_
#define CCBSO_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include "ccbso_local_planner/CCBSOPlannerConfig.h" //cfg的名字加上Config

class CCBSOConfig
{
public:
    int psize;
    int iteration;

    double max_vel_x, max_rot_vel;
    double acc_lim_x, acc_lim_theta;
    double Pelitist, Pone;
    double Rstep, Astep;
    double pheromone_lasting;
    double searchDis, searchAngle;
    double markedDis, markedAngle;
    double threDis, threAngle;

    double weight_aa;

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
        psize = 20;
        iteration = 50;
        max_vel_x = 0.22;
        max_rot_vel = 2.75;
        acc_lim_x = 2.5;
        acc_lim_theta = 3.2;
        Pelitist = 0.5;
        Pone = 0.7;
        Rstep = 0.1;
        Astep = 0.1;
        pheromone_lasting = 30.0;
        searchDis = 1.0;
        searchAngle = 1.0472;// pi/3
        markedDis = 0.5;
        markedAngle = 0.785;
        threDis = 0.2;
        threAngle = 0.17; // 3.14/18
        weight_aa = 0.1;
    }
    static CCBSOConfig *instance;
};

CCBSOConfig* CCBSOConfig::instance = NULL;

//从Parameter Server中加载参数， 如果参数不存在， 则使用CCBSOConfig构造函数中初始化的值。
void CCBSOConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
    nh.param("psize", psize, psize);
    nh.param("iteration", iteration, iteration);
    nh.param("max_vel_x", max_vel_x, max_vel_x);
    nh.param("max_rot_vel", max_rot_vel, max_rot_vel);
    nh.param("acc_lim_x", acc_lim_x, acc_lim_x);
    nh.param("acc_lim_theta", acc_lim_theta, acc_lim_theta);
    nh.param("Pelitist", Pelitist, Pelitist);
    nh.param("Pone", Pone, Pone);
    nh.param("Rstep", Rstep, Rstep);
    nh.param("Astep", Astep, Astep);
    nh.param("pheromone_lasting", pheromone_lasting, pheromone_lasting);
    nh.param("searchDis", searchDis, searchDis);
    nh.param("searchAngle", searchAngle, searchAngle);
    nh.param("markedDis", markedDis, markedDis);
    nh.param("markedAngle", markedAngle, markedAngle);
    nh.param("threDis", threDis, threDis);
    nh.param("threAngle", threAngle, threAngle);
    nh.param("weight_aa", weight_aa, weight_aa);
    
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
    psize = cfg.psize;
    iteration = cfg.iteration;
    max_vel_x = cfg.max_vel_x;
    max_rot_vel = cfg.max_rot_vel;
    acc_lim_x = cfg.acc_lim_x;
    acc_lim_theta = cfg.acc_lim_theta;
    Pelitist = cfg.Pelitist;
    Pone = cfg.Pone;
    Rstep = cfg.Rstep;
    Astep = cfg.Astep;
    pheromone_lasting = cfg.pheromone_lasting;
    searchDis = cfg.searchDis;
    searchAngle = cfg.searchAngle;
    markedDis = cfg.markedDis;
    markedAngle = cfg.markedAngle;
    threDis = cfg.threDis;
    threAngle = cfg.threAngle;
    weight_aa = cfg.weight_aa;

    checkParameters();
}
    
    
void CCBSOConfig::checkParameters() const
{
  
}    

void CCBSOConfig::checkDeprecated(const ros::NodeHandle& nh) const
{
  
}

#endif