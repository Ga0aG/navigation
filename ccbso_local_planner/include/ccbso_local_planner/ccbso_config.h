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

    bool show_poseArray;
    bool pheromone_forever;
    bool clear_pheromone;

    double max_vel_x, max_rot_vel;
    double acc_lim_x, acc_lim_theta;
    double Pelitist, Pone;
    double Rstep, Astep;
    double pheromone_lasting;
    double searchDis, searchAngle;
    double markedDis, markedAngle;
    double threDis, threAngle;

    double weight_aa;
    double weight_cruiser,cruiser_max;
    double pheromone;
    double weight_pc;
    double weight_k;
    double weight_d, disToTarget;
    

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
        show_poseArray = true;
        pheromone_forever = false;
        clear_pheromone = false;
        max_vel_x = 0.22;
        max_rot_vel = 2.75;
        acc_lim_x = 2.5;
        acc_lim_theta = 3.2;
        Pelitist = 0.5;
        Pone = 0.7;
        Rstep = 0.1;
        Astep = 0.1;
        pheromone_lasting = 100.0;
        searchDis = 1.0;
        searchAngle = 1.9;
        markedDis = 0.7;
        markedAngle = 1.0472;// pi/3
        threDis = 0.2;
        threAngle = 0.17; // 3.14/18
        weight_aa = 0.01;
        weight_cruiser = 2.0;
        cruiser_max = 5.0;
        pheromone = 2.0;
        weight_pc = 5.0;//bigger than pheromone
        weight_k = 2.0;
        weight_d = 1.0;
        disToTarget = 1.0;
    }
    static CCBSOConfig *instance;
};

CCBSOConfig* CCBSOConfig::instance = NULL;

//从Parameter Server中加载参数， 如果参数不存在， 则使用CCBSOConfig构造函数中初始化的值。
void CCBSOConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
    
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
    show_poseArray = cfg.show_poseArray;
    pheromone_forever = cfg.pheromone_forever;
    clear_pheromone = cfg.clear_pheromone;
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
    weight_cruiser = cfg.weight_cruiser;
    cruiser_max = cfg.cruiser_max;
    pheromone = cfg.pheromone;
    weight_pc = cfg.weight_pc;
    weight_k = cfg.weight_k;
    weight_d = cfg.weight_d;
    disToTarget = cfg.disToTarget;

    checkParameters();
}
    
    
void CCBSOConfig::checkParameters() const
{
  
}    

void CCBSOConfig::checkDeprecated(const ros::NodeHandle& nh) const
{
  
}

#endif