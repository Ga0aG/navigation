
#include <ccbso_local_planner/ccbso_local_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ccbso_local_planner::CCBSOPlanner, nav_core::BaseLocalPlanner)

using namespace ccbso_local_planner;

CCBSOPlanner::CCBSOPlanner(){

}
CCBSOPlanner::~CCBSOPlanner(){

}
void CCBSOPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    state_ = SEARCHING;
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("id"                        , id, 2);
    private_nh.param("pnum"                      , pnum, 1);

    private_nh.param("max_vel_x"                 , max_vel_x, 0.22);
    private_nh.param("max_rot_vel"               , max_rot_vel, 2.75);
    private_nh.param("acc_lim_x"                 , acc_lim_x, 2.5);
    private_nh.param("acc_lim_theta"             , acc_lim_theta, 3.2);

    CCBSOCONFIG.loadRosParamFromNodeHandle(private_nh);
    dsrv_ = boost::make_shared<dynamic_reconfigure::Server<CCBSOPlannerConfig>>(private_nh);
    dynamic_reconfigure::Server<CCBSOPlannerConfig>::CallbackType cb = boost::bind(&CCBSOPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    ROS_INFO("Robot%d: initialized ccbso_local_planner",id);
    
}

void CCBSOPlanner::reconfigureCB(CCBSOPlannerConfig &config, uint32_t level){
    CCBSOCONFIG.reconfigure(config);
}

bool CCBSOPlanner::setState(int &state){
    if(state_ != state){
        ROS_DEBUG("robot%d: state transform from %s to %s", id, states[state_].c_str(), states[state].c_str());
        state_ = state;
    }
}

bool CCBSOPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    
}

bool CCBSOPlanner::isGoalReached(){}

bool CCBSOPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){}






















