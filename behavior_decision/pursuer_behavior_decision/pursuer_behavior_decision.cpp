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

void PursuerBehaviorDecision::initialize(std::string name, tf::TransformListener* tf,costmap_2d::Costmap2DROS* costmap_ros){
    tf::TransformListener* m_p_tf;
    m_p_tf = tf;
    // ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle private_nh("~");
    private_nh.param("id", id, 1);
    private_nh.param("pnum", pnum, 1);
    ros::Subscriber sub2 = private_nh.subscribe("tag_detections", 1000, &PursuerBehaviorDecision::detectionCallback,this);
    sub = private_nh.subscribe("/robot1/tag_detections", 1000, &PursuerBehaviorDecision::detectionCallback,this);
    state_ = SEARCHING;
    ROS_INFO("*********id:%d\n*********pnum:%d\n*********state_:%d",id,pnum,state_);
}

void PursuerBehaviorDecision::getState(int &state){
    state =  state_;
    std::string ns = ros::this_node::getNamespace();
    // ROS_INFO("namespace:%s",ns.c_str());
    ns = ros::this_node::getName();
    // ROS_INFO("namespace:%s",ns.c_str());
}

void PursuerBehaviorDecision::detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
    ROS_INFO("received");
    if(!msg->detections.empty()){
        for (int i = 0; i != msg->detections.size();i++)
        {
            int tag_id = msg->detections[i].id[0];
            ROS_INFO("Detect id:%d",tag_id);
        }
    }
}

bool PursuerBehaviorDecision::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path){
}
void PursuerBehaviorDecision::test(){
    ROS_INFO("==================================Working");
    ROS_INFO("==================================Working");
}