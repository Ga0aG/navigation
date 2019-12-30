#include "pursuer_behavior_decision/pursuer_behavior_decision.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(behavior_decision::PursuerBehaviorDecision, behavior_decision::BehaviorDecision)

using namespace behavior_decision;
using namespace std;

PursuerBehaviorDecision::PursuerBehaviorDecision():tf_(NULL),state_(SEARCHING),target_id(-1)
{

}

PursuerBehaviorDecision::~PursuerBehaviorDecision()
{

}

void PursuerBehaviorDecision::initialize(std::string name, tf::TransformListener* tf,costmap_2d::Costmap2DROS* costmap_ros){
    tf_ = tf;
    ns = ros::this_node::getNamespace();
    int slash = ns.rfind('/');
    if( slash != std::string::npos ){
        ns_tf2 = ns.substr(slash+1);
    }
    ros::NodeHandle private_nh("~");
    private_nh.param("id", id, 1);
    private_nh.param("pnum", pnum, 1);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("fixed_frame", fixed_frame_, std::string("/map"));
    ROS_DEBUG("======id:%d\n============================pnum:%d\n==================================state_:%d",id,pnum,state_);  
    detection_sub = private_nh.subscribe(ns+"/tag_detections", 1000, &PursuerBehaviorDecision::detectionCallback,this);
    evaderState_pub = private_nh.advertise<behavior_decision::evaderState>(ns+"/evader_state", 100);
}

void PursuerBehaviorDecision::detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
    int tag_id;
    Eigen::Matrix<double,4,1> estimatedState;
    // ROS_DEBUG("RECEIVED");
    if(!msg->detections.empty()){
        for (int i = 0; i != msg->detections.size();i++)
        {
            tag_id = msg->detections[i].id[0];
            //ROS_DEBUG("Detect id:%d",tag_id);
            if(tag_id == target_id){
                try{
                    tf::StampedTransform evader_transform;
                    geometry_msgs::Pose2D evader_pose;
                    tf_->waitForTransform("/map", "tag_"+to_string(tag_id), ros::Time(0), ros::Duration(0.01));
                    tf_->lookupTransform("/map", "tag_"+to_string(tag_id), ros::Time(0), evader_transform);
                    evader_pose.x = evader_transform.getOrigin().x();
                    evader_pose.y = evader_transform.getOrigin().y();
                    kf->update(evader_pose, estimatedState); 
                    ROS_DEBUG("evader %d 's pos: %f, %f; current state:%f, %f, %f, %f",target_id, evader_pose.x, evader_pose.y, estimatedState(0,0),estimatedState(1,0),estimatedState(2,0),estimatedState(3,0));
                }
                catch(...){
                    ROS_ERROR("Kalman Filter ERROR");
                }
                break;
            }
            if(detected.empty() || detected.find(tag_id) != detected.end()){
                tf::TransformListener listener;
                tf::StampedTransform robot_transform;
                tf::StampedTransform evader_transform;
                geometry_msgs::Pose2D robot_pose;
                geometry_msgs::Pose2D evader_pose;
                double vx, vy;
                target_id = tag_id;
                try
                {
                    tf_->waitForTransform("/map", ns_tf2+"_tf/base_footprint", ros::Time(0), ros::Duration(0.01));
                    tf_->lookupTransform("/map", ns_tf2+"_tf/base_footprint", ros::Time(0), robot_transform);
                    tf_->waitForTransform("/map", "tag_"+to_string(tag_id), ros::Time(0), ros::Duration(0.01));
                    tf_->lookupTransform("/map", "tag_"+to_string(tag_id), ros::Time(0), evader_transform);
                    robot_pose.x = robot_transform.getOrigin().x();
                    robot_pose.y = robot_transform.getOrigin().y();
                    evader_pose.x = evader_transform.getOrigin().x();
                    evader_pose.y = evader_transform.getOrigin().y();
                    
                    double v;
                    vx = robot_pose.x - evader_pose.x;
                    vy = robot_pose.y - evader_pose.y;
                    v = hypot(vx, vy);
                    
                    estimatedState << evader_pose.x, evader_pose.y, vx/v, vy/v;
                    ROS_DEBUG("evader %d 's initial state:%f, %f, %f, %f",target_id, evader_pose.x, evader_pose.y, vx/v,  vy/v);

                    detected[tag_id] = estimatedState;
                    kf = new KalmanFilter(estimatedState, 1.0/controller_frequency_);
                    // kf.initialize(estimatedState, 1.0/controller_frequency_);
                    state_ = KEEPING;
                    break;
                }
                catch(tf::LookupException& ex){
                    ROS_ERROR("PathFollow::getRobotPose(): No Transform available Error: %s\n", ex.what());
                }
                catch(tf::ConnectivityException& ex){
                    ROS_ERROR("PathFollow::getRobotPose(): Connectivity Error: %s\n", ex.what());
                }
                catch(tf::ExtrapolationException& ex){
                    ROS_ERROR("PathFollow::getRobotPose(): Extrapolation Error: %s\n", ex.what());
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("PathFollow::getRobotPose(): Transform Error: %s",ex.what());
                }
            }
        }
        if(state_ == KEEPING){
            behavior_decision::evaderState evaderState_;
            evaderState_.header.stamp = ros::Time::now();
            evaderState_.header.frame_id = fixed_frame_;
            evaderState_.id = tag_id;
            evaderState_.lost = false;
            evaderState_.x = estimatedState(0,0);
            evaderState_.y = estimatedState(1,0);
            evaderState_.vx = estimatedState(2,0);
            evaderState_.vy = estimatedState(3,0);
            evaderState_pub.publish(evaderState_);
        }
    }
}

void PursuerBehaviorDecision::getState(int &state){
    state =  state_;
}

void PursuerBehaviorDecision::resetFrequency(double controller_frequency){
    controller_frequency_ = controller_frequency;
    if(state_ == KEEPING){
        // kf.resetDeltat(1.0/controller_frequency);
    }
}

bool PursuerBehaviorDecision::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path){
}
void PursuerBehaviorDecision::test(){
    ROS_INFO("==================================Decision plugin is Working");
}