#ifndef PURSUER_DECISION_MAKING_H
#define PURSUER_DECISION_MAKING_H
// from image_loader.cpp
#define MAP_IDX(sx,i,j) ((sx) * (j) + (i))
// from Astar .cpp
#define POT_HIGH 1.0e10 

#define MAX_COST 1e10

#include <mutex>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <deque>
// #include <queue>          // std::priority_queue
// #include <functional>     // std::greater
// #include <std_msgs/Int8.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "pursuer_behavior_decision/Hungarian.h"
#include <pursuer_behavior_decision/KalmanFilter.h>
#include "behavior_decision/behavior_decision_base.h"
#include "behavior_decision/evaderState.h"
#include "behavior_decision/dis2evaders.h"

namespace behavior_decision{
    class PursuerBehaviorDecision : public BehaviorDecision {
    public:
        enum state{
            SEARCHING,
            KEEPING,
            TRACKING,
            FOLLOWING
        };
        PursuerBehaviorDecision();
        ~PursuerBehaviorDecision();

        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
        void resetFrequency(double controller_frequency);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path);

        void getState(int &state);

        void test();
        // bool selectOptimalPolicy(Policy& policy);
    private:
        void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
        void evaderStateCallback(const behavior_decision::evaderState::ConstPtr& msg);
        void dis2evadersCallback(const behavior_decision::dis2evaders::ConstPtr& msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void calculateDistance(int start_x, int start_y, int target_x, int target_y, int target_id);
        void targetAssignment();
        // consider value 0 as free. Might be TODO
        bool Astar(int start_x, int start_y, int target_x, int target_y, std::vector<int>& path);
        //TODO : Lost target recovery;

        std::string ns, ns_tf2, gns;
        std::string fixed_frame_;
        int id, pnum, state_, target_id;
        int maxiFollower;
        int AdujustPathThre;
        double controller_frequency_;
        double lost_time_threshold_; // Check for targetState's availity
        double tracking2followingThre;
        tf::TransformListener* tf_;
        
        ros::Subscriber map_sub;
        ros::Subscriber detection_sub;
        ros::Publisher evaderState_pub;
        ros::Publisher inflationMap_pub;
        ros::Publisher dis2evaders_pub;
        ros::Publisher path_pub;
        // ros::Publisher target_pub;
        std::vector<ros::Subscriber> evaderState_subs; // Subscribe evaderState from other pursuers;
        std::vector<ros::Subscriber> dis2evader_subs; 
        std::recursive_mutex mutex_tags, mutex_evaderStates, mutex_map;
        apriltag_ros::AprilTagDetectionArray tags;
        std::map<int,behavior_decision::evaderState> detected;// detected[target_id] = behavior_decision::evaderState
        
        KalmanFilter* kf;//### why it has to be pointer, or --> undefined symbol

        // Astar
        nav_msgs::OccupancyGrid map_;
        double origin_x;
        double origin_y;
        int width;
        int height;
        double resolution;

        double robot_radius;
        // Target assignment
        // if state_ != KEEPING, robot needs to calculate it's distance to each evader for targetAssignment 
        std::map<int,std::deque<int>> path2evaders; // store index of waypoint
        std::map<int,double> dis2evaders;        // single pursuer, dis2evaders[evader_id]=distance
        // std::map<int,std::vector<double>> costs; // all pursuers, costs[robot_id]={dis,..,dis}(enum)
        std::map<int,std::map<int,double>> costs; // costs[robot_id]={evader_id: dis, ...} (Other pursuers')
        std::recursive_mutex mutex_costs;
        float grid_dis(int x1, int y1, int x2, int y2);
      
        // copied from global_planner 
        bool worldToMap(double wx, double wy, int& mx, int& my);
        void mapToWorld(int& mx, int& my, double& wx, double& wy);
        };
}
#endif

/*         data       |       mutex
        ============== ===================
           tags       |     mutex_tags
           map_       |     mutex_map
          costs       |    mutex_costs
         detected     | mutex_evaderStates
*/