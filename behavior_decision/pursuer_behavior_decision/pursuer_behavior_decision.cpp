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
    // Remove slash of ns
    int slash = ns.rfind('/');
    if( slash != std::string::npos ){
        ns_tf2 = ns.substr(slash+1);
    }

    ros::NodeHandle private_nh("~");
    private_nh.param("id"                        , id, 1);
    private_nh.param("pnum"                      , pnum, 1);
    private_nh.param("local_costmap/robot_radius", robot_radius, 0.105);
    private_nh.param("fixed_frame"               , fixed_frame_, std::string("/map"));
    private_nh.param("/global_namespace"         , gns, std::string("/robot"));
    private_nh.param("controller_frequency"      , controller_frequency_, 20.0);
    private_nh.param("lost_time_threshold"       , lost_time_threshold_, 3.0);
    private_nh.param("maxiFollower"              , maxiFollower, 3); // Now is fixed
    private_nh.param("tracking2followingThre"    , tracking2followingThre, 2.0);
    
    ROS_DEBUG("====================  id:%d\n====================  pnum:%d\n====================  state_:%d",id,pnum,state_);  
    map_sub = private_nh.subscribe(ns+"/map", 1, &PursuerBehaviorDecision::mapCallback,this);
    detection_sub = private_nh.subscribe(ns+"/tag_detections", 1, &PursuerBehaviorDecision::detectionCallback,this);
    evaderState_pub = private_nh.advertise<behavior_decision::evaderState>(ns+"/evader_state", 1, true);
    dis2evaders_pub = private_nh.advertise<behavior_decision::dis2evaders>(ns+"/dis2evaders", 1, true);
    for(int i = 0; i < pnum; i++){
        if(i != id){
            ros::Subscriber evaderState_sub = private_nh.subscribe(gns+to_string(i)+"/evader_state", 10, &PursuerBehaviorDecision::evaderStateCallback, this);
            ros::Subscriber dis2evader_sub = private_nh.subscribe(gns+to_string(i)+"/dis2evaders", 1, &PursuerBehaviorDecision::dis2evadersCallback, this);
            evaderState_subs.push_back(evaderState_sub);
            dis2evader_subs.push_back(dis2evader_sub);
        }
    }
    // inflationMap_pub = private_nh.advertise<nav_msgs::OccupancyGrid>(ns+"/inflation_map", 1, true);
}

void PursuerBehaviorDecision::detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
    std::unique_lock<std::recursive_mutex> lock(mutex_tags);
    tags = *msg;
}

void PursuerBehaviorDecision::evaderStateCallback(const behavior_decision::evaderState::ConstPtr& msg){
    std::unique_lock<std::recursive_mutex> lock(mutex_evaderStates);
    std::unique_lock<std::recursive_mutex> lock_costs(mutex_costs);
    // received evader state from other keeper
    // delete recorded keeper's cost
    if( msg->lost )
        // Remove lost target
        if(detected.find(msg->target_id) != detected.end())
            detected.erase(msg->target_id);
    else{
        detected[msg->target_id] = *msg;
        if(costs.find(msg->robot_id)!=costs.end())
            costs.erase(msg->robot_id);
    }
}

void PursuerBehaviorDecision::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std::unique_lock<std::recursive_mutex> lock(mutex_map);
    map_ = *msg;
    unsigned int width = map_.info.width, height = map_.info.height;
    int InflateRadius = std::ceil(robot_radius / map_.info.resolution);
    // dilate
    ROS_DEBUG("InflateRadius:%d",InflateRadius);
    for(int j=0;j<height;j++){
        for(int i=0;i<width;i++){
            int min_i, min_j, max_i, max_j;
            min_i = std::max(0, i - InflateRadius);
            min_j = std::max(0, j - InflateRadius);
            max_i = std::min(int(width), i + InflateRadius + 1);
            max_j = std::min(int(height), j + InflateRadius + 1);
            int8_t value = -1;
            for(int y=min_j;y<max_j;y++){
                for(int x=min_i;x<max_i;x++)
                    if(msg->data[MAP_IDX(width,x,y)] > value)
                        value = msg->data[MAP_IDX(width,x,y)];
            }
            map_.data[MAP_IDX(width,i,j)] = value;
        }
    }
    // different with matlab, left corner is grid's coodinate original
    // right->x,up->y
    // for(int j=height-10;j<height;j++){
    //     for(int i=width-20;i<width;i++){
    //         map_.data[MAP_IDX(width,i,j)] = 0;
    //     }
    // }
    // // left corner,i is x, j is y
    // for(int j=0;j<10;j++){
    //     for(int i=0;i<40;i++){
    //         map_.data[MAP_IDX(width,i,j)] = 0;
    //     }
    // }
    // inflationMap_pub.publish(map_);
}
// other non-Keeper's distace to each evader
void PursuerBehaviorDecision::dis2evadersCallback(const behavior_decision::dis2evaders::ConstPtr& msg){
    std::unique_lock<std::recursive_mutex> lock_costs(mutex_costs);
    std::map<int,double> cost;
    for(int i = 0; i != msg->evader_ids.size();i++){
        cost[msg->evader_ids[i]] = msg->dis[i];
    }
    costs[msg->robot_id] = cost;
}

// core function
void PursuerBehaviorDecision::getState(int &state){
    std::unique_lock<std::recursive_mutex> lock_tags(mutex_tags);
    apriltag_ros::AprilTagDetectionArray msg = tags;
    lock_tags.unlock();

    // calculate robot pose
    tf::StampedTransform robot_transform;
    geometry_msgs::Pose2D robot_pose;
    try{
        tf_->waitForTransform("/map", ns_tf2+"_tf/base_footprint", ros::Time(0), ros::Duration(0.01));
        tf_->lookupTransform("/map", ns_tf2+"_tf/base_footprint", ros::Time(0), robot_transform);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("tf transform error: %s",ex.what());
    }
    robot_pose.x = robot_transform.getOrigin().x();
    robot_pose.y = robot_transform.getOrigin().y();

    
    int tag_id;
    bool lostTarget = true;
    Eigen::Matrix<double,4,1> estimatedState; // estimate position and velocity(2d) of target evader

    std::unique_lock<std::recursive_mutex> states_lock(mutex_evaderStates);
    // Check for the robot's detection first
    // if(!msg.detections.empty()){
    for (int i = 0; i != msg.detections.size();i++){
        tag_id = msg.detections[i].id[0];
        // update target state
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
            catch(tf::TransformException& ex){
                ROS_ERROR("tf transform error: %s",ex.what());
            }
            catch(...){
                ROS_ERROR("Kalman Filter ERROR or Transform error");
            }
            lostTarget = false;
            break;
        }
        // detect a new target, state change to KEEPING
        if(detected.empty() || detected.find(tag_id) != detected.end()){
            tf::StampedTransform evader_transform;
            geometry_msgs::Pose2D evader_pose;
            double vx, vy;
            target_id = tag_id;
            try
            {
                tf_->waitForTransform("/map", "tag_"+to_string(tag_id), ros::Time(0), ros::Duration(0.01));
                tf_->lookupTransform("/map", "tag_"+to_string(tag_id), ros::Time(0), evader_transform);
                evader_pose.x = evader_transform.getOrigin().x();
                evader_pose.y = evader_transform.getOrigin().y();
                
                double v;
                vx = robot_pose.x - evader_pose.x;
                vy = robot_pose.y - evader_pose.y;
                v = hypot(vx, vy);
                // initial velocity is set to 1 m/s towards robot
                estimatedState << evader_pose.x, evader_pose.y, vx/v, vy/v;
                ROS_DEBUG("robot id : %d, evader %d 's initial state:%f, %f, %f, %f", id, target_id, evader_pose.x, evader_pose.y, vx/v,  vy/v);

                kf = new KalmanFilter(estimatedState, 1.0/controller_frequency_);
                state_ = KEEPING;
                ROS_DEBUG("robot %d state transform from SEARCHING to KEEPING", id);
                lostTarget = false;
                dis2evaders.clear();
                path2evaders.clear();
                break;
            }
            catch(tf::TransformException& ex){
                ROS_ERROR("tf transform error: %s",ex.what());
            }
        }
    }
    // }

    // Keeper publish evader's state
    if(state_ == KEEPING){
        behavior_decision::evaderState evaderState_;
        evaderState_.header.stamp = ros::Time::now();
        evaderState_.header.frame_id = fixed_frame_;
        evaderState_.robot_id = id;
        evaderState_.target_id = tag_id;
        evaderState_.lost = lostTarget;
        evaderState_.x = estimatedState(0,0);
        evaderState_.y = estimatedState(1,0);
        evaderState_.vx = estimatedState(2,0);
        evaderState_.vy = estimatedState(3,0);
        evaderState_pub.publish(evaderState_);
        detected[target_id] = evaderState_;
        if(lostTarget){
            state_ = SEARCHING;
            target_id = -1;
        }
    }

    // Check for estimate evader state published by other pursuer is available (time)
    if( detected.size()){
        for (std::map<int,behavior_decision::evaderState>::iterator it=detected.begin(); it!=detected.end(); ++it){
            behavior_decision::evaderState evaderState_ = it -> second;
            ros::Duration diff_time = ros::Time::now() - evaderState_.header.stamp;
            if(diff_time.toSec() > lost_time_threshold_){
                ROS_DEBUG("Lost target : %lu", evaderState_.target_id);
                detected.erase(it -> first);
            }
        }
    }
    
    // if not Keeper, publish distance to each detected evader 
    int start_x, start_y;
    std::map<int, std::pair<int,int>> target_poses;
    
    if(state_ != KEEPING && !detected.empty()){
        // transform robot and evader' poses from world to map
        // and calculate the distance between robot and each evader
        if(!worldToMap(robot_pose.x, robot_pose.y, start_x, start_y)){
            ROS_ERROR("Pursuer position transform error, Pw:(%f,%f); Pm:(%d,%d)",robot_pose.x, robot_pose.y,start_x, start_y);
        }
        for(std::map<int,behavior_decision::evaderState>::iterator it=detected.begin(); it!=detected.end(); ++it){
            int target_x, target_y;
            if(!worldToMap(it->second.x, it->second.y, target_x, target_y)){
                ROS_ERROR("target position is invalid, Pw:(%f,%f); Pm:(%d,%d)",it->second.x, it->second.y, target_x, target_y);
            }
            calculateDistance(start_x, start_y, target_x, target_y, it->second.target_id);
            std::pair<int,int> pair_(target_x, target_y);
            target_poses[it->second.target_id] = pair_;
        }

        // publish dis2evaders
        behavior_decision::dis2evaders dis2evaders_;
        dis2evaders_.header.stamp = ros::Time::now();
        dis2evaders_.header.frame_id = fixed_frame_;
        dis2evaders_.robot_id = id;
        for (std::map<int,double>::iterator it=dis2evaders.begin(); it!=dis2evaders.end(); ++it){
            dis2evaders_.evader_ids.push_back(it->first);
            dis2evaders_.dis.push_back(it->second);
        }
        dis2evaders_pub.publish(dis2evaders_);
        
        targetAssignment();
    }
    states_lock.unlock();

    std::unique_lock<std::recursive_mutex> lock_map(mutex_map);
    double resolution = map_.info.resolution;
    lock_map.unlock();

    int target_x = target_poses[target_id].first, target_y = target_poses[target_id].second;
    if(state_ == TRACKING && grid_dis(start_x, start_y, target_x, target_y)/resolution < tracking2followingThre){
        state_ = FOLLOWING;
        ROS_DEBUG("robot %d state transform from TRACKING to FOLLOWING ",id);
    }
    state =  state_;
}

// using data costs, dis2evaders
void PursuerBehaviorDecision::targetAssignment(){
    
    std::unique_lock<std::recursive_mutex> lock_costs(mutex_costs);
    // calculateDistance is above targetAssignment, so dis2evaders dont need mutex
    // build costa_matrix
    std::vector< std::vector<double> > costs_matrix;
    std::vector<int> evaderIdOrder;
    std::vector<double> cost;
    for (std::map<int,double>::iterator it=dis2evaders.begin(); it!=dis2evaders.end(); ++it){
        cost.push_back(it->second);
        evaderIdOrder.push_back(it->first);
    }
    costs_matrix.push_back(cost);

    for (std::map<int,std::map<int,double>>::iterator it=costs.begin(); it!=costs.end(); ++it){
        cost.clear();
        for(std::vector<int>::iterator it_id=evaderIdOrder.begin(); it_id!=evaderIdOrder.end(); ++it_id){
            if(it->second.find(*it_id) != it->second.end()){
                cost.push_back(it->second[*it_id]);
            }
            else{
                cost.push_back(MAX_COST);
                ROS_DEBUG("robot i %d dont have dis to evader %d", it->first, *it_id);
            }
        }
        costs_matrix.push_back(cost);
    }
    lock_costs.unlock();
    
    // non-Keeper is more than detected evader 
    if(costs_matrix.size()>dis2evaders.size()){
        for(std::vector<std::vector<double>>::iterator it=costs_matrix.begin();it!=costs_matrix.end();++it){
            std::vector<double> copy = *it;
            (*it).insert((*it).end(),copy.begin(),copy.end());
        }
    }

    HungarianAlgorithm HungAlgo;
	std::vector<int> assignment;
    double cost_ = HungAlgo.Solve(costs_matrix, assignment);
    if(assignment.front()>=0){
        if(assignment.front() != target_id){
            state_ = TRACKING;
            if(assignment.front() > dis2evaders.size()) 
                target_id = assignment.front() - dis2evaders.size();
            else 
                target_id = assignment.front();
            ROS_DEBUG("robot %d state transform from SEARCHING to TRACKING ",id);
        }     
    }
    else{
        //ROS_DEBUG();
    }
    
}
// Sacrifice memory
// using data map_, dis2evaders, path2evaders
void PursuerBehaviorDecision::calculateDistance(int start_x, int start_y, int target_x, int target_y,int target_id){
    std::unique_lock<std::recursive_mutex> lock(mutex_map);
    int width = map_.info.width;

    // new target case, build path
    if (path2evaders.find(target_id) == path2evaders.end()){
        std::vector<int> path;
        if(Astar(start_x, start_y, target_x, target_y, path)){// path is from [target -> start]
            dis2evaders[target_id] = 0;
            int pre_x = start_x, pre_y  = start_y;
            for (std::vector<int>::reverse_iterator rit = path.rbegin(); rit!= path.rend(); ++rit){
                int curr_x = *rit % width, curr_y = *rit / width;
                // store path from start to target 
                path2evaders[target_id].push_back(*rit);
                dis2evaders[target_id] += hypot(curr_x - pre_x, curr_y - pre_y);
                pre_x = curr_x, pre_y  = curr_y;
            }
        }
        else{
            ROS_ERROR("FIND PATH FAILED :1");
        }
    }
    else{
        // dynamic Astar
        // looking back : 5

        // Adjust from head of path
        std::deque<int>::iterator hit = path2evaders[target_id].begin();
        while( grid_dis(start_x     , start_y     , *hit % width    , *hit / width)   + 
               grid_dis(*hit % width, *hit / width, *(hit+4) % width, *(hit+4)/width) >
               grid_dis(start_x     , start_y     , *(hit+4) % width, *(hit+4) / width)){
                dis2evaders[target_id] -= hypot(*hit % width - *(hit+1) % width, 
                                                *hit / width - *(hit+1) / width);
                path2evaders[target_id].pop_front();
                ++hit;
        }
        std::vector<int> hpath;
        int head = path2evaders[target_id].front();
        if(Astar(start_x, start_y, head % width, head / width, hpath)){
            int pre_x = head % width, pre_y  = head / width;
            for (std::vector<int>::iterator it = hpath.begin()+1; it!= hpath.end(); ++it){
                int curr_x = *it % width, curr_y = *it / width;
                path2evaders[target_id].push_front(*it);
                dis2evaders[target_id] += hypot(curr_x - pre_x, curr_y - pre_y);
                pre_x = curr_x, pre_y  = curr_y;
            }
        }
        else{
            ROS_ERROR("FIND PATH FAILED :2");
        }
        // Adjust from end of path
        std::deque<int>::iterator eit = path2evaders[target_id].end() - 1;
        while( grid_dis(target_x    , target_y    , *eit % width    , *eit/ width)    + 
               grid_dis(*eit % width, *eit / width, *(eit-4) % width, *(eit-4)/width) >
               grid_dis(target_x    , target_y    , *(eit-4) % width, *(eit-4) / width)){
                dis2evaders[target_id] -= hypot(*eit % width - *(eit -1) % width, 
                                                *eit / width - *(eit -1) / width);
                path2evaders[target_id].pop_back();
                --eit;
        }
        std::vector<int> epath;
        int tail = path2evaders[target_id].back();
        if(Astar(target_x, target_y, tail % width, tail / width, epath)){
            int pre_x = tail % width, pre_y  = tail / width;
            for (std::vector<int>::iterator it = epath.begin()+1; it!= epath.end(); ++it){
                int curr_x = *it % width, curr_y = *it / width;
                path2evaders[target_id].push_back(*it);
                dis2evaders[target_id] += hypot(curr_x - pre_x, curr_y - pre_y);
                pre_x = curr_x, pre_y  = curr_y;
            }
        }
        else{
            ROS_ERROR("FIND PATH FAILED :2");
        }

    }
    
    lock.unlock();
}

float PursuerBehaviorDecision::grid_dis(int x1, int y1, int x2, int y2){
    int mini = std::min(std::abs(x2-x1),std::abs(y2-y1));
    int maxi = std::max(std::abs(x2-x1),std::abs(y2-y1));
    int residual = maxi - mini;
    float gdis = mini*std::sqrt(2) + residual;
    return gdis;
}

bool PursuerBehaviorDecision::Astar(int start_x, int start_y, int target_x, int target_y, std::vector<int>& path){
    int width = map_.info.width;
    int height = map_.info.height;
    int ns_ = width* height;
    std::vector<double> costs(ns_,POT_HIGH);
    std::vector<int> parents(ns_,-1);
    std::priority_queue<std::pair<float,int>,std::vector<std::pair<float,int> >,std::greater<std::pair<float,int> > > ind2visit;
    int currInd = MAP_IDX(width,start_x,start_y);
    
    costs[currInd] = 0.0;
    std::pair<float,int> pair_(0.0,currInd);
    ind2visit.push(pair_);
    int* nbrs = new int[8];
    float weights[] = {1.0, 1.41, 1.0, 1.41, 1.0, 1.41, 1.0, 1.41};
    bool path_found = false;
    while(!ind2visit.empty()){
        currInd = ind2visit.top().second;
        if(currInd==MAP_IDX(width,target_x,target_y)){
            break;
            path_found = true;
        }
        ind2visit.pop();
        int row = currInd / width;
        int col = currInd % width;
        nbrs[3] = (col > 0)                              ? currInd - 1           : -1;
        nbrs[0] = (row > 0 && col > 0)                   ? currInd - width - 1   : -1;
        nbrs[1] = (row > 0)                              ? currInd - width       : -1;
        nbrs[2] = (row > 0 && col + 1 < width)           ? currInd - width + 1   : -1;
        nbrs[4] = (col + 1 < width)                      ? currInd + 1           : -1;
        nbrs[7] = (row + 1 < height && col + 1 < width ) ? currInd + width + 1   : -1;
        nbrs[6] = (row + 1 < height)                     ? currInd + width       : -1;
        nbrs[5] = (row + 1 < height && col > 0)          ? currInd + width - 1   : -1;
        for (int i = 0; i < 8; ++i) {
            if (nbrs[i] >= 0) {
                float new_cost = costs[currInd] + weights[i];
                if (new_cost < costs[nbrs[i]] && map_.data[nbrs[i]]==0) {
                    float heuristic_cost, priority;
                    heuristic_cost = std::hypot(nbrs[i]%width - target_x, nbrs[i]/width - target_y);
                    priority = new_cost + heuristic_cost;
                    std::pair<float,int> _pair(priority,nbrs[i]);
                    ind2visit.push(_pair);

                    costs[nbrs[i]] = new_cost;
                    parents[nbrs[i]] = currInd;
                }   
            }
        }
    }
    delete[] nbrs;
    if(path_found){
        path.push_back(currInd);
        while(currInd != MAP_IDX(width,start_x,start_y)){
            currInd = parents[currInd];
            path.push_back(currInd);
        }
    }
    return path_found;
}

void PursuerBehaviorDecision::resetFrequency(double controller_frequency){
    controller_frequency_ = controller_frequency;
    if(state_ == KEEPING){
        // kf.resetDeltat(1.0/controller_frequency);
    }
}

bool PursuerBehaviorDecision::worldToMap(double wx, double wy, int& mx, int& my){
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    int width = map_.info.width;
    int height = map_.info.height;
    double resolution = map_.info.resolution;

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = round((wx - origin_x) / resolution );
    my = round((wy - origin_y) / resolution );

    if (mx < width && my < height)
        return true;

    return false;
}

bool PursuerBehaviorDecision::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_path){
}
void PursuerBehaviorDecision::test(){
    ROS_INFO("==================================Decision plugin is Working");
}