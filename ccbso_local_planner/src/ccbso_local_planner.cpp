
#include <ccbso_local_planner/ccbso_local_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ccbso_local_planner::CCBSOPlanner, nav_core::BaseLocalPlanner)

using namespace ccbso_local_planner;

CCBSOPlanner::CCBSOPlanner():received_scan(false), state_(SEARCHING){

}
CCBSOPlanner::~CCBSOPlanner(){

}
void CCBSOPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // get namespace
    ns = ros::this_node::getNamespace();
    // Remove slash of ns
    int slash = ns.rfind('/');
    if( slash != std::string::npos ){
        ns_tf2 = ns.substr(slash+1);
    }
    ros::NodeHandle nh(ns);
    ROS_INFO("NAMESPACE:%s",ns.c_str());
    nh.param("tf_prefix",tf_prefix,std::string("robot"));
    ROS_INFO("tf_prefix:%s",tf_prefix.c_str());

    nh.param("id"                        , id, 2);
    nh.param("pnum"                      , pnum, 1);
    nh.param("robot_radius"              , robot_radius, 0.105);
    nh.param("/global_namespace"         , gns, std::string("/robot"));
    nh.param("fixed_frame"               , fixed_frame_, std::string("/map"));
    
    sub_laser = nh.subscribe(ns+"/scan",1,&CCBSOPlanner::laserCallback,this);
    sub_path = nh.subscribe(ns+"/trackingPath",1 ,&CCBSOPlanner::pathCallback,this);
    for(int i=1;i<=pnum;i++){
        ros::Subscriber sub_evaderState = nh.subscribe(ns+"/evader_state", 1, &CCBSOPlanner::evaderCallback,this);
        sub_evaderStates.push_back(sub_evaderState);
        if(i!=id){
            ros::Subscriber sub_trail = nh.subscribe<nav_msgs::Path>(gns+std::to_string(i)+"/pheromoneTrail",1,boost::bind(&CCBSOPlanner::trailCallback, this, _1, i));
            sub_trails.push_back(sub_trail);
        }
    }
    pub_pheromoneTrail = nh.advertise<nav_msgs::Path>(ns+"/pheromoneTrail",1);
    pub_pheromoneTrail_ = nh.advertise<geometry_msgs::PoseArray>(ns+"/pheromoneTrail_",1);
    pub_lookAheadPoint = nh.advertise<visualization_msgs::Marker>(ns+"/lookAheadPoint",1);
    pub_checkPoint = nh.advertise<visualization_msgs::Marker>(ns+"/checkPoint",1);
    pub_state = nh.advertise<visualization_msgs::Marker>(ns+"/state",1);
    srv_check_fitness = nh.advertiseService("check_fitness", &CCBSOPlanner::checkFitnessService, this);

    ros::NodeHandle private_nh(ns+"/" + name);
    CCBSOCONFIG.loadRosParamFromNodeHandle(private_nh);
    dsrv_ = boost::make_shared<dynamic_reconfigure::Server<CCBSOPlannerConfig>>(private_nh);
    dynamic_reconfigure::Server<CCBSOPlannerConfig>::CallbackType cb = boost::bind(&CCBSOPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    

    ROS_INFO("Robot%d: initialized ccbso_local_planner",id);
    
}

void CCBSOPlanner::reconfigureCB(CCBSOPlannerConfig &config, uint32_t level){
    CCBSOCONFIG.reconfigure(config);
}

void CCBSOPlanner::laserCallback(const sensor_msgs::LaserScanConstPtr &msg){
    std::unique_lock<std::recursive_mutex> lock_tags(mutex_scan);
    scan = *msg;
    received_scan = true;
}
// pheromone trail
void CCBSOPlanner::trailCallback(const nav_msgs::PathConstPtr &msg, int i){
    std::unique_lock<std::recursive_mutex> lock_trails(mutex_trails);
    trails[i] = *msg;
}
// path to target
void CCBSOPlanner::pathCallback(const nav_msgs::PathConstPtr &msg){
    std::unique_lock<std::recursive_mutex> lock_path(mutex_path);
    trackingPath = *msg;
}

void CCBSOPlanner::evaderCallback(const behavior_decision::evaderStateConstPtr &msg){
    std::unique_lock<std::recursive_mutex> lock_evaders(mutex_evaders);
    evaderStates[msg->target_id] = *msg;
    // ROS_WARN("Received evader");
}

bool CCBSOPlanner::checkFitnessService(check_fitness::Request &req,
                                       check_fitness::Response &res){
    // state_ = res.state
    std::unique_lock<std::recursive_mutex> lock_pursuers(mutex_pursuers);
    std::unique_lock<std::recursive_mutex> lock_trails(mutex_trails);
    std::unique_lock<std::recursive_mutex> lock_scan(mutex_scan);
    std::unique_lock<std::recursive_mutex> lock_path(mutex_path);
    std::unique_lock<std::recursive_mutex> lock_evaders(mutex_evaders);
    std::pair<float,float> pos(req.delta_trans, req.delta_rot1);
    switch(state_){
        case SEARCHING:{
            res.Fpheromone = Fpheromone(pos);
            res.Fobstacle =  Fobstacle(pos);
            res.FangularAcc = FangularAcc(pos);
            res.FpotentialCollision = FpotentialCollision(pos);
            res.sum = res.Fpheromone+res.Fobstacle+res.FpotentialCollision+res.FangularAcc;
            break;
        }
        case KEEPING:{
            res.Fobstacle =  Fobstacle(pos);
            res.FangularAcc = FangularAcc(pos);
            res.FpotentialCollision = FpotentialCollision(pos);
            res.Fkeeping = Fkeeping(pos);
            res.FdisToTarget = FdisToTarget(pos);
            res.sum = res.Fobstacle+res.FpotentialCollision+res.FangularAcc+res.Fkeeping+res.FdisToTarget;
            // res.element1 = evaderStates[targets[id]].x;
            // res.element2 = evaderStates[targets[id]].y;
        }
        default:{
            break;
        }
    }
    publishLookAheadPoint(pos,true);
}

bool CCBSOPlanner::setState(int &state){
    // TODO, bool alow_received
    if(state_ != state){
        ROS_DEBUG("robot%d: state transform from %s to %s", id, states[state_].c_str(), states[state].c_str());
        state_ = state;
    }
}

bool CCBSOPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    // thread safety
    std::unique_lock<std::recursive_mutex> lock_pursuers(mutex_pursuers);
    std::unique_lock<std::recursive_mutex> lock_trails(mutex_trails);
    std::unique_lock<std::recursive_mutex> lock_scan(mutex_scan);
    std::unique_lock<std::recursive_mutex> lock_path(mutex_path);
    std::unique_lock<std::recursive_mutex> lock_evaders(mutex_evaders);
    geometry_msgs::PoseStamped currPose;
    
    // Access pursuers' targets
    ros::NodeHandle nh;
    for(int i=1;i<=pnum;i++){
        int target_idx;
        if(nh.getParam(gns+std::to_string(i)+"/target_id", target_idx)){
            targets[i] = target_idx;
        }
        else{
            targets[i] = -1;
        }
    }

    // get pursuers' poses
    for(int i=1;i<=pnum;i++){
        tf::StampedTransform pursuser_transform;
        geometry_msgs::Pose2D pursuer_pose;
        try{
            tf_->waitForTransform("/map", gns+std::to_string(i)+"_tf/base_footprint", ros::Time(0), ros::Duration(0.01));
            tf_->lookupTransform("/map", gns+std::to_string(i)+"_tf/base_footprint", ros::Time(0), pursuser_transform);
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("tf transform error: %s",ex.what());
        }
        pursuer_pose.x = pursuser_transform.getOrigin().x();
        pursuer_pose.y = pursuser_transform.getOrigin().y();
        pursuer_pose.theta = tf::getYaw(pursuser_transform.getRotation());
        pursuer_poses[i] = pursuer_pose;
        if(id==i){
            currPose.header.stamp = ros::Time::now();
            currPose.header.frame_id = fixed_frame_;
            currPose.pose.position.x = pursuer_pose.x;
            currPose.pose.position.y = pursuer_pose.y;
            currPose.pose.orientation.x = pursuser_transform.getRotation().x();
            currPose.pose.orientation.y = pursuser_transform.getRotation().y();
            currPose.pose.orientation.z = pursuser_transform.getRotation().z();
            currPose.pose.orientation.w = pursuser_transform.getRotation().w();
        }
    }
    
    publishState();

    // Publish trail
    // TODO, clear trail if state_ != SEARCING
    if(CCBSOCONFIG.clear_pheromone){
        trails[id].poses.clear();
        CCBSOCONFIG.clear_pheromone = false;
    }
    trails[id].header.stamp = ros::Time::now();
    if(trails[id].poses.empty()){
        trails[id].header.frame_id = fixed_frame_;
        trails[id].header.frame_id = fixed_frame_;
        trails[id].poses.push_back(currPose);
    }
    else{
        // Pheromones disappear over time
        ros::Duration diff_time = trails[id].header.stamp - trails[id].poses.front().header.stamp;
        while(diff_time.toSec() > CCBSOCONFIG.pheromone_lasting && !CCBSOCONFIG.pheromone_forever && !trails[id].poses.empty()){
            trails[id].poses.erase(trails[id].poses.begin());
            diff_time = trails[id].header.stamp - trails[id].poses.front().header.stamp;
        }
        float mdis, mangle;// status changed
        mdis = diffDis(trails[id].poses.back(),currPose);
        mangle = diffAngle(trails[id].poses.back(),currPose);
        // ros::Duration diff_time2 = trails[id].header.stamp - trails[id].poses.back().header.stamp;
        if(mdis > CCBSOCONFIG.threDis || std::abs(mangle)> CCBSOCONFIG.threAngle){
            trails[id].poses.push_back(currPose);
        }
    }
    //publish nav::Path
    pub_pheromoneTrail.publish(trails[id]);
    //publish geometry_msgs::PoseArray
    if(CCBSOCONFIG.show_poseArray){
        geometry_msgs::PoseArray poseArray;
        poseArray.header = trails[id].header;
        for(std::vector<geometry_msgs::PoseStamped>::iterator it = trails[id].poses.begin() ; it != trails[id].poses.end(); ++it){
            poseArray.poses.push_back(it->pose);
        }
        pub_pheromoneTrail_.publish(poseArray);
    }
    
    std::pair<float,float> nextPos;//delta_trans,delta_rot1
    bsoSelection(nextPos);
    
    // compute velocity
    cmd_vel.linear.x = 0.0;
    if(std::abs(nextPos.first)>1e-2){
        if(std::abs(nextPos.second)>1e-2){
            // pos.first/2 / sin(pos.second) * 2 * pos.second = arc
            cmd_vel.linear.x = nextPos.first*nextPos.second/sin(nextPos.second);
        }
        else{
            cmd_vel.linear.x = nextPos.first;
        }
    }
    if(std::abs(cmd_vel.linear.x) > CCBSOCONFIG.max_vel_x){
        cmd_vel.linear.x = cmd_vel.linear.x > 0 ? CCBSOCONFIG.max_vel_x: -CCBSOCONFIG.max_vel_x;
    }
    // cmd_vel.angular.z = nextPos.second * 0.1;//performs bad when times 1/control_frequency
    cmd_vel.angular.z = nextPos.first > 0? nextPos.second: -nextPos.second;
    publishLookAheadPoint(nextPos);
}

void CCBSOPlanner::bsoSelection(std::pair<float,float>& nextPos){
    // // Roulette Wheel Selection
    // // Sampling within a sector
    // Eigen::Matrix<float, 11, 1> pros;
    // float sum = 1;
    // // if(state_ == SEARCHING)
    // //     sum = 1;
    // // else
    // //     sum = 5;//give ro=0 bigger propertity
    // for(int i=0;i<=10;i++){
    //     sum += i;
    //     pros(i,0)=sum;
    // }
    // pros = pros/sum;

    // odometry motion model
    std::vector<std::pair<float,float>> nextPoses;//delta_trans,delta_rot1
    std::multiset<std::pair<double,int>> individuals;//fitness,index, Ascending order
    
    // uniformly sampling
    for(int i=0;i<CCBSOCONFIG.psize;i++){
        double p = rand()/double(RAND_MAX);
        std::pair<float,float> pos(p*CCBSOCONFIG.searchDis,i*CCBSOCONFIG.searchAngle/CCBSOCONFIG.psize - CCBSOCONFIG.searchAngle/2.0);
        std::pair<double,int> individual(calFitness(pos),i);
        nextPoses.push_back(pos);
        individuals.emplace(individual);
    }

    if(state_== KEEPING || state_ == FOLLOWING){
        for(int i=0;i<CCBSOCONFIG.psize;i++){
            double p = rand()/double(RAND_MAX);
            std::pair<float,float> newPos(-p*CCBSOCONFIG.searchDis,i*CCBSOCONFIG.searchAngle/CCBSOCONFIG.psize - CCBSOCONFIG.searchAngle/2.0);
            double newFitness = calFitness(newPos);
            std::pair<double,int> worseIndi;
            getNthElement(individuals.begin(),CCBSOCONFIG.psize-1,worseIndi);
            if(newFitness < worseIndi.first){
                std::pair<double,int> newIndi(newFitness,worseIndi.second);
                nextPoses[worseIndi.second] = newPos;
                individuals.erase(worseIndi);
                individuals.emplace(newIndi);
            }
        }
    }

    // Add zero velocity individual
    for(int i=0;i<6;i++){
        std::pair<double,int> worseIndi;
        getNthElement(individuals.begin(),CCBSOCONFIG.psize-1,worseIndi);
        std::pair<float,float> newPos(0,i*CCBSOCONFIG.searchAngle/6.0 - CCBSOCONFIG.searchAngle/2.0);
        double newFitness = calFitness(newPos);
        if(newFitness < worseIndi.first){
            std::pair<double,int> newIndi(newFitness,worseIndi.second);
            nextPoses[worseIndi.second] = newPos;
            individuals.erase(worseIndi);
            individuals.emplace(newIndi);
        }
    }
    
    // for(int i=0;i<CCBSOCONFIG.psize;i++){
    //     double p = rand()/double(RAND_MAX);
    //     for(int j=0;j<=10;j++){
    //         if(p <= pros(j,0)){
    //             //std::pair<float,float> pos(CCBSOCONFIG.searchDis/10.0*j,rand()%20*CCBSOCONFIG.searchAngle/20.0 - CCBSOCONFIG.searchAngle/2.0);
    //             std::pair<float,float> pos;
    //             pos.second = i*CCBSOCONFIG.searchAngle/CCBSOCONFIG.psize - CCBSOCONFIG.searchAngle/2.0;
    //             if(state_ == SEARCHING || state_ == FOLLOWING){
    //                 pos.first = CCBSOCONFIG.searchDis/10.0*j;
    //             }
    //             else{
    //                 pos.first = CCBSOCONFIG.searchDis/10.0*j - CCBSOCONFIG.searchDis/2.0;
    //             }
    //             std::pair<double,int> individual(calFitness(pos),i);
    //             nextPoses.push_back(pos);
    //             individuals.emplace(individual);
    //             break;
    //         }
    //     }
    // }



    // update individual
    for(int i=0;i<CCBSOCONFIG.iteration;i++){
        //generate new individual
        std::pair<float,float> newPos;
        double p1 = rand()/double(RAND_MAX);
        double p2 = rand()/double(RAND_MAX);
        // Divide indiviudals into half elitist and half normal
        if(p1<CCBSOCONFIG.Pelitist){
            if(p2<CCBSOCONFIG.Pone){
                std::pair<double,int> pair_;
                int ind = rand()%(CCBSOCONFIG.psize/2);
                getNthElement(individuals.begin(),ind,pair_);
                newPos = nextPoses[pair_.second];
                newPos.first += CCBSOCONFIG.Rstep*(rand()/double(RAND_MAX)-0.5)*2;
                newPos.second += CCBSOCONFIG.Astep*(rand()/double(RAND_MAX)-0.5)*2;
            }
            else{
                std::pair<double,int> pair1,pair2;
                int ind1 = rand()%(CCBSOCONFIG.psize/2);
                int ind2 = rand()%(CCBSOCONFIG.psize/2);
                getNthElement(individuals.begin(),ind1,pair1);
                getNthElement(individuals.begin(),ind2,pair2);
                newPos.first = nextPoses[pair1.second].first/2.0+nextPoses[pair2.second].first/2.0;
                newPos.second = nextPoses[pair1.second].second/2.0+nextPoses[pair2.second].second/2.0;
            }
        }
        else{
            if(p2<CCBSOCONFIG.Pone){
                std::pair<double,int> pair_;
                int ind = rand()%(CCBSOCONFIG.psize/2);
                getNthElement(individuals.begin(),ind+CCBSOCONFIG.psize/2,pair_);
                newPos = nextPoses[pair_.second];
                newPos.first += CCBSOCONFIG.Rstep*(rand()/double(RAND_MAX)-0.5)*2;
                newPos.second += CCBSOCONFIG.Astep*(rand()/double(RAND_MAX)-0.5)*2;
            }
            else{
                std::pair<double,int> pair1,pair2;
                int ind1 = rand()%(CCBSOCONFIG.psize/2);
                int ind2 = rand()%(CCBSOCONFIG.psize/2);
                getNthElement(individuals.begin(),ind1+CCBSOCONFIG.psize/2,pair1);
                getNthElement(individuals.begin(),ind2+CCBSOCONFIG.psize/2,pair2);
                newPos.first = nextPoses[pair1.second].first/2.0+nextPoses[pair2.second].first/2.0;
                newPos.second = nextPoses[pair1.second].second/2.0+nextPoses[pair2.second].second/2.0;
            }
        }
        // verify the effectiveness of individual
        if(std::abs(newPos.second) > CCBSOCONFIG.searchAngle / 2.0 || std::abs(newPos.first) > CCBSOCONFIG.searchDis){
            i--;
            continue;
        } 
        if((state_ == SEARCHING || state_ == TRACKING) && newPos.first < 0){// only moving forward
            // newPos.first = 0.0;
            i--;
            continue;
        }
        double newFitness = calFitness(newPos);
        std::pair<double,int> worseIndi;
        getNthElement(individuals.begin(),CCBSOCONFIG.psize-1,worseIndi);
        if(newFitness < worseIndi.first){
            std::pair<double,int> newIndi(newFitness,worseIndi.second);
            nextPoses[worseIndi.second] = newPos;
            individuals.erase(worseIndi);
            individuals.emplace(newIndi);
        }
        //Avoid hitting wall, spinning to find an angle to escape
        std::pair<double,int> bestIndi;
        getNthElement(individuals.begin(),0,bestIndi);
        if(bestIndi.first >= fmaxi){
            newPos.first = 0.0;
            newPos.second = 0.3;
            newFitness = calFitness(newPos);
            // newFitness = 2*fmaxi;
            // for(int j=0;j<CCBSOCONFIG.psize;j++){
            //     std::pair<float,float> tempPos(0.0, CCBSOCONFIG.searchAngle/CCBSOCONFIG.psize * j - CCBSOCONFIG.searchAngle/2.0);
            //     double tempFitness = calFitness(tempPos);
            //     if(tempFitness < newFitness){
            //         newFitness = tempFitness;
            //         newPos = tempPos;
            //     }
            // }
            std::pair<double,int> newIndi(newFitness,bestIndi.second);
            individuals.clear();
            individuals.emplace(newIndi);
            nextPoses[bestIndi.second] = newPos;
            break;
        }
    }
    // choose the best one
    if(individuals.size()){
        int ind = (*individuals.begin()).second;
        nextPos = nextPoses[ind];
        // std::pair<double,int> worseIndi;
        // getNthElement(individuals.begin(),CCBSOCONFIG.psize-1,worseIndi);
        // float posTheta = theta2pi(nextPos.second);
        // int index = round(posTheta/scan.angle_increment);
        // float irange = 0.0;
        // if(received_scan){
        //     irange = scan.ranges[index];
        // }
        // ROS_INFO("robot%i:distance:%f, angle:%f, value:%lf, scan range: %f",id,nextPos.first,nextPos.second,(*individuals.begin()).first,irange);
        ROS_DEBUG_NAMED("result","robot%i:distance:%f, angle:%f, value:%lf",id,nextPos.first,nextPos.second,(*individuals.begin()).first);
        switch(state_){
            case SEARCHING:{
                //ROS_INFO("robot%i:Fpheromone:%f, Fobstacle:%f, Fcruise:%f, FangularAcc:%f",id,Fpheromone(nextPos),Fobstacle(nextPos),Fcruise(nextPos), FangularAcc(nextPos));
                break;
            }
            default:{
                break;
            }
        }
    }
    else{
        ROS_ERROR("individuals is empty");
    }
}

double CCBSOPlanner::calFitness(std::pair<float,float>& pos){
    geometry_msgs::Twist cmd_vel;
    double fitness = 0;
    switch(state_){
        case SEARCHING:{
            // fitness += Fpheromone(pos) + Fobstacle(pos) + FangularAcc(pos);
            fitness += Fpheromone(pos) + Fobstacle(pos) + FangularAcc(pos) + FpotentialCollision(pos);
            break;
        }
        case KEEPING:{
            fitness += Fobstacle(pos) + FangularAcc(pos) + FpotentialCollision(pos) + Fkeeping(pos) + FdisToTarget(pos);
            break;
        }
        default:{

        }
    }
    return fitness;
}

float CCBSOPlanner::Fobstacle(std::pair<float,float>& pos){
    if(!received_scan) return 0.0;
    // TODO
    // if there's pursuer between robot and pos, return 0;
    float posTheta = pos.first < 0? theta2pi(pos.second+PI) : theta2pi(pos.second);
    // geometry_msgs::Pose2D ipursuer = pursuer_poses[id];
    // for(int i=1;i<=pnum;i++){
    //     if(i != id){
    //         geometry_msgs::Pose2D pursuerx = pursuer_poses[i];
    //         float dis = hypot(pursuerx.y - ipursuer.y, pursuerx.x -  ipursuer.x);
    //         float azimuth = theta2pi(atan2(pursuerx.y - ipursuer.y, pursuerx.x -  ipursuer.x));
    //         int ind = round(azimuth/scan.angle_increment);
    //         if(scan.ranges[ind] < pos.first - robot_radius/2.0 && sqrt(pow(dis,2)+pow(pos.first,2)-2*dis*pos.first*cos(azimuth-pos.second)) < robot_radius)
    //             return 0.0;
    //     }
    // }
    int index = round(posTheta/scan.angle_increment);
    try{
        if(scan.ranges[index] < std::abs(pos.first) || scan.ranges[index] < robot_radius || isNearToObstacle(pos))
            return fmaxi;
        else{
            //(1) Can not turn around  when the wall is ahead
            // return scan.ranges[index] < CCBSOCONFIG.searchDis? exp(-(CCBSOCONFIG.searchDis - scan.ranges[index])/CCBSOCONFIG.searchDis): 0.0;
            //(2) Too close with the wall, it can turn direction when the wall is ahead,
            // but cannot judge the newPos is close to the wall or not. 
            int index_l = index - 1 < 0? scan.ranges.size()-1: index-1;
            int index_h = index + 1 > scan.ranges.size()-1? 0: index+1;
            // Avoid hitting corner, consider near scan
            double mini = scan.ranges[index_l] < scan.ranges[index_h]?scan.ranges[index_l]:scan.ranges[index_h];
            mini = mini < scan.ranges[index]? mini: scan.ranges[index];
            // return scan.ranges[index] < CCBSOCONFIG.searchDis? exp(-(scan.ranges[index]-pos.first)/CCBSOCONFIG.searchDis): 0.0;
            return mini < CCBSOCONFIG.searchDis? exp(-(mini-std::abs(pos.first))/CCBSOCONFIG.searchDis): 0.0;

        }
    }
    catch(...){
        ROS_ERROR("posTheta:%f index:%d",posTheta,index);
    }
    
}

bool CCBSOPlanner::isNearToObstacle(std::pair<float,float>& pos){
    costmap_2d::Costmap2D * costmap_ = costmap_ros_->getCostmap();
    double resolution = costmap_->getResolution();
    double originX = costmap_->getOriginX();
    double originY = costmap_->getOriginY();
    double sizeX = costmap_->getSizeInCellsX();
    double sizeY = costmap_->getSizeInCellsY();

    float nextPose_x = pursuer_poses[id].x + pos.first*cos(pursuer_poses[id].theta+pos.second);
    float nextPose_y = pursuer_poses[id].y + pos.first*sin(pursuer_poses[id].theta+pos.second);
    int nextPose_xl = (int)((nextPose_x-robot_radius-originX)/resolution);
    int nextPose_xh = (int)((nextPose_x+robot_radius-originX)/resolution);
    int nextPose_yl = (int)((nextPose_y-robot_radius-originY)/resolution);
    int nextPose_yh = (int)((nextPose_y+robot_radius-originY)/resolution);
    if(nextPose_xl<0 || nextPose_xh>=sizeX || nextPose_yl<0 || nextPose_yh>=sizeY)
        return true;
    for(unsigned int mx=nextPose_xl;mx<=nextPose_xh;mx++){
        for(unsigned int my=nextPose_yl;my<=nextPose_yh;my++){
            if(costmap_->getCost(mx,my)>costmap_2d::FREE_SPACE)
                return true;
        }
    }
    return false;
}

float CCBSOPlanner::Fpheromone(std::pair<float,float>& pos){
    float value = 0;
    float nextPose_x = pursuer_poses[id].x + pos.first*cos(pursuer_poses[id].theta+pos.second);
    float nextPose_y = pursuer_poses[id].y + pos.first*sin(pursuer_poses[id].theta+pos.second);
    if(trails[id].poses.size()>1){
        //TODO
        //float normlized_time = (trails[id].poses.back().header.stamp - trails[id].poses.front().header.stamp).toSec();
        for(int i=1;i<=pnum;i++){
            //Searching waypoint from new to old
            for(std::vector<geometry_msgs::PoseStamped>::reverse_iterator it = trails[i].poses.rbegin() ; it != trails[i].poses.rend(); ++it){
                float dis, dir, diffy, diffx; 
                diffy = nextPose_y - it->pose.position.y ;
                diffx = nextPose_x - it->pose.position.x ;
                dis = hypot(diffy,diffx);
                tf::Quaternion quat;
                tf::quaternionMsgToTF(it->pose.orientation, quat);
                dir = atan2(diffy,diffx)-tf::getYaw(quat);//pi-(-pi) could happen, so robot is easily make mistake when he is in the negative x-axis
                // dir = theta2pi(atan2(diffy,diffx))-theta2pi(tf::getYaw(quat));//2pi -0  positive x-axis
                if( dis < CCBSOCONFIG.markedDis && (std::abs(dir) < CCBSOCONFIG.markedAngle/2.0 || std::abs(std::abs(dir)-2*PI) < CCBSOCONFIG.markedAngle/2.0)){
                    ros::Duration diff_time = ros::Time::now() - it->header.stamp;
                    // Avoid stuck in visited place
                    // value = CCBSOCONFIG.pheromone*(1+diff_time.toSec()/normlized_time);
                    value = CCBSOCONFIG.pheromone*(1-diff_time.toSec()/CCBSOCONFIG.pheromone_lasting);
                    break;
                }
            }
        }
    }
    return value;
}

// float CCBSOPlanner::Fcruise(std::pair<float,float>& pos){
//     float nextPose_x = pursuer_poses[id].x + pos.first*cos(pursuer_poses[id].theta+pos.second);
//     float nextPose_y = pursuer_poses[id].y + pos.first*sin(pursuer_poses[id].theta+pos.second);
//     // return CCBSOCONFIG.weight_cruiser*(CCBSOCONFIG.cruiser_max - std::max(std::abs(nextPose_y), std::abs(nextPose_x)))/CCBSOCONFIG.cruiser_max;
//     return CCBSOCONFIG.weight_cruiser*hypot(nextPose_x, nextPose_y);
// }

float CCBSOPlanner::FangularAcc(std::pair<float,float>& pos){
    // preference : 
    // id is odd, straight > turn right > turn left
    // id is even, straight > turn left > turn right
    // float value;
    // if(id%2){
    //     value = pos.second > 0? 1: 0;
    // }
    // else{
    //     value = pos.second < 0? 1: 0;
    // }
    float value = pos.second > 0? 1: 0;
    value = CCBSOCONFIG.weight_aa*std::abs(pos.second)*(1+value);
    return value;
}

float CCBSOPlanner::FpotentialCollision(std::pair<float,float>& pos){
    float value = 0.0;
    float individualx = pursuer_poses[id].x+pos.first*cos(pursuer_poses[id].theta+pos.second);
    float individualy = pursuer_poses[id].y+pos.first*sin(pursuer_poses[id].theta+pos.second);
    geometry_msgs::Pose2D ri_vec;
    ri_vec.x = individualx - pursuer_poses[id].x;
    ri_vec.y = individualy - pursuer_poses[id].y;
    ri_vec.theta = atan2(ri_vec.y,ri_vec.x);
    for(int i=1;i<=pnum;i++){
        if(i!=id){
            // ipursuer point to xpursuer
            geometry_msgs::Pose2D rr_vec;
            rr_vec.x = pursuer_poses[i].x - pursuer_poses[id].x;
            rr_vec.y = pursuer_poses[i].y - pursuer_poses[id].y;
            rr_vec.theta = atan2(rr_vec.y, rr_vec.x);
            // xpursuer
            geometry_msgs::Pose2D xr_vec;
            xr_vec.x = cos(pursuer_poses[i].theta);
            xr_vec.y = sin(pursuer_poses[i].theta);
            xr_vec.theta = pursuer_poses[i].theta;
            float rrdis = hypot(rr_vec.x, rr_vec.y);
            ROS_DEBUG_NAMED("FpotentialCollision","tran:%f, rot:%f",pos.first ,pos.second); 
            if(std::abs(pos.first) < 1e-3){
                value += rrdis  < CCBSOCONFIG.searchDis? exp(-rrdis/CCBSOCONFIG.searchDis/2.0) : 0.0;
                ROS_DEBUG_NAMED("FpotentialCollision","case1,rrdis: %f, value: %f", rrdis, value);
            }
            // face to back
            else if( ri_vec.x * rr_vec.x + ri_vec.y * rr_vec.y < 0 ||
               -xr_vec.x * rr_vec.x - xr_vec.y * rr_vec.y < 0){
                float dis = hypot(individualx - pursuer_poses[i].x, individualy - pursuer_poses[i].y);
                value += dis < CCBSOCONFIG.searchDis? exp(-dis/CCBSOCONFIG.searchDis) : 0.0;
                ROS_DEBUG_NAMED("FpotentialCollision","case2,dis: %f, value: %f", dis, value);
            }
            else{// face to face, 
                float rrdis = hypot(rr_vec.x, rr_vec.y);
                float sign1 = rr_vec.x * ri_vec.y  - rr_vec.y * ri_vec.x ;
                float sign2 = rr_vec.x * xr_vec.y  - rr_vec.y * xr_vec.x ;
                float k1 = tan(ri_vec.theta);
                float k2 = tan(xr_vec.theta);
                ROS_DEBUG_NAMED("FpotentialCollision","sign1:%f, sign2: %f, k1: %f, k2: %f", sign1, sign2, k1, k2);
                // same side, has intersection
                if(sign1 * sign2 <= 0) ROS_DEBUG_NAMED("FpotentialCollision","Here1");
                if(abs(k1-k2) <= 1e-2) ROS_DEBUG_NAMED("FpotentialCollision","Here2");
                if(sign1 * sign2 > 0 && abs(k1-k2)>1e-2){
                    float interx = (rr_vec.y+k1*pursuer_poses[id].x-k2*pursuer_poses[i].x)/(k1-k2);
                    float intery = k1*(interx - pursuer_poses[id].x) + pursuer_poses[id].y;
                    // compare the distance of ipursuer-intersection and xpursuer-intersection
                    // if intersection is closer to ipursuer, needs ipursuer to move faster, 
                    // otherwise, move slower
                    float dis_ii = hypot(interx - pursuer_poses[id].x, intery - pursuer_poses[id].y);
                    float dis_ix = hypot(interx - pursuer_poses[i].x, intery - pursuer_poses[i].y);
                    // value += dis < CCBSOCONFIG.searchDis? exp(-dis/CCBSOCONFIG.searchDis) : 0.0;
                    if(dis_ii < CCBSOCONFIG.searchDis){
                        value += (dis_ii > dis_ix ? exp(std::abs(pos.first)/CCBSOCONFIG.searchDis-1):exp(-std::abs(pos.first)/CCBSOCONFIG.searchDis));
                    }
                    ROS_DEBUG_NAMED("FpotentialCollision","case3,dis: %f, value: %f", dis_ii, value);
                }
                // different side
                else if(rrdis<CCBSOCONFIG.searchDis){
                    float dis1 = std::abs(tan(xr_vec.theta - rr_vec.theta)) * rrdis;
                    float dis2 = std::abs(tan(xr_vec.theta - rr_vec.theta)) * rrdis;
                    float dis = dis1 + dis2;
                    value += dis < robot_radius*3? exp(-dis/robot_radius/3) : 0.0;
                    ROS_DEBUG_NAMED("FpotentialCollision","case4,dis: %f, value: %f", dis, value);
                }
            }
        }
    }
    value = value * CCBSOCONFIG.weight_pc;
    ROS_DEBUG_NAMED("FpotentialCollision","weight:%lf, value: %f", CCBSOCONFIG.weight_pc,value);
    return value;
}

float CCBSOPlanner::Fkeeping(std::pair<float,float>& pos){
    geometry_msgs::Pose2D nextPos;
    float theta = pos.first < 0? pos.second + PI: pos.second;
    nextPos.x = pursuer_poses[id].x+pos.first*cos(pursuer_poses[id].theta+pos.second);
    nextPos.y = pursuer_poses[id].y+pos.first*sin(pursuer_poses[id].theta+pos.second);
    nextPos.theta = theta2pi(pursuer_poses[id].theta + 2*pos.second);
    if(targets[id]==-1){
        ROS_ERROR("Keeper don't have a target");
    }
    float target_dir = theta2pi(atan2(evaderStates[targets[id]].y - nextPos.y, evaderStates[targets[id]].x - nextPos.x));
    float value = CCBSOCONFIG.weight_k * std::abs(target_dir - nextPos.theta);
    return value;
}

float CCBSOPlanner::FdisToTarget(std::pair<float,float>& pos){
    geometry_msgs::Pose2D nextPos;
    nextPos.x = pursuer_poses[id].x+pos.first*cos(pursuer_poses[id].theta+pos.second);
    nextPos.y = pursuer_poses[id].y+pos.first*sin(pursuer_poses[id].theta+pos.second);
    float dis = hypot(evaderStates[targets[id]].y - nextPos.y, evaderStates[targets[id]].x - nextPos.x);
    float value = std::abs(dis-CCBSOCONFIG.disToTarget) * CCBSOCONFIG.weight_d;
    return value;
}

void CCBSOPlanner::resetFrequency(double frequency){
    controller_frequency_ = frequency;
}

void CCBSOPlanner::publishLookAheadPoint(std::pair<float,float>& pos, bool switch_){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    // marker.id = id;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = pursuer_poses[id].x+pos.first*cos(pursuer_poses[id].theta+pos.second);
    marker.pose.position.y = pursuer_poses[id].y+pos.first*sin(pursuer_poses[id].theta+pos.second);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    if(switch_){
        marker.lifetime = ros::Duration(1.0);
        pub_checkPoint.publish(marker);
    }else{
        marker.lifetime = ros::Duration(0.1);
        pub_lookAheadPoint.publish(marker);
    }
}

void CCBSOPlanner::publishState(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = pursuer_poses[id].x;
    marker.pose.position.y = pursuer_poses[id].y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.01;
    marker.color.a = 0.2; 
    marker.lifetime = ros::Duration(0.1);
    switch(state_){
        case SEARCHING:{//blue
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            break;
        }
        case KEEPING:{//cyan
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            break;
        }
        case TRACKING:{//green
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
        }
        case FOLLOWING:{//red
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            break;
        }
    }
    pub_state.publish(marker);
}

bool CCBSOPlanner::isGoalReached(){}

bool CCBSOPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){}






















