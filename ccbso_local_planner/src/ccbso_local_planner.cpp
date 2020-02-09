
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
    for(int i=1;i<=pnum;i++){
        if(i!=id){
            ros::Subscriber sub_trail = nh.subscribe<nav_msgs::Path>(gns+std::to_string(i)+"/pheromoneTrail",1,boost::bind(&CCBSOPlanner::trailCallback, this, _1, i));
            sub_trails.push_back(sub_trail);
        }
    }
    pub_pheromoneTrail = nh.advertise<nav_msgs::Path>(ns+"/pheromoneTrail",1);
    pub_pheromoneTrail_ = nh.advertise<geometry_msgs::PoseArray>(ns+"/pheromoneTrail_",1);
    pub_lookAheadPoint = nh.advertise<visualization_msgs::Marker>(ns+"/lookAheadPoint",1);
    pub_checkPoint = nh.advertise<visualization_msgs::Marker>(ns+"/checkPoint",1);
    srv_check_fitness = nh.advertiseService("check_fitness", &CCBSOPlanner::checkFitnessService, this);

    ros::NodeHandle private_nh("/" + name);
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

void CCBSOPlanner::trailCallback(const nav_msgs::PathConstPtr &msg, int i){
    std::unique_lock<std::recursive_mutex> lock_trails(mutex_trails);
    trails[i] = *msg;
}

bool CCBSOPlanner::checkFitnessService(check_fitness::Request &req,
                                       check_fitness::Response &res){
    // state_ = res.state
    std::pair<float,float> pos(req.delta_trans, req.delta_rot1);
    switch(state_){
        case SEARCHING:{
            res.Fpheromone = Fpheromone(pos);
            res.Fobstacle =  Fobstacle(pos);
            res.Fcruise = Fcruise(pos);
            res.FangularAcc = FangularAcc(pos);
            break;
        }
        default:{
            break;
        }
    }
    res.sum = res.Fpheromone+res.Fobstacle+res.Fcruise+res.FangularAcc;
    std::unique_lock<std::recursive_mutex> lock_pursuers(mutex_pursuers);
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
    // get pursuers' poses
    std::unique_lock<std::recursive_mutex> lock_pursuers(mutex_pursuers);
    std::unique_lock<std::recursive_mutex> lock_trails(mutex_trails);
    geometry_msgs::PoseStamped currPose;
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
    
    // Publish trail
    trails[id].header.stamp = ros::Time::now();
    if(trails[id].poses.empty()){
        trails[id].header.frame_id = fixed_frame_;
        trails[id].header.frame_id = fixed_frame_;
        trails[id].poses.push_back(currPose);
    }
    else{
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

    // Access pursuers' targets
    ros::NodeHandle nh;
    for(int i=1;i<=pnum;i++){
        int target_idx;
        nh.getParam(gns+std::to_string(i)+"/target_id", target_idx);
        targets[i] = target_idx;
    }
    
    std::pair<float,float> nextPos;//delta_trans,delta_rot1
    bsoSelection(nextPos);
    
    if(std::abs(nextPos.first)>1e-2){
        if(std::abs(nextPos.second)>1e-2){
            cmd_vel.linear.x = nextPos.first*nextPos.second/sin(nextPos.second);
        }
        else{
            cmd_vel.linear.x = nextPos.first;
        }
    }
    if(std::abs(cmd_vel.linear.x) > CCBSOCONFIG.max_vel_x){
        cmd_vel.linear.x = cmd_vel.linear.x > 0 ? CCBSOCONFIG.max_vel_x: -CCBSOCONFIG.max_vel_x;
    }
    cmd_vel.angular.z = nextPos.second;
    
    publishLookAheadPoint(nextPos);
    // ROS_INFO("Next pose:%f,%f; cmd_vel:%lf; ",nextPos.first,nextPos.second,cmd_vel.linear.x);
}

void CCBSOPlanner::bsoSelection(std::pair<float,float>& nextPos){
    // Roulette Wheel Selection
    // Sampling within a sector
    Eigen::Matrix<float, 11, 1> pros;
    float sum;
    if(state_ == SEARCHING)
        sum = 1;
    else
        sum = 5;//give ro=0 bigger propertity
    for(int i=0;i<=10;i++){
        sum += i;
        pros(i,0)=sum;
    }
    pros = pros/sum;

    // odometry motion model
    std::vector<std::pair<float,float>> nextPoses;//delta_trans,delta_rot1
    std::multiset<std::pair<double,int>> individuals;//fitness,index, Ascending order
    for(int i=0;i<CCBSOCONFIG.psize;i++){
        double p = rand()/double(RAND_MAX);
        for(int j=0;j<=10;j++){
            if(p <= pros(j,0)){
                //std::pair<float,float> pos(CCBSOCONFIG.searchDis/10.0*j,rand()%20*CCBSOCONFIG.searchAngle/20.0 - CCBSOCONFIG.searchAngle/2.0);
                std::pair<float,float> pos(CCBSOCONFIG.searchDis/10.0*j,i*CCBSOCONFIG.searchAngle/CCBSOCONFIG.psize - CCBSOCONFIG.searchAngle/2.0);
                std::pair<double,int> individual(calFitness(pos),i);
                nextPoses.push_back(pos);
                individuals.emplace(individual);
                break;
            }
        }
    }

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
        if(newPos.second > CCBSOCONFIG.searchAngle / 2.0 || newPos.first > CCBSOCONFIG.searchDis){
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
        //Avoid hitting wall
        std::pair<double,int> bestIndi;
        getNthElement(individuals.begin(),0,bestIndi);
        if(bestIndi.first > fmaxi){
            newPos.first = 0.0;
            newPos.second = rand()%20*CCBSOCONFIG.searchAngle/20.0 - CCBSOCONFIG.searchAngle/2.0;
            newFitness = calFitness(newPos);
            std::pair<double,int> newIndi(newFitness,bestIndi.second);
            individuals.erase(bestIndi);
            individuals.emplace(newIndi);
            nextPoses[bestIndi.second] = newPos;
        }
    }
    // choose the best one
    int ind;
    if(individuals.size()){
        ind = (*individuals.begin()).second;
        nextPos = nextPoses[ind];
        std::pair<double,int> worseIndi;
        getNthElement(individuals.begin(),CCBSOCONFIG.psize-1,worseIndi);
        ROS_INFO("distance:%f, angle:%f, value:%lf, worse value:%lf",nextPos.first,nextPos.second,(*individuals.begin()).first,worseIndi.first);
        // ROS_DEBUG_NAMED("ccbso","%d, distance:%f, angle:%f",ind,nextPos.first,nextPos.second);
        switch(state_){
            case SEARCHING:{
                ROS_INFO("Fpheromone:%f, Fobstacle:%f, Fcruise:%f, FangularAcc:%f",Fpheromone(nextPos),Fobstacle(nextPos),Fcruise(nextPos), FangularAcc(nextPos));
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
            // fitness += Fpheromone(pos) + Fobstacle(pos) + Fcruise(pos) + FangularAcc(pos);
            fitness += Fpheromone(pos) + Fobstacle(pos) + FangularAcc(pos);
            break;
        }
        default:{

        }
    }
    return fitness;
}

float CCBSOPlanner::Fobstacle(std::pair<float,float>& pos){
    if(!received_scan) return 0.0;
    std::unique_lock<std::recursive_mutex> lock_tags(mutex_scan);
    // if there's pursuer between robot and pos, return 0;
    float posTheta = theta2pi(pos.second);
    geometry_msgs::Pose2D ipursuer = pursuer_poses[id];
    for(int i=1;i<=pnum;i++){
        if(i != id){
            geometry_msgs::Pose2D pursuerx = pursuer_poses[i];
            float dis = hypot(pursuerx.y - ipursuer.y, pursuerx.x -  ipursuer.x);
            float azimuth = theta2pi(atan2(pursuerx.y - ipursuer.y, pursuerx.x -  ipursuer.x));
            int ind = round(azimuth/scan.angle_increment);
            if(scan.ranges[ind] < pos.first - robot_radius/2.0 && sqrt(pow(dis,2)+pow(pos.first,2)-2*dis*pos.first*cos(azimuth-pos.second)) < robot_radius)
                return 0.0;
        }
    }
    int index = round(posTheta/scan.angle_increment);
    if(scan.ranges[index] < pos.first || scan.ranges[index] < robot_radius || isNearToObstacle(pos))
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
        return mini < CCBSOCONFIG.searchDis? exp(-(mini-pos.first)/CCBSOCONFIG.searchDis): 0.0;

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
        float normlized_time = (trails[id].poses.back().header.stamp - trails[id].poses.front().header.stamp).toSec();
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
                    value = CCBSOCONFIG.pheromone*(2-diff_time.toSec()/normlized_time);
                    break;
                }
            }
        }
    }
    return value;
}

float CCBSOPlanner::Fcruise(std::pair<float,float>& pos){
    float nextPose_x = pursuer_poses[id].x + pos.first*cos(pursuer_poses[id].theta+pos.second);
    float nextPose_y = pursuer_poses[id].y + pos.first*sin(pursuer_poses[id].theta+pos.second);
    // return CCBSOCONFIG.weight_cruiser*(CCBSOCONFIG.cruiser_max - std::max(std::abs(nextPose_y), std::abs(nextPose_x)))/CCBSOCONFIG.cruiser_max;
    return CCBSOCONFIG.weight_cruiser*hypot(nextPose_x, nextPose_y);
}

float CCBSOPlanner::FangularAcc(std::pair<float,float>& pos){
    //preference : straight > turn right > turn left
    float value = pos.second > 0? 1: 0;
    value = CCBSOCONFIG.weight_aa*std::abs(pos.second)*(1+value);
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

bool CCBSOPlanner::isGoalReached(){}

bool CCBSOPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){}






















