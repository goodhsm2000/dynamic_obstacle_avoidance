// /*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2009, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of Willow Garage, Inc. nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * Author: Eitan Marder-Eppstein
// * Modified by: Jeeseon Kim
// *********************************************************************/

// #include <dwa_local_planner2/dwa_planner_ros2.h>
// #include <Eigen/Core>
// #include <cmath>

// #include <ros/console.h>

// #include <pluginlib/class_list_macros.h>

// #include <base_local_planner/goal_functions.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <tf2/utils.h>

// //#!
// #include <algorithm>
// #include <nav_msgs/GetMap.h>
// #define MAX_VAL 10000
// #define MIN_VAL -10000
// #define COLL_PROB_ALPHA 0.8
// #define COLL_PROB_BETA  0.05
// #define SIGMA   0.4
// #define CORR    1/sqrt(2 * M_PI * SIGMA)
// #define GAUSS_ALPHA  0.1
// #define EPSILON 0.0001

// bool no_obstacles_ = false;
// //#!

// //register this planner as a BaseLocalPlanner plugin
// PLUGINLIB_EXPORT_CLASS(dwa_local_planner2::DWAPlannerROS2, nav_core::BaseLocalPlanner)

// using namespace std;

// namespace dwa_local_planner2 {

//   void DWAPlannerROS2::reconfigureCB(DWAPlanner2Config &config, uint32_t level) {
//       if (setup_ && config.restore_defaults) {
//         config = default_config_;
//         config.restore_defaults = false;
//       }
//       if ( ! setup_) {
//         default_config_ = config;
//         setup_ = true;
//       }

//       // update generic local planner params
//       base_local_planner::LocalPlannerLimits limits;
//       limits.max_vel_trans = config.max_vel_trans;
//       limits.min_vel_trans = config.min_vel_trans;
//       limits.max_vel_x = config.max_vel_x;
//       limits.min_vel_x = config.min_vel_x;
//       limits.max_vel_y = config.max_vel_y;
//       limits.min_vel_y = config.min_vel_y;
//       limits.max_vel_theta = config.max_vel_theta;
//       limits.min_vel_theta = config.min_vel_theta;
//       limits.acc_lim_x = config.acc_lim_x;
//       limits.acc_lim_y = config.acc_lim_y;
//       limits.acc_lim_theta = config.acc_lim_theta;
//       limits.acc_lim_trans = config.acc_lim_trans;
//       limits.xy_goal_tolerance = config.xy_goal_tolerance;
//       limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
//       limits.prune_plan = config.prune_plan;
//       limits.trans_stopped_vel = config.trans_stopped_vel;
//       limits.theta_stopped_vel = config.theta_stopped_vel;
//       planner_util_.reconfigureCB(limits, config.restore_defaults);

//       // update dwa specific configuration
//       dp_->reconfigure(config);
//   }

//   DWAPlannerROS2::DWAPlannerROS2() : initialized_(false),
//       odom_helper_("odom"), setup_(false) {

//   }

//   // map 
//   void DWAPlannerROS2::mapProcess(const nav_msgs::OccupancyGrid& map){

//       ROS_INFO_STREAM(map.header.frame_id);
//       ROS_INFO("map initialize");

//       map_inf map_info;
//       map_info.size_x = map.info.width;
//       map_info.size_y = map.info.height;
//       map_info.scale = map.info.resolution;
//       map_info.origin_x = map.info.origin.position.x + (map_info.size_x / 2) * map_info.scale;
//       map_info.origin_y = map.info.origin.position.y + (map_info.size_y / 2) * map_info.scale;

//       float w_x, w_y;

//       map_position_.clear();

//       for (std::size_t j=0; j < map_info.size_y; j++) {
//           for (std::size_t i=0; i < map_info.size_x; i++) {
//               if(map.data[MAP_INDEX(map_info, i, j)]==100){
//                   // convert to world position
//                   w_x = MAP_WXGX(map_info, i);
//                   w_y = MAP_WYGY(map_info, j);
//                   map_position_.push_back(std::make_pair(w_x, w_y));
//                   // ROS_INFO("map pos w_x: %f, w_y: %f", w_x, w_y);
//               }
//           }
//       }
//   }

//   // 장애물 이전 좌표 저장(최대 10개)
//   float obstacles_prev[10][2];
//   float time_to_collide[10][2];
//   int cnt_ = 1;
  
//   // 충돌시간 계산, 안전 확률 계산 함수
//   void DWAPlannerROS2::computeTTC(){
//     float obs_curr_x, obs_curr_y;     //obstacle position

//     if(cnt_ % 2 != 0){
      
//         findObstacles(); 

//         if(no_obstacles_){
//             // 장애물이 없으면 안전 확률을 최대로 설정하고 반환
//             for(int idx = 0; idx < rcv_msg_.ranges.size(); idx++){
//                 robot_safe_dir_.push_back(1.0);
//                 ROS_INFO("safe_dir push if no obstacle");
//             }
//             dp_->setProbability(robot_safe_dir_);
            
//             //clear
//             // curr_obs: 현재 인지한 동적 장애물
//             curr_obs_.clear();
//             obs_direction_.clear();
//             obs_safe_prob_.clear();
//             return;
//         }

//         // 로봇 벡터
//         double robot_vec[2] = {current_pose_.pose.position.x - previous_pose_.pose.position.x,
//                                current_pose_.pose.position.y - previous_pose_.pose.position.y};   //robot vec
        
//         // 동적 장애물 벡터들 0으로 초기화
//         float obs_vec[2] = {0, 0};
//         // 상대 속도
//         float v_rel[2];
        
//         // 
//         for(int idx = 0; idx < curr_obs_.size(); idx++){
//             obs_curr_x = curr_obs_[idx].first;
//             obs_curr_y = curr_obs_[idx].second;

//             float min_dist_ = MAX_VAL;
//             int min_idx;

//             //find previous obstacle j who has minimum distance with obstacle idx
//             for(int j = 0; j < 10 ; j++){
//                 float dist_ = sqrt(powf(obs_curr_x - obstacles_prev[j][0], 2.0) + powf(obs_curr_y - obstacles_prev[j][1], 2.0));
//                 if(dist_ < min_dist_){
//                     min_dist_ = dist_;
//                     min_idx = j;
//                 }
//             }
//             obs_vec[0] = obs_curr_x - obstacles_prev[min_idx][0];
//             obs_vec[1] = obs_curr_y - obstacles_prev[min_idx][1];   //obs vec

//             v_rel[0] = robot_vec[0] - obs_vec[0];
//             v_rel[1] = robot_vec[1] - obs_vec[1];

//             float robot_vec_s  = sqrt(powf(robot_vec[0], 2.0) + powf(robot_vec[1], 2.0));
//             float obs_vec_s = sqrt(powf(obs_vec[0], 2.0) + powf(obs_vec[1], 2.0));
//             float f_dot = robot_vec[0] * obs_vec[0] + robot_vec[1] * obs_vec[1];    //inner product
//             float cos_theta = f_dot / (robot_vec_s * obs_vec_s);    //cosine theta between 2 vec

//             float d_rel_s = sqrt(powf(obs_curr_x - current_pose_.pose.position.x, 2.0)
//                                + powf(obs_curr_y - current_pose_.pose.position.y, 2.0));
//             float v_rel_s = sqrt(powf(v_rel[0], 2.0) + powf(v_rel[1], 2.0));

//             // 충돌 시간 구하기
//             float ttc = d_rel_s / (v_rel_s * cos_theta);

            
            
//             // 회피 확률 구하기
//             float safety_prob = 1 - COLL_PROB_ALPHA * powf(M_E, -1 * (COLL_PROB_BETA * ttc) * (COLL_PROB_BETA * ttc) );

//             obs_safe_prob_.push_back(safety_prob);
//        }//end for

//         // 이전 상태 업데이트
//         //update previous state to current state
//         previous_pose_ = current_pose_;

//         for(int idx = 0; idx < 10; idx++){
//             if(idx < curr_obs_.size()){
//                 obstacles_prev[idx][0] = curr_obs_[idx].first;
//                 obstacles_prev[idx][1] = curr_obs_[idx].second;
//             }
//             else{
//                 obstacles_prev[idx][0] = MAX_VAL;
//                 obstacles_prev[idx][1] = MAX_VAL;
//             }
//         }

//         double gauss_prob;
//         double min_prob = MAX_VAL;

//         //compute safe probability for all directions
//         // 모든 방향에 대해서 회피 확률을 계산해 저장, 이때 장애물과 정면각도의 주변 각도도 회피 확률을 낮게 설정 
//         for(int idx = 0; idx < rcv_msg_.ranges.size(); idx++){
//             min_prob = MAX_VAL;
//             for(int j = 0;  j < curr_obs_.size(); j++){
//                 gauss_prob = 1 - (CORR * powf(M_E, -1 * (GAUSS_ALPHA*(idx - obs_direction_[j])) * (GAUSS_ALPHA*(idx - obs_direction_[j])) / (2*SIGMA))
//                          * (1/CORR)) * (1 - obs_safe_prob_[j]);
//                 if(min_prob > gauss_prob){
//                     min_prob = gauss_prob;
//                 }
//             }
//             robot_safe_dir_.push_back(min_prob);
//         }

//         int min_prob_idx;
//         int max_prob_idx;
     
//         if( !((1 - robot_safe_dir_[0]) < EPSILON && (1 - robot_safe_dir_[rcv_msg_.ranges.size() - 1]) < EPSILON) ){ //obs in front of robot head direction
     
//             if((1 - robot_safe_dir_[0]) >= EPSILON){ // 0 ~
//                 min_prob_idx = 0;   max_prob_idx = 0;

//                 for(int i = 0 ; i < rcv_msg_.ranges.size(); i++){   //find index with minimum prob value, last index which robot_safe_dir_[index] < 1
//                     if(robot_safe_dir_[i] < robot_safe_dir_[min_prob_idx]){
//                         min_prob_idx = i;
//                     }
//                     if((1 - robot_safe_dir_[i]) < EPSILON){
//                         max_prob_idx = i;
//                         break;
//                     }
//                 }

//                 for(int i = min_prob_idx + 1; i <= max_prob_idx; i++){  //update probabilities for the opposite side
//                     if(min_prob_idx - (i - min_prob_idx) < 0){
//                         int offset = min_prob_idx - (i - min_prob_idx);
//                         robot_safe_dir_[rcv_msg_.ranges.size() + offset] = robot_safe_dir_[i];
//                     }
//                 }
//             }

//             else if((1 - robot_safe_dir_[rcv_msg_.ranges.size() - 1]) >= EPSILON){   // ~ 359
//                 int max_i = rcv_msg_.ranges.size() - 1;
//                 min_prob_idx = max_i;   max_prob_idx = max_i;

//                 for(int i = max_i ; i >= 0; i--){   //find index with minimum prob value, last index which robot_safe_dir_[index] < 1
                     
//                     if(robot_safe_dir_[i] < robot_safe_dir_[min_prob_idx]){
//                         min_prob_idx = i;
//                     }
//                     if((1 - robot_safe_dir_[i]) < EPSILON){
//                         max_prob_idx = i;
//                         break;
//                     }
//                 }

//                 for(int i = min_prob_idx - 1; i >= max_prob_idx; i--){  //update probabilities for the opposite side
//                     if(min_prob_idx - (i - min_prob_idx) > max_i){
//                         int offset = min_prob_idx - (i - min_prob_idx);
//                         robot_safe_dir_[offset - rcv_msg_.ranges.size()] = robot_safe_dir_[i];
//                     }
//                 }
//             }
//         }

//         //send robot_safe_dir_ to base_local_planner::ProbabilityCostFunction
//         dp_->setProbability(robot_safe_dir_);

//         // for(int i=0; i<robot_safe_dir_.size(); i++)
//         // {
//         //   ROS_INFO("robot safe dir is %f",robot_safe_dir_[i]);
//         //   ROS_INFO("curr_obs %ld", curr_obs_.size() );  
//         // }

  
//         //clear
//         curr_obs_.clear();
//         obs_direction_.clear();
//         obs_safe_prob_.clear();
//     }
//     cnt_++;
//   }

//   // 동적 장애물 찾기 & 중심점 찾기
//   void DWAPlannerROS2::findObstacles(){
//       float pt_x, pt_y;
//       float rb_yaw; //robot yaw
//       float w_x = 0, w_y = 0, dist_sq = 0;

//       int obs_count = 0;
//       float min_dist;
//       bool dynamic;

//       int *p1, *p2, *p3;
//       int *grp_start_end;
//       float *center_pos;

//       rb_yaw = tf2::getYaw(current_pose_.pose.orientation);

//       if(rb_yaw < 0){
//           rb_yaw += 2 * M_PI;
//           // ROS_INFO("robot current yaw is minus");
//       }

//       // obs_idx_ : 동적 장애물로 감지한 레이더 각도 index
//       //find dynamic points, add dynamic points to obs_idx_
//       for(int i = 0; i < rcv_msg_.ranges.size() ; i++){
//           if(rcv_msg_.ranges[i] < rcv_msg_.range_max){
//               // ROS_INFO("lidar distance < lidar max");
//               pt_x = current_pose_.pose.position.x
//                       + rcv_msg_.ranges[i] * std::cos(rcv_msg_.angle_increment * i + rb_yaw);
//               pt_y = current_pose_.pose.position.y
//                       + rcv_msg_.ranges[i] * std::sin(rcv_msg_.angle_increment * i + rb_yaw);    //sensed position
//               // ROS_INFO("(pt_x, pt_y) : lidar absolute position in map");
//               int j;
//               for(j = 0; j < map_position_.size(); j++){

//                   w_x = map_position_[j].first;     //world map position ??
//                   w_y = map_position_[j].second;
//                   dist_sq = sqrt(powf(w_x - pt_x, 2.0) + powf(w_y - pt_y, 2.0));
//                   ROS_INFO("distance: %f ", dist_sq);
//                   if(dist_sq <= 5.0){ //near map
//                       break;
//                       // ROS_INFO("dist_sq <= 0.20 : lidar point is near by static map -> static obstacle");
//                   }
//               }
//               if(j == map_position_.size()){
//                   dynamic = true;
//                   // ROS_INFO("no static obstacle");
//               }
//               if(dynamic){
//                   obs_idx_.push_back(i);
//                   // ROS_INFO("lidar point index i add to dynamic obstacle");
//               }
//           }
//           dynamic = false;
//       }

//       no_obstacles_ = false;
//       if(obs_idx_.size() < 1){
//           no_obstacles_ = true;
//           obs_idx_.clear();
//           return;
//       }
//       // ROS_INFO("count dynmaic obstacle: %ld", obs_idx_.size());
//       int former_idx = obs_idx_.at(0);

//       // 인덱스 간의 차이가 1이 아닌 경우, 즉 연속되지 않은 인덱스일 경우에만 obs_count를 증가
//       for (std::vector<int>::iterator itr = obs_idx_.begin(); itr != obs_idx_.end(); ++itr) {
//           // ROS_INFO("difference: %d", *itr - former_idx);
//           if(*itr - former_idx != 1){
//               obs_count++;
//           }
//           former_idx = *itr;
//           // ROS_INFO("count dynmaic obstacle group by classifying discrete points ");
//       }

//       ROS_INFO("count dynmaic obstacle: %d", obs_count);
//       // 라이더의 시작, 최소, 끝 점 3개를 이용해 외심으로 중심 구하기
//       //allocate index array for each obstacle
//       p1 = new int[obs_count];    //start
//       p2 = new int[obs_count];    //min
//       p3 = new int[obs_count];    //end
//       grp_start_end = new int[obs_count * 2];

//       for (int i = 0; i < obs_count; i++) {
//           p2[i] = 0;  // 또는 다른 적절한 초기값으로 설정
//       }

//       //fill grp_start_end[]: [i*2]: start idx for i-th obs / [i*2+1]: end idx for i-th obs
//       int sep_idx_ = 0;
//       grp_start_end[sep_idx_] = obs_idx_.front();
//       grp_start_end[obs_count * 2 - 1] = obs_idx_.back();

//       former_idx = obs_idx_.at(0);
      

//       for (std::vector<int>::iterator itr = obs_idx_.begin() + 1; itr != obs_idx_.end(); ++itr) {
//           if(*itr - former_idx != 1){ //new obstacle
//               sep_idx_++;
//               grp_start_end[sep_idx_] = *(--itr);
//               sep_idx_++;
//               grp_start_end[sep_idx_] = *(++itr);
//           }
//           former_idx = *itr;
//       }

//       // p1, p2, p3에 각각 장애물에 해당하는 라이더 각도 저장
//       //find lsr index for each obstacle
//       for (int i = 0; i < obs_count; i++) {
//           p1[i] = grp_start_end[i*2];     //start index
//           p3[i] = grp_start_end[i*2+1];   //end index
//           min_dist = rcv_msg_.range_max;

//           for (int j = p1[i]; j <= p3[i]; j++) {
//               if(rcv_msg_.ranges[j] < min_dist){   //find index with minimum distance
//                   min_dist = rcv_msg_.ranges[j];
//                   p2[i] = j; // j is index, not distance value
//               }
//           }
//       }

//       //Obstacle exists at robot head direction (remove in case for duplicate count)
//       if(obs_idx_.front() == 0 && obs_idx_.back() == rcv_msg_.ranges.size() - 1){
//           if(rcv_msg_.ranges[p2[0]] >= rcv_msg_.ranges[p2[obs_count - 1]]){
//              p2[0] = p2[obs_count - 1];
//           }
//           p1[0] = p1[obs_count - 1];
//           p1[obs_count - 1] = 0;
//           p2[obs_count - 1] = 0;
//           p3[obs_count - 1] = 0;

//           obs_count--;
//       }
    

//       center_pos = new float[obs_count * 2];

//       //form triangle with p1, p2, p3 and assume center position of obstacles
//       for (int i = 0; i < obs_count; i++) {
          
//           float tri_a_x, tri_a_y, tri_c_x, tri_c_y, tri_b_x, tri_b_y;
//           float a_vec_x, a_vec_y;
//           float b_vec_x, b_vec_y;
//           float a_vec_sq, b_vec_sq, a_b_in;
//           float p, q;

//           // ROS_INFO("p1[i] %d", p1[i]);
//           // ROS_INFO("p2[i] %d", p2[i]);
//           // ROS_INFO("p3[i] %d", p3[i]);
//           // ROS_INFO("lidar %f", rcv_msg_.ranges[i]);

//           tri_a_x = rcv_msg_.ranges[p1[i]] * std::cos(rcv_msg_.angle_increment * p1[i]);    //A
//           tri_a_y = rcv_msg_.ranges[p1[i]] * std::sin(rcv_msg_.angle_increment * p1[i]);
//           tri_c_x = rcv_msg_.ranges[p2[i]] * std::cos(rcv_msg_.angle_increment * p2[i]);    //C
//           tri_c_y = rcv_msg_.ranges[p2[i]] * std::sin(rcv_msg_.angle_increment * p2[i]);
//           tri_b_x = rcv_msg_.ranges[p3[i]] * std::cos(rcv_msg_.angle_increment * p3[i]);    //B
//           tri_b_y = rcv_msg_.ranges[p3[i]] * std::sin(rcv_msg_.angle_increment * p3[i]);

//           a_vec_x = tri_a_x - tri_c_x; //CA == OA - OC
//           a_vec_y = tri_a_y - tri_c_y;
//           b_vec_x = tri_b_x - tri_c_x; //CB == OB - OC
//           b_vec_y = tri_b_y - tri_c_y;

//           a_vec_sq = a_vec_x * a_vec_x + a_vec_y * a_vec_y;   //  |CA|^2
//           b_vec_sq = b_vec_x * b_vec_x + b_vec_y * b_vec_y;   //  |CB|^2
//           a_b_in = a_vec_x * b_vec_x + a_vec_y * b_vec_y;     //  CA dot CB

//           p = 1 / (a_vec_sq * a_b_in - b_vec_sq * a_b_in) * (a_b_in * 0.5 * a_vec_sq - a_b_in * 0.5 * b_vec_sq); //inverse matrix
//           q = 1 / (a_vec_sq * a_b_in - b_vec_sq * a_b_in) * (-1*b_vec_sq * 0.5 * a_vec_sq + a_vec_sq * 0.5 * b_vec_sq);

//           center_pos[2*i] = tri_c_x + p * a_vec_x + q * b_vec_x;
//           center_pos[2*i + 1] = tri_c_y + p * a_vec_y + q * b_vec_y;

//           if(std::isnan(center_pos[2 * i + 1]) || std::isnan(center_pos[2 * i])){   //remove sensed obstacle if assumed position is nan value
//                 //obs_count_mod--;
//           }
//           else{
//               // 장애물이 있는 방향을 나타내는 인덱스 즉, 장애물이 로봇에서 가장 가까운 지점의 인덱스를 obs_direction_에 추가
//               curr_obs_.push_back(make_pair(current_pose_.pose.position.x + center_pos[2*i], current_pose_.pose.position.y + center_pos[2*i + 1]));    //obs position
//               ROS_INFO("%d th obstacle center axis x: %f, y: %f", i+1, current_pose_.pose.position.x + center_pos[2*i], current_pose_.pose.position.y + center_pos[2*i + 1]);
//               obs_direction_.push_back(p2[i]);   //direction of min
//           }
//       }

//       delete [] p1;
//       delete [] p2;
//       delete [] p3;
//       delete [] grp_start_end;
//       delete [] center_pos;

//       p1 = NULL;
//       p2 = NULL;
//       p3 = NULL;
//       grp_start_end = NULL;
//       center_pos = NULL;

//       obs_idx_.clear();
//   }

//   void DWAPlannerROS2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
//       //laser data to rcv_msg_
//       rcv_msg_.header = msg->header;
//       rcv_msg_.angle_min = msg->angle_min;
//       rcv_msg_.angle_max = msg->angle_max;
//       rcv_msg_.angle_increment = msg->angle_increment;
//       rcv_msg_.time_increment = msg->time_increment;
//       rcv_msg_.scan_time = msg->scan_time;
//       rcv_msg_.range_min = msg->range_min;
//       rcv_msg_.range_max = msg->range_max;
//       rcv_msg_.ranges = msg->ranges;
//       rcv_msg_.intensities = msg->intensities;
//   }

//   void DWAPlannerROS2::initialize(
//       std::string name,
//       tf2_ros::Buffer* tf,
//       costmap_2d::Costmap2DROS* costmap_ros) {
        
//     if (! isInitialized()) {
      
//       // ROS 노드와 퍼블리셔, TF(Buffer) 및 Costmap 관련 객체들을 초기화
//       ros::NodeHandle private_nh("~/" + name);
//       g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
//       l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
//       tf_ = tf;
//       costmap_ros_ = costmap_ros;
//       costmap_ros_->getRobotPose(current_pose_);


//       // make sure to update the costmap we'll use for this cycle
//       costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

//       planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

//       //dp_ 가 실제 알고리즘이 돌아가는 dwa.cpp 파일의 함수
//       dp_ = boost::shared_ptr<DWAPlanner2>(new DWAPlanner2(name, &planner_util_));

//       if( private_nh.getParam( "odom_topic", odom_topic_ ))
//       {
//         odom_helper_.setOdomTopic( "/odom" );
//       }
      
//       //latchedStopRotateController_.initialize(name);

//       initialized_ = true;

//       // dynamic_reconfigure를 사용하여 파라미터를 런타임에서 동적으로 변경할 수 있도록 함
//       dsrv_ = new dynamic_reconfigure::Server<DWAPlanner2Config>(private_nh);
//       // reconfigureCB 함수는 local planner의 값을 바꾸도록 만들어줌
//       // dynamic_reconfigure에 의해 호출되는 함수로, 매개변수가 변경될 때마다 호출
//       dynamic_reconfigure::Server<DWAPlanner2Config>::CallbackType cb = boost::bind(&DWAPlannerROS2::reconfigureCB, this, _1, _2);
//       dsrv_->setCallback(cb);


//       // scan 값을 subscribe 함
//       scan_sub = private_nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &DWAPlannerROS2::scanCallBack, this);


//       nav_msgs::GetMap::Request  req;
//       nav_msgs::GetMap::Response resp;
//       ROS_INFO("Requesting the map..");

//       while(!ros::service::call("static_map", req, resp))
//       {
//         ROS_WARN("Request for map failed; trying again...");
//       }
//       ROS_INFO("Recieve the map!"); 

//       // 현재 static map 받아와서 MapProcess 함수로 넘기기
//       current_map_ = resp.map;
//       mapProcess(current_map_);
//       //#!

//     }
//     else{
//       ROS_WARN("This planner has already been initialized, doing nothing.");
//     }
//   }

//   bool DWAPlannerROS2::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
//     if (! isInitialized()) {
//       ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
//       return false;
//     }
//     //when we get a new plan, we also want to clear any latch we may have on goal tolerances
//     latchedStopRotateController_.resetLatching();
//     // ROS_INFO(orig_global_plan)
//     ROS_INFO("Got new plan1");
//     return dp_->setPlan(orig_global_plan);
//   }

//   bool DWAPlannerROS2::isGoalReached() {
//     if (! isInitialized()) {
//       ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
//       return false;
//     }
//     if ( ! costmap_ros_->getRobotPose(current_pose_)) {
//       ROS_ERROR("Could not get robot pose");
//       return false;
//     }

//     if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
//       ROS_INFO("Goal reached");
//       return true;
//     } else {
//       return false;
//     }
//   }

//   void DWAPlannerROS2::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//     base_local_planner::publishPlan(path, l_plan_pub_);
//   }


//   void DWAPlannerROS2::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//     base_local_planner::publishPlan(path, g_plan_pub_);
//   }

//   DWAPlannerROS2::~DWAPlannerROS2(){
//     //make sure to clean things up
//     delete dsrv_;
//   }


//   bool DWAPlannerROS2::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {

//     // dynamic window sampling approach to get useful velocity commands
//     if(! isInitialized()){
//       ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
//       return false;
//     }

//     geometry_msgs::PoseStamped robot_vel;
//     odom_helper_.getRobotVel(robot_vel);

//     /* For timing uncomment
//     struct timeval start, end;
//     double start_t, end_t, t_diff;
//     gettimeofday(&start, NULL);
//     */

//     //compute what trajectory to drive along
//     geometry_msgs::PoseStamped drive_cmds;
//     drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

//     //


//     // call with updated footprint
//     base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
//     //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

//     /* For timing uncomment
//     gettimeofday(&end, NULL);
//     start_t = start.tv_sec + double(start.tv_usec) / 1e6;
//     end_t = end.tv_sec + double(end.tv_usec) / 1e6;
//     t_diff = end_t - start_t;
//     ROS_INFO("Cycle time: %.9f", t_diff);
//     */

//     //pass along drive commands
//     cmd_vel.linear.x = drive_cmds.pose.position.x;
//     cmd_vel.linear.y = drive_cmds.pose.position.y;
//     cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

//     //if we cannot move... tell someone
//     std::vector<geometry_msgs::PoseStamped> local_plan;
//     if(path.cost_ < 0) {
//       ROS_DEBUG_NAMED("dwa_local_planner2",
//           "The dwa local planner2 failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
//       local_plan.clear();
      
//       publishLocalPlan(local_plan);
//       return false;
//     }

//     // ROS_INFO("A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
//     //                 cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

//     // Fill out the local plan
//     for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
//       double p_x, p_y, p_th;
//       path.getPoint(i, p_x, p_y, p_th);

//       // geometry_msgs::PoseStamped p;
//       //         // tf::Stamped<tf::Pose>(tf::Pose(
//       //         //         tf::createQuaternionFromYaw(p_th),
//       //         //         tf::Point(p_x, p_y, 0.0)),
//       //         //         ros::Time::now(),
//       //         //         costmap_ros_->getGlobalFrameID());
//       // geometry_msgs::PoseStamped pose;
//       // tf::poseStampedTFToMsg(p, pose);
//       // local_plan.push_back(pose);
//       geometry_msgs::PoseStamped p;
//       p.header.frame_id = costmap_ros_->getGlobalFrameID();
//       p.header.stamp = ros::Time::now();
//       p.pose.position.x = p_x;
//       p.pose.position.y = p_y;
//       p.pose.position.z = 0.0;
//       tf2::Quaternion q;
//       q.setRPY(0, 0, p_th);
//       tf2::convert(q, p.pose.orientation);
//       local_plan.push_back(p);
      
//     }

//     //publish information to the visualizer

//     // ROS_INFO("using publish local plan");

//     publishLocalPlan(local_plan);
//     return true;
//   }


//   bool DWAPlannerROS2::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
//     // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal

//     // ROS_INFO("2");
//     if ( ! costmap_ros_->getRobotPose(current_pose_)) {
//       ROS_ERROR("Could not get robot pose");
//       return false;
//     }
//     std::vector<geometry_msgs::PoseStamped> transformed_plan;

//     //#!
//     robot_safe_dir_.clear();
//     computeTTC();
//     //#!

//     if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
//       ROS_ERROR("Could not get local plan");
//       return false;
//     }

//     //if the global plan passed in is empty... we won't do anything
//     if(transformed_plan.empty()) {
//       ROS_WARN_NAMED("dwa_local_planner2", "Received an empty transformed plan.");
//       return false;
//     }
//     ROS_DEBUG_NAMED("dwa_local_planner2", "Received a transformed plan with %zu points.", transformed_plan.size());

//     // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
//     dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

//     if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
//       //publish an empty plan because we've reached our goal position
//       std::vector<geometry_msgs::PoseStamped> local_plan;
//       std::vector<geometry_msgs::PoseStamped> transformed_plan;
      
//       publishGlobalPlan(transformed_plan);
//       publishLocalPlan(local_plan);
//       base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
//       return latchedStopRotateController_.computeVelocityCommandsStopRotate(
//           cmd_vel,
//           limits.getAccLimits(),
//           dp_->getSimPeriod(),
//           &planner_util_,
//           odom_helper_,
//           current_pose_,
//           boost::bind(&DWAPlanner2::checkTrajectory, dp_, _1, _2, _3));
//     } else {
//       bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
//       if (isOk) {
//         publishGlobalPlan(transformed_plan);
//       } else {
//         ROS_WARN_NAMED("dwa_local_planner2", "DWA planner failed to produce path.");
//         std::vector<geometry_msgs::PoseStamped> empty_plan;
//         publishGlobalPlan(empty_plan);
//       }
//       return isOk;
//     }
//   }



// };

#include <dwa_local_planner2/dwa_planner_ros2.h>
#include <Eigen/Core>
#include <cmath>
#include <Eigen/Dense>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

//#!
#include <algorithm>
#include <nav_msgs/GetMap.h>
#define MAX_VAL 10000
#define MIN_VAL -10000
#define COLL_PROB_ALPHA 0.8
#define COLL_PROB_BETA  0.05
#define SIGMA   0.4
#define CORR    1/sqrt(2 * M_PI * SIGMA)
#define GAUSS_ALPHA  0.1
#define EPSILON 0.0001

bool no_obstacles_ = false;
//#!

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner2::DWAPlannerROS2, nav_core::BaseLocalPlanner)

using namespace std;

namespace dwa_local_planner2 {

  void DWAPlannerROS2::reconfigureCB(DWAPlanner2Config &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS2::DWAPlannerROS2() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  // 장애물 이전 좌표 저장(최대 10개)
  float obstacles_prev[10][2];
  float time_to_collide[10][2];
  int cnt_ = 1;
  
  // 충돌시간 계산, 안전 확률 계산 함수
  void DWAPlannerROS2::computeTTC(){
    float obs_curr_x, obs_curr_y;     //obstacle position

    if(cnt_ % 2 != 0){
      
        findObstacles(); 

        if(no_obstacles_){
            // 장애물이 없으면 안전 확률을 최대로 설정하고 반환
            for(int idx = 0; idx < rcv_msg_.ranges.size(); idx++){
                robot_safe_dir_.push_back(1.0);
                // ROS_INFO("safe_dir push if no obstacle");
            }
            dp_->setProbability(robot_safe_dir_);
            
            //clear
            // curr_obs: 현재 인지한 동적 장애물
            curr_obs_.clear();
            
            // obs_direction: 장애물이 있는 라이더 각도 
            obs_direction_.clear();

            // obs_safe_prob: 장애물 별 안전 확률
            obs_safe_prob_.clear();
            return;
        }

        // 로봇 벡터
        double robot_vec[2] = {current_pose_.pose.position.x - previous_pose_.pose.position.x,
                               current_pose_.pose.position.y - previous_pose_.pose.position.y};   //robot vec
        
        // 동적 장애물 벡터들 0으로 초기화
        float obs_vec[2] = {0, 0};
        // 상대 속도
        float v_rel[2];
        
        // 
        for(int idx = 0; idx < curr_obs_.size(); idx++){
            obs_curr_x = curr_obs_[idx].first;
            obs_curr_y = curr_obs_[idx].second;

            float min_dist_ = MAX_VAL;
            int min_idx;

            //find previous obstacle j who has minimum distance with obstacle idx
            for(int j = 0; j < 10 ; j++){
                float dist_ = sqrt(powf(obs_curr_x - obstacles_prev[j][0], 2.0) + powf(obs_curr_y - obstacles_prev[j][1], 2.0));
                if(dist_ < min_dist_){
                    min_dist_ = dist_;
                    min_idx = j;
                }
            }
            obs_vec[0] = obs_curr_x - obstacles_prev[min_idx][0];
            obs_vec[1] = obs_curr_y - obstacles_prev[min_idx][1];   //obs vec

            v_rel[0] = robot_vec[0] - obs_vec[0];
            v_rel[1] = robot_vec[1] - obs_vec[1];

            float robot_vec_s  = sqrt(powf(robot_vec[0], 2.0) + powf(robot_vec[1], 2.0));
            float obs_vec_s = sqrt(powf(obs_vec[0], 2.0) + powf(obs_vec[1], 2.0));
            float f_dot = robot_vec[0] * obs_vec[0] + robot_vec[1] * obs_vec[1];    //inner product
            float cos_theta = f_dot / (robot_vec_s * obs_vec_s);    //cosine theta between 2 vec

            float d_rel_s = sqrt(powf(obs_curr_x - current_pose_.pose.position.x, 2.0)
                               + powf(obs_curr_y - current_pose_.pose.position.y, 2.0));
            float v_rel_s = sqrt(powf(v_rel[0], 2.0) + powf(v_rel[1], 2.0));

            // 충돌 시간 구하기
            float ttc = d_rel_s / (v_rel_s * cos_theta);
            // ROS_INFO("safety prob : %f", safety_prob);

            
            
            // 회피 확률 구하기
            float safety_prob = 1 - COLL_PROB_ALPHA * powf(M_E, -1 * (COLL_PROB_BETA * ttc) * (COLL_PROB_BETA * ttc) );
            // ROS_INFO("safety prob : %f", safety_prob);

            obs_safe_prob_.push_back(safety_prob);
       }//end for

        // 이전 상태 업데이트
        //update previous state to current state
        previous_pose_ = current_pose_;

        for(int idx = 0; idx < 10; idx++){
            if(idx < curr_obs_.size()){
                obstacles_prev[idx][0] = curr_obs_[idx].first;
                obstacles_prev[idx][1] = curr_obs_[idx].second;
            }
            else{
                obstacles_prev[idx][0] = MAX_VAL;
                obstacles_prev[idx][1] = MAX_VAL;
            }
        }

        double gauss_prob;
        double min_prob = MAX_VAL;

        //compute safe probability for all directions
        // 모든 방향에 대해서 회피 확률을 계산해 저장, 이때 장애물과 정면각도의 주변 각도도 회피 확률을 낮게 설정 
        for(int idx = 0; idx < rcv_msg_.ranges.size(); idx++){
            min_prob = MAX_VAL;
            for(int j = 0;  j < curr_obs_.size(); j++){
                gauss_prob = 1 - (CORR * powf(M_E, -1 * (GAUSS_ALPHA*(idx - obs_direction_[j])) * (GAUSS_ALPHA*(idx - obs_direction_[j])) / (2*SIGMA))
                         * (1/CORR)) * (1 - obs_safe_prob_[j]);
                // ROS_INFO("min prob : %f", gauss_prob);
                if(min_prob > gauss_prob){
                    min_prob = gauss_prob;
                }
                // ROS_INFO("min prob : %f", min_prob);
            }
            robot_safe_dir_.push_back(min_prob);
        }

        // for(int j = 0;  j < curr_obs_.size(); j++){
        //   ROS_INFO("min robot_safe_dir_ : %f, index: %d", robot_safe_dir_[obs_direction_[j]],obs_direction_[j] );
        // }



        int min_prob_idx;
        int max_prob_idx;
     
        if( !((1 - robot_safe_dir_[0]) < EPSILON && (1 - robot_safe_dir_[rcv_msg_.ranges.size() - 1]) < EPSILON) ){ //obs in front of robot head direction
     
            if((1 - robot_safe_dir_[0]) >= EPSILON){ // 0 ~
                min_prob_idx = 0;   max_prob_idx = 0;

                for(int i = 0 ; i < rcv_msg_.ranges.size(); i++){   //find index with minimum prob value, last index which robot_safe_dir_[index] < 1
                    if(robot_safe_dir_[i] < robot_safe_dir_[min_prob_idx]){
                        min_prob_idx = i;
                    }
                    if((1 - robot_safe_dir_[i]) < EPSILON){
                        max_prob_idx = i;
                        break;
                    }
                }

                for(int i = min_prob_idx + 1; i <= max_prob_idx; i++){  //update probabilities for the opposite side
                    if(min_prob_idx - (i - min_prob_idx) < 0){
                        int offset = min_prob_idx - (i - min_prob_idx);
                        robot_safe_dir_[rcv_msg_.ranges.size() + offset] = robot_safe_dir_[i];
                    }
                }
            }

            else if((1 - robot_safe_dir_[rcv_msg_.ranges.size() - 1]) >= EPSILON){   // ~ 359
                int max_i = rcv_msg_.ranges.size() - 1;
                min_prob_idx = max_i;   max_prob_idx = max_i;

                for(int i = max_i ; i >= 0; i--){   //find index with minimum prob value, last index which robot_safe_dir_[index] < 1
                     
                    if(robot_safe_dir_[i] < robot_safe_dir_[min_prob_idx]){
                        min_prob_idx = i;
                    }
                    if((1 - robot_safe_dir_[i]) < EPSILON){
                        max_prob_idx = i;
                        break;
                    }
                }

                for(int i = min_prob_idx - 1; i >= max_prob_idx; i--){  //update probabilities for the opposite side
                    if(min_prob_idx - (i - min_prob_idx) > max_i){
                        int offset = min_prob_idx - (i - min_prob_idx);
                        robot_safe_dir_[offset - rcv_msg_.ranges.size()] = robot_safe_dir_[i];
                    }
                }
            }
        }

        
        //send robot_safe_dir_ to base_local_planner::ProbabilityCostFunction
        dp_->setProbability(robot_safe_dir_);

        // for(int i=0; i<robot_safe_dir_.size(); i++)
        // {
        //   ROS_INFO("robot safe dir is %f",robot_safe_dir_[i]);
        //   ROS_INFO("curr_obs %ld", curr_obs_.size() );  
        // }

  
        //clear
        curr_obs_.clear();
        obs_direction_.clear();
        obs_safe_prob_.clear();
    }
    cnt_++;
  }

  // 동적 장애물 찾기 & 중심점 찾기
  void DWAPlannerROS2::findObstacles(){
      float pt_x, pt_y;
      float rb_yaw; //robot yaw
      float w_x = 0, w_y = 0, dist_sq = 0;
      float curr_x, curr_y; // robot position
      float x_diff, y_diff; // robot-obstacle pos differrence
      float obs_yaw, rcv_yaw; // obs yaw 

      int obs_count = 0;
      float min_dist;

      rb_yaw = tf2::getYaw(current_pose_.pose.orientation);
      pt_x = current_pose_.pose.position.x;
      pt_y = current_pose_.pose.position.y;

      if(rb_yaw < 0){
          rb_yaw += 2 * M_PI;
          // ROS_INFO("robot current yaw is minus");
      }
      
      // obs_list - input obs pose by custom topic
      if (obs_list.poses.size() > 0){
        no_obstacles_ = false;
        for (int i = 0; i < obs_list.poses.size(); i++){
            curr_x = obs_list.poses[i].position.x;
            curr_y = obs_list.poses[i].position.y;

            // 장애물 중점
            curr_obs_.push_back(make_pair(curr_x, curr_y)); 
            // x_diff = abs(curr_x - pt_x);
            // y_diff = abs(curr_y - pt_y);
            // obs_yaw = atan2(x_diff, y_diff);
            // rcv_yaw = obs_yaw + rb_yaw;
            
            // 로봇에서 장애물까지의 상대적인 벡터
            Eigen::Vector2d obstacle_position(curr_x, curr_y);  // 장애물의 위치
            Eigen::Vector2d robot_position(pt_x, pt_y);  // 로봇의 위치
            Eigen::Vector2d robot_obstacle = obstacle_position - robot_position;

            // 상대적인 벡터를 로봇(원점) 좌표계로 변환
            Eigen::Rotation2D<double> rotation(-rb_yaw);
            Eigen::Vector2d robot_obstacle_transformed = rotation * robot_obstacle;

            // 각도 계산
            double rcv_yaw = std::atan2(robot_obstacle_transformed.y(),
                                                                robot_obstacle_transformed.x());

            // 반시계 방향의 각도 얻기
            if (rcv_yaw < 0) {
                rcv_yaw += 2 * M_PI;
               }

            // ROS_INFO("current rb angle: %f, obs_raider angle: %f", rb_yaw / 3.14 * 180.0, rcv_yaw / 3.14 * 180.0);

            float min_diff_ = MAX_VAL;
            float min_ang = rcv_msg_.angle_min;
            float lidar_ang;
            int min_idx = 0;
            float det_yaw; 
            // ROS_INFO("min angle: %f, angle_increment %f", min_ang / 3.14 * 180.0,  rcv_msg_.angle_increment / 3.14 * 180.0);
            // ROS_INFO("0 angle distance: %f", rcv_msg_.ranges[0]);

            if ((rcv_yaw - M_PI) <= 0) {
              det_yaw = abs(rcv_yaw - M_PI);
            }
            else {
              det_yaw = abs(rcv_yaw - 3 * M_PI);
            }


            // 장애물이 있는 각도와 가장 유사한 라이더 각도의 index 값 저장
            for(int idx = 0; idx < rcv_msg_.ranges.size(); idx++){
                lidar_ang = 0 +  rcv_msg_.angle_increment * idx;
                float ang_diff = abs(lidar_ang - det_yaw);
                if(ang_diff < min_diff_){
                    min_diff_ = ang_diff;
                    min_idx = idx;
                }
            }

            // ROS_INFO("idx of lidar: %d, real lidar angle: %f", min_idx, (min_ang + rcv_msg_.angle_increment * float(min_idx)) / 3.14 * 180.0);
            // 장애물이 있는 라이더 각도의 index값 저장
            obs_direction_.push_back(min_idx);
        }
      }
      else{
        obs_count = 0;
        no_obstacles_ = true;
      }

  }

  void DWAPlannerROS2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
      //laser data to rcv_msg_
      rcv_msg_.header = msg->header;
      rcv_msg_.angle_min = msg->angle_min;
      rcv_msg_.angle_max = msg->angle_max;
      rcv_msg_.angle_increment = msg->angle_increment;
      rcv_msg_.time_increment = msg->time_increment;
      rcv_msg_.scan_time = msg->scan_time;
      rcv_msg_.range_min = msg->range_min;
      rcv_msg_.range_max = msg->range_max;
      rcv_msg_.ranges = msg->ranges;
      rcv_msg_.intensities = msg->intensities;
  }

    // ********************************************************************
    void DWAPlannerROS2::obstacleCallBack(const geometry_msgs::PoseArray::ConstPtr& msg){
      //laser data to rcv_msg_
      obs_list.poses = msg->poses;
      // ROS_INFO("obs_list size: %ld", obs_list.poses.size());
  }

  void DWAPlannerROS2::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
        
    if (! isInitialized()) {
      
      // ROS 노드와 퍼블리셔, TF(Buffer) 및 Costmap 관련 객체들을 초기화
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);


      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //dp_ 가 실제 알고리즘이 돌아가는 dwa.cpp 파일의 함수
      dp_ = boost::shared_ptr<DWAPlanner2>(new DWAPlanner2(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( "/odom" );
      }
      
      //latchedStopRotateController_.initialize(name);

      initialized_ = true;

      // dynamic_reconfigure를 사용하여 파라미터를 런타임에서 동적으로 변경할 수 있도록 함
      dsrv_ = new dynamic_reconfigure::Server<DWAPlanner2Config>(private_nh);
      // reconfigureCB 함수는 local planner의 값을 바꾸도록 만들어줌
      // dynamic_reconfigure에 의해 호출되는 함수로, 매개변수가 변경될 때마다 호출
      dynamic_reconfigure::Server<DWAPlanner2Config>::CallbackType cb = boost::bind(&DWAPlannerROS2::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);


      // scan 값을 subscribe 함
      scan_sub = private_nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &DWAPlannerROS2::scanCallBack, this);

      //*************************************
      obspos_sub = private_nh.subscribe<geometry_msgs::PoseArray>("/obs_pose", 1, &DWAPlannerROS2::obstacleCallBack, this);


      nav_msgs::GetMap::Request  req;
      nav_msgs::GetMap::Response resp;
      ROS_INFO("Requesting the map..");

      while(!ros::service::call("static_map", req, resp))
      {
        ROS_WARN("Request for map failed; trying again...");
      }
      ROS_INFO("Recieve the map!"); 

      // // 현재 static map 받아와서 MapProcess 함수로 넘기기
      // current_map_ = resp.map;
      // mapProcess(current_map_);
      //#!

    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  bool DWAPlannerROS2::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();
    // ROS_INFO(orig_global_plan)
    ROS_INFO("Got new plan1");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS2::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void DWAPlannerROS2::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void DWAPlannerROS2::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS2::~DWAPlannerROS2(){
    //make sure to clean things up
    delete dsrv_;
  }


  bool DWAPlannerROS2::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {

    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

    //


    // call with updated footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("dwa_local_planner2",
          "The dwa local planner2 failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      
      publishLocalPlan(local_plan);
      return false;
    }

    // ROS_INFO("A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
    //                 cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      // geometry_msgs::PoseStamped p;
      //         // tf::Stamped<tf::Pose>(tf::Pose(
      //         //         tf::createQuaternionFromYaw(p_th),
      //         //         tf::Point(p_x, p_y, 0.0)),
      //         //         ros::Time::now(),
      //         //         costmap_ros_->getGlobalFrameID());
      // geometry_msgs::PoseStamped pose;
      // tf::poseStampedTFToMsg(p, pose);
      // local_plan.push_back(pose);
      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
      
    }

    //publish information to the visualizer

    // ROS_INFO("using publish local plan");

    publishLocalPlan(local_plan);
    return true;
  }


  bool DWAPlannerROS2::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal

    // ROS_INFO("2");
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;

    //#!
    robot_safe_dir_.clear();
    computeTTC();
    //#!

    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("dwa_local_planner2", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner2", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner2::checkTrajectory, dp_, _1, _2, _3));
    } else {
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("dwa_local_planner2", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }



};