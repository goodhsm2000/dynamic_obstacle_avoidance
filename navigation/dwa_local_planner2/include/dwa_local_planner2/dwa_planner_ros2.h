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
// #ifndef DWA_LOCAL_PLANNER2_DWA_PLANNER_ROS2_H_
// #define DWA_LOCAL_PLANNER2_DWA_PLANNER_ROS2_H_

// #include <boost/shared_ptr.hpp>
// #include <boost/thread.hpp>

// // #include <tf/transform_listener.h>
// #include <tf2_ros/buffer.h>

// #include <dynamic_reconfigure/server.h>
// #include <dwa_local_planner2/DWAPlanner2Config.h>

// #include <angles/angles.h>

// #include <nav_msgs/Odometry.h>

// #include <costmap_2d/costmap_2d_ros.h>

// #include <nav_core/base_local_planner.h>

// #include <base_local_planner/latched_stop_rotate_controller.h>

// #include <base_local_planner/odometry_helper_ros.h>

// #include <dwa_local_planner2/dwa_planner2.h>

// //#!
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <sensor_msgs/LaserScan.h>

// // Some definitions of functions
// #define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
// #define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
// #define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)

// // Information of the map
// struct map_inf {
//     double size_x;
//     double size_y;
//     double scale;
//     double origin_x;
//     double origin_y;
// };
// //#!

// namespace dwa_local_planner2 {
//   /**
//    * @class DWAPlannerROS
//    * @brief ROS Wrapper for the DWAPlanner that adheres to the
//    * BaseLocalPlanner interface and can be used as a plugin for move_base.
//    */
//   class DWAPlannerROS2 : public nav_core::BaseLocalPlanner {
//     public:
//       /**
//        * @brief  Constructor for DWAPlannerROS wrapper
//        */
//       DWAPlannerROS2();

//       /**
//        * @brief  Constructs the ros wrapper
//        * @param name The name to give this instance of the trajectory planner
//        * @param tf A pointer to a transform listener
//        * @param costmap The cost map to use for assigning costs to trajectories
//        */
//       void initialize(std::string name, tf2_ros::Buffer* tf,
//           costmap_2d::Costmap2DROS* costmap_ros);

//       /**
//        * @brief  Destructor for the wrapper
//        */
//       ~DWAPlannerROS2();

//       /**
//        * @brief  Given the current position, orientation, and velocity of the robot,
//        * compute velocity commands to send to the base
//        * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
//        * @return True if a valid trajectory was found, false otherwise
//        */
//       bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


//       /**
//        * @brief  Given the current position, orientation, and velocity of the robot,
//        * compute velocity commands to send to the base, using dynamic window approach
//        * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
//        * @return True if a valid trajectory was found, false otherwise
//        */
//       bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

//       /**
//        * @brief  Set the plan that the controller is following
//        * @param orig_global_plan The plan to pass to the controller
//        * @return True if the plan was updated successfully, false otherwise
//        */
//       bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

//       /**
//        * @brief  Check if the goal pose has been achieved
//        * @return True if achieved, false otherwise
//        */
//       bool isGoalReached();



//       bool isInitialized() {
//         return initialized_;
//       }




//     private:
//       /**
//        * @brief Callback to update the local planner's parameters based on dynamic reconfigure
//        */
//       void reconfigureCB(DWAPlanner2Config &config, uint32_t level);

//       void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

//       void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);



//       //#!
//       void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
// 	  /**
// 	  * @brief Compute Time-to-Collision(TTC) when dynamic obstacles detected
// 	  */
//       void computeTTC();
// 	  /**
// 	  * @brief Sense dynamic points and segment to each obstacle
// 	  */
//       void findObstacles();
//       /**
//        * @brief Given static map, assume occupied positions in the map
//        */
//       void mapProcess(const nav_msgs::OccupancyGrid& map);
// 	  //#!


//       tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

//       // for visualisation, publishers of global and local plan
//       ros::Publisher g_plan_pub_, l_plan_pub_;

//       base_local_planner::LocalPlannerUtil planner_util_;

//       boost::shared_ptr<DWAPlanner2> dp_; ///< @brief The trajectory controller

//       costmap_2d::Costmap2DROS* costmap_ros_;

//       dynamic_reconfigure::Server<DWAPlanner2Config> *dsrv_;
//       dwa_local_planner2::DWAPlanner2Config default_config_;
//       bool setup_;
//       geometry_msgs::PoseStamped current_pose_;

//       base_local_planner::LatchedStopRotateController latchedStopRotateController_;


//       bool initialized_;


//       base_local_planner::OdometryHelperRos odom_helper_;
//       std::string odom_topic_;


//       //#!
//       nav_msgs::OccupancyGrid current_map_;
//       std::vector<std::pair<float, float> > map_position_;
//       std::vector<int> obs_idx_;      //index data for valid ranges[] values
//       geometry_msgs::PoseStamped previous_pose_; // modify by SG

//       sensor_msgs::LaserScan rcv_msg_;
//       sensor_msgs::LaserScan lsr_msg_;
//       ros::Subscriber scan_sub;

//       std::vector<std::pair<float, float> > curr_obs_;

//       std::vector<int> obs_direction_;
//       std::vector<float> obs_safe_prob_;
//       std::vector<double> robot_safe_dir_;

//       base_local_planner::ProbabilityCostFunction prob_cost_function_;
//       //#!
//   };
// };
// #endif

#ifndef DWA_LOCAL_PLANNER2_DWA_PLANNER_ROS2_H_
#define DWA_LOCAL_PLANNER2_DWA_PLANNER_ROS2_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// #include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner2/DWAPlanner2Config.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <nav_core/base_local_planner.h>

#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner2/dwa_planner2.h>

//#!
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>

//***********************************
#include <geometry_msgs/PoseArray.h>

// Some definitions of functions
#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)

// Information of the map
struct map_inf {
    double size_x;
    double size_y;
    double scale;
    double origin_x;
    double origin_y;
};
//#!

namespace dwa_local_planner2 {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerROS2 : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerROS2();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerROS2();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }




    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DWAPlanner2Config &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);



      //#!
      void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
	  /**
	  * @brief Compute Time-to-Collision(TTC) when dynamic obstacles detected
	  */
      void computeTTC();
	  /**
	  * @brief Sense dynamic points and segment to each obstacle
	  */
      void findObstacles();
      /**
       * @brief Given static map, assume occupied positions in the map
       */

      // *************************************
      void obstacleCallBack(const geometry_msgs::PoseArray::ConstPtr& msg);


      tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner2> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<DWAPlanner2Config> *dsrv_;
      dwa_local_planner2::DWAPlanner2Config default_config_;
      bool setup_;
      geometry_msgs::PoseStamped current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;


      //#!
      nav_msgs::OccupancyGrid current_map_;
      std::vector<std::pair<float, float> > map_position_;
      std::vector<int> obs_idx_;      //index data for valid ranges[] values
      geometry_msgs::PoseStamped previous_pose_; // modify by SG

      sensor_msgs::LaserScan rcv_msg_;
      sensor_msgs::LaserScan lsr_msg_;
      
      geometry_msgs::PoseArray obs_list;

      ros::Subscriber scan_sub;
      ros::Subscriber obspos_sub;

      std::vector<std::pair<float, float> > curr_obs_;

      std::vector<int> obs_direction_;
      std::vector<float> obs_safe_prob_;
      std::vector<double> robot_safe_dir_;

      base_local_planner::ProbabilityCostFunction prob_cost_function_;
      //#!
  };
};
#endif