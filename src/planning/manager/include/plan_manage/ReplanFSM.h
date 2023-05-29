#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/BsplineOptimizer.h>
#include <mapping/GridMap.h>
#include <air_pilot/Bspline.h>
#include <air_pilot/DataDisp.h>
#include <plan_manage/PlannerManager.h>
#include <minimumSnapTrajGen/planning_visualization.h>

using std::vector;

namespace air_pilot
{

  class ReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    PlannerManager::Ptr pm_;
    PlanningVisualization::Ptr visualization_;
    air_pilot::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_;

    bool consistancy_check_flag_ = false;
    /* ROS utils */ 
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    /* helper functions */
    bool callOptimizeOnce(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool iterativeReplan();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void FSMStateTransition(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, ReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSM();

    void miniSnapPlanOptimization();
    void getLocalTarget();

    /* ROS functions */
    void mainPlanLoop(const ros::TimerEvent &e);
    void checkTrajCollision(const ros::TimerEvent &e);
    void targetCb(const nav_msgs::PathConstPtr &msg);
    void odomCb(const nav_msgs::OdometryConstPtr &msg);

    bool checkConsistancy();
    bool checkCollision();

  public:
    ReplanFSM(/* args */)
    {
    }
    ~ReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace air_pilot

#endif