#ifndef MYLOCALPLANNER_H_
#define MYLOCALPLANNER_H_

#include <ros/ros.h>
#include <tf2/utils.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <dynamic_reconfigure/server.h>
#include <my_local_planner/MyLocalPlannerConfig.h>

#include <pluginlib/class_list_macros.h>


#include <cmath>
#include <algorithm>

static const double PI = 3.14;

namespace my_local_planner {

class MyLocalPlanner : public nav_core::BaseLocalPlanner 
{
	public:

		MyLocalPlanner();

		~MyLocalPlanner();

		bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

		bool isGoalReached() override;

		bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

		void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

	private:

		//methods
		double LinearPID(nav_msgs::Odometry& base_odometry, double next_t_x, double next_t_y);
		double AngularPID(nav_msgs::Odometry& base_odometry, double target_th_w, double robot_orien);
		void GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void LaserCB(const sensor_msgs::LaserScan::ConstPtr& msg);
		bool EmergencyStop(sensor_msgs::LaserScan scan) const;
		static geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw);

		//members
		tf2_ros::Buffer* tf_;
		costmap_2d::Costmap2DROS* costmap_ros_;
		costmap_2d::Costmap2D* costmap_;
		base_local_planner::OdometryHelperRos *odom_helper_;
		std::vector<geometry_msgs::PoseStamped> global_plan_;
		geometry_msgs::PoseStamped trace_target_, goal_;
		sensor_msgs::LaserScan laser_;

		float ctrl_time_;
		double goal_pose_[3]; // x,y,theta
		bool initialized_, goal_reached_, need_final_rotation_, waypoint_reached_, lin_pid_ok_, ang_pid_ok_ ;
		int plan_index_, plan_hrzn_, obstacle_range_left_, obstacle_range_right_;
		double stop_range_;

		//frames
		std::string map_frame_, odom_frame_;

		// Dynamic reconfigure
		dynamic_reconfigure::Server<MyLocalPlannerConfig> *server_;
  		void reconfigureCB(MyLocalPlannerConfig &config, uint32_t level);

		//pid parameters
		double pos_tolerance_, ang_tolerance_;
		double max_vel_lin_, min_vel_lin_, max_incr_lin_;
		double max_vel_ang_, min_vel_ang_, max_incr_ang_;
		double k_p_lin_, k_i_lin_, k_d_lin_;
		double k_p_ang_, k_i_ang_, k_d_ang_;
		double error_lin_, error_ang_;
		double integral_lin_, integral_ang_;

        //publishers
  		ros::Publisher target_pose_pub_, curr_pose_pub_;

  		// subscribers
  		ros::Subscriber goal_sub_, laser_sub_;

	};
}

#endif /* MYLOCALPLANNER_H_ */
