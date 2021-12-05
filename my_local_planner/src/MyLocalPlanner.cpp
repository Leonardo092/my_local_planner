#include "../include/MyLocalPlanner.h"

PLUGINLIB_EXPORT_CLASS(my_local_planner::MyLocalPlanner, nav_core::BaseLocalPlanner)

namespace my_local_planner {

	MyLocalPlanner::MyLocalPlanner() : initialized_(false), goal_reached_(false) {}

	MyLocalPlanner::~MyLocalPlanner() 
	{
    	delete server_;
	}

	void MyLocalPlanner::reconfigureCB(MyLocalPlannerConfig& config,uint32_t level) 
	{
		// target (final goal included) tolerance
		pos_tolerance_ = config.position_tolerance;
		ang_tolerance_ = config.orientation_tolerance;

		//laser range for obstacle detection
		obstacle_range_left_ = config.lidar_detection_range_left;
		obstacle_range_right_ = config.lidar_detection_range_right;
		stop_range_ = config.stopping_distance;

		//local planner target among global waypoints
		plan_hrzn_ = config.local_plan_horizon;

		// linear
		max_vel_lin_ = config.max_vel_lin;
		min_vel_lin_ = config.min_vel_lin;
		max_incr_lin_ = config.max_incr_lin;

		// angular
		max_vel_ang_ = config.max_vel_ang;
		min_vel_ang_ = config.min_vel_ang;
		max_incr_ang_ = config.max_incr_ang;

		// pid controller params
		k_p_lin_ = config.k_p_lin;
		k_i_lin_ = config.k_i_lin;
		k_d_lin_ = config.k_d_lin;

		k_p_ang_ = config.k_p_ang;
		k_i_ang_ = config.k_i_ang;
		k_d_ang_ = config.k_d_ang;
	}

	void MyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
	{
		if (!initialized_)
		{
			ros::NodeHandle nh("/");
			ros::NodeHandle private_nh("~/" + name);

			initialized_ = true;	

			lin_pid_ok_ = false;
			ang_pid_ok_ = false;

			tf_ = tf;
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			odom_helper_ = new base_local_planner::OdometryHelperRos(odom_frame_);

			float ctrl_freq = 20;
			ctrl_time_ = 1/ctrl_freq;
			goal_pose_[0] = 0;
			goal_pose_[1] = 0;
			goal_pose_[2] = 0;

			need_final_rotation_ = false;
			goal_reached_ = false;
			waypoint_reached_ = true;
			plan_index_ = 0;

			//frames
			map_frame_= "map";
			odom_frame_ = "/odom";	

			//dynamic reconfigure
			server_ = new dynamic_reconfigure::Server<MyLocalPlannerConfig>(private_nh);
    		dynamic_reconfigure::Server<MyLocalPlannerConfig>::CallbackType cb = boost::bind(&MyLocalPlanner::reconfigureCB, this, _1, _2);
    		server_->setCallback(cb);

			//pub and sub
			target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 20);
			curr_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 20);
			curr_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 20);
			goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 20, &MyLocalPlanner::GoalPoseCB, this);
			laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/scan", 20, &MyLocalPlanner::LaserCB, this);

		}
		else
		{
			ROS_WARN("Local planner has already been initialized.");
		}
	}

	bool MyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
	{
		if (!initialized_) 
		{
    		ROS_ERROR("Local planner has not been initialized.");
    		return false;
  		}
		else
		{
			// reset planner
			global_plan_.clear();
			global_plan_ = plan;

			// //assign local planner horizon
			plan_index_ = plan_hrzn_;

			//if robot is close to goal, let the goal be the local planner horizon
			if (plan_index_ > (int)(global_plan_.size()))
				plan_index_ = (int)((global_plan_.size()));

			//set new target if previous is reached
			if (waypoint_reached_)
			{
				trace_target_ = global_plan_[plan_index_];
				waypoint_reached_ = false;
			}

			//reset controller
  			integral_lin_ = 0; 
			integral_ang_ = 0.0;
  			error_lin_ = 0;
			error_ang_ = 0.0;

			return true;
		}
	}

	bool MyLocalPlanner::isGoalReached()
	{
		if (!initialized_) 
		{
    		ROS_ERROR("Local planner has not been initialized.");
    		goal_reached_ = false;
  		}
  		if (goal_reached_) 
		{
    		ROS_ERROR("Local LocalPlanner goal reached...");
    		goal_reached_ = true;
  		}

		if (plan_index_ >= (int)(global_plan_.size() -1)) 
		{
			// last pose
			double pos_to_target_dist = std::hypot((global_plan_.back().pose.position.x - goal_pose_[0]), 
													(global_plan_.back().pose.position.y - goal_pose_[1]));
			double pos_to_target_angle = fabs(tf2::getYaw(goal_.pose.orientation) - goal_pose_[2]);
			
			//when final position is reached, the robot alignes with the goal pose
			if (pos_to_target_dist <= pos_tolerance_)
			{
				if ((pos_to_target_angle <= ang_tolerance_))
					goal_reached_ = true;
				else
					need_final_rotation_ = true;

			}
		}
  		return goal_reached_;
	}

	bool MyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		if (!initialized_) 
		{
    		ROS_WARN("PID planner has not been initialized.");
    		return false;
  		}

  		if (goal_reached_) 
		  {
    		ROS_WARN("Robot has reached the goal position.");
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.angular.z = 0;
    		return true;
  		}

		if(global_plan_.empty())
		{
			ROS_WARN("Global plan is empty!");
			return false;
		}

		//variables that need reset at each cycle
		double lin_vel, ang_vel;
		geometry_msgs::PoseStamped target;
		geometry_msgs::PoseStamped curr_pose;

		//set current target waypoint
		target = trace_target_;		

		// invoking robot pose and orientaiton
		geometry_msgs::PoseStamped global_pose;
		costmap_ros_->getRobotPose(global_pose);

		// trace robot global pose for isGoalReached
		goal_pose_[0] = global_pose.pose.position.x;
		goal_pose_[1] = global_pose.pose.position.y;
		goal_pose_[2] = tf2::getYaw(global_pose.pose.orientation);

		// receive odometry info
		nav_msgs::Odometry odometry;
		odom_helper_->getOdom(odometry);

		//check for obstacles
		if (EmergencyStop(laser_))
		{
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.angular.z = 0;
			ROS_WARN("Obstacle too close: emergency stop activated");
		}
		else
		{
			//calculate next waypoint orientation
			float target_theta;

			if (need_final_rotation_) //robot is near the goal position
				target_theta = tf2::getYaw(goal_.pose.orientation);
			else
				target_theta = atan2((target.pose.position.y - global_pose.pose.position.y), 
										(target.pose.position.x - global_pose.pose.position.x));

			//retreive robot current orientation
			float robot_theta = tf2::getYaw(global_pose.pose.orientation);

			//calculate ungular displacement
			float pos_to_target_ang = robot_theta - target_theta;

			//calculate euler distance
			float pos_to_target_dist = std::hypot((target.pose.position.x - global_pose.pose.position.x), 
													(target.pose.position.y - global_pose.pose.position.y));

			//motion state machine
			if (fabs(pos_to_target_ang) <= ang_tolerance_)
			{
				ang_pid_ok_ = true;
				cmd_vel.linear.x = 0.0;
				cmd_vel.linear.y = 0.0;
				cmd_vel.angular.z = 0.0;

				if (pos_to_target_dist <= pos_tolerance_)
				{
					lin_pid_ok_ = true;
					cmd_vel.linear.x = 0.0;
					cmd_vel.linear.y = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				else
				{
					lin_vel = LinearPID(odometry, target.pose.position.x, target.pose.position.y);
					ang_vel = AngularPID(odometry, target_theta, robot_theta); //make sure the robot goes as straight as possible
					cmd_vel.linear.x = lin_vel;
					cmd_vel.linear.y = 0;
					cmd_vel.angular.z = ang_vel;
				}
			}
			else
			{
				ang_vel = AngularPID(odometry, target_theta, robot_theta);
				cmd_vel.linear.x = 0;
				cmd_vel.linear.y = 0;
				cmd_vel.angular.z = ang_vel;
			}
			
			// publish next target pose
			target.header.frame_id = map_frame_;
			target.header.stamp = ros::Time::now();
			auto target_orien_quat = createQuaternionMsgFromYaw(target_theta);
			target.pose.orientation.x = target_orien_quat.x;
			target.pose.orientation.y = target_orien_quat.y;
			target.pose.orientation.z = target_orien_quat.z;
			target.pose.orientation.w = target_orien_quat.w;
			target_pose_pub_.publish(target);

			// publish robot pose
			curr_pose.header.frame_id = map_frame_;
			curr_pose.header.stamp = ros::Time::now();
			auto curr_orien_quat = createQuaternionMsgFromYaw(robot_theta);
			curr_pose.pose.position.x = global_pose.pose.position.x;
			curr_pose.pose.position.y = global_pose.pose.position.y;
			curr_pose.pose.position.z = global_pose.pose.position.z;
			curr_pose.pose.orientation.x = curr_orien_quat.x;
			curr_pose.pose.orientation.y = curr_orien_quat.y;
			curr_pose.pose.orientation.z = curr_orien_quat.z;
			curr_pose.pose.orientation.w = curr_orien_quat.w;
			curr_pose_pub_.publish(curr_pose);

			//update target when the robot reached current target
			if (ang_pid_ok_ && lin_pid_ok_)
			{
				waypoint_reached_ = true;
				ang_pid_ok_ = false;
				lin_pid_ok_ = false;
			}
		}

		return true;
	}

	double MyLocalPlanner::LinearPID(nav_msgs::Odometry& odometry, double target_x, double target_y) 
	{
		double vel_curr = hypot(odometry.twist.twist.linear.y, odometry.twist.twist.linear.x);
		double vel_target = hypot(target_x, target_y) / ctrl_time_;

		if (fabs(vel_target) > max_vel_lin_) 
		{
			vel_target = std::copysign(max_vel_lin_, vel_target);
		}

		// calculate pid parameters
		double err_vel = vel_target - vel_curr;
		integral_lin_ += err_vel * ctrl_time_;
		double derivative_lin = (err_vel - error_lin_) / ctrl_time_;

		// apply pid
		double incr_lin = k_p_lin_ * err_vel + k_i_lin_ * integral_lin_ + k_d_lin_ * derivative_lin;

		// update error
		error_lin_ = err_vel;

		if (fabs(incr_lin) > max_incr_lin_) 
			incr_lin = std::copysign(max_incr_lin_, incr_lin);

		//calculate output linear velocity
		double out_velocity = vel_curr + incr_lin;

		if (fabs(out_velocity) > max_vel_lin_) 
			out_velocity = std::copysign(max_vel_lin_, out_velocity);

		if (fabs(out_velocity) < min_vel_lin_) 
			out_velocity = std::copysign(min_vel_lin_, out_velocity);

		return out_velocity;
	}

	double MyLocalPlanner::AngularPID(nav_msgs::Odometry& odometry, double target_ang, double robot_ang) 
	{

		double orien_err = target_ang - robot_ang;

		//bind angle in range [-180; 180]
		if(orien_err > PI) 
			orien_err -= (2* PI);

		if(orien_err < -PI) 
			orien_err += (2* PI);

		double target_vel_ang = (orien_err) / ctrl_time_;

		if (fabs(target_vel_ang) > max_vel_ang_) 
			target_vel_ang = std::copysign(max_vel_ang_, target_vel_ang);

		// calculate pid parameters
		double vel_ang = odometry.twist.twist.angular.z;
		double error_ang = target_vel_ang - vel_ang;
		integral_ang_ += error_ang * ctrl_time_;
		double derivative_ang = (error_ang - error_ang_) / ctrl_time_;

		// apply pid
		double incr_ang = k_p_ang_ * error_ang + k_i_ang_ * integral_ang_ + k_d_ang_ * derivative_ang;

		// update error
		error_ang_ = error_ang;

		if (fabs(incr_ang) > max_incr_ang_) 
			incr_ang = std::copysign(max_incr_ang_, incr_ang);

		//calculate output angular velocity
		double out_velocity = std::copysign(vel_ang + incr_ang, target_vel_ang);

		if (fabs(out_velocity) > max_vel_ang_) 
			out_velocity = std::copysign(max_vel_ang_, out_velocity);

		if (fabs(out_velocity) < min_vel_ang_) 
			out_velocity = std::copysign(min_vel_ang_, out_velocity);

		return out_velocity;
	}

	void MyLocalPlanner::GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
		goal_ = *msg;
    }

	void MyLocalPlanner::LaserCB(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
		laser_ = *msg;
    }

	bool MyLocalPlanner::EmergencyStop(sensor_msgs::LaserScan scan) const
	{
		//robot checks for obstacles frontward in the range [-60,60] deg (configurable)
		for (int i = 0; i < obstacle_range_left_; i++)
		{
			if (scan.ranges[i] < stop_range_)
				return true;
		}

		for (int i = obstacle_range_left_; i < scan.ranges.size(); i++)
		{
			if (scan.ranges[i] < stop_range_)
				return true;
		}

		return false;
	}

	geometry_msgs::Quaternion MyLocalPlanner::createQuaternionMsgFromYaw(double yaw)
	{
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		return tf2::toMsg(q);
	}	

}
