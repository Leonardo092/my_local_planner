#include "../include/operational_checker.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "operational_checker_node");  

    ros::NodeHandle nh("~/");

    //pub and sub
    ros::Publisher stuck_pub = nh.advertise<std_msgs::String>("/stuck", 20);
    ros::Subscriber get_robot_pose = nh.subscribe<geometry_msgs::PoseStamped>("/current_pose", 20, &RobotPoseCB);
    ros::Subscriber get_goal_pose = nh.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 20, &GoalPoseCB);
    ros::Subscriber get_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 20, &VelocityCB);

    ros::Rate loop_rate(10.0);

    //string output for stuck message
    std::string stuck_str = "Robot is unable to move";
    std_msgs::String stuck_msg;
    stuck_msg.data = stuck_str;

    //at initialization, the robot has covered no path yet
    double goal_dist;
    double r_to_g_dist;
    bool is_stationary;
    int completion_rate;
    int stuck_counter = 0;
    int stuck_limit = 300; //30 sec of null velocity to trigger the stuck condition
    int goal_tolerance = 95; //over that path percentage, the robot is considered to be at the goal position (not stuck in goal)

    goal_received = false;

    while (ros::ok())
    {
        if (goal_received)
        {

            //calculate distance from start (always [0,0]) to goal
            goal_dist = hypot((goal.goal.target_pose.pose.position.x), (goal.goal.target_pose.pose.position.y));

            //calculate current robot-goal dist
            r_to_g_dist = hypot((goal.goal.target_pose.pose.position.x - robot.pose.position.x), 
                                        (goal.goal.target_pose.pose.position.y - robot.pose.position.y));
            //check robot velocity 
            if (vel.linear.x == 0.0 && vel.linear.y == 0.0)
                is_stationary = true;
            else
                is_stationary = false;

            completion_rate = (int)((1 - (r_to_g_dist/goal_dist))*100);

            //verify whether the robot is stuck
            if ((is_stationary) && (completion_rate > 0))
                stuck_counter++;
            else
                stuck_counter = 0;
            
            //the robot is considered stuck if the counter is incremented until a certain limit and the robot is not near the goal
            if ((stuck_counter >= stuck_limit) && (completion_rate < goal_tolerance))
            {
                stuck_pub.publish(stuck_msg);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RobotPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    robot = *msg;
}

void GoalPoseCB(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg)
{
    goal = *msg;
    goal_received = true;
}

void VelocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel = *msg;
}