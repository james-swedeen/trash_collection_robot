/**
 * @File: controller_node.cpp
 *
 * @brief
 * A behavior based controller that will make the bot look for trash, pick it up, and throw it away.
 **/

/* ROS Headers */
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>
#include<interbotix_xs_msgs/JointGroupCommand.h>
#include<interbotix_xs_msgs/JointSingleCommand.h>

/* Local Headers */
#include<trash_collection_robot/pose_managers/nav_odometry_manager.hpp>
#include<trash_collection_robot/rx200_arm_edge_generator.hpp>
#include<trash_collection_robot/laser_scan_processor.hpp>
#include<trash_collection_robot/targets_queue.hpp>
#include<trash_collection_robot/sensor_joint_state_manager.hpp>
#include<trash_collection_robot/arm_controller.hpp>


#define GRAB_ANGLE           double(-5)
#define OBSTICAL_RADIUS      double(0.15)
#define OBS_CHECK_LEN        double(0.25)
#define LINEAR_EPS           double(0.01)
#define ANGULAR_EPS          math::angleToRadian<double>(0.01)
#define LINEAR_GAIN          double(0.15)
#define ANGULAR_GAIN         double(0.35)
#define LINEAR_MAX           double(0.2)
#define ANGULAR_MAX          math::angleToRadian<double>(20)
#define MAX_SPIN_TIME        double(6)
#define MIN_SPIN_TIME        double(2)
#define WAIT_FOR_TARGET_TIME double(2)
#define REACHING_RADIUS      double(0.4)

void rand_walk(LaserScanProcessor&   obstacle_manager,
               TargetsQueue&         targets_queue,
               const ros::Publisher& go_to_goal_pub,
               ros::Rate&            loop_rate);
bool move_bot(LaserScanProcessor&   obstacle_manager,
              TargetsQueue&         targets_queue,
              const ros::Publisher& go_to_goal_pub,
              ros::Rate&            loop_rate);
bool move_arm(const ros::Publisher&                            arm_pub,
              TargetsQueue&                                    targets_queue,
              SensorJointStateManager<double,Eigen::RowMajor>& joint_listener,
              ros::Rate&                                       loop_rate,
              const ros::Publisher&                            gripper_pub);
void move_arm_over_bin(const ros::Publisher&                            arm_pub,
                       SensorJointStateManager<double,Eigen::RowMajor>& joint_listener,
                       ros::Rate&                                       loop_rate,
                       const ros::Publisher&                            gripper_pub);
void grabber_op(const bool open, const ros::Publisher& gripper_pub);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle m_nh;
  ros::NodeHandle p_nh("~");
  ros::Rate       loop_rate(30);

  srand(time(NULL));

  // To get targets
  TargetsQueue               targets_queue("targets", "rx200/base_link", 0.1, 0.5);
  geometry_msgs::PoseStamped target_point;

  // To get current location of bot
  SensorJointStateManager<double,Eigen::RowMajor> joint_listener("/robot/joint_states");

  ros::Publisher arm_pub     = m_nh.advertise<interbotix_xs_msgs::JointGroupCommand>("/robot/commands/joint_group", 5);
  ros::Publisher gripper_pub = m_nh.advertise<interbotix_xs_msgs::JointSingleCommand>("/robot/commands/joint_single", 5);
  ros::Publisher base_pub    = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  // To get the obstacles
  LaserScanProcessor obstacle_manager("/scan", "rx200/base_link", 50);

  tf::TransformListener tf_listener;

  while(m_nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    // Random walk
    rand_walk(obstacle_manager, targets_queue, base_pub, loop_rate);

    // Go to target
    if(move_bot(obstacle_manager, targets_queue, base_pub, loop_rate))
    {
      if(move_arm(arm_pub, targets_queue, joint_listener, loop_rate, gripper_pub))
      {
        // Put object in bin
        move_arm_over_bin(arm_pub, joint_listener, loop_rate, gripper_pub);
      }
    }
  }

  exit(EXIT_SUCCESS);
}


//Move Bot

//Move Arm Function
bool move_arm(const ros::Publisher&                            arm_pub,
              TargetsQueue&                                    targets_queue,
              SensorJointStateManager<double,Eigen::RowMajor>& joint_listener,
              ros::Rate&                                       loop_rate,
              const ros::Publisher&                            gripper_pub)
{
  //declare variables
  geometry_msgs::Point                 robot_pose_msg;
  geometry_msgs::PoseStamped           target_point;
  interbotix_xs_msgs::JointGroupCommand arm_message;
  Eigen::Matrix<double,1,5,Eigen::RowMajor> position, position1, current,  error, target;

  robot_pose_msg.x = 0;
  robot_pose_msg.y = 0;
  robot_pose_msg.z = 0;

  //Open the gripper
  grabber_op(true, gripper_pub);

  target_point.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  target_point.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  target_point.pose.position.z = std::numeric_limits<double>::quiet_NaN();

  // Wait until final target position is available
  const ros::Time start_time = ros::Time::now();
  while(ros::ok() and
        !targets_queue.getNearestTarget(robot_pose_msg, target_point) and
        ((ros::Time::now() - start_time).toSec() < WAIT_FOR_TARGET_TIME))
  {
    ROS_WARN_THROTTLE(3, "Failing to get robot goal information.");
    ros::spinOnce();
    loop_rate.sleep();
  }

  //send in target position to get out arm angle positions
  if(!RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(target_point.pose.position.x,
                                                              target_point.pose.position.y,
                                                              target_point.pose.position.z,
                                                              math::angleToRadian<double>(GRAB_ANGLE),
                                                              math::angleToRadian<double>(0), position))
  {
    ROS_WARN("Thats not valid");
    return false;
  }

  //move arm to the position of the target
  arm_message = mover(position);

  //reinitialize error vector
  for(size_t it = 0; it < 5; ++it)
  {
    error[it] = 1;
  }

  //Wait until the arm has moved into correct position
  target = position;
  while((error.array().abs() > math::angleToRadian<double>(5)).any()){
    arm_pub.publish(arm_message);
    //Get final target position
    joint_listener.update();
    if(joint_listener.getPose(current))
    {
      position = target;

      error = target.array() - current.array();

      position[0] += error[0] * double(0.2);

      arm_message = mover(position);
    }
    loop_rate.sleep();
  }

  //close the gripper
  grabber_op(false, gripper_pub);
  return true;
}

//Bin Function
void move_arm_over_bin(const ros::Publisher&                            arm_pub,
                       SensorJointStateManager<double,Eigen::RowMajor>& joint_listener,
                       ros::Rate&                                       loop_rate,
                       const ros::Publisher&                            gripper_pub)
{
  //Declare Variables
  interbotix_xs_msgs::JointGroupCommand arm_message;
  Eigen::Matrix<double,1,5,Eigen::RowMajor> position, error, current;

  //Move the arm over the bin
  if(!RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(double(-0.03),
                                                              double(-0.12),
                                                              double(0.12),
                                                              math::angleToRadian<double>(90),
                                                              math::angleToRadian<double>(85), position))
  {
    ROS_WARN_THROTTLE(1, "Thats not valid");
    return;
  }

  //Send arm position data and publish message
  arm_message = mover(position);
  arm_pub.publish(arm_message);

  //initialize error vector
  for(size_t it = 0; it < 5; ++it) { error[it] = 1; }

  //wait until the arm is in position over the bin
  while((error.array() > math::angleToRadian<double>(5)).any()){
    joint_listener.update();
    if(joint_listener.getPose(current))
    {
      for(size_t it = 0; it < 5; ++it)
      {
        error[it] = std::abs<double>(position[it] - current[it]);
      }
    }
    loop_rate.sleep();
  }

  //open the gripper
  grabber_op(true, gripper_pub);

  position = Eigen::Matrix<double,1,5,Eigen::RowMajor>::Zero();
  position[2] = math::angleToRadian<double>(10);
  arm_message = mover(position);
  arm_pub.publish(arm_message);

  for(size_t it = 0; it < 5; ++it) { error[it] = std::numeric_limits<double>::infinity(); }
  while((error.array() > math::angleToRadian<double>(5)).any()){
    joint_listener.update();
    if(joint_listener.getPose(current))
    {
      for(size_t it = 0; it < 5; ++it)
      {
        error[it] = std::abs<double>(position[it] - current[it]);
      }
    }
    loop_rate.sleep();
  }

  //Move the arm to home position
  position = RX200EdgeGenerator<double, Eigen::RowMajor>::HOME_POSITION;

  //Publish message
  arm_message = mover(position);
  arm_pub.publish(arm_message);

  //Reinitialize error vector
  for(size_t it = 0; it < 5; ++it)
  {
    error[it] = 1;
  }

  //wait until the arm is in position
  while((error.array() > math::angleToRadian<double>(5)).any()){
    joint_listener.update();
    if(joint_listener.getPose(current))
    {
      for(size_t it = 0; it < 5; ++it)
      {
        error[it] = std::abs<double>(position[it] - current[it]);
      }
    }
    loop_rate.sleep();
  }
}

void grabber_op(const bool open, const ros::Publisher& gripper_pub)
{
  //declare variable
  interbotix_xs_msgs::JointSingleCommand gripper_message;

  //publish message
  gripper_message = grip(open);
  gripper_pub.publish(gripper_message);

  //sleep for 1 second
  std::this_thread::sleep_for(std::chrono::seconds(3));
}

void rand_walk(LaserScanProcessor&   obstacle_manager,
               TargetsQueue&         targets_queue,
               const ros::Publisher& go_to_goal_pub,
               ros::Rate&            loop_rate)
{
  geometry_msgs::Twist                 bot_command;
  std::list<Eigen::Matrix<double,2,1>> obstacles;
  Eigen::Matrix<double,2,1>            target;
  bool                                 wall_ahead = false;

  ROS_INFO("Starting random walk");
  while(ros::ok() and not targets_queue.hasTarget())
  {
    // Choose a spot to go
    if(wall_ahead)
    {
      wall_ahead = false;
      bot_command.linear.x = 0;
      bot_command.linear.y = 0;
      bot_command.linear.z = 0;
      bot_command.angular.x = 0;
      bot_command.angular.y = 0;
      bot_command.angular.z = ((0 == (std::rand() % 2)) ? double(1) : double(-1)) * ANGULAR_MAX;
      go_to_goal_pub.publish(bot_command);
      ros::spinOnce();

      const double rand_val = double(std::rand())/double(RAND_MAX);
      const double time_to_spin = (rand_val * (MAX_SPIN_TIME - MIN_SPIN_TIME)) + MIN_SPIN_TIME;
      const ros::Time start_time = ros::Time::now();
      while(ros::ok() and (time_to_spin > (ros::Time::now() - start_time).toSec()) and not targets_queue.hasTarget())
      {
        ros::spinOnce();
        // Check new obstacles
        std::list<Eigen::Matrix<double,2,1>> local_new_obstacles = obstacle_manager.getObstacles();
        if(not local_new_obstacles.empty())
        {
          obstacles.swap(local_new_obstacles);
        }
        loop_rate.sleep();
      }
      continue;
    }

    const Eigen::Index length_of_trajectory = std::max<Eigen::Index>(std::ceil<Eigen::Index>(OBS_CHECK_LEN/OBSTICAL_RADIUS)*Eigen::Index(2), 2);

    Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> trajectory(length_of_trajectory, 2);
    trajectory.leftCols <1>().setLinSpaced(length_of_trajectory, 0, OBS_CHECK_LEN);
    trajectory.rightCols<1>().setLinSpaced(length_of_trajectory, 0, 0);

    bot_command.linear.x = 0;
    bot_command.linear.y = 0;
    bot_command.linear.z = 0;
    bot_command.angular.x = 0;
    bot_command.angular.y = 0;
    bot_command.angular.z = 0;

    // Check new obstacles
    std::list<Eigen::Matrix<double,2,1>> new_obstacles = obstacle_manager.getObstacles();
    if(not new_obstacles.empty())
    {
      obstacles.swap(new_obstacles);
    }

    // Check for obstacles
    for(auto obs_it = obstacles.cbegin(); obs_it != obstacles.cend(); ++obs_it)
    {
      if((OBSTICAL_RADIUS > (trajectory.rowwise() - obs_it->transpose()).rowwise().norm().array()).any())
      {
        wall_ahead = true;
        // Send stop command
        bot_command.linear.x = 0;
        bot_command.linear.y = 0;
        bot_command.linear.z = 0;
        bot_command.angular.x = 0;
        bot_command.angular.y = 0;
        bot_command.angular.z = 0;
        go_to_goal_pub.publish(bot_command);
        ros::spinOnce();
        break;
      }
    }
    if(wall_ahead) { continue; }

    bot_command.linear.x = LINEAR_MAX;
    bot_command.linear.y = 0;
    bot_command.linear.z = 0;
    bot_command.angular.x = 0;
    bot_command.angular.y = 0;
    bot_command.angular.z = 0;

    // Send go command
    go_to_goal_pub.publish(bot_command);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Leaving random walk");

  // Make the bot stop moving
  bot_command.linear.x = 0;
  bot_command.linear.y = 0;
  bot_command.linear.z = 0;
  bot_command.angular.x = 0;
  bot_command.angular.y = 0;
  bot_command.angular.z = 0;
  go_to_goal_pub.publish(bot_command);
  ros::spinOnce();
}

bool move_bot(LaserScanProcessor&   obstacle_manager,
              TargetsQueue&         targets_queue,
              const ros::Publisher& go_to_goal_pub,
              ros::Rate&            loop_rate)
{
  geometry_msgs::Twist                 bot_command;
  geometry_msgs::PoseStamped           target_point;
  std::list<Eigen::Matrix<double,2,1>> obstacles;

  geometry_msgs::Point                      robot_pose_msg;
  Eigen::Matrix<double,1,5,Eigen::RowMajor> position;

  robot_pose_msg.x = 0;
  robot_pose_msg.y = 0;
  robot_pose_msg.z = 0;

  ROS_INFO("Starting move to goal");
  ros::Time start_time = ros::Time::now();
  bool      got_target = false;
  while(((ros::Time::now() - start_time).toSec() < WAIT_FOR_TARGET_TIME) and !(got_target = targets_queue.getNearestTarget(robot_pose_msg, target_point)))
  {
    ros::spinOnce();
    ROS_WARN_THROTTLE(2, "Failing to get robot goal information in move bot.");
  }
  if(not got_target)
  {
    ROS_INFO("Leaving move to goal because there is no target");
    return false;
  }

  start_time = ros::Time::now();
  while(ros::ok() and
        not (RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(target_point.pose.position.x,
                                                                     target_point.pose.position.y,
                                                                     target_point.pose.position.z,
                                                                     math::angleToRadian<double>(GRAB_ANGLE),
                                                                     math::angleToRadian<double>(0), position) and
	 (std::sqrt(std::pow(target_point.pose.position.x, 2) + std::pow(target_point.pose.position.y, 2)) < REACHING_RADIUS)))
  {
    // Make trajectory
/*    const Eigen::Index length_of_trajectory = std::max<Eigen::Index>(std::ceil<Eigen::Index>(dist_to_target/OBSTICAL_RADIUS)*Eigen::Index(2), 2);

    Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> trajectory(length_of_trajectory, 2);
    trajectory.leftCols <1>().setLinSpaced(length_of_trajectory, 0, target_point.pose.position.x);
    trajectory.rightCols<1>().setLinSpaced(length_of_trajectory, 0, target_point.pose.position.y);

    // Check new obstacles
    std::list<Eigen::Matrix<double,2,1>> new_obstacles = obstacle_manager.getObstacles();
    if(not new_obstacles.empty())
    {
      obstacles.swap(new_obstacles);
    }

    // Check for obstacles
    for(auto obs_it = obstacles.cbegin(); obs_it != obstacles.cend(); ++obs_it)
    {
      if((OBSTICAL_RADIUS > (trajectory.rowwise() - obs_it->transpose()).rowwise().norm().array()).any())
      {
        return false;
      }
    }
*/
    const double dist_to_target = std::sqrt(std::pow(target_point.pose.position.x, 2) + std::pow(target_point.pose.position.y, 2));

    Eigen::Matrix<double,1,2> bot_to_target_vec;
    bot_to_target_vec[0] = target_point.pose.position.x;
    bot_to_target_vec[1] = target_point.pose.position.y;
    bot_to_target_vec.array() /= bot_to_target_vec.norm();

    const double target_direction = std::atan2(bot_to_target_vec[1], bot_to_target_vec[0]);

    bot_command.linear.x = 0;
    bot_command.linear.y = 0;
    bot_command.linear.z = 0;
    bot_command.angular.x = 0;
    bot_command.angular.y = 0;
    bot_command.angular.z = 0;

    if(dist_to_target > LINEAR_EPS)
    {
      bot_command.linear.x = std::min<double>(LINEAR_GAIN * dist_to_target, LINEAR_MAX);
    }
    if(std::fabs(target_direction) > ANGULAR_EPS)
    {
      bot_command.angular.z = std::min<double>(ANGULAR_GAIN * target_direction, ANGULAR_MAX);
    }

    // Send go command
    go_to_goal_pub.publish(bot_command);
    ros::spinOnce();
    loop_rate.sleep();
    if(targets_queue.getNearestTarget(robot_pose_msg, target_point))
    {
      start_time = ros::Time::now();
    }
    else if((ros::Time::now() - start_time).toSec() > WAIT_FOR_TARGET_TIME)
    {
      ROS_INFO("Leaving move to goal because I lost the target");
      return false;
    }
  }

  // Make the bot stop moving
  bot_command.linear.x = 0;
  bot_command.linear.y = 0;
  bot_command.linear.z = 0;
  bot_command.angular.x = 0;
  bot_command.angular.y = 0;
  bot_command.angular.z = 0;
  go_to_goal_pub.publish(bot_command);
  ros::spinOnce();

  ROS_INFO("Leaving move to goal");
  return true;
}

/* controller_node.cpp */
