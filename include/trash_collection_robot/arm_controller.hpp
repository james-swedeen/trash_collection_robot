#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include "ros/ros.h"
#include "interbotix_xs_msgs/JointGroupCommand.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include <cmath>
#include <Eigen/Dense>

interbotix_xs_msgs::JointGroupCommand mover(const Eigen::Matrix<double,5,1>& angles);
interbotix_xs_msgs::JointSingleCommand grip(bool open_it);
#endif
