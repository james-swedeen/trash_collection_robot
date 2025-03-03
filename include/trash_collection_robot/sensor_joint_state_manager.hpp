/**
 * @File: sensor_joint_state_manager.hpp
 * @Date:
 * @Author: James Swedeen
 *
 * @brief
 * A class that maintains an instance of a sensor_msgs::JointState subscriber.
 **/

#ifndef ANYTIME_RRT_SENSOR_JOINT_STATE_MANAGER_HPP
#define ANYTIME_RRT_SENSOR_JOINT_STATE_MANAGER_HPP

/* C++ Headers */
#include<memory>
#include<string>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<tf/transform_listener.h>

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class SensorJointStateManager;

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
using SensorJointStateManagerPtr = std::shared_ptr<SensorJointStateManager<SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class SensorJointStateManager
{
public:
  /**
   * @Default Constructor
   **/
  SensorJointStateManager() = delete;
  /**
   * @Copy Constructor
   **/
  SensorJointStateManager(const SensorJointStateManager&) = delete;
  /**
   * @Move Constructor
   **/
  SensorJointStateManager(SensorJointStateManager&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets the object up for use.
   *
   * @parameters
   * topic: The topic this object will listen to
   **/
  SensorJointStateManager(const std::string& topic);
  /**
   * @Deconstructor
   **/
  ~SensorJointStateManager() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  SensorJointStateManager& operator=(const SensorJointStateManager&) = delete;
  /**
   * @Move Assignment Operator
   **/
  SensorJointStateManager& operator=(SensorJointStateManager&&) = default;
  /**
   * @hasPose
   *
   * @brief
   * Tests whether or not this object has received the pose information at all.
   *
   * @return
   * True if and only of the pose this object holds is valid.
   **/
  bool hasPose();
  /**
   * @newPose
   *
   * @brief
   * Tests whether or not there is new information to collect.
   *
   * @return
   * True if and only of the pose this object holds has changed sense the last time getPose was called.
   **/
  bool newPose();
  /**
   * @getPose
   *
   * @brief
   * Used to get the pose this object holds.
   *
   * @parameters
   * output: Filled with the most up to date pose that this object is holding
   *
   * @return
   * True if and only if the operation worked.
   **/
  bool getPose(Eigen::Matrix<SCALAR,1,5,OPTIONS>& output);
  /**
   * @update
   *
   * @brief
   * Updates any internally held state.
   **/
  void update();
private:
  ros::NodeHandle                 nh;
  ros::CallbackQueue              msg_queue;
  ros::Subscriber                 pose_sub;
  bool                            has_pose;
  bool                            new_pose;
  sensor_msgs::JointStateConstPtr pose_msg;
  /**
   * @poseCallback
   *
   * @brief
   * Updates the internally held pose with the one passed in.
   *
   * @parameters
   * pose: The new pose
   **/
  void poseCallback(const sensor_msgs::JointStateConstPtr& pose);
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
SensorJointStateManager<SCALAR,OPTIONS>::SensorJointStateManager(const std::string& topic)
 : has_pose(false),
   new_pose(false)
{
  this->nh.setCallbackQueue(&this->msg_queue);
  this->pose_sub = this->nh.subscribe(topic, 1, &SensorJointStateManager::poseCallback, this);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool SensorJointStateManager<SCALAR,OPTIONS>::hasPose()
{
  return this->has_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool SensorJointStateManager<SCALAR,OPTIONS>::newPose()
{
  return this->new_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool SensorJointStateManager<SCALAR,OPTIONS>::getPose(Eigen::Matrix<SCALAR,1,5,OPTIONS>& output)
{
  if(this->hasPose())
  {
    for(size_t joint_it = 0; joint_it < 5; ++joint_it)
    {
      output[joint_it] = this->pose_msg->position[joint_it];
    }

    this->new_pose = false;
    return true;
  }

  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
void SensorJointStateManager<SCALAR,OPTIONS>::update()
{
  this->msg_queue.callAvailable();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
void SensorJointStateManager<SCALAR,OPTIONS>::poseCallback(const sensor_msgs::JointStateConstPtr& pose)
{
  if(8 == pose->name.size()) // This is a hack
  {
    this->pose_msg = pose;
    this->has_pose = true;
    this->new_pose = true;
  }
}

#endif
/* sensor_joint_state_manager.hpp */
