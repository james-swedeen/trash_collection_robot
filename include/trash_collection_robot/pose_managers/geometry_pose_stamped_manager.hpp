/**
 * @File: geometry_pose_stamped_manager.hpp
 * @Date: December 2020
 * @Author: James Swedeen
 *
 * @brief
 * A class that maintains an instance of a geometry_msgs::PoseStamped subscriber.
 **/

#ifndef TRASH_COLLECTION_ROBOT_GEOMETRY_POSE_STAMPED_MANAGER_HPP
#define TRASH_COLLECTION_ROBOT_GEOMETRY_POSE_STAMPED_MANAGER_HPP

/* C++ Headers */
#include<memory>
#include<string>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>

/* Local Headers */
#include<trach_collection_robot/pose_managers/pose_manager.hpp>

namespace rrt
{
namespace anytime
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class GeometryPoseStampedManager;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
using GeometryPoseStampedManagerPtr = std::shared_ptr<GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using GeometryPoseStampedManager2d   = GeometryPoseStampedManager<2,0,0,double,Eigen::RowMajor>;
using GeometryPoseStampedManager21d  = GeometryPoseStampedManager<3,1,0,double,Eigen::RowMajor>;
using GeometryPoseStampedManager211d = GeometryPoseStampedManager<4,1,1,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @S
 * The number of angular dimensions each point will have at the end of q but before NON_STATE.
 *
 * @NON_STATE
 * Dimensions that shouldn't be considered in KD tree calculations and other
 * similar operations. They appear at the end of q.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class GeometryPoseStampedManager
 : public PoseManager<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  GeometryPoseStampedManager() = delete;
  /**
   * @Copy Constructor
   **/
  GeometryPoseStampedManager(const GeometryPoseStampedManager&) = delete;
  /**
   * @Move Constructor
   **/
  GeometryPoseStampedManager(GeometryPoseStampedManager&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets the object up for use.
   *
   * @parameters
   * topic: The topic this object will listen to
   * tf_frame: The
   **/
  GeometryPoseStampedManager(const std::string& topic,
                             const std::string& tf_frame);
  /**
   * @Deconstructor
   **/
  ~GeometryPoseStampedManager() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  GeometryPoseStampedManager& operator=(const GeometryPoseStampedManager&) = delete;
  /**
   * @Move Assignment Operator
   **/
  GeometryPoseStampedManager& operator=(GeometryPoseStampedManager&&) = default;
  /**
   * @hasPose
   *
   * @brief
   * Tests whether or not this object has received the pose information at all.
   *
   * @return
   * True if and only of the pose this object holds is valid.
   **/
  bool hasPose() override;
  /**
   * @newPose
   *
   * @brief
   * Tests whether or not there is new information to collect.
   *
   * @return
   * True if and only of the pose this object holds has changed sense the last time getPose was called.
   **/
  bool newPose() override;
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
  bool getPose(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& output) override;
  /**
   * @update
   *
   * @brief
   * Updates any internally held state.
   **/
  void update() override;
private:
  std::string                tf_frame;
  ros::NodeHandle            nh;
  ros::CallbackQueue         msg_queue;
  tf::TransformListener      tf_listener;
  ros::Subscriber            pose_sub;
  bool                       has_pose;
  bool                       new_pose;
  geometry_msgs::PoseStamped pose_msg;
  /**
   * @poseCallback
   *
   * @brief
   * Updates the internally held pose with the one passed in.
   *
   * @parameters
   * pose: The new pose
   **/
  void poseCallback(const geometry_msgs::PoseStamped& pose);
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  GeometryPoseStampedManager(const std::string& topic,
                             const std::string& tf_frame)
 : PoseManager<DIM,SCALAR,OPTIONS>(),
   tf_frame(tf_frame),
   has_pose(false),
   new_pose(false)
{
  this->nh.setCallbackQueue(&this->msg_queue);
  this->pose_sub = this->nh.subscribe(topic, 1, &GeometryPoseStampedManager::poseCallback, this);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>::hasPose()
{
  return this->has_pose;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>::newPose()
{
  return this->new_pose;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>::getPose(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& output)
{
  if(this->hasPose())
  {
    try
    {
      this->tf_listener.waitForTransform(this->tf_frame, this->pose_msg.header.frame_id, ros::Time::now(), ros::Duration(3));
      this->tf_listener.transformPose(this->tf_frame, this->pose_msg, this->pose_msg);
    }
    catch(const tf::TransformException& ex)
    {
      ROS_WARN_DELAYED_THROTTLE(3, "Failed to transform pose.");
      return false;
    }

    if constexpr((DIM == 2) and (S == 0) and (NON_STATE == 0))
    {
      output[0] = this->pose_msg.pose.position.x;
      output[1] = this->pose_msg.pose.position.y;
    }
    else if constexpr((DIM == 3) and (S == 1) and (NON_STATE == 0))
    {
      output[0] = this->pose_msg.pose.position.x;
      output[1] = this->pose_msg.pose.position.y;
      output[2] = tf::getYaw(this->pose_msg.pose.orientation);
    }
    else if constexpr((DIM == 4) and (S == 1) and (NON_STATE == 1))
    {
      output[0] = this->pose_msg.pose.position.x;
      output[1] = this->pose_msg.pose.position.y;
      output[2] = tf::getYaw(this->pose_msg.pose.orientation);
      output[3] = 0;
    }
    else
    {
      throw std::runtime_error("Template dimension inputs are not supported");
    }

    this->new_pose = false;
    return true;
  }

  return false;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
void GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>::update()
{
  this->msg_queue.callAvailable();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
void GeometryPoseStampedManager<DIM,S,NON_STATE,SCALAR,OPTIONS>::poseCallback(const geometry_msgs::PoseStamped& pose)
{
  this->pose_msg = pose;
  this->has_pose = true;
  this->new_pose = true;
}
} // anytime
} // rrt

#endif
/* geometry_pose_stamped_manager.hpp */
