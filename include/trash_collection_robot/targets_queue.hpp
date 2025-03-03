/**
 * @File: targets_queue.hpp
 *
 * @brief
 * Reads in targets from the target server.
 **/

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

/* C++ Headers */
#include<list>
#include<thread>
#include<mutex>

class TargetsQueue
{
public:
  /**
   * @Default Constructor
   **/
  TargetsQueue() = delete;
  /**
   * @Copy Constructor
   **/
  TargetsQueue(const TargetsQueue&) = delete;
  /**
   * @Move Constructor
   **/
  TargetsQueue(TargetsQueue&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Sets the object up for use.
   *
   * @parameters
   * topic: The topic this object will listen to
   * tf_frame: The tf_frame to output the data in
   * target_size: Targets closer then this distance will be considered the same target
   * target_timeout: Targets older then this in seconds are discarded
   **/
  TargetsQueue(const std::string& topic,
               const std::string& tf_frame,
               const double       target_size,
               const double       target_timeout);
  /**
   * @Deconstructor
   **/
  ~TargetsQueue() noexcept;
  /**
   * @Copy Assignment Operator
   **/
  TargetsQueue& operator=(const TargetsQueue&) = delete;
  /**
   * @Move Assignment Operator
   **/
  TargetsQueue& operator=(TargetsQueue&&) = delete;
  /**
   * @hasTarget
   **/
  bool hasTarget() const;
  /**
   * @getNearestTarget
   *
   * @brief
   * Gets the target that is closes to the input point.
   **/
  bool getNearestTarget(const geometry_msgs::Point& ref_point, geometry_msgs::PoseStamped& output_point);
private:
  // General ROS interface
  ros::NodeHandle    nh;
  ros::CallbackQueue msg_queue;
  // Target server connection
  ros::Subscriber                                 target_sub;
  std::list<geometry_msgs::PoseStamped::ConstPtr> new_targets;
  std::list<geometry_msgs::PoseStamped>           targets;
  // Worker thread
  mutable std::mutex targets_mux;
  std::thread        worker_thread;
  /**
   * @poseCallback
   **/
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  /**
   * @workerThreadFunc
   **/
  void workerThreadFunc(const std::string tf_frame, const double target_size, const double target_timeout);
};

TargetsQueue::TargetsQueue(const std::string& topic,
                           const std::string& tf_frame,
                           const double       target_size,
                           const double       target_timeout)
{
  this->nh.setCallbackQueue(&this->msg_queue);
  this->target_sub = this->nh.subscribe(topic, 15, &TargetsQueue::poseCallback, this);
  this->worker_thread = std::thread(&TargetsQueue::workerThreadFunc, this, tf_frame, target_size, target_timeout);
}

TargetsQueue::~TargetsQueue() noexcept
{
  this->nh.shutdown();
  this->worker_thread.join();
}

bool TargetsQueue::hasTarget() const
{
  std::unique_lock<std::mutex> lock(this->targets_mux);
  return (0 != this->targets.size());
}

bool TargetsQueue::getNearestTarget(const geometry_msgs::Point& ref_point,
                                    geometry_msgs::PoseStamped& output_point)
{
  std::unique_lock<std::mutex> lock(this->targets_mux);

  if(0 == this->targets.size()) { return false; }

  double                                                best_dist = std::numeric_limits<double>::infinity();
  std::list<geometry_msgs::PoseStamped>::const_iterator best_point;

  const auto targets_end = this->targets.cend();
  for(auto target_it = this->targets.cbegin(); target_it != targets_end; ++target_it)
  {
    const double dist = std::pow(ref_point.x - target_it->pose.position.x, 2) +
                        std::pow(ref_point.y - target_it->pose.position.y, 2) +
                        std::pow(ref_point.z - target_it->pose.position.z, 2);
    if(dist < best_dist)
    {
      best_dist  = dist;
      best_point = target_it;
    }
  }

  output_point = *best_point;
  //this->targets.erase(best_point);
  return true;
}

void TargetsQueue::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  this->new_targets.push_back(msg);
}

void TargetsQueue::workerThreadFunc(const std::string tf_frame, const double target_size, const double target_timeout)
{
  ros::Rate loop_rate(30);

  // TF interface
  tf2_ros::Buffer            tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  while(this->nh.ok())
  {
    std::list<geometry_msgs::PoseStamped> local_targets;
    // Get new messages
    this->msg_queue.callAvailable();

    const auto new_target_end = this->new_targets.cend();
    for(auto new_target_it = this->new_targets.cbegin(); new_target_it != new_target_end; ++new_target_it)
    {
      try
      {
        local_targets.emplace_back(tf_buffer.transform<geometry_msgs::PoseStamped>(**new_target_it, tf_frame, ros::Duration(1.0)));
      }
      catch(tf2::TransformException &ex)
      {
        ROS_WARN_THROTTLE(1, "Could not transform target to correct frame: %s", ex.what());
      }
    }
    this->new_targets.clear();

    this->targets_mux.lock();
    // Add local target to full list
    this->targets.splice(this->targets.end(), local_targets);
    // Remove redundant targets
    for(auto target_it = this->targets.begin(); target_it != this->targets.end(); ++target_it)
    {
      for(auto forward_it = std::next(target_it); forward_it != this->targets.end(); ++forward_it)
      {
        const double dist = std::sqrt(std::pow(forward_it->pose.position.x - target_it->pose.position.x, 2) +
                                      std::pow(forward_it->pose.position.y - target_it->pose.position.y, 2) +
                                      std::pow(forward_it->pose.position.z - target_it->pose.position.z, 2));
        // If it is the same object
        if(dist < target_size)
        {
          //ROS_WARN("Target queue merged near targets");
          // Keep the newest one
          if(target_it->header.stamp < forward_it->header.stamp)
          {
            *target_it = *forward_it;
          }
          forward_it = std::prev(this->targets.erase(forward_it));
        }
      }
    }
    // Remove if it's too old
    const ros::Time current_time = ros::Time::now();
    //const size_t old_size = this->targets.size();
    this->targets.remove_if([&current_time, target_timeout] (const geometry_msgs::PoseStamped& point_it)
      {
        return target_timeout < (current_time - point_it.header.stamp).toSec();
      });
    /*if(old_size != this->targets.size())
    {
      ROS_WARN("Target queue removed a out dated target");
    }*/

    this->targets_mux.unlock();
    loop_rate.sleep();
  }
}

/* targets_queue.hpp */
