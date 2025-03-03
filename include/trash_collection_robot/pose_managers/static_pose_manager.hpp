/**
 * @File: static_pose_manager.hpp
 * @Date: January 2021
 * @Author: James Swedeen
 *
 * @brief
 * A class that defines a pose that doesn't change.
 **/

#ifndef TRASH_COLLECTION_ROBOT_STATIC_POSE_MANAGER_HPP
#define TRASH_COLLECTION_ROBOT_STATIC_POSE_MANAGER_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<trash_collection_robot/pose_managers/pose_manager.hpp>

namespace rrt
{
namespace anytime
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class StaticPoseManager;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
using StaticPoseManagerPtr = std::shared_ptr<StaticPoseManager<DIM,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class StaticPoseManager
 : public PoseManager<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  StaticPoseManager() = delete;
  /**
   * @Copy Constructor
   **/
  StaticPoseManager(const StaticPoseManager&) = default;
  /**
   * @Move Constructor
   **/
  StaticPoseManager(StaticPoseManager&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets all internally held state.
   *
   * @parameters
   * pose: The pose that this object represents.
   **/
  StaticPoseManager(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& pose);
  /**
   * @Deconstructor
   **/
  ~StaticPoseManager() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  StaticPoseManager& operator=(const StaticPoseManager&) = default;
  /**
   * @Move Assignment Operator
   **/
  StaticPoseManager& operator=(StaticPoseManager&&) = default;
  /**
   * @hasPose
   *
   * @brief
   * Tests whether or not this object has received the pose information at all.
   *
   * @return
   * True.
   **/
  bool hasPose() override;
  /**
   * @newPose
   *
   * @brief
   * Tests whether or not there is new information to collect.
   *
   * @return
   * False.
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
   * True.
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
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> pose;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
StaticPoseManager<DIM,SCALAR,OPTIONS>::StaticPoseManager(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& pose)
 : pose(pose)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool StaticPoseManager<DIM,SCALAR,OPTIONS>::hasPose()
{
  return true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool StaticPoseManager<DIM,SCALAR,OPTIONS>::newPose()
{
  return false;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool StaticPoseManager<DIM,SCALAR,OPTIONS>::getPose(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& output)
{
  output = this->pose;
  return true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void StaticPoseManager<DIM,SCALAR,OPTIONS>::update()
{}
} // anytime
} // rrt

#endif
/* pose_manager.hpp */
