/**
 * @File: pose_manager.hpp
 * @Date: December 2020
 * @Author: James Swedeen
 *
 * @brief
 * A base class that defines a class that continuously updates where something is a in a simulation.
 **/

#ifndef TRASH_COLLECTION_ROBOT_POSE_MANAGER_HPP
#define TRASH_COLLECTION_ROBOT_POSE_MANAGER_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace anytime
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class PoseManager;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
using PoseManagerPtr = std::shared_ptr<PoseManager<DIM,SCALAR,OPTIONS>>;

using PoseManager2d = PoseManager<2,double,Eigen::RowMajor>;
using PoseManager3d = PoseManager<3,double,Eigen::RowMajor>;
using PoseManager4d = PoseManager<4,double,Eigen::RowMajor>;

using PoseManagerPtr2d = PoseManagerPtr<2,double,Eigen::RowMajor>;
using PoseManagerPtr3d = PoseManagerPtr<3,double,Eigen::RowMajor>;
using PoseManagerPtr4d = PoseManagerPtr<4,double,Eigen::RowMajor>;

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
class PoseManager
{
public:
  /**
   * @Default Constructor
   **/
  PoseManager() = default;
  /**
   * @Copy Constructor
   **/
  PoseManager(const PoseManager&) = default;
  /**
   * @Move Constructor
   **/
  PoseManager(PoseManager&&) = default;
  /**
   * @Deconstructor
   **/
  virtual ~PoseManager() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  PoseManager& operator=(const PoseManager&) = default;
  /**
   * @Move Assignment Operator
   **/
  PoseManager& operator=(PoseManager&&) = default;
  /**
   * @hasPose
   *
   * @brief
   * Tests whether or not this object has received the pose information at all.
   *
   * @return
   * True if and only of the pose this object holds is valid.
   **/
  virtual bool hasPose() = 0;
  /**
   * @newPose
   *
   * @brief
   * Tests whether or not there is new information to collect.
   *
   * @return
   * True if and only of the pose this object holds has changed sense the last time getPose was called.
   **/
  virtual bool newPose() = 0;
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
  virtual bool getPose(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& output) = 0;
  /**
   * @update
   *
   * @brief
   * Updates any internally held state.
   **/
  virtual void update() = 0;
};
} // anytime
} // rrt

#endif
/* pose_manager.hpp */
