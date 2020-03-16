/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <Eigen/Core>
#include <units/units.h>

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/Trajectory.h"

namespace frc {

class LTVUnicycleController {
 public:
  /**
   * Construct a linear time-varying unicycle controller.
   *
   * @param Qelems The maximum desired error tolerance for each state.
   * @param Relems The maximum desired control effort for each input.
   * @param dt     Discretization timestep.
   */
  LTVUnicycleController(const std::array<double, 3>& Qelems,
                        const std::array<double, 2>& Relems,
                        units::second_t dt);

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  bool AtReference() const;

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   */
  void SetTolerance(const Pose2d& poseTolerance);

  /**
   * Returns the next output of the controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose        The current pose.
   * @param poseRef            The desired pose.
   * @param linearVelocityRef  The desired linear velocity.
   * @param angularVelocityRef The desired angular velocity.
   */
  ChassisSpeeds Calculate(const Pose2d& currentPose, const Pose2d& poseRef,
                          units::meters_per_second_t linearVelocityRef,
                          units::radians_per_second_t angularVelocityRef);

  /**
   * Returns the next output of the controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose  The current pose.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   */
  ChassisSpeeds Calculate(const Pose2d& currentPose,
                          const Trajectory::State& desiredState);

 private:
  Eigen::Matrix<double, 2, 3> m_K0;
  Eigen::Matrix<double, 2, 3> m_K1;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;
};

}  // namespace frc
