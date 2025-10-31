#ifndef HHFC_MJ_COMMON_HPP
#define HHFC_MJ_COMMON_HPP

#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "utils/parallel_ankle.hpp"

namespace ovinf {

// enum MotorIdx {
//   LShoulderPitchMotor = 0,

//   LHipPitchMotor = 1,
//   LHipRollMotor = 2,
//   LHipYawMotor = 3,
//   LKneeMotor = 4,
//   // LAnkleLongMotor = 4,
//   // LAnkleShortMotor = 5,
//   LAnklePitchMotor = 5,
//   LAnkleRollMotor = 6,

//   RShoulderPitchMotor = 7,
//   RHipPitchMotor = 8,
//   RHipRollMotor = 9,
//   RHipYawMotor = 10,
//   RKneeMotor = 11,
//   // RAnkleLongMotor = 10,
//   // RAnkleShortMotor = 11,
//   RAnklePitchMotor = 12,
//   RAnkleRollMotor = 13,
// };

enum MotorIdx {
  LHipPitchMotor = 0,
  LHipRollMotor = 1,
  LHipYawMotor = 2,
  LKneeMotor = 3,
  // LAnkleLongMotor = 4,
  // LAnkleShortMotor = 5,
  LAnklePitchMotor = 4,
  LAnkleRollMotor = 5,

  RHipPitchMotor = 6,
  RHipRollMotor = 7,
  RHipYawMotor = 8,
  RKneeMotor = 9,
  // RAnkleLongMotor = 10,
  // RAnkleShortMotor = 11,
  RAnklePitchMotor = 10,
  RAnkleRollMotor = 11,

  LShoulderPitchMotor = 12,
  RShoulderPitchMotor = 13,
};

// enum JointIdx {
//   LShoulderPitchJoint = 0,

//   LHipPitchJoint = 1,
//   LHipRollJoint = 2,
//   LHipYawJoint = 3,
//   LKneeJoint = 4,
//   LAnklePitchJoint = 5,
//   LAnkleRollJoint = 6,

//   RShoulderPitchJoint = 7,

//   RHipPitchJoint = 8,
//   RHipRollJoint = 9,
//   RHipYawJoint = 10,
//   RKneeJoint = 11,
//   RAnklePitchJoint = 12,
//   RAnkleRollJoint = 13,

// };

enum JointIdx {
  LHipPitchJoint = 0,
  LHipRollJoint = 1,
  LHipYawJoint = 2,
  LKneeJoint = 3,
  LAnklePitchJoint = 4,
  LAnkleRollJoint = 5,

  RHipPitchJoint = 6,
  RHipRollJoint = 7,
  RHipYawJoint = 8,
  RKneeJoint = 9,
  RAnklePitchJoint = 10,
  RAnkleRollJoint = 11,

  LShoulderPitchJoint = 12,
  RShoulderPitchJoint = 13,
};

}  // namespace ovinf

using KernelBus = bitbot::MujocoBus;
using ImuDevice = bitbot::MujocoImu;
using ImuPtr = ImuDevice*;
using MotorDevice = bitbot::MujocoJoint;
using MotorPtr = MotorDevice*;
using AnklePtr = std::shared_ptr<ovinf::ParallelAnkle<float>>;

struct UserData {};

using Kernel = bitbot::MujocoKernel<UserData>;

#endif  // !HHFC_MJ_COMMON_HPP
