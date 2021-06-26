/*! @file IronHorse.h
 *  @brief Utility function to build a IronHorse Quadruped object
 *
 * This file is based on vrep model.md and builds a model
 * of the IronHorse robot.  The inertia parameters of all bodies are
 * determined from vrep.
 *
 */

#ifndef PROJECT_IRONHORSE_H
#define PROJECT_IRONHORSE_H

#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildIronHorse() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  cheetah._bodyMass = 10;
  cheetah._bodyLength = 0.315 * 2;
  cheetah._bodyWidth = 0.18 * 2;
  cheetah._bodyHeight = 0.15 * 2;
  cheetah._abadGearRatio = 0.0;
  cheetah._hipGearRatio = 0.0;
  cheetah._kneeGearRatio = 0.0;
  cheetah._abadLinkLength = 0.06;
  cheetah._hipLinkLength = 0.4;
  cheetah._kneeLinkY_offset = 0.0;
  cheetah._kneeLinkLength = 0.372;
  cheetah._maxLegLength = 0.772;


  cheetah._motorTauMax = 30.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = 0.0;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.0;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.18, 0);
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, -cheetah._bodyHeight) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, 0, -cheetah._abadLinkLength);
  cheetah._hipRotorLocation = Vec3<T>(0, 0, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
