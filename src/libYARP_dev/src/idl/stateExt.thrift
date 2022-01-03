/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

namespace yarp yarp.dev.impl

struct jointData
{
  1: list<double> jointPosition;
  2: bool jointPosition_isValid;
  3: list<double> jointVelocity;
  4: bool jointVelocity_isValid;
  5: list<double> jointAcceleration;
  6: bool jointAcceleration_isValid;
  7: list<double> motorPosition;
  8: bool motorPosition_isValid;
  9: list<double> motorVelocity;
  10: bool motorVelocity_isValid;
  11: list<double> motorAcceleration;
  12: bool motorAcceleration_isValid;
  13: list<double> torque;
  14: bool torque_isValid;
  15: list<double> pwmDutycycle;
  16: bool pwmDutycycle_isValid;
  17: list<double> current;
  18: bool current_isValid;
  19: list<i32> controlMode;
  20: bool controlMode_isValid;
  21: list<i32> interactionMode;
  22: bool interactionMode_isValid;
} (
    yarp.api.include = "yarp/dev/api.h"
    yarp.api.keyword = "YARP_dev_API"
)
