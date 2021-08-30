/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

struct yarp_sig_Matrix{
} (
  yarp.name = "yarp::sig::Matrix"
  yarp.includefile="yarp/sig/Matrix.h"
)

struct yarp_dev_Nav2D_Map2DLocation{
} (
  yarp.name = "yarp::dev::Nav2D::Map2DLocation"
  yarp.includefile="yarp/dev/Map2DLocation.h"
)

struct yarp_dev_OdometryData{
} (
  yarp.name = "yarp::dev::OdometryData"
  yarp.includefile="yarp/dev/OdometryData.h"
)

enum yarp_dev_Nav2D_LocalizationStatusEnum {
} (
  yarp.name = "yarp::dev::Nav2D::LocalizationStatusEnum"
  yarp.includefile = "yarp/dev/ILocalization2D.h"
)

struct return_getLocalizationStatusRPC {
  1: bool ret = false;
  2: yarp_dev_Nav2D_LocalizationStatusEnum status;
}

struct return_getEstimatedPosesRPC {
  1: bool ret = false;
  2: list<yarp_dev_Nav2D_Map2DLocation> poses;
}

struct return_getCurrentPositionRPC1 {
  1: bool ret = false;
  2: yarp_dev_Nav2D_Map2DLocation loc;
}

struct return_getCurrentPositionRPC2 {
  1: bool ret = false;
  2: yarp_dev_Nav2D_Map2DLocation loc;
  3: yarp_sig_Matrix cov;
}

struct return_getEstimatedOdometryRPC {
  1: bool ret = false;
  2: yarp_dev_OdometryData odom;
}

service ILocalization2DRPC
{
  bool                             startLocalizationServiceRPC ();
  bool                             stopLocalizationServiceRPC  ();
  return_getLocalizationStatusRPC  getLocalizationStatusRPC    ();
  return_getEstimatedPosesRPC      getEstimatedPosesRPC        ();
  return_getCurrentPositionRPC1    getCurrentPositionRPC1      ();
  return_getCurrentPositionRPC2    getCurrentPositionRPC2      ();
  return_getEstimatedOdometryRPC   getEstimatedOdometryRPC     ();
  bool                             setInitialPoseRPC1          (1: yarp_dev_Nav2D_Map2DLocation loc);
  bool                             setInitialPoseRPC2          (1: yarp_dev_Nav2D_Map2DLocation loc, 2: yarp_sig_Matrix cov);
}
