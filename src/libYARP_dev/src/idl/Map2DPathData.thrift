/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

namespace yarp yarp.dev.Nav2D

struct Map2DLocation {
} (
  yarp.name = "yarp::dev::Nav2D::Map2DLocation"
  yarp.includefile="yarp/dev/Map2DLocation.h"
  yarp.customSerializerName =    "yarp::dev::Nav2D::Map2DLocationDataSerializer"
  yarp.customSerializerInclude = "yarp/dev/Nav2D/Map2DLocationDataSerializer.h"
)

struct Map2DPathData
{
    /** list of waypoints which define the path*/
    1: list<Map2DLocation> waypoints;
    /** user defined string*/
    2: string description;
}
(
    yarp.api.include = "yarp/dev/api.h"
    yarp.api.keyword = "YARP_dev_API"
)
