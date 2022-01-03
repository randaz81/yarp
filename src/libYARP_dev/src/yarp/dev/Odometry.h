/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_NAV2D_ODOMETRY_H
#define YARP_NAV2D_ODOMETRY_H

#include <yarp/dev/api.h>
#include <yarp/dev/NavTypes.h>
#include <yarp/dev/Nav2D/OdometryDataStorage.h>

namespace yarp::dev::Nav2D {

class Odometry: public yarp::dev::Nav2D::OdometryDataStorage
{
    public:
    Odometry() = default;
    virtual ~Odometry() = default;

    std::string toString() const
    {
        std::ostringstream stringStream;
        stringStream.precision(-1);
        stringStream.width(-1);
        stringStream << std::string(" x:") << odom_x << std::string(" y:") << odom_y << std::string(" theta:") << odom_theta;
        return stringStream.str();
    }
};

} // namespace yarp::dev::Nav2D

#endif // YARP_NAV2D_ODOMETRY_H
