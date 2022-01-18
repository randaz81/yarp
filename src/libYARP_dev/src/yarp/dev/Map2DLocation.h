/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_MAP2DLOCATION_H
#define YARP_DEV_MAP2DLOCATION_H

#include <sstream>
#include <string>

//#include <yarp/math/Vec2D.h>
#include <yarp/dev/api.h>
#include <yarp/dev/NavTypes.h>
#include <yarp/dev/Nav2d/Map2DLocationDataStorage.h>

/**
* \file Map2DLocation.h contains the definition of a Map2DLocation type
*/
namespace yarp::dev::Nav2D {
struct YARP_dev_API Map2DLocation : public Map2DLocationDataStorage
{
    /**
     * Constructor
     * @param map_name: the name of the map the location refers to.
     * @param inX: location coordinates w.r.t. map reference frame (expressed in meters)
     * @param inY: location coordinates w.r.t. map reference frame (expressed in meters)
     * @param inT: location orientation w.r.t. map reference frame (expressed in degrees)
     */
    Map2DLocation(const std::string& map_name, const double& inX, const double& inY, const double& inT, const std::string& desc = "")
    {
        map_id = map_name;
        x = inX;
        y = inY;
        theta = inT;
        description = desc;
    }

    /**
     * A constructor which accepts a yarp::math::Vec2D, defining the location by its x,y coordinates.
     * The orientation is set to 0 by default.
     * @param map_name: the name of the map the location refers to.
     * @param inX: location coordinates w.r.t. map reference frame (expressed in meters)
     * @param inY: location coordinates w.r.t. map reference frame (expressed in meters)
     * @param inT: location orientation w.r.t. map reference frame (expressed in degrees)
     */
    Map2DLocation(const std::string& map_name, const yarp::dev::Nav2D::XYWorld& location, const std::string& desc = "")
    {
        map_id = map_name;
        x = location.x;
        y = location.y;
        theta = 0;
        description = desc;
    }

    /**
     * Default constructor: the map name is empty, coordinates are set to zero.
     */
    Map2DLocation()
    {
        map_id = "";
        description = "";
        x = 0;
        y = 0;
        theta = 0;
    }

    /**
     * Default destructor
     */
    virtual ~Map2DLocation() = default;

    /**
     * Returns text representation of the location.
     * @return a human readable string containing the location infos.
     */
    std::string toString() const
    {
        std::ostringstream stringStream;
        stringStream.precision(-1);
        stringStream.width(-1);
        stringStream << std::string("map_id:") << map_id << std::string(" x:") << x << std::string(" y:") << y << std::string(" theta:") << theta;
        stringStream << std::string("desc:") << description;
        return stringStream.str();
    }

    /**
     * Compares two Map2DLocations
     * @return true if the two locations are different.
     */
    inline bool operator!=(const Map2DLocation& r) const
    {
        if (
            map_id != r.map_id || x != r.x || y != r.y || theta != r.theta) {
            return true;
        }
        return false;
    }

    /**
     * Compares two Map2DLocations
     * @return true if the two locations are identical.
     */
    inline bool operator==(const Map2DLocation& r) const
    {
        if (
            map_id == r.map_id && x == r.x && y == r.y && theta == r.theta) {
            return true;
        }
        return false;
    }

    /**
     * Compares two Map2DLocations
     * @return true if the two locations are near.
     */
    bool is_near_to(const Map2DLocation& other_loc, double linear_tolerance, double angular_tolerance) const;

};
} // namespace yarp::dev::Nav2D

#endif // YARP_DEV_MAP2DLOCATION_H
