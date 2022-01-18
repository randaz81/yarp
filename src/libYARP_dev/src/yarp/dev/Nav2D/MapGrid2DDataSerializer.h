/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_MAPGRID2D_SERIALIZER_H
#define YARP_DEV_MAPGRID2D_SERIALIZER_H

#include <yarp/dev/api.h>

#include <yarp/os/Serializer.h>

#include <yarp/dev/Nav2D/MapGrid2DDataStorage.h>

#include <string>

/**
* \file MapGrid2DDataSerializer.h contains the definition of a map type
*/
namespace yarp::dev::Nav2D {
class YARP_dev_API MapGrid2DDataSerializer : public yarp::os::Serializer<MapGrid2DDataStorage>
{
public:
    using Serializer::Serializer;

    virtual ~MapGrid2DDataSerializer() = default;

    /*
     * Read vector from a connection.
     * return true iff a vector was read correctly
     */
    bool read(yarp::os::ConnectionReader& connection) override;

    /**
     * Write vector to a connection.
     * return true iff a vector was written correctly
     */
    bool write(yarp::os::ConnectionWriter& connection) const override;
};
} // namespace yarp::dev::Nav2D

#endif // YARP_DEV_MAPGRID2D_SERIALIZER_H
