/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_QUATERNION_DATA
#define YARP_QUATERNION_DATA

#include <yarp/math/api.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Serializer.h>

// network stuff
#include <yarp/os/NetInt32.h>

namespace yarp::math {
class QuaternionDataStorage;
class QuaternionDataSerializer;
}

class YARP_math_API yarp::math::QuaternionDataStorage
{
    friend class QuaternionDataSerializer;

    double internal_data[4]; // stored as [w x y z]

public:
    QuaternionDataStorage();
    QuaternionDataStorage(double x, double y, double z, double w);
    double* data();
    const double* data() const;
    double x() const;
    double y() const;
    double z() const;
    double w() const;
    double& x() ;
    double& y() ;
    double& z() ;
    double& w() ;

    std::string toString(int precision = -1, int width = -1) const;
};

class YARP_math_API yarp::math::QuaternionDataSerializer : public yarp::os::Serializer<yarp::math::QuaternionDataStorage>
{
public:
    using Serializer::Serializer;

    /*
     * Read Quaternion from a connection.
     * return true iff a matrix was read correctly
     */
    bool read(yarp::os::ConnectionReader& connection) override;

    /**
     * Write Quaternion to a connection.
     * return true iff a matrix was written correctly
     */
    bool write(yarp::os::ConnectionWriter& connection) const override;

    yarp::os::Type getType() const override
    {
        return yarp::os::Type::byName("yarp/quaternion");
    }
};

#endif
