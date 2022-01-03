/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/math/QuaternionData.h>

#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogComponent.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <cstdio>

using namespace yarp::math;

namespace {
YARP_LOG_COMPONENT(QUATERNION, "yarp.math.Quaternion")
}

YARP_BEGIN_PACK
class QuaternionPortContentHeader
{
public:
    yarp::os::NetInt32 listTag{0};
    yarp::os::NetInt32 listLen{0};
    QuaternionPortContentHeader() = default;
};
YARP_END_PACK

QuaternionDataStorage::QuaternionDataStorage()
{
    internal_data[0] = 1;
    internal_data[1] = 0;
    internal_data[2] = 0;
    internal_data[3] = 0;
}

QuaternionDataStorage::QuaternionDataStorage(double x, double y, double z, double w)
{
    internal_data[0] = w;
    internal_data[1] = x;
    internal_data[2] = y;
    internal_data[3] = z;
}

const double* QuaternionDataStorage::data() const
{
    return internal_data;
}

double* QuaternionDataStorage::data()
{
    return internal_data;
}

double QuaternionDataStorage::w() const
{
    return internal_data[0];
}

double QuaternionDataStorage::x() const
{
    return internal_data[1];
}

double QuaternionDataStorage::y() const
{
    return internal_data[2];
}

double QuaternionDataStorage::z() const
{
    return internal_data[3];
}

double& QuaternionDataStorage::w()
{
    return internal_data[0];
}

double& QuaternionDataStorage::x()
{
    return internal_data[1];
}

double& QuaternionDataStorage::y()
{
    return internal_data[2];
}

double& QuaternionDataStorage::z()
{
    return internal_data[3];
}

bool QuaternionDataSerializer::read(yarp::os::ConnectionReader& connection)
{
    // auto-convert text mode interaction
    connection.convertTextMode();
    QuaternionPortContentHeader header;
    bool ok = connection.expectBlock((char*)&header, sizeof(header));
    if (!ok) {
        return false;
    }

    if (header.listLen == 4 &&  header.listTag == (BOTTLE_TAG_LIST | BOTTLE_TAG_FLOAT64))
    {
        mStorage->internal_data[0] = connection.expectFloat64();
        mStorage->internal_data[1] = connection.expectFloat64();
        mStorage->internal_data[2] = connection.expectFloat64();
        mStorage->internal_data[3] = connection.expectFloat64();
    }
    else
    {
        return false;
    }

    return !connection.isError();
}

bool QuaternionDataSerializer::write(yarp::os::ConnectionWriter& connection) const
{
    QuaternionPortContentHeader header;

    header.listTag = (BOTTLE_TAG_LIST | BOTTLE_TAG_FLOAT64);
    header.listLen = 4;

    connection.appendBlock((char*)&header, sizeof(header));

    connection.appendFloat64(mStorage->internal_data[0]);
    connection.appendFloat64(mStorage->internal_data[1]);
    connection.appendFloat64(mStorage->internal_data[2]);
    connection.appendFloat64(mStorage->internal_data[3]);

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();

    return !connection.isError();
}

std::string QuaternionDataStorage::toString(int precision, int width) const
{
    std::string ret;
    char tmp[350];
    if (width<0)
    {
        sprintf(tmp, "w=% .*lf\t", precision, internal_data[0]);   ret += tmp;
        sprintf(tmp, "x=% .*lf\t", precision, internal_data[1]);   ret += tmp;
        sprintf(tmp, "y=% .*lf\t", precision, internal_data[2]);   ret += tmp;
        sprintf(tmp, "z=% .*lf\t", precision, internal_data[3]);   ret += tmp;
    }
    else
    {
        sprintf(tmp, "w=% *.*lf ", width, precision, internal_data[0]);    ret += tmp;
        sprintf(tmp, "x=% *.*lf ", width, precision, internal_data[1]);    ret += tmp;
        sprintf(tmp, "y=% *.*lf ", width, precision, internal_data[2]);    ret += tmp;
        sprintf(tmp, "z=% *.*lf ", width, precision, internal_data[3]);    ret += tmp;
    }

    return ret.substr(0, ret.length() - 1);
}
