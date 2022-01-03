/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/math/Vec2DSerializer.h>

#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogComponent.h>
#include <yarp/math/Math.h>
#include <sstream>
#include <cmath>
#include <cstdio>

// network stuff
#include <yarp/os/NetInt32.h>

using namespace yarp::math;

namespace {
YARP_LOG_COMPONENT(VEC2D, "yarp.math.Vec2D")
}

YARP_BEGIN_PACK
class Vec2DPortContentHeader
{
public:
    yarp::os::NetInt32 listTag{0};
    yarp::os::NetInt32 listLen{0};
    Vec2DPortContentHeader() = default;
};
YARP_END_PACK

namespace yarp::math {
template<>
bool Vec2DSerializer<double>::read(yarp::os::ConnectionReader& connection)
{
    // auto-convert text mode interaction
    connection.convertTextMode();
    Vec2DPortContentHeader header;
    bool ok = connection.expectBlock(reinterpret_cast<char*>(&header), sizeof(header));
    if (!ok) {
        return false;
    }

    if (header.listLen == 2 && header.listTag == (BOTTLE_TAG_LIST | BOTTLE_TAG_FLOAT64))
    {
        mStorage->x = connection.expectFloat64();
        mStorage->y = connection.expectFloat64();
    }
    else
    {
        return false;
    }

    return !connection.isError();
}

template<>
bool Vec2DSerializer<int>::read(yarp::os::ConnectionReader& connection)
{
    // auto-convert text mode interaction
    connection.convertTextMode();
    Vec2DPortContentHeader header;
    bool ok = connection.expectBlock(reinterpret_cast<char*>(&header), sizeof(header));
    if (!ok) {
        return false;
    }

    if (header.listLen == 2 && header.listTag == (BOTTLE_TAG_LIST | BOTTLE_TAG_INT32))
    {
        mStorage->x = connection.expectInt32();
        mStorage->y = connection.expectInt32();
    }
    else
    {
        return false;
    }

    return !connection.isError();
}

template<>
bool Vec2DSerializer<size_t>::read(yarp::os::ConnectionReader& connection)
{
    // auto-convert text mode interaction
    connection.convertTextMode();
    Vec2DPortContentHeader header;
    bool ok = connection.expectBlock(reinterpret_cast<char*>(&header), sizeof(header));
    if (!ok) {
        return false;
    }

    if (header.listLen == 2 && header.listTag == (BOTTLE_TAG_LIST | BOTTLE_TAG_INT64))
    {
        mStorage->x = connection.expectInt64();
        mStorage->y = connection.expectInt64();
    }
    else
    {
        return false;
    }

    return !connection.isError();
}

template<>
bool Vec2DSerializer<double>::write(yarp::os::ConnectionWriter& connection) const
{
    Vec2DPortContentHeader header;

    header.listTag = (BOTTLE_TAG_LIST | BOTTLE_TAG_FLOAT64);
    header.listLen = 2;

    connection.appendBlock(reinterpret_cast<char*>(&header), sizeof(header));

    connection.appendFloat64(mStorage->x);
    connection.appendFloat64(mStorage->y);

    connection.convertTextMode();

    return !connection.isError();
}

template<>
bool Vec2DSerializer<int>::write(yarp::os::ConnectionWriter& connection) const
{
    Vec2DPortContentHeader header;

    header.listTag = (BOTTLE_TAG_LIST | BOTTLE_TAG_INT32);
    header.listLen = 2;

    connection.appendBlock(reinterpret_cast<char*>(&header), sizeof(header));

    connection.appendInt32(mStorage->x);
    connection.appendInt32(mStorage->y);

    connection.convertTextMode();

    return !connection.isError();
}

template<>
bool Vec2DSerializer<size_t>::write(yarp::os::ConnectionWriter& connection) const
{
    Vec2DPortContentHeader header;

    header.listTag = (BOTTLE_TAG_LIST | BOTTLE_TAG_INT64);
    header.listLen = 2;

    connection.appendBlock(reinterpret_cast<char*>(&header), sizeof(header));

    connection.appendInt64(mStorage->x);
    connection.appendInt64(mStorage->y);

    connection.convertTextMode();

    return !connection.isError();
}

} // namespace yarp::math
