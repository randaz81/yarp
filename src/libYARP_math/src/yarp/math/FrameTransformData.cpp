/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/math/FrameTransformData.h>

#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <cstdio>
#include <cmath>

namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORM, "yarp.math.FrameTransformData")
}

yarp::math::FrameTransformDataStorage::FrameTransformDataStorage()
{
    translation.set(0, 0, 0);
}

yarp::math::FrameTransformDataStorage::FrameTransformDataStorage(const std::string& parent,
                                            const std::string& child,
                                            double             inTX,
                                            double             inTY,
                                            double             inTZ,
                                            double             inRX,
                                            double             inRY,
                                            double             inRZ,
                                            double             inRW) :
        src_frame_id(parent),
        dst_frame_id(child),
        translation{inTX, inTY, inTZ},
        rotation(inRX, inRY, inRZ, inRW)
{
}

yarp::math::FrameTransformDataStorage::~FrameTransformDataStorage() = default;

bool yarp::math::FrameTransformDataSerializer::read(yarp::os::ConnectionReader& connection)
{
    // auto-convert text mode interaction
    connection.convertTextMode();

    connection.expectInt32();
    connection.expectInt32();

    connection.expectInt32();
    mStorage->src_frame_id = connection.expectString();
    connection.expectInt32();
    mStorage->dst_frame_id = connection.expectString();
    connection.expectInt32();
    mStorage->timestamp = connection.expectFloat64();
    connection.expectInt32();
    mStorage->isStatic = (connection.expectInt8()==1);

    connection.expectInt32();
    mStorage->translation.tX = connection.expectFloat64();
    connection.expectInt32();
    mStorage->translation.tY = connection.expectFloat64();
    connection.expectInt32();
    mStorage->translation.tZ = connection.expectFloat64();

    connection.expectInt32();
    mStorage->rotation.x() = connection.expectFloat64();
    connection.expectInt32();
    mStorage->rotation.y() = connection.expectFloat64();
    connection.expectInt32();
    mStorage->rotation.z() = connection.expectFloat64();
    connection.expectInt32();
    mStorage->rotation.w() = connection.expectFloat64();

    return !connection.isError();
}

bool yarp::math::FrameTransformDataSerializer::write(yarp::os::ConnectionWriter& connection) const
{
    connection.appendInt32(BOTTLE_TAG_LIST);
    connection.appendInt32(4+3+4);

    connection.appendInt32(BOTTLE_TAG_STRING);
    connection.appendString(mStorage->src_frame_id);
    connection.appendInt32(BOTTLE_TAG_STRING);
    connection.appendString(mStorage->dst_frame_id);
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->timestamp);
    connection.appendInt32(BOTTLE_TAG_INT8);
    connection.appendInt8(int8_t(mStorage->isStatic));

    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->translation.tX);
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->translation.tY);
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->translation.tZ);

    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->rotation.x());
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->rotation.y());
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->rotation.z());
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->rotation.w());

    connection.convertTextMode();

    return !connection.isError();
}
