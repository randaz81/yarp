/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_MATH_TRANSFORM_DATA_H
#define YARP_MATH_TRANSFORM_DATA_H

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/api.h>
#include <yarp/math/Quaternion.h>
#include <yarp/os/Serializer.h>

namespace yarp::math {

class YARP_math_API FrameTransformDataStorage
{
public:
    YARP_SUPPRESS_DLL_INTERFACE_WARNING_ARG(std::string) src_frame_id;
    YARP_SUPPRESS_DLL_INTERFACE_WARNING_ARG(std::string) dst_frame_id;
    double timestamp = 0;
    bool   isStatic  = false;

    struct Translation_t
    {
        double tX;
        double tY;
        double tZ;

        void set(double x, double y, double z)
        {
            tX = x;
            tY = y;
            tZ = z;
        }
    } translation;

    Quaternion rotation;

    FrameTransformDataStorage();

    FrameTransformDataStorage(const std::string& parent,
                   const std::string& child,
                   double             inTX,
                   double             inTY,
                   double             inTZ,
                   double             inRX,
                   double             inRY,
                   double             inRZ,
                   double             inRW);

    ~FrameTransformDataStorage();
};

class YARP_math_API FrameTransformDataSerializer : public yarp::os::Serializer<yarp::math::FrameTransformDataStorage>
{
public:
    using Serializer::Serializer;

    /*
     * Read frameTransform from a connection.
     * return true iff a matrix was read correctly
     */
    bool read(yarp::os::ConnectionReader& connection) override;

    /**
     * Write frameTransform to a connection.
     * return true iff a matrix was written correctly
     */
    bool write(yarp::os::ConnectionWriter& connection) const override;

    yarp::os::Type getType() const override
    {
        return yarp::os::Type::byName("yarp/frameTransform");
    }
};

} // namespace yarp::math

#endif // YARP_MATH_TRANSFORM_DATA_H
