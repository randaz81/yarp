/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_MATH_TRANSFORM_H
#define YARP_MATH_TRANSFORM_H

#include <yarp/math/FrameTransformData.h>

namespace yarp::math {

class YARP_math_API FrameTransform : public yarp::math::FrameTransformDataStorage
{
    public:
    bool isValid() const;

    FrameTransform();

    FrameTransform(const std::string& parent,
                   const std::string& child,
                   double             inTX,
                   double             inTY,
                   double             inTZ,
                   double             inRX,
                   double             inRY,
                   double             inRZ,
                   double             inRW);

    ~FrameTransform();

    void transFromVec(double X, double Y, double Z);
    void rotFromRPY(double R, double P, double Y);
    yarp::sig::Vector getRPYRot() const;

    yarp::sig::Matrix toMatrix() const;
    bool fromMatrix(const yarp::sig::Matrix& mat);

    enum display_transform_mode_t
    {
       rotation_as_quaternion=0,
       rotation_as_matrix=1,
       rotation_as_rpy=2
    };

    std::string toString(display_transform_mode_t format= rotation_as_quaternion) const;
};

} // namespace yarp::math

#endif // YARP_MATH_TRANSFORM_H
