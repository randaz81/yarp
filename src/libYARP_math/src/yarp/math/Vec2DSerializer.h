/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_MATH_VEC2D_SERIALIZER_H
#define YARP_MATH_VEC2D_SERIALIZER_H

#include <yarp/math/api.h>

#include <yarp/math/Vec2D.h>
#include <yarp/os/Serializer.h>

#include <type_traits>

namespace yarp::math {

template <typename T>
class YARP_math_API Vec2DSerializer :
        public yarp::os::Serializer <yarp::math::Vec2D<T>>
{
    static_assert(std::is_same<size_t, T>::value || std::is_same<int, T>::value || std::is_same<double, T>::value, "Vec2D can be specialized only as size_t, int, double");

public:
    using yarp::os::Serializer<yarp::math::Vec2D<T>>::Serializer;

    virtual ~Vec2DSerializer() = default;

    ///////// Serialization methods
    /*
    * Read vector from a connection.
    * return true if a Vec2D was read correctly
    */
    bool read(yarp::os::ConnectionReader& connection) override;

    /**
    * Write vector to a connection.
    * return true if a Vec2D was written correctly
    */
    bool write(yarp::os::ConnectionWriter& connection) const override;

    yarp::os::Type getType() const override
    {
        return yarp::os::Type::byName("yarp/vec2D");
    }
};

// Forward declarations of specialized methods
template<> bool YARP_math_API yarp::math::Vec2DSerializer<double>::read(yarp::os::ConnectionReader& connection);
template<> bool YARP_math_API yarp::math::Vec2DSerializer<int>::read(yarp::os::ConnectionReader& connection);
template<> bool YARP_math_API yarp::math::Vec2DSerializer<size_t>::read(yarp::os::ConnectionReader& connection);
template<> bool YARP_math_API yarp::math::Vec2DSerializer<double>::write(yarp::os::ConnectionWriter& connection) const;
template<> bool YARP_math_API yarp::math::Vec2DSerializer<int>::write(yarp::os::ConnectionWriter& connection) const;
template<> bool YARP_math_API yarp::math::Vec2DSerializer<size_t>::write(yarp::os::ConnectionWriter& connection) const;

} // namespace yarp::math

#endif // YARP_MATH_VEC2D_SERIALIZER_H
