/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/math/Vec2D.h>

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

template <typename T>
std::string yarp::math::Vec2D<T>::toString(int precision, int width) const
{
    std::ostringstream stringStream;
    stringStream.precision(precision);
    stringStream.width(width);
    stringStream << std::string("x:") << x << std::string(" y:") << y;
    return stringStream.str();
}

template <typename T>
T yarp::math::Vec2D<T>::norm() const
{
    return T(sqrt(x*x + y*y));
}

//constructors
template <typename T>
yarp::math::Vec2D<T>::Vec2D() : x(0), y(0)
{
}

template <typename T>
yarp::math::Vec2D<T>::Vec2D(const yarp::sig::Vector& v)
{
    yCAssert(VEC2D, v.size() == 2);
    x = T(v[0]);
    y = T(v[1]);
}

template <typename T>
yarp::math::Vec2D<T>::Vec2D(const T& x_value, const T& y_value)
{
    x = x_value;
    y = y_value;
}

template <typename T>
 yarp::math::Vec2D<T>  operator * (const yarp::sig::Matrix& lhs, yarp::math::Vec2D<T> rhs)
{
    yCAssert(VEC2D, lhs.rows() == 2 && lhs.cols() == 2);
    T x = rhs.x; T y = rhs.y;
    rhs.x = T(lhs[0][0] * x + lhs[0][1] * y);
    rhs.y = T(lhs[1][0] * x + lhs[1][1] * y);
    return rhs;
}

template <typename T>
yarp::math::Vec2D<T> operator + (yarp::math::Vec2D<T> lhs, const yarp::math::Vec2D<T>& rhs)
{
    lhs += rhs;
    return lhs;
}

template <typename T>
yarp::math::Vec2D<T> operator - (yarp::math::Vec2D<T> lhs, const yarp::math::Vec2D<T>& rhs)
{
    lhs -= rhs;
    return lhs;
}

template <typename T>
yarp::math::Vec2D<T>& yarp::math::Vec2D<T>::operator+=(const yarp::math::Vec2D<T>& rhs)
{
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
}

template <typename T>
yarp::math::Vec2D<T>& yarp::math::Vec2D<T>::operator-=(const yarp::math::Vec2D<T>& rhs)
{
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
}

template <typename T>
bool yarp::math::Vec2D<T>::operator ==(const yarp::math::Vec2D<T>& rhs) const
{
    if (this->x == rhs.x &&
        this->y == rhs.y) {
        return true;
    }
    return false;
}

template <typename T>
bool yarp::math::Vec2D<T>::operator !=(const yarp::math::Vec2D<T>& rhs) const
{
    if (this->x == rhs.x &&
        this->y == rhs.y) {
        return false;
    }
    return true;
}

template yarp::math::Vec2D<double> YARP_math_API operator + (yarp::math::Vec2D<double> lhs, const yarp::math::Vec2D<double>& rhs);
template yarp::math::Vec2D<int>    YARP_math_API operator + (yarp::math::Vec2D<int> lhs, const yarp::math::Vec2D<int>& rhs);
template yarp::math::Vec2D<double> YARP_math_API operator - (yarp::math::Vec2D<double> lhs, const yarp::math::Vec2D<double>& rhs);
template yarp::math::Vec2D<int>    YARP_math_API operator - (yarp::math::Vec2D<int> lhs, const yarp::math::Vec2D<int>& rhs);
template yarp::math::Vec2D<double> YARP_math_API operator * (const yarp::sig::Matrix& lhs, yarp::math::Vec2D<double> rhs);
template yarp::math::Vec2D<int>    YARP_math_API operator * (const yarp::sig::Matrix& lhs, yarp::math::Vec2D<int> rhs);

template class yarp::math::Vec2D<double>;
template class yarp::math::Vec2D<int>;
template class yarp::math::Vec2D<size_t>;
