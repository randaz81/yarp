/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_OS_WIRESERIALIZER_H
#define YARP_OS_WIRESERIALIZER_H

#include <yarp/os/Serializer.h>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace yarp::os {

/**
 * \ingroup comm_class
 *
 * A base template wireserializer. Methods read/write should be overrided by the specific data type to serialize.
 *
 */
template <typename T>
class WireSerializer : public yarp::os::Serializer<T>
{
public:
    using Serializer::Serializer;
/*    Serializer() :
        mStorage(new T),
        mStorageConst(std::as_const(mStorage)),
        isOwned(true)
    {
    }

    explicit Serializer(T& in) :
        mStorage(&in),
        mStorageConst(std::as_const(mStorage))
    {
    }

    // This constructor takes a const T as input.
    // If the non-const get() method is used, it should probably throw an
    // exception.
    explicit Serializer(const T& in) :
        mStorageConst(&in)
    {
    }

    explicit Serializer(T&& in) noexcept :
        mStorage(new T(std::move(in))),
        mStorageConst(std::as_const(mStorage)),
        isOwned(true)
    {
    }

    Serializer(const Serializer& in) = delete;
    Serializer(Serializer&& in) noexcept= delete;
    Serializer& operator=(const Serializer& in) = delete;
    Serializer& operator=(Serializer&&) = delete;

    ~Serializer() override
    {
        if (isOwned) {
            delete mStorage;
        }
    }
*/

    virtual bool read(yarp::os::idl::WireReader& reader) = 0;  //?????
    //bool read(yarp::os::idl::WireReader& reader) override = 0; //?????

    virtual bool write(const yarp::os::idl::WireWriter& writer) const = 0;  //?????
    //bool write(const yarp::os::idl::WireWriter& writer) override const = 0;  //?????

};

} // namespace yarp::os

#endif // YARP_OS_SERIALIZER_H
