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
    using Serializer<T>::Serializer;
    using Serializer<T>::read;
    using Serializer<T>::write;

    virtual bool read(yarp::os::idl::WireReader& reader) = 0;
    virtual bool write(const yarp::os::idl::WireWriter& writer) const = 0;
};

} // namespace yarp::os

#endif // YARP_OS_SERIALIZER_H
