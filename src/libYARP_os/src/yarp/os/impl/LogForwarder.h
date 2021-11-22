/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_OS_IMPL_LOGFORWARDER_H
#define YARP_OS_IMPL_LOGFORWARDER_H

#include <yarp/os/api.h>

#include <yarp/os/Port.h>
#include <yarp/os/PortWriterBuffer.h>

#include <mutex>
#include <string>

namespace yarp::os::impl {

class YARP_os_impl_API LogForwarder
{
public:
    ~LogForwarder();
    static LogForwarder& getInstance();

    void forward(const std::string& message);
    static void shutdown();

private:
    LogForwarder();
    LogForwarder(LogForwarder const&) = delete;
    LogForwarder& operator=(LogForwarder const&) = delete;

    std::mutex mutex;
    yarp::os::Port outputPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> outputPort_buffer;
    static bool started;
};

} // namespace yarp::os::impl

#endif // YARP_OS_IMPL_LOGFORWARDER_H
