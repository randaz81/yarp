/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_OS_SYSTEMINFOSERIALIZER_H
#define YARP_OS_SYSTEMINFOSERIALIZER_H

#include <yarp/os/Portable.h>
#include <yarp/os/SystemInfo.h>

namespace yarp::os {

/**
 * \ingroup key_class
 * @brief A helper class to pass the SystemInfo object around the YARP network
 */
class YARP_os_API SystemInfoSerializer : public yarp::os::Portable
{
public:
    /**
     * @brief SystemInfoSerializer constructor
     */
    SystemInfoSerializer();

    /**
     * @brief ~SystemInfoSerializer deconstructor
     */
    virtual ~SystemInfoSerializer();

    /**
     * @brief reads from a ConnectionReader and fill into the SystemInfo structs.
     * @param connection a ConnectionReader
     * @return true/false upon success or failure
     */
    bool read(yarp::os::ConnectionReader& connection) override;

    /**
     * @brief write the SystemInfo structs using a ConnectionWriter.
     * @param connection a ConnectionWriter
     * @return true/false upon success or failure
     */
    bool write(yarp::os::ConnectionWriter& connection) const override;

    /**
     * @brief system memory information
     */
    mutable yarp::os::SystemInfo::MemoryInfo memory;

    /**
     * @brief system storage information
     */
    mutable yarp::os::SystemInfo::StorageInfo storage;

    /**
     * @brief system processor type information
     */
    mutable yarp::os::SystemInfo::ProcessorInfo processor;

    /**
     * @brief operating system information
     */
    mutable yarp::os::SystemInfo::PlatformInfo platform;

    /**
     * @brief current cpu load information
     */
    mutable yarp::os::SystemInfo::LoadInfo load;

    /**
     * @brief current user information
     */
    mutable yarp::os::SystemInfo::UserInfo user;

    // yarp::os::SystemInfo::NetworkInfo network;

private:
    void updateSystemInfo() const;
};

} // namespace yarp::os

#endif // YARP_OS_SYSTEMINFOSERIALIZER_H
