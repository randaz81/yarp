/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IAXISINFO_H
#define YARP_DEV_IAXISINFO_H

#include <string>

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>
#include <yarp/dev/api.h>
#include <yarp/dev/ReturnValue.h>

/*! \file IAxisInfo.h define control board standard interfaces*/

namespace yarp::dev {
class IAxisInfo;
class IAxisInfoRaw;

enum JointTypeEnum
{
    VOCAB_JOINTTYPE_REVOLUTE = yarp::os::createVocab32('a', 't', 'r', 'v'),
    VOCAB_JOINTTYPE_PRISMATIC = yarp::os::createVocab32('a', 't', 'p', 'r'),
    VOCAB_JOINTTYPE_UNKNOWN = yarp::os::createVocab32('u', 'n', 'k', 'n')
};
} // namespace yarp::dev

/**
 * @ingroup dev_iface_motor
 *
 * Interface for getting information about specific axes, if available.
 */
class YARP_dev_API yarp::dev::IAxisInfo
{
public:
    /**
     * Destructor.
     */
    virtual ~IAxisInfo() {}

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax storage to return param
     * @return true/false.
     */
    virtual yarp::dev::yarp_ret_value getAxes(int* ax) = 0;

    /* Get the name for a particular axis.
    * @param axis joint number
    * @param name the axis name
    * @return true if everything goes fine, false otherwise.
    */
    virtual yarp::dev::yarp_ret_value getAxisName(int axis, std::string& name) = 0;

    /* Get the joint type (e.g. revolute/prismatic) for a particular axis.
    * @param axis joint number
    * @param type the joint type
    * @return true if everything goes fine, false otherwise.
    */
    virtual yarp::dev::yarp_ret_value getJointType(int axis, yarp::dev::JointTypeEnum& type) { yFatal("getJointType() not implemented on your device, cannot proceed further. Please report the problem on yarp issue tracker"); return yarp::dev::NOT_YET_IMPLEMENTED(); }
};

/**
 * @ingroup dev_iface_motor_raw
 *
 * Interface for getting information about specific axes, if available.
 */
class YARP_dev_API yarp::dev::IAxisInfoRaw
{
public:
    /**
    * Destructor.
    */
    virtual ~IAxisInfoRaw() {}

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax storage to return param
     * @return true/false.
     */
    virtual yarp::dev::yarp_ret_value getAxes(int* ax) = 0;

    /* Get the name for a particular axis.
    * @param axis joint number
    * @param name the axis name
    * @return true if everything goes fine, false otherwise.
    */
    virtual yarp::dev::yarp_ret_value getAxisNameRaw(int axis, std::string& name) = 0;

    /* Get the joint type (e.g. revolute/prismatic) for a particular axis.
    * @param axis joint number
    * @param type the joint type
    * @return true if everything goes fine, false otherwise.
    */
    virtual yarp::dev::yarp_ret_value getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)  { yFatal("getJointType() not implemented on your device, cannot proceed further. Please report the problem on yarp issue tracker"); return yarp::dev::NOT_YET_IMPLEMENTED(); };
};

// interface IAxisInfo
constexpr yarp::conf::vocab32_t VOCAB_INFO_NAME            = yarp::os::createVocab32('n','a','m','e');
constexpr yarp::conf::vocab32_t VOCAB_INFO_TYPE            = yarp::os::createVocab32('t','y','p','e');

#endif // YARP_DEV_IAXISINFO_H
