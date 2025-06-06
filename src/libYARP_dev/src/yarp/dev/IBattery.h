/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IBATTERY_H
#define YARP_DEV_IBATTERY_H

#include <string>

#include <yarp/os/Vocab.h>
#include <yarp/dev/api.h>
#include <yarp/dev/ReturnValue.h>

/*! \file Ibattery.h battery interface */
namespace yarp::dev {
class IBattery;
}

/**
 * @ingroup dev_iface_other
 *
 * A generic battery interface
 */
class YARP_dev_API yarp::dev::IBattery
{
public:
    enum Battery_status
    {
        BATTERY_OK_STANDBY       = 0,
        BATTERY_OK_IN_CHARGE     = 1,
        BATTERY_OK_IN_USE        = 2,
        BATTERY_GENERAL_ERROR    = 3,
        BATTERY_TIMEOUT          = 4,
        BATTERY_LOW_WARNING      = 5,
        BATTERY_CRITICAL_WARNING = 6
    };

    virtual ~IBattery();

    /**
     * Get the instantaneous voltage measurement
     * @param voltage the voltage measurement
     * @return true/false.
     */
    virtual yarp::dev::ReturnValue  getBatteryVoltage(double &voltage) = 0;

    /**
     * Get the instantaneous current measurement
     * @param current the current measurement
     * @return true/false.
     */
    virtual yarp::dev::ReturnValue getBatteryCurrent(double &current) = 0;

    /**
     * get the battery status of charge
     * @param charge the charge measurement (0-100%)
     * @return true/false.
     */
    virtual yarp::dev::ReturnValue getBatteryCharge(double &charge) = 0;

    /**
    * get the battery status
    * @param status the battery status
    * @return true/false.
    */
    virtual yarp::dev::ReturnValue getBatteryStatus(Battery_status &status) = 0;

    /**
    * get the battery temperature
    * @param temperature the battery temperature
    * @return true/false.
    */
    virtual yarp::dev::ReturnValue getBatteryTemperature(double &temperature) = 0;

    /**
    * get the battery hardware characteristics (e.g. max voltage etc)
    * @param a string containing the battery infos
    * @return true/false.
    */
    virtual yarp::dev::ReturnValue getBatteryInfo(std::string &battery_info) = 0;
};

#endif // YARP_DEV_IBATTERY_H
