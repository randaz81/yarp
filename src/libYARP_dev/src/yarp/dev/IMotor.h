/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMOTOR_H
#define YARP_DEV_IMOTOR_H

#include <yarp/os/Vocab.h>
#include <yarp/os/Log.h>
#include <yarp/dev/api.h>
#include <yarp/dev/ReturnValue.h>

namespace yarp::dev {
class IMotorRaw;
class IMotor;
}

/**
 * @ingroup dev_iface_motor_raw
 *
 * Control board, encoder interface.
 */
class YARP_dev_API yarp::dev::IMotorRaw
{
public:
    /**
     * Destructor.
     */
    virtual ~IMotorRaw() {}

    /**
     * Get the number of available motors.
     * @param num retrieved number of available motors
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getNumberOfMotorsRaw(int *num) = 0;

    /**
     * Get temperature of a motor.
     * @param m motor number
     * @param val retrieved motor temperature
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getTemperatureRaw(int m, double* val)=0;

    /**
     * Get temperature of all the motors.
     * @param vals pointer to an array containing all motor temperatures
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getTemperaturesRaw(double *vals)=0;

    /**
     * Retrieves the current temperature limit for a specific motor.
     * The specific behavior of the motor when the temperature limit is exceeded depends on the implementation (power off recommended)
     * @param m motor number
     * @param temp the current temperature limit.
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getTemperatureLimitRaw(int m, double *temp)=0;

    /**
     * Set the temperature limit for a specific motor.
     * The specific behavior of the motor when the temperature limit is exceeded depends on the implementation (power off recommended)
     * @param m motor number
     * @param temp the temperature limit to be set
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setTemperatureLimitRaw(int m, const double temp)=0;

    /**
    * Get the gearbox ratio for a specific motor
    * @param m motor number
    * @param val retrieved gearbox ratio
    * @return true/false
    */
    virtual yarp::dev::yarp_ret_value getGearboxRatioRaw(int m, double *val) { return yarp::dev::NOT_YET_IMPLEMENTED(); };

    /**
    * Set the gearbox ratio for a specific motor
    * @param m motor number
    * @param gearbox ratio to be set
    * @return true/false
    */
    virtual yarp::dev::yarp_ret_value setGearboxRatioRaw(int m, const double val) { return yarp::dev::NOT_YET_IMPLEMENTED(); };
};

/**
 * @ingroup dev_iface_motor
 *
 * Control board, encoder interface.
 */
class YARP_dev_API yarp::dev::IMotor
{
public:
    /**
     * Destructor.
     */
    virtual ~IMotor() {}

    /**
     * Get the number of available motors.
     * @param num retrieved number of available motors
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getNumberOfMotors(int *num) = 0;

    /**
     * Get temperature of a motor.
     * @param m motor number
     * @param val retrieved motor temperature
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getTemperature(int m, double *val)=0;

    /**
     * Get temperature of all the motors.
     * @param vals pointer to an array containing all motor temperatures
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getTemperatures(double *vals)=0;

    /**
     * Retrieves the current temperature limit for a specific motor.
     * The specific behavior of the motor when the temperature limit is exceeded depends on the implementation (power off recommended)
     * @param m motor number
     * @param temp the current temperature limit.
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getTemperatureLimit(int m, double *temp)=0;

    /**
     * Set the temperature limit for a specific motor.
     * The specific behavior of the motor when the temperature limit is exceeded depends on the implementation (power off recommended)
     * @param m motor number
     * @param temp the temperature limit to be set
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setTemperatureLimit(int m, const double temp)=0;

    /**
    * Get the gearbox ratio for a specific motor
    * @param m motor number
    * @param val retrieved gearbox ratio
    * @return true/false
    */
    virtual yarp::dev::yarp_ret_value getGearboxRatio(int m, double *val) { return yarp::dev::NOT_YET_IMPLEMENTED(); };

    /**
    * Set the gearbox ratio for a specific motor
    * @param m motor number
    * @param gearbox ratio to be set
    * @return true/false
    */
    virtual yarp::dev::yarp_ret_value setGearboxRatio(int m, const double val) { return yarp::dev::NOT_YET_IMPLEMENTED(); };
};

// interface IMotorEncoders gets
constexpr yarp::conf::vocab32_t VOCAB_MOTORS_NUMBER        = yarp::os::createVocab32('m','t','n','m');
constexpr yarp::conf::vocab32_t VOCAB_TEMPERATURE          = yarp::os::createVocab32('t','m','p');
constexpr yarp::conf::vocab32_t VOCAB_GEARBOX_RATIO        = yarp::os::createVocab32('g','b','x','r');
constexpr yarp::conf::vocab32_t VOCAB_TEMPERATURES         = yarp::os::createVocab32('t','m','p','s');
constexpr yarp::conf::vocab32_t VOCAB_TEMPERATURE_LIMIT    = yarp::os::createVocab32('t','m','p','l');


#endif // YARP_DEV_IMOTOR_H
