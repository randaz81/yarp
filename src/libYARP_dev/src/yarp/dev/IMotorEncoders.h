/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMOTORENCODERS_H
#define YARP_DEV_IMOTORENCODERS_H

#include <yarp/os/Vocab.h>
#include <yarp/dev/api.h>
#include <yarp/dev/ReturnValue.h>

namespace yarp::dev {
class IMotorEncodersRaw;
class IMotorEncoders;
}

/**
 * @ingroup dev_iface_motor_raw
 *
 * Control board, encoder interface.
 */
class YARP_dev_API yarp::dev::IMotorEncodersRaw
{
public:
    /**
     * Destructor.
     */
    virtual ~IMotorEncodersRaw() {}

    /**
     * Get the number of available motor encoders.
     * @param m pointer to a value representing the number of available motor encoders.
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getNumberOfMotorEncodersRaw(int *num) = 0;

    /**
     * Reset motor encoder, single motor. Set the encoder value to zero.
     * @param m motor number
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetMotorEncoderRaw(int m)=0;

    /**
     * Reset motor encoders. Set the motor encoders value to zero.
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetMotorEncodersRaw()=0;

    /**
     * Sets number of counts per revolution for motor encoder m.
     * @param m motor number
     * @param cpr new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr)=0;

    /**
     * Gets number of counts per revolution for motor encoder m.
     * @param m motor number
     * @param cpr vals pointer to the new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr)=0;

    /**
     * Set the value of the motor encoder for a given motor.
     * @param m motor number
     * @param val new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setMotorEncoderRaw(int m, const double val)=0;

    /**
     * Set the value of all motor encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setMotorEncodersRaw(const double *vals)=0;

    /**
     * Read the value of a motor encoder.
     * @param m motor encoder number
     * @param v pointer to storage for the return value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderRaw(int m, double *v)=0;

    /**
     * Read the position of all motor encoders.
     * @param encs pointer to the array that will contain the output
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getMotorEncodersRaw(double *encs)=0;

    /**
     * Read the instantaneous position of all motor encoders.
     * @param encs pointer to the array that will contain the output
     * @param time pointer to the array that will contain individual timestamps
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncodersTimedRaw(double *encs, double *stamps)=0;

   /**
    * Read the instantaneous position of a motor encoder.
    * @param m motor index
    * @param encs encoder value (pointer to)
    * @param time corresponding timestamp (pointer to)
    * @return true if successful, false otherwise.
    */
    virtual yarp::dev::yarp_ret_value getMotorEncoderTimedRaw(int m, double *encs, double *stamp)=0;

    /**
     * Read the instantaneous speed of a motor encoder.
     * @param m motor number
     * @param sp pointer to storage for the output
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderSpeedRaw(int m, double *sp)=0;

    /**
     * Read the instantaneous speed of all motor encoders.
     * @param spds pointer to storage for the output values
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderSpeedsRaw(double *spds)=0;

    /**
     * Read the instantaneous acceleration of a motor encoder
     * @param m motor number
     * @param acc pointer to the array that will contain the output
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderAccelerationRaw(int m, double *spds)=0;

    /**
     * Read the instantaneous acceleration of all motor encoders.
     * @param accs pointer to the array that will contain the output
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderAccelerationsRaw(double *accs)=0;
};

/**
 * @ingroup dev_iface_motor
 *
 * Control board, encoder interface.
 */
class YARP_dev_API yarp::dev::IMotorEncoders
{
public:
    /**
     * Destructor.
     */
    virtual ~IMotorEncoders() {}

    /**
     * Get the number of available motor encoders.
     * @param m pointer to a value representing the number of available motor encoders.
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getNumberOfMotorEncoders(int *num) = 0;

    /**
     * Reset motor encoder, single motor. Set the encoder value to zero.
     * @param m motor number
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetMotorEncoder(int m)=0;

    /**
     * Reset motor encoders. Set the motor encoders value to zero.
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetMotorEncoders()=0;

    /**
     * Sets number of counts per revolution for motor encoder m.
     * @param m motor number
     * @param cpr new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setMotorEncoderCountsPerRevolution(int m, const double cpr)=0;

    /**
     * Gets number of counts per revolution for motor encoder m.
     * @param m motor number
     * @param cpr vals pointer to the new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderCountsPerRevolution(int m, double *cpr)=0;

    /**
     * Set the value of the motor encoder for a given motor.
     * @param m motor number
     * @param val new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setMotorEncoder(int m, const double val)=0;

    /**
     * Set the value of all motor encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setMotorEncoders(const double *vals)=0;

    /**
     * Read the value of a motor encoder.
     * @param m motor encoder number
     * @param v pointer to storage for the return value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoder(int m, double *v)=0;

    /**
     * Read the position of all motor encoders.
     * @param encs pointer to the array that will contain the output
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoders(double *encs)=0;

    /**
     * Read the instantaneous position of all motor encoders.
     * @param encs pointer to the array that will contain the output
     * @param time pointer to the array that will contain individual timestamps
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncodersTimed(double *encs, double *time)=0;

   /**
    * Read the instantaneous position of a motor encoder.
    * @param m motor index
    * @param encs encoder value (pointer to)
    * @param time corresponding timestamp (pointer to)
    * @return true if successful, false otherwise.
    */
    virtual yarp::dev::yarp_ret_value getMotorEncoderTimed(int m, double *encs, double *time)=0;

    /**
     * Read the istantaneous speed of a motor encoder.
     * @param m motor number
     * @param sp pointer to storage for the output
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderSpeed(int m, double *sp)=0;

    /**
     * Read the instantaneous speed of all motor encoders.
     * @param spds pointer to storage for the output values
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderSpeeds(double *spds)=0;

    /**
     * Read the instantaneous acceleration of a motor encoder
     * @param m motor number
     * @param acc pointer to the array that will contain the output
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderAcceleration(int m, double *acc)=0;

    /**
     * Read the instantaneous acceleration of all motor encoders.
     * @param accs pointer to the array that will contain the output
     * @return true if successful, false otherwise.
     */
    virtual yarp::dev::yarp_ret_value getMotorEncoderAccelerations(double *accs)=0;
};

// interface IMotorEncoders sets
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_E_RESET  = yarp::os::createVocab32('m','r','e');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_E_RESETS = yarp::os::createVocab32('m','r','e','s');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODER  = yarp::os::createVocab32('m','n','c');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODERS = yarp::os::createVocab32('m','n','c','s');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_CPR      = yarp::os::createVocab32('m','c','p','r');

// interface IMotorEncoders gets
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODER_NUMBER        =  yarp::os::createVocab32('m','n','u','m');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODER_SPEED         = yarp::os::createVocab32('m','s','p');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODER_SPEEDS        = yarp::os::createVocab32('m','s','p','s');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODER_ACCELERATION  = yarp::os::createVocab32('m','a','c');
constexpr yarp::conf::vocab32_t VOCAB_MOTOR_ENCODER_ACCELERATIONS = yarp::os::createVocab32('m','a','c','s');

#endif // YARP_DEV_IMOTORENCODERS_H
