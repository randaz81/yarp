/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IENCODERS_H
#define YARP_DEV_IENCODERS_H

#include <yarp/os/Vocab.h>
#include <yarp/dev/api.h>
#include <yarp/dev/ReturnValue.h>

namespace yarp::dev {
class IEncodersRaw;
class IEncoders;
}

/**
 * @ingroup dev_iface_motor_raw
 *
 * Control board, encoder interface.
 */
class YARP_dev_API yarp::dev::IEncodersRaw
{
public:
    /**
     * Destructor.
     */
    virtual ~IEncodersRaw() {}

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @return the number of controlled axes.
     */
    virtual yarp::dev::yarp_ret_value getAxes(int *ax) = 0;

    /**
     * Reset encoder, single joint. Set the encoder value to zero
     * @param j encoder number
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetEncoderRaw(int j)=0;

    /**
     * Reset encoders. Set the encoders value to zero
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetEncodersRaw()=0;

    /**
     * Set the value of the encoder for a given joint.
     * @param j encoder number
     * @param val new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setEncoderRaw(int j, double val)=0;

    /**
     * Set the value of all encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setEncodersRaw(const double *vals)=0;

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure (you knew it, uh?)
     */
    virtual yarp::dev::yarp_ret_value getEncoderRaw(int j, double *v)=0;

    /**
     * Read the position of all axes.
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getEncodersRaw(double *encs)=0;

    /**
     * Read the instantaneous speed of an axis.
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return true if successful, false ... otherwise.
     */
    virtual yarp::dev::yarp_ret_value getEncoderSpeedRaw(int j, double *sp)=0;

    /**
     * Read the instantaneous acceleration of an axis.
     * @param spds pointer to storage for the output values
     * @return guess what? (true/false on success or failure).
     */
    virtual yarp::dev::yarp_ret_value getEncoderSpeedsRaw(double *spds)=0;

    /**
     * Read the instantaneous acceleration of an axis.
     * @param j axis number
     * @param spds pointer to the array that will contain the output
     */
    virtual yarp::dev::yarp_ret_value getEncoderAccelerationRaw(int j, double *spds)=0;

    /**
     * Read the instantaneous acceleration of all axes.
     * @param accs pointer to the array that will contain the output
     * @return true if all goes well, false if anything bad happens.
     */
    virtual yarp::dev::yarp_ret_value getEncoderAccelerationsRaw(double *accs)=0;
};

/**
 * @ingroup dev_iface_motor
 *
 * Control board, encoder interface.
 */
class YARP_dev_API yarp::dev::IEncoders
{
public:
    /**
     * Destructor.
     */
    virtual ~IEncoders() {}

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @return the number of controlled axes.
     */
    virtual yarp::dev::yarp_ret_value getAxes(int *ax) = 0;

    /**
     * Reset encoder, single joint. Set the encoder value to zero
     * @param j encoder number
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetEncoder(int j)=0;

    /**
     * Reset encoders. Set the encoders value to zero
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value resetEncoders()=0;

    /**
     * Set the value of the encoder for a given joint.
     * @param j encoder number
     * @param val new value
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setEncoder(int j, double val)=0;

    /**
     * Set the value of all encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    virtual yarp::dev::yarp_ret_value setEncoders(const double *vals)=0;

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure (you knew it, uh?)
     */
    virtual yarp::dev::yarp_ret_value getEncoder(int j, double *v)=0;

    /**
     * Read the position of all axes.
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getEncoders(double *encs)=0;

    /**
     * Read the instantaneous speed of an axis.
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return true if successful, false ... otherwise.
     */
    virtual yarp::dev::yarp_ret_value getEncoderSpeed(int j, double *sp)=0;

    /**
     * Read the instantaneous speed of all axes.
     * @param spds pointer to storage for the output values
     * @return guess what? (true/false on success or failure).
     */
    virtual yarp::dev::yarp_ret_value getEncoderSpeeds(double *spds)=0;

    /**
     * Read the instantaneous acceleration of an axis.
     * @param j axis number
     * @param spds pointer to the array that will contain the output
     */
    virtual yarp::dev::yarp_ret_value getEncoderAcceleration(int j, double *spds)=0;

    /**
     * Read the instantaneous acceleration of all axes.
     * @param accs pointer to the array that will contain the output
     * @return true if all goes well, false if anything bad happens.
     */
    virtual yarp::dev::yarp_ret_value getEncoderAccelerations(double *accs)=0;
};

// interface IEncoders sets
constexpr yarp::conf::vocab32_t VOCAB_E_RESET  = yarp::os::createVocab32('e','r','e');
constexpr yarp::conf::vocab32_t VOCAB_E_RESETS = yarp::os::createVocab32('e','r','e','s');
constexpr yarp::conf::vocab32_t VOCAB_ENCODER  = yarp::os::createVocab32('e','n','c');
constexpr yarp::conf::vocab32_t VOCAB_ENCODERS = yarp::os::createVocab32('e','n','c','s');

// interface IEncoders gets
constexpr yarp::conf::vocab32_t VOCAB_ENCODER_SPEED         = yarp::os::createVocab32('e','s','p');
constexpr yarp::conf::vocab32_t VOCAB_ENCODER_SPEEDS        = yarp::os::createVocab32('e','s','p','s');
constexpr yarp::conf::vocab32_t VOCAB_ENCODER_ACCELERATION  = yarp::os::createVocab32('e','a','c');
constexpr yarp::conf::vocab32_t VOCAB_ENCODER_ACCELERATIONS = yarp::os::createVocab32('e','a','c','s');

#endif // YARP_DEV_IENCODERS_H
