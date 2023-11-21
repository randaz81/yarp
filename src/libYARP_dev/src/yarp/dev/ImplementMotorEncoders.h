/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMPLEMENTMOTORENCODERS_H
#define YARP_DEV_IMPLEMENTMOTORENCODERS_H

#include <yarp/dev/IMotorEncoders.h>

namespace yarp::dev {
class ImplementMotorEncoders;
}

namespace yarp::dev::impl {

template <typename T>
class FixedSizeBuffersManager;

} // namespace yarp::dev::impl

class YARP_dev_API yarp::dev::ImplementMotorEncoders: public IMotorEncoders
{
protected:
    IMotorEncodersRaw *iMotorEncoders;
    void *helper;
    yarp::dev::impl::FixedSizeBuffersManager<double> *buffManager;


    /**
     * Initialize the internal data and alloc memory.
     * @param size is the number of controlled axes the driver deals with.
     * @param amap is a lookup table mapping axes onto physical drivers.
     * @param enc is an array containing the encoder to angles conversion factors.
     * @param zos is an array containing the zeros of the encoders.
     * @return true if initialized succeeded, false if it wasn't executed, or assert.
     */
    bool initialize (int size, const int *amap, const double *enc, const double *zos);

    /**
     * Clean up internal data and memory.
     * @return true if uninitialization is executed, false otherwise.
     */
    bool uninitialize ();

public:
    /* Constructor.
     * @param y is the pointer to the class instance inheriting from this
     *  implementation.
     */
    ImplementMotorEncoders(yarp::dev::IMotorEncodersRaw *y);


    virtual ~ImplementMotorEncoders();

    yarp::dev::yarp_ret_value getNumberOfMotorEncoders(int *num) override;

    yarp::dev::yarp_ret_value resetMotorEncoder(int m) override;
    yarp::dev::yarp_ret_value resetMotorEncoders() override;
    yarp::dev::yarp_ret_value setMotorEncoder(int m, const double val) override;
    yarp::dev::yarp_ret_value setMotorEncoders(const double *vals) override;
    yarp::dev::yarp_ret_value setMotorEncoderCountsPerRevolution(int m, const double cpr) override;
    yarp::dev::yarp_ret_value getMotorEncoderCountsPerRevolution(int m, double *cpr) override;
    yarp::dev::yarp_ret_value getMotorEncoder(int m, double *v) override;
    yarp::dev::yarp_ret_value getMotorEncodersTimed(double *encs, double *time) override;
    yarp::dev::yarp_ret_value getMotorEncoderTimed(int m, double *v, double *t) override;
    yarp::dev::yarp_ret_value getMotorEncoders(double *encs) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeed(int m, double *spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeeds(double *spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAcceleration(int m, double *spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAccelerations(double *accs) override;
};

#endif // YARP_DEV_IMPLEMENTMOTORENCODERS_H
