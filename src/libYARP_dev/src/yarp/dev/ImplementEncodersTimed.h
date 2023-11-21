/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMPLEMENTENCODERSTIMED_H
#define YARP_DEV_IMPLEMENTENCODERSTIMED_H

#include <yarp/dev/IEncodersTimed.h>

namespace yarp::dev {
class ImplementEncodersTimed;
}

namespace yarp::dev::impl {

template <typename T>
class FixedSizeBuffersManager;

} // namespace yarp::dev::impl

class YARP_dev_API yarp::dev::ImplementEncodersTimed: public IEncodersTimed
{
protected:
    IEncodersTimedRaw *iEncoders;
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
    ImplementEncodersTimed(yarp::dev::IEncodersTimedRaw *y);


    virtual ~ImplementEncodersTimed();

    yarp::dev::yarp_ret_value getAxes(int *ax) override;

    yarp::dev::yarp_ret_value resetEncoder(int j) override;
    yarp::dev::yarp_ret_value resetEncoders() override;
    yarp::dev::yarp_ret_value setEncoder(int j, double val) override;
    yarp::dev::yarp_ret_value setEncoders(const double *vals) override;
    yarp::dev::yarp_ret_value getEncoder(int j, double *v) override;
    yarp::dev::yarp_ret_value getEncodersTimed(double *encs, double *time) override;
    yarp::dev::yarp_ret_value getEncoderTimed(int j, double *v, double *t) override;
    yarp::dev::yarp_ret_value getEncoders(double *encs) override;
    yarp::dev::yarp_ret_value getEncoderSpeed(int j, double *spds) override;
    yarp::dev::yarp_ret_value getEncoderSpeeds(double *spds) override;
    yarp::dev::yarp_ret_value getEncoderAcceleration(int j, double *spds) override;
    yarp::dev::yarp_ret_value getEncoderAccelerations(double *accs) override;
};

#endif // YARP_DEV_IMPLEMENTENCODERSTIMED_H
