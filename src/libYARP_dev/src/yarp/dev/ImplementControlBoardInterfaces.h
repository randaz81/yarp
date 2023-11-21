/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMPLEMENTCONTROLBOARDINTERFACES_H
#define YARP_DEV_IMPLEMENTCONTROLBOARDINTERFACES_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ImplementControlCalibration.h>
#include <yarp/dev/api.h>

namespace yarp::dev {
class StubImplPositionControlRaw;
class StubImplEncodersRaw;
}

/**
 * Stub implementation of IEncodersRaw interface.
 * Inherit from this class if you want a stub implementation
 * of methods in IPositionControlRaw. This class allows to
 * gradually implement an interface; you just have to implement
 * functions that are useful for the underlying device.
 * Another way to see this class is as a means to convert
 * compile time errors in runtime errors.
 *
 * If you use this class please be aware that the device
 * you are wrapping might not function properly because you
 * missed to implement useful functionalities.
 *
 */
class YARP_dev_API yarp::dev::StubImplEncodersRaw: public IEncodersRaw
{
public:
    virtual ~StubImplEncodersRaw(){}

    yarp::dev::yarp_ret_value getAxes(int *ax) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value resetEncoderRaw(int j) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value resetEncodersRaw() override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setEncoderRaw(int j, double val) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setEncodersRaw(const double *vals) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getEncoderRaw(int j, double *v) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getEncodersRaw(double *encs) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getEncoderSpeedRaw(int j, double *sp) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getEncoderSpeedsRaw(double *spds) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getEncoderAccelerationRaw(int j, double *spds) override
    {return NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getEncoderAccelerationsRaw(double *accs) override
    {return NOT_YET_IMPLEMENTED();}
};

#endif // YARP_DEV_IMPLEMENTCONTROLBOARDINTERFACES_H
