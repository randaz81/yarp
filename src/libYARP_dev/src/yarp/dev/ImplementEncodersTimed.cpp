/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "yarp/dev/ControlBoardInterfacesImpl.h"
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>

#include <cstdio>
using namespace yarp::dev;
using namespace yarp::os;

////////////////////////
// Encoder Interface Timed Implementation
ImplementEncodersTimed::ImplementEncodersTimed(IEncodersTimedRaw *y):
    iEncoders(y),
    helper(nullptr),
    buffManager(nullptr)
{;}

ImplementEncodersTimed::~ImplementEncodersTimed()
{
    uninitialize();
}

bool ImplementEncodersTimed:: initialize (int size, const int *amap, const double *enc, const double *zos)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, enc, zos));
    yAssert (helper != nullptr);

    buffManager = new yarp::dev::impl::FixedSizeBuffersManager<double> (size);
    yAssert (buffManager != nullptr);
    return true;
}

/**
* Clean up internal data and memory.
* @return true if uninitialization is executed, false otherwise.
*/
bool ImplementEncodersTimed::uninitialize ()
{
    if (helper!=nullptr)
    {
        delete castToMapper(helper);
        helper=nullptr;
    }

    if (buffManager!=nullptr)
    {
        delete buffManager;
        buffManager=nullptr;
    }
    return true;
}

yarp_ret_value ImplementEncodersTimed::getAxes(int *ax)
{
    (*ax)=castToMapper(helper)->axes();
    return yarp_ret_value_ok;
}

yarp_ret_value ImplementEncodersTimed::resetEncoder(int j)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);

    return iEncoders->resetEncoderRaw(k);
}

yarp_ret_value ImplementEncodersTimed::resetEncoders()
{
    return iEncoders->resetEncodersRaw();
}

yarp_ret_value ImplementEncodersTimed::setEncoder(int j, double val)
{
    JOINTIDCHECK(j)
    int k;
    double enc;

    castToMapper(helper)->posA2E(val, j, enc, k);

    return iEncoders->setEncoderRaw(k, enc);
}

yarp_ret_value ImplementEncodersTimed::setEncoders(const double *val)
{
    yarp::dev::impl::Buffer<double> buffValues = buffManager->getBuffer();
    castToMapper(helper)->posA2E(val, buffValues.getData());
    yarp_ret_value ret = iEncoders->setEncodersRaw(buffValues.getData());
    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoder(int j, double *v)
{
    JOINTIDCHECK(j)
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderRaw(k, &enc);

    *v=castToMapper(helper)->posE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoders(double *v)
{
    yarp::dev::impl::Buffer<double> buffValues =buffManager->getBuffer();
    yarp_ret_value ret = iEncoders->getEncodersRaw(buffValues.getData());
    castToMapper(helper)->posE2A(buffValues.getData(), v);
    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoderSpeed(int j, double *v)
{
    JOINTIDCHECK(j)
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderSpeedRaw(k, &enc);

    *v=castToMapper(helper)->velE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoderSpeeds(double *v)
{
    yarp::dev::impl::Buffer<double> buffValues = buffManager->getBuffer();
    yarp_ret_value ret=iEncoders->getEncoderSpeedsRaw(buffValues.getData());
    castToMapper(helper)->velE2A(buffValues.getData(), v);
    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoderAcceleration(int j, double *v)
{
    JOINTIDCHECK(j)
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderAccelerationRaw(k, &enc);

    *v=castToMapper(helper)->accE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoderAccelerations(double *v)
{
    yarp::dev::impl::Buffer<double> buffValues = buffManager->getBuffer();
    yarp_ret_value ret = iEncoders->getEncoderAccelerationsRaw(buffValues.getData());
    castToMapper(helper)->accE2A(buffValues.getData(), v);
    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementEncodersTimed::getEncoderTimed(int j, double *v, double *t)
{
    JOINTIDCHECK(j)
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderTimedRaw(k, &enc, t);

    *v=castToMapper(helper)->posE2A(enc, k);

    return ret;
}


yarp_ret_value ImplementEncodersTimed::getEncodersTimed(double *v, double *t)
{
    yarp::dev::impl::Buffer<double> b_v = buffManager->getBuffer();
    yarp::dev::impl::Buffer<double> b_t = buffManager->getBuffer();
    yarp_ret_value ret=iEncoders->getEncodersTimedRaw(b_v.getData(), b_t.getData());

    castToMapper(helper)->posE2A(b_v.getData(), v);
    castToMapper(helper)->toUser(b_t.getData(), t);

    buffManager->releaseBuffer(b_v);
    buffManager->releaseBuffer(b_t);
    return ret;
}
