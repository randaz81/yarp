/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "yarp/dev/ImplementPWMControl.h"
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>
#include <iostream>

using namespace yarp::dev;
using namespace yarp::os;

/////////////// implement ImplemenPWMControl
ImplementPWMControl::ImplementPWMControl(IPWMControlRaw *r) :
    helper(nullptr),
    raw(r),
    doubleBuffManager(nullptr)
{;}

bool ImplementPWMControl::initialize(int size, const int *amap, const double* dutyToPWM)
{
    if (helper != nullptr) {
        return false;
    }

    helper = (void *)(new ControlBoardHelper(size, amap, nullptr, nullptr, nullptr, nullptr, nullptr, dutyToPWM));
    yAssert(helper != nullptr);

    doubleBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<double> (size);
    yAssert (doubleBuffManager != nullptr);

    return true;
}

ImplementPWMControl::~ImplementPWMControl()
{
    uninitialize();
}

bool ImplementPWMControl::uninitialize()
{
    if (helper != nullptr)
    {
        delete castToMapper(helper);
        helper = nullptr;
    }

    if(doubleBuffManager)
    {
        delete doubleBuffManager;
        doubleBuffManager=nullptr;
    }

    return true;
}

yarp_ret_value ImplementPWMControl::getNumberOfMotors(int *axes)
{
    return raw->getNumberOfMotorsRaw(axes);
}

yarp_ret_value ImplementPWMControl::setRefDutyCycle(int j, double duty)
{
    JOINTIDCHECK(j)
    int k;
    double pwm;
    castToMapper(helper)->dutycycle2PWM(duty, j, pwm, k);
    return raw->setRefDutyCycleRaw(k, pwm);
}

yarp_ret_value ImplementPWMControl::setRefDutyCycles(const double *duty)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->dutycycle2PWM(duty, buffValues.getData());
    yarp_ret_value ret = raw->setRefDutyCyclesRaw( buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPWMControl::getRefDutyCycle(int j, double *v)
{
    JOINTIDCHECK(j)
    double pwm;
    int k = castToMapper(helper)->toHw(j);
    yarp_ret_value ret = raw->getRefDutyCycleRaw(k, &pwm);
    *v = castToMapper(helper)->PWM2dutycycle(pwm, k);
    return ret;
}

yarp_ret_value ImplementPWMControl::getRefDutyCycles(double *duty)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = raw->getRefDutyCyclesRaw(buffValues.getData());
    castToMapper(helper)->PWM2dutycycle(buffValues.getData(), duty);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPWMControl::getDutyCycle(int j, double *duty)
{
    JOINTIDCHECK(j)
    double pwm;
    int k = castToMapper(helper)->toHw(j);
    yarp_ret_value ret = raw->getDutyCycleRaw(k, &pwm);
    *duty = castToMapper(helper)->PWM2dutycycle(pwm, k);
    return ret;
}

yarp_ret_value ImplementPWMControl::getDutyCycles(double *duty)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = raw->getDutyCyclesRaw(buffValues.getData());
    castToMapper(helper)->PWM2dutycycle(buffValues.getData(), duty);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}
