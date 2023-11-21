/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/ImplementEncoders.h>
#include <yarp/dev/ControlBoardHelper.h>

#include <cmath>

// Be careful: this file contains template implementations and is included by translation
// units that use the template (e.g. .cpp files). Avoid putting here non-template functions to
// avoid multiple definitions.

using namespace yarp::dev;

////////////////////////
// Encoder Interface Implementation
ImplementEncoders::ImplementEncoders(yarp::dev::IEncodersRaw  *y)
{
    iEncoders= y;
    helper = nullptr;
    temp=nullptr;
}

ImplementEncoders::~ImplementEncoders()
{
    uninitialize();
}

bool ImplementEncoders:: initialize (int size, const int *amap, const double *enc, const double *zos)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, enc, zos));
    yAssert (helper != nullptr);
    temp=new double [size];
    yAssert (temp != nullptr);
    return true;
}

/**
* Clean up internal data and memory.
* @return true if uninitialization is executed, false otherwise.
*/
bool ImplementEncoders::uninitialize ()
{
    if (helper!=nullptr)
    {
        delete castToMapper(helper);
        helper=nullptr;
    }

    checkAndDestroy(temp);

    return true;
}

yarp_ret_value ImplementEncoders::getAxes(int *ax)
{
    (*ax)=castToMapper(helper)->axes();
    return yarp_ret_value_ok;
}

yarp_ret_value ImplementEncoders::resetEncoder(int j)
{
    int k;
    k=castToMapper(helper)->toHw(j);

    return iEncoders->resetEncoderRaw(k);
}


yarp_ret_value ImplementEncoders::resetEncoders()
{
    return iEncoders->resetEncodersRaw();
}

yarp_ret_value ImplementEncoders::setEncoder(int j, double val)
{
    int k;
    double enc;

    castToMapper(helper)->posA2E(val, j, enc, k);

    return iEncoders->setEncoderRaw(k, enc);
}

yarp_ret_value ImplementEncoders::setEncoders(const double *val)
{
    castToMapper(helper)->posA2E(val, temp);

    return iEncoders->setEncodersRaw(temp);
}

yarp_ret_value ImplementEncoders::getEncoder(int j, double *v)
{
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderRaw(k, &enc);

    *v=castToMapper(helper)->posE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementEncoders::getEncoders(double *v)
{
    castToMapper(helper)->axes();

    yarp_ret_value ret=iEncoders->getEncodersRaw(temp);

    castToMapper(helper)->posE2A(temp, v);

    return ret;
}

yarp_ret_value ImplementEncoders::getEncoderSpeed(int j, double *v)
{
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderSpeedRaw(k, &enc);

    *v=castToMapper(helper)->velE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementEncoders::getEncoderSpeeds(double *v)
{
    yarp_ret_value ret=iEncoders->getEncoderSpeedsRaw(temp);

    castToMapper(helper)->velE2A(temp, v);

    return ret;
}

yarp_ret_value ImplementEncoders::getEncoderAcceleration(int j, double *v)
{
    int k;
    double enc;

    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret=iEncoders->getEncoderAccelerationRaw(k, &enc);

    *v=castToMapper(helper)->accE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementEncoders::getEncoderAccelerations(double *v)
{
    yarp_ret_value ret=iEncoders->getEncoderAccelerationsRaw(temp);

    castToMapper(helper)->accE2A(temp, v);

    return ret;
}
