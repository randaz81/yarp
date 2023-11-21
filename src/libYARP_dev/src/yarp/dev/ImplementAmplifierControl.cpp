/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/ImplementAmplifierControl.h>
#include <yarp/dev/ControlBoardHelper.h>

#include <cmath>

// Be careful: this file contains template implementations and is included by translation
// units that use the template (e.g. .cpp files). Avoid putting here non-template functions to
// avoid multiple definitions.

using namespace yarp::dev;

ImplementAmplifierControl::ImplementAmplifierControl(yarp::dev::IAmplifierControlRaw  *y)
{
    iAmplifier= y;
    helper = nullptr;
    dTemp=nullptr;
    iTemp=nullptr;
}

ImplementAmplifierControl::~ImplementAmplifierControl()
{
    uninitialize();
}

bool ImplementAmplifierControl:: initialize (int size, const int *amap, const double *enc, const double *zos, const double *ampereFactor, const double *voltFactor)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, enc, zos,nullptr, ampereFactor, voltFactor));
    yAssert (helper != nullptr);
    dTemp=new double[size];
    yAssert (dTemp != nullptr);
    iTemp=new int[size];
    yAssert (iTemp != nullptr);

    return true;
}

/**
* Clean up internal data and memory.
* @return true if uninitialization is executed, false otherwise.
*/
bool ImplementAmplifierControl::uninitialize ()
{
    if (helper != nullptr) {
        delete castToMapper(helper);
    }

    delete [] dTemp;
    delete [] iTemp;

    helper=nullptr;
    dTemp=nullptr;
    iTemp=nullptr;
    return true;
}

yarp_ret_value ImplementAmplifierControl::enableAmp(int j)
{
    int k=castToMapper(helper)->toHw(j);

    return iAmplifier->enableAmpRaw(k);
}

yarp_ret_value ImplementAmplifierControl::disableAmp(int j)
{
    int k=castToMapper(helper)->toHw(j);

    return iAmplifier->disableAmpRaw(k);
}

yarp_ret_value ImplementAmplifierControl::getCurrents(double *currs)
{
    yarp_ret_value ret=iAmplifier->getCurrentsRaw(dTemp);
    castToMapper(helper)->ampereS2A(dTemp, currs);
    return ret;
}

yarp_ret_value ImplementAmplifierControl::getCurrent(int j, double *c)
{
    double temp = 0;
    int k = castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iAmplifier->getCurrentRaw(k, &temp);
    castToMapper(helper)->ampereS2A(temp, k, *c, j);
    return ret;
}

yarp_ret_value ImplementAmplifierControl::getAmpStatus(int *st)
{
    yarp_ret_value ret=iAmplifier->getAmpStatusRaw(iTemp);
    castToMapper(helper)->toUser(iTemp, st);

    return ret;
}

yarp_ret_value ImplementAmplifierControl::getAmpStatus(int k, int *st)
{
    int j=castToMapper(helper)->toHw(k);
    yarp_ret_value ret=iAmplifier->getAmpStatusRaw(j, st);

    return ret;
}

yarp_ret_value ImplementAmplifierControl::setMaxCurrent(int m, double v)
{
    int k;
    double curr;
    castToMapper(helper)->ampereA2S(v, m, curr, k);
    return iAmplifier->setMaxCurrentRaw(k, curr);
}

yarp_ret_value ImplementAmplifierControl::getMaxCurrent(int j, double* v)
{
    double val;
    int k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iAmplifier->getMaxCurrentRaw(k, &val);
    *v = castToMapper(helper)->ampereS2A(val, k);
    return ret;
}

yarp_ret_value ImplementAmplifierControl::getNominalCurrent(int m, double *curr)
{
    int k;
    double tmp;

    k=castToMapper(helper)->toHw(m);
    yarp_ret_value ret=iAmplifier->getNominalCurrentRaw(k, &tmp);
    *curr=castToMapper(helper)->ampereS2A(tmp, k);
    return ret;
}

yarp_ret_value ImplementAmplifierControl::getPeakCurrent(int m, double *curr)
{
    int k;
    double tmp;

    k=castToMapper(helper)->toHw(m);
    yarp_ret_value ret=iAmplifier->getPeakCurrentRaw(k, &tmp);
    *curr=castToMapper(helper)->ampereS2A(tmp, k);
    return ret;
}

yarp_ret_value ImplementAmplifierControl::setPeakCurrent(int m, const double curr)
{
    int k;
    double val;
    castToMapper(helper)->ampereA2S(curr, m, val, k);
    return iAmplifier->setPeakCurrentRaw(k, val);
}

yarp_ret_value ImplementAmplifierControl::setNominalCurrent(int m, const double curr)
{
    int k;
    double val;
    castToMapper(helper)->ampereA2S(curr, m, val, k);
    return iAmplifier->setNominalCurrentRaw(k, val);
}

yarp_ret_value ImplementAmplifierControl::getPWM(int m, double* pwm)
{
    int k;
    k=castToMapper(helper)->toHw(m);
    return iAmplifier->getPWMRaw(k, pwm);
}

yarp_ret_value ImplementAmplifierControl::getPWMLimit(int m, double* limit)
{
    int k;
    k=castToMapper(helper)->toHw(m);
    return iAmplifier->getPWMLimitRaw(k, limit);
}

yarp_ret_value ImplementAmplifierControl::setPWMLimit(int m, const double limit)
{
    int k;
    k=castToMapper(helper)->toHw(m);
    return iAmplifier->setPWMLimitRaw(k, limit);
}

yarp_ret_value ImplementAmplifierControl::getPowerSupplyVoltage(int m, double *voltage)
{
    int k;
    k=castToMapper(helper)->toHw(m);
    return iAmplifier->getPowerSupplyVoltageRaw(k, voltage);
}
