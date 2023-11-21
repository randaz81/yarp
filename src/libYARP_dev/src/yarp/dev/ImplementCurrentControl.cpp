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

ImplementCurrentControl::ImplementCurrentControl(ICurrentControlRaw *tq):
    iCurrentRaw(tq),
    helper(nullptr),
    intBuffManager(nullptr),
    doubleBuffManager(nullptr)
{;}

ImplementCurrentControl::~ImplementCurrentControl()
{
    uninitialize();
}

bool ImplementCurrentControl::initialize(int size, const int *amap, const double* ampsToSens)
{
    if (helper != nullptr) {
        return false;
    }

    intBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<int> (size);
    yAssert (intBuffManager != nullptr);

    doubleBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<double> (size);
    yAssert (doubleBuffManager != nullptr);

    helper = (void *)(new ControlBoardHelper(size, amap, nullptr, nullptr, nullptr, ampsToSens, nullptr, nullptr));
    yAssert (helper != nullptr);

    return true;
}

bool ImplementCurrentControl::uninitialize()
{
    if (helper!=nullptr)
    {
        delete castToMapper(helper);
        helper=nullptr;
    }

    if(intBuffManager)
    {
        delete intBuffManager;
        intBuffManager=nullptr;
    }

    if(doubleBuffManager)
    {
        delete doubleBuffManager;
        doubleBuffManager=nullptr;
    }

    return true;
}

yarp_ret_value ImplementCurrentControl::getNumberOfMotors(int *axes)
{
    return iCurrentRaw->getNumberOfMotorsRaw(axes);
}

yarp_ret_value ImplementCurrentControl::getRefCurrent(int j, double *r)
{
    JOINTIDCHECK(j)
    int k;
    double current;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iCurrentRaw->getRefCurrentRaw(k, &current);
    *r = castToMapper(helper)->ampereS2A(current, k);
    return ret;
}

yarp_ret_value ImplementCurrentControl::getRefCurrents(double *t)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iCurrentRaw->getRefCurrentsRaw(buffValues.getData());
    castToMapper(helper)->ampereS2A(buffValues.getData(),t);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementCurrentControl::setRefCurrents(const double *t)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->ampereA2S(t, buffValues.getData());
    yarp_ret_value ret = iCurrentRaw->setRefCurrentsRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementCurrentControl::setRefCurrent(int j, double t)
{
    JOINTIDCHECK(j)
    int k;
    double sens;
    castToMapper(helper)->ampereA2S(t,j,sens,k);
    return iCurrentRaw->setRefCurrentRaw(k, sens);
}

yarp_ret_value ImplementCurrentControl::getCurrents(double *t)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iCurrentRaw->getCurrentsRaw(buffValues.getData());
    castToMapper(helper)->ampereS2A(buffValues.getData(), t);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementCurrentControl::setRefCurrents(const int n_joints, const int *joints, const double *t)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
        buffValues[idx] = castToMapper(helper)->ampereA2S(t[idx], joints[idx]);
    }

    yarp_ret_value ret = iCurrentRaw->setRefCurrentsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementCurrentControl::getCurrent(int j, double *t)
{
    JOINTIDCHECK(j)
    int k;
    double current;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iCurrentRaw->getCurrentRaw(k, &current);
    *t = castToMapper(helper)->ampereS2A(current, k);
    return ret;
}

yarp_ret_value ImplementCurrentControl::getCurrentRanges(double *min, double *max)
{
    yarp::dev::impl::Buffer<double> b_min = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> b_max = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iCurrentRaw->getCurrentRangesRaw(b_min.getData(), b_max.getData());
    castToMapper(helper)->ampereS2A(b_min.getData(), min);
    castToMapper(helper)->ampereS2A(b_max.getData(), max);
    doubleBuffManager->releaseBuffer(b_min);
    doubleBuffManager->releaseBuffer(b_max);
    return ret;
}

yarp_ret_value ImplementCurrentControl::getCurrentRange(int j, double *min, double *max)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    double min_t, max_t;
    yarp_ret_value ret = iCurrentRaw->getCurrentRangeRaw(k, &min_t, &max_t);
    *min = castToMapper(helper)->ampereS2A(min_t, k);
    *max = castToMapper(helper)->ampereS2A(max_t, k);
    return ret;
}
