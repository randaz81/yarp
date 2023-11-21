/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/ImplementControlMode.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>

#include <cstdio>
using namespace yarp::dev;
using namespace yarp::os;

ImplementControlMode::ImplementControlMode(IControlModeRaw *r):
    helper(nullptr),
    raw(r),
    buffManager(nullptr)
{;}

bool ImplementControlMode::initialize(int size, const int *amap)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap));
    yAssert (helper != nullptr);

    buffManager = new yarp::dev::impl::FixedSizeBuffersManager<int> (size);
    yAssert (buffManager != nullptr);
    return true;
}

ImplementControlMode::~ImplementControlMode()
{
    uninitialize();
}

bool ImplementControlMode::uninitialize ()
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

yarp_ret_value ImplementControlMode::getControlMode(int j, int *f)
{
    JOINTIDCHECK(j)
    int k=castToMapper(helper)->toHw(j);
    return raw->getControlModeRaw(k, f);
}

yarp_ret_value ImplementControlMode::getControlModes(int *modes)
{
    yarp::dev::impl::Buffer<int> buffValues = buffManager->getBuffer();

    yarp_ret_value ret=raw->getControlModesRaw(buffValues.getData());
    castToMapper(helper)->toUser(buffValues.getData(), modes);

    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementControlMode::getControlModes(const int n_joints, const int *joints, int *modes)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffValues = buffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffValues[idx] = castToMapper(helper)->toHw(joints[idx]);
    }
    yarp_ret_value ret = raw->getControlModesRaw(n_joints, buffValues.getData(), modes);

    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementControlMode::setControlMode(const int j, const int mode)
{
    JOINTIDCHECK(j)
    int k=castToMapper(helper)->toHw(j);
    return raw->setControlModeRaw(k, mode);
}

yarp_ret_value ImplementControlMode::setControlModes(const int n_joints, const int *joints, int *modes)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffValues  = buffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffValues[idx] = castToMapper(helper)->toHw(joints[idx]);
    }
    yarp_ret_value ret = raw->setControlModesRaw(n_joints, buffValues.getData(), modes);

    buffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementControlMode::setControlModes(int *modes)
{
    yarp::dev::impl::Buffer<int> buffValues  = buffManager->getBuffer();
    for(int idx=0; idx<castToMapper(helper)->axes(); idx++)
    {
        buffValues[castToMapper(helper)->toHw(idx)] = modes[idx];
    }
    yarp_ret_value ret = raw->setControlModesRaw(buffValues.getData());
    buffManager->releaseBuffer(buffValues);
    return ret;
}
