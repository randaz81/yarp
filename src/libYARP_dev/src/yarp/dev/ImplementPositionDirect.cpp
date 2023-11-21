/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/os/Log.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>

using namespace yarp::dev;
using namespace yarp::os;

ImplementPositionDirect::ImplementPositionDirect(IPositionDirectRaw *y):
    iPDirect(y),
    helper(nullptr),
    intBuffManager(nullptr),
    doubleBuffManager(nullptr)
{;}


ImplementPositionDirect::~ImplementPositionDirect()
{
    uninitialize();
}

bool ImplementPositionDirect::initialize(int size, const int *amap, const double *enc, const double *zos)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, enc, zos));
    yAssert(helper != nullptr);

    intBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<int> (size);
    yAssert (intBuffManager != nullptr);

    doubleBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<double> (size);
    yAssert (doubleBuffManager != nullptr);

    return true;
}

bool ImplementPositionDirect::uninitialize()
{
    if(helper!=nullptr)
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

yarp_ret_value ImplementPositionDirect::getAxes(int *axes)
{
    (*axes)=castToMapper(helper)->axes();
    return yarp_ret_value_ok;
}

yarp_ret_value ImplementPositionDirect::setPosition(int j, double ref)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    castToMapper(helper)->posA2E(ref, j, enc, k);
    return iPDirect->setPositionRaw(k, enc);
}

yarp_ret_value ImplementPositionDirect::setPositions(const int n_joints, const int *joints, const double *refs)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();

    if (n_joints > (int)intBuffManager->getBufferSize()) {
        return false;
    }

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints.setValue(idx, castToMapper(helper)->toHw(joints[idx]));
        buffValues.setValue(idx, castToMapper(helper)->posA2E(refs[idx], joints[idx]));
    }

    yarp_ret_value ret = iPDirect->setPositionsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);

    return ret;
}

yarp_ret_value ImplementPositionDirect::setPositions(const double *refs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->posA2E(refs, buffValues.getData());
    yarp_ret_value ret = iPDirect->setPositionsRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionDirect::getRefPosition(const int j, double* ref)
{
    JOINTIDCHECK(j)
    int k;
    double tmp;
    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret = iPDirect->getRefPositionRaw(k, &tmp);

    *ref=(castToMapper(helper)->posE2A(tmp, k));
    return ret;
}

yarp_ret_value ImplementPositionDirect::getRefPositions(const int n_joints, const int* joints, double* refs)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }

    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iPDirect->getRefPositionsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    for(int idx=0; idx<n_joints; idx++)
    {
        refs[idx]=castToMapper(helper)->posE2A(buffValues[idx], buffJoints[idx]);
    }

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);

    return ret;
}

yarp_ret_value ImplementPositionDirect::getRefPositions(double* refs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iPDirect->getRefPositionsRaw(buffValues.getData());
    castToMapper(helper)->posE2A(buffValues.getData(), refs);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}
