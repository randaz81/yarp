/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include <yarp/dev/ImplementVelocityControl.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/os/Log.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>

using namespace yarp::dev;
using namespace yarp::os;

ImplementVelocityControl::ImplementVelocityControl(IVelocityControlRaw *y) :
    iVelocity(y),
    helper(nullptr),
    intBuffManager(nullptr),
    doubleBuffManager(nullptr)
{;}

ImplementVelocityControl::~ImplementVelocityControl()
{
    uninitialize();
}

bool ImplementVelocityControl::initialize(int size, const int *axis_map, const double *enc, const double *zeros)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, axis_map, enc, zeros));
    yAssert (helper != nullptr);

    intBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<int> (size);
    yAssert (intBuffManager != nullptr);

    doubleBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<double> (size);
    yAssert (doubleBuffManager != nullptr);

    return true;
}

bool ImplementVelocityControl::uninitialize()
{
    if(helper != nullptr)
    {
        delete castToMapper(helper);
        helper = nullptr;
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

yarp_ret_value ImplementVelocityControl::getAxes(int *ax)
{
    (*ax)=castToMapper(helper)->axes();
    return yarp_ret_value_ok;
}

yarp_ret_value ImplementVelocityControl::velocityMove(int j, double sp)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    castToMapper(helper)->velA2E(sp, j, enc, k);
    return iVelocity->velocityMoveRaw(k, enc);
}

yarp_ret_value ImplementVelocityControl::velocityMove(const int n_joints, const int *joints, const double *spds)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<int> buffJoints =  intBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
        buffValues[idx] = castToMapper(helper)->velA2E(spds[idx], joints[idx]);
    }
    yarp_ret_value ret = iVelocity->velocityMoveRaw(n_joints, buffJoints.getData(), buffValues.getData());

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementVelocityControl::velocityMove(const double *sp)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->velA2E(sp, buffValues.getData());
    yarp_ret_value ret = iVelocity->velocityMoveRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementVelocityControl::getRefVelocity(const int j, double* vel)
{
    JOINTIDCHECK(j)
    int k;
    double tmp;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iVelocity->getRefVelocityRaw(k, &tmp);
    *vel=castToMapper(helper)->velE2A(tmp, k);
    return ret;
}

yarp_ret_value ImplementVelocityControl::getRefVelocities(double *vels)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret=iVelocity->getRefVelocitiesRaw(buffValues.getData());
    castToMapper(helper)->velE2A(buffValues.getData(), vels);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementVelocityControl::getRefVelocities(const int n_joints, const int *joints, double *vels)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }

    yarp_ret_value ret = iVelocity->getRefVelocitiesRaw(n_joints, buffJoints.getData(), buffValues.getData());

    for(int idx=0; idx<n_joints; idx++)
    {
        vels[idx]=castToMapper(helper)->velE2A(buffValues[idx], buffJoints[idx]);
    }

    intBuffManager->releaseBuffer(buffJoints);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementVelocityControl::setRefAcceleration(int j, double acc)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    castToMapper(helper)->accA2E_abs(acc, j, enc, k);
    return iVelocity->setRefAccelerationRaw(k, enc);
}

yarp_ret_value ImplementVelocityControl::setRefAccelerations(const int n_joints, const int *joints, const double *accs)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints =  intBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        castToMapper(helper)->accA2E_abs(accs[idx], joints[idx], buffValues[idx], buffJoints[idx]);
    }
    yarp_ret_value ret = iVelocity->setRefAccelerationsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);

    return ret;
}

yarp_ret_value ImplementVelocityControl::setRefAccelerations(const double *accs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->accA2E_abs(accs, buffValues.getData());
    yarp_ret_value ret = iVelocity->setRefAccelerationsRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementVelocityControl::getRefAcceleration(int j, double *acc)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iVelocity->getRefAccelerationRaw(k, &enc);
    *acc=castToMapper(helper)->accE2A_abs(enc, k);
    return ret;
}

yarp_ret_value ImplementVelocityControl::getRefAccelerations(const int n_joints, const int *joints, double *accs)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<int> buffJoints =  intBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx]=castToMapper(helper)->toHw(joints[idx]);
    }

    yarp_ret_value ret = iVelocity->getRefAccelerationsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    for(int idx=0; idx<n_joints; idx++)
    {
        accs[idx]=castToMapper(helper)->accE2A_abs(buffValues[idx], buffJoints[idx]);
    }

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}


yarp_ret_value ImplementVelocityControl::getRefAccelerations(double *accs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret=iVelocity->getRefAccelerationsRaw(buffValues.getData());
    castToMapper(helper)->accE2A_abs(buffValues.getData(), accs);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}


yarp_ret_value ImplementVelocityControl::stop(int j)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    return iVelocity->stopRaw(k);
}


yarp_ret_value ImplementVelocityControl::stop(const int n_joints, const int *joints)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints =  intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }
    yarp_ret_value ret = iVelocity->stopRaw(n_joints, buffJoints.getData());
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}


yarp_ret_value ImplementVelocityControl::stop()
{
    return iVelocity->stopRaw();
}
