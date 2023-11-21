/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include <yarp/dev/ImplementPositionControl.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/os/Log.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>

using namespace yarp::dev;
using namespace yarp::os;

ImplementPositionControl::ImplementPositionControl(yarp::dev::IPositionControlRaw *y) :
    iPosition(y),
    helper(nullptr),
    intBuffManager(nullptr),
    doubleBuffManager(nullptr),
    boolBuffManager(nullptr)
{;}


ImplementPositionControl::~ImplementPositionControl()
{
    uninitialize();
}

/**
 * Allocate memory for internal data
 * @param size the number of joints
 * @param amap axis map for this device wrapper
 * @param enc encoder conversion factor, from high level to hardware
 * @param zos offset for setting the zero point. Units are relative to high level user interface (degrees)
 * @return true if uninitialization is executed, false otherwise.
 */
bool ImplementPositionControl::initialize(int size, const int *amap, const double *enc, const double *zos)
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

    boolBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<bool> (size, 1);
    yAssert (boolBuffManager != nullptr);
    return true;
}

/**
 * Clean up internal data and memory.
 * @return true if uninitialization is executed, false otherwise.
 */
bool ImplementPositionControl::uninitialize()
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

    if(boolBuffManager)
    {
        delete boolBuffManager;
        boolBuffManager=nullptr;
    }

    return true;
}

yarp_ret_value ImplementPositionControl::positionMove(int j, double ang)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    castToMapper(helper)->posA2E(ang, j, enc, k);
    return iPosition->positionMoveRaw(k, enc);
}

yarp_ret_value ImplementPositionControl::positionMove(const int n_joints, const int *joints, const double *refs)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
        buffValues[idx] = castToMapper(helper)->posA2E(refs[idx], joints[idx]);
    }
    yarp_ret_value ret = iPosition->positionMoveRaw(n_joints, buffJoints.getData(), buffValues.getData());

    intBuffManager->releaseBuffer(buffJoints);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::positionMove(const double *refs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->posA2E(refs, buffValues.getData());

    yarp_ret_value ret = iPosition->positionMoveRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::relativeMove(int j, double delta)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    castToMapper(helper)->velA2E(delta, j, enc, k);

    return iPosition->relativeMoveRaw(k,enc);
}

yarp_ret_value ImplementPositionControl::relativeMove(const int n_joints, const int *joints, const double *deltas)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
        buffValues[idx] = castToMapper(helper)->velA2E(deltas[idx], joints[idx]);
    }
    yarp_ret_value ret = iPosition->relativeMoveRaw(n_joints, buffJoints.getData(), buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementPositionControl::relativeMove(const double *deltas)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->velA2E(deltas, buffValues.getData());
    yarp_ret_value ret = iPosition->relativeMoveRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::checkMotionDone(int j, bool *flag)
{
    JOINTIDCHECK(j)
    int k=castToMapper(helper)->toHw(j);

    return iPosition->checkMotionDoneRaw(k,flag);
}

yarp_ret_value ImplementPositionControl::checkMotionDone(const int n_joints, const int *joints, bool *flags)
{
    JOINTSIDSCHECK()

    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }
    yarp_ret_value ret = iPosition->checkMotionDoneRaw(n_joints, buffJoints.getData(), flags);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementPositionControl::checkMotionDone(bool *flag)
{
    return iPosition->checkMotionDoneRaw(flag);
}

yarp_ret_value ImplementPositionControl::setRefSpeed(int j, double sp)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    castToMapper(helper)->velA2E_abs(sp, j, enc, k);
    return iPosition->setRefSpeedRaw(k, enc);
}

yarp_ret_value ImplementPositionControl::setRefSpeeds(const int n_joints, const int *joints, const double *spds)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        castToMapper(helper)->velA2E_abs(spds[idx], joints[idx], buffValues[idx], buffJoints[idx]);
    }
    yarp_ret_value ret = iPosition->setRefSpeedsRaw(n_joints, buffJoints.getData(), buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementPositionControl::setRefSpeeds(const double *spds)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->velA2E_abs(spds, buffValues.getData());
    yarp_ret_value ret = iPosition->setRefSpeedsRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::setRefAcceleration(int j, double acc)
{
    JOINTIDCHECK(j)
    int k;
    double enc;

    castToMapper(helper)->accA2E_abs(acc, j, enc, k);
    return iPosition->setRefAccelerationRaw(k, enc);
}

yarp_ret_value ImplementPositionControl::setRefAccelerations(const int n_joints, const int *joints, const double *accs)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        castToMapper(helper)->accA2E_abs(accs[idx], joints[idx], buffValues[idx], buffJoints[idx]);
    }

    yarp_ret_value ret = iPosition->setRefAccelerationsRaw(n_joints, buffJoints.getData(), buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementPositionControl::setRefAccelerations(const double *accs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->accA2E_abs(accs, buffValues.getData());

    yarp_ret_value ret = iPosition->setRefAccelerationsRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::getRefSpeed(int j, double *ref)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    k=castToMapper(helper)->toHw(j);

    yarp_ret_value ret = iPosition->getRefSpeedRaw(k, &enc);

    *ref=(castToMapper(helper)->velE2A_abs(enc, k));

    return ret;
}

yarp_ret_value ImplementPositionControl::getRefSpeeds(const int n_joints, const int *joints, double *spds)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }

    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iPosition->getRefSpeedsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    for(int idx=0; idx<n_joints; idx++)
    {
        spds[idx]=castToMapper(helper)->velE2A_abs(buffValues[idx], buffJoints[idx]);
    }
    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementPositionControl::getRefSpeeds(double *spds)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iPosition->getRefSpeedsRaw(buffValues.getData());
    castToMapper(helper)->velE2A_abs(buffValues.getData(), spds);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::getRefAccelerations(double *accs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret=iPosition->getRefAccelerationsRaw(buffValues.getData());
    castToMapper(helper)->accE2A_abs(buffValues.getData(), accs);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::getRefAccelerations(const int n_joints, const int *joints, double *accs)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints = intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }

    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iPosition->getRefAccelerationsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    for(int idx=0; idx<n_joints; idx++)
    {
        accs[idx]=castToMapper(helper)->accE2A_abs(buffValues[idx], buffJoints[idx]);
    }
    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementPositionControl::getRefAcceleration(int j, double *acc)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iPosition->getRefAccelerationRaw(k, &enc);

    *acc=castToMapper(helper)->accE2A_abs(enc, k);

    return ret;
}

yarp_ret_value ImplementPositionControl::stop(int j)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);

    return iPosition->stopRaw(k);
}

yarp_ret_value ImplementPositionControl::stop(const int n_joint, const int *joints)
{
    yarp::dev::impl::Buffer<int> buffValues =intBuffManager->getBuffer();
    for(int idx=0; idx<n_joint; idx++)
    {
        buffValues[idx] = castToMapper(helper)->toHw(joints[idx]);
    }

    yarp_ret_value ret = iPosition->stopRaw(n_joint, buffValues.getData());
    intBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::stop()
{
    return iPosition->stopRaw();
}

yarp_ret_value ImplementPositionControl::getAxes(int *axis)
{
    (*axis)=castToMapper(helper)->axes();

    return yarp_ret_value_ok;
}


yarp_ret_value ImplementPositionControl::getTargetPosition(const int j, double* ref)
{
    JOINTIDCHECK(j)
    int k;
    double enc;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iPosition->getTargetPositionRaw(k, &enc);

    *ref=castToMapper(helper)->posE2A(enc, k);

    return ret;
}

yarp_ret_value ImplementPositionControl::getTargetPositions(double* refs)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret=iPosition->getTargetPositionsRaw(buffValues.getData());
    castToMapper(helper)->posE2A(buffValues.getData(), refs);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementPositionControl::getTargetPositions(const int n_joints, const int* joints, double* refs)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints =intBuffManager->getBuffer();
    for(int idx=0; idx<n_joints; idx++)
    {
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iPosition->getTargetPositionsRaw(n_joints, buffJoints.getData(), buffValues.getData());

    for(int idx=0; idx<n_joints; idx++)
    {
        refs[idx]=castToMapper(helper)->posE2A(buffValues[idx], buffJoints[idx]);
    }
    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}
/////////////////// End Implement PostionControl
