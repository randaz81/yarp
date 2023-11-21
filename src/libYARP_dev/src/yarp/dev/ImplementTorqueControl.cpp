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

ImplementTorqueControl::ImplementTorqueControl(ITorqueControlRaw *tq):
    iTorqueRaw(tq),
    helper(nullptr),
    intBuffManager(nullptr),
    doubleBuffManager(nullptr)
{;}

ImplementTorqueControl::~ImplementTorqueControl()
{
    uninitialize();
}

bool ImplementTorqueControl::initialize(int size, const int *amap, const double *enc, const double *zos, const double *nw, const double* amps, const double* dutys, const double* bemfs, const double* ktaus)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, enc, zos, nw, amps, nullptr, dutys,bemfs,ktaus));
    yAssert (helper != nullptr);

    intBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<int> (size);
    yAssert (intBuffManager != nullptr);

    doubleBuffManager = new yarp::dev::impl::FixedSizeBuffersManager<double> (size);
    yAssert (doubleBuffManager != nullptr);

    return true;
}

bool ImplementTorqueControl::uninitialize ()
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

yarp_ret_value ImplementTorqueControl::getAxes(int *axes)
{
    return iTorqueRaw->getAxes(axes);
}

yarp_ret_value ImplementTorqueControl::getRefTorque(int j, double *r)
{
    JOINTIDCHECK(j)
    int k;
    double torque;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iTorqueRaw->getRefTorqueRaw(k, &torque);
    *r=castToMapper(helper)->trqS2N(torque, k);
    return ret;
}

yarp_ret_value ImplementTorqueControl::setMotorTorqueParams(int j,  const yarp::dev::MotorTorqueParameters params)
{
    JOINTIDCHECK(j)
    int k;

    yarp::dev::MotorTorqueParameters params_raw;
    castToMapper(helper)->bemf_user2raw(params.bemf, j, params_raw.bemf, k);
    castToMapper(helper)->ktau_user2raw(params.ktau, j, params_raw.ktau, k);
    params_raw.bemf_scale = params.bemf_scale;
    params_raw.ktau_scale = params.ktau_scale;

    castToMapper(helper)->viscousPos_user2raw(params.viscousPos, j, params_raw.viscousPos, k);
    castToMapper(helper)->viscousNeg_user2raw(params.viscousNeg, j, params_raw.viscousNeg, k);
    castToMapper(helper)->coulombPos_user2raw(params.coulombPos, j, params_raw.coulombPos, k);
    castToMapper(helper)->coulombNeg_user2raw(params.coulombNeg, j, params_raw.coulombNeg, k);
    castToMapper(helper)->velocityThres_user2raw(params.velocityThres, j, params_raw.velocityThres, k);

    return iTorqueRaw->setMotorTorqueParamsRaw(k, params_raw);
}

yarp_ret_value ImplementTorqueControl::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params)
{
    JOINTIDCHECK(j)
    int k=castToMapper(helper)->toHw(j);

    yarp::dev::MotorTorqueParameters params_raw;
    yarp_ret_value b = iTorqueRaw->getMotorTorqueParamsRaw(k, &params_raw);
    int tmp_j;

    if (b)
    {
        *params = params_raw;
        castToMapper(helper)->bemf_raw2user(params_raw.bemf, k, (*params).bemf, tmp_j);
        castToMapper(helper)->ktau_raw2user(params_raw.ktau, k, (*params).ktau, tmp_j);
        (*params).bemf_scale = params_raw.bemf_scale;
        (*params).ktau_scale = params_raw.ktau_scale;
        castToMapper(helper)->viscousPos_raw2user(params_raw.viscousPos, k, (*params).viscousPos, tmp_j);
        castToMapper(helper)->viscousNeg_raw2user(params_raw.viscousNeg, k, (*params).viscousNeg, tmp_j);
        castToMapper(helper)->coulombPos_raw2user(params_raw.coulombPos, k, (*params).coulombPos, tmp_j);
        castToMapper(helper)->coulombNeg_raw2user(params_raw.coulombNeg, k, (*params).coulombNeg, tmp_j);
        castToMapper(helper)->velocityThres_raw2user(params_raw.velocityThres, k, (*params).velocityThres, tmp_j);
    }
    return b;
}

yarp_ret_value ImplementTorqueControl::getRefTorques(double *t)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iTorqueRaw->getRefTorquesRaw(buffValues.getData());
    castToMapper(helper)->trqS2N(buffValues.getData(),t);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementTorqueControl::setRefTorques(const double *t)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    castToMapper(helper)->trqN2S(t, buffValues.getData());
    yarp_ret_value ret = iTorqueRaw->setRefTorquesRaw(buffValues.getData());
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementTorqueControl::setRefTorque(int j, double t)
{
    JOINTIDCHECK(j)
    int k;
    double sens;
    castToMapper(helper)->trqN2S(t,j,sens,k);
    return iTorqueRaw->setRefTorqueRaw(k, sens);
}

yarp_ret_value ImplementTorqueControl::getTorques(double *t)
{
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();
    yarp_ret_value ret = iTorqueRaw->getTorquesRaw(buffValues.getData());
    castToMapper(helper)->toUser(buffValues.getData(), t);
    doubleBuffManager->releaseBuffer(buffValues);
    return ret;
}

yarp_ret_value ImplementTorqueControl::setRefTorques(const int n_joints, const int *joints, const double *t)
{
    JOINTSIDSCHECK()
    yarp::dev::impl::Buffer<int> buffJoints =  intBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffValues = doubleBuffManager->getBuffer();

    for(int idx=0; idx<n_joints; idx++)
    {
        buffValues[idx] =  castToMapper(helper)->trqN2S(t[idx], joints[idx]);
        buffJoints[idx] = castToMapper(helper)->toHw(joints[idx]);
    }
    yarp_ret_value ret = iTorqueRaw->setRefTorquesRaw(n_joints, buffJoints.getData(), buffValues.getData());

    doubleBuffManager->releaseBuffer(buffValues);
    intBuffManager->releaseBuffer(buffJoints);
    return ret;
}

yarp_ret_value ImplementTorqueControl::getTorque(int j, double *t)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    return iTorqueRaw->getTorqueRaw(k, t);
}

yarp_ret_value ImplementTorqueControl::getTorqueRanges(double *min, double *max)
{
    yarp::dev::impl::Buffer<double> buffMin = doubleBuffManager->getBuffer();
    yarp::dev::impl::Buffer<double> buffMax = doubleBuffManager->getBuffer();

    yarp_ret_value ret = iTorqueRaw->getTorqueRangesRaw(buffMin.getData(),buffMax.getData());
    castToMapper(helper)->toUser(buffMin.getData(), min);
    castToMapper(helper)->toUser(buffMax.getData(), max);
    doubleBuffManager->releaseBuffer(buffMin);
    doubleBuffManager->releaseBuffer(buffMax);
    return ret;
}

yarp_ret_value ImplementTorqueControl::getTorqueRange(int j, double *min, double *max)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    return iTorqueRaw->getTorqueRangeRaw(k, min, max);
}
