/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _USE_MATH_DEFINES

#include "fakeLaserWithMotor.h"

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/Vec2D.h>
#include <iostream>
#include <limits>
#include <cstring>
#include <cstdlib>
#include <cmath>

//#define LASER_DEBUG
#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

YARP_LOG_COMPONENT(FAKE_LASER_MOTORS, "yarp.devices.fakeLaserWithMotor.motors")

using namespace yarp::os;
using namespace yarp::dev;

template <class T,
    std::enable_if_t<std::is_trivial<T>::value, int> = 0>
    T* allocAndCheck(int size)
{
    T* t = new T[size];
    yAssert(t != 0);
    memset(t, 0, sizeof(T) * size);
    return t;
}

template <class T,
    std::enable_if_t<!std::is_trivial<T>::value, int> = 0>
    T* allocAndCheck(int size)
{
    T* t = new T[size];
    yAssert(t != 0);
    return t;
}

template <class T>
void checkAndDestroy(T*& p) {
    if (p != 0) {
        delete[] p;
        p = 0;
    }
}


//Motor interfaces
yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncodersTimedRaw(double* encs, double* stamps)
{
    yarp::dev::yarp_ret_value ret = getEncodersRaw(encs);
    m_mutex.lock();
    for (int i = 0; i < m_njoints; i++) {
        stamps[i] = m_timestamp.getTime();
    }
    m_mutex.unlock();
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncoderTimedRaw(int j, double* encs, double* stamp)
{
    yarp::dev::yarp_ret_value ret = getEncoderRaw(j, encs);
    m_mutex.lock();
    *stamp = m_timestamp.getTime();
    m_mutex.unlock();

    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getAxes(int* ax)
{
    *ax = m_njoints;
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::positionMoveRaw(int j, double ref)
{
    int mode = 0;
    getControlModeRaw(j, &mode);
    if ((mode != VOCAB_CM_POSITION) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_POS) &&
        (mode != VOCAB_CM_IDLE))
    {
        yCError(FAKE_LASER_MOTORS) << "positionMoveRaw: skipping command because joint " << j << " is not in VOCAB_CM_POSITION mode";
    }
    _posCtrl_references[j] = ref;
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::positionMoveRaw(const double* refs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0, index = 0; j < m_njoints; j++, index++)
    {
        ret &= positionMoveRaw(j, refs[index]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::relativeMoveRaw(int j, double delta)
{
    int mode = 0;
    getControlModeRaw(j, &mode);
    if ((mode != VOCAB_CM_POSITION) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_POS) &&
        (mode != VOCAB_CM_IDLE))
    {
        yCError(FAKE_LASER_MOTORS) << "relativeMoveRaw: skipping command because joint " << j << " is not in VOCAB_CM_POSITION mode";
    }
    _posCtrl_references[j] += delta;
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::relativeMoveRaw(const double* deltas)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0, index = 0; j < m_njoints; j++, index++)
    {
        ret &= relativeMoveRaw(j, deltas[index]);
    }
    return ret;
}


yarp::dev::yarp_ret_value FakeLaserWithMotor::checkMotionDoneRaw(int j, bool* flag)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    *flag = false;
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::checkMotionDoneRaw(bool* flag)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    bool val, tot_res = true;

    for (int j = 0, index = 0; j < m_njoints; j++, index++)
    {
        ret &= checkMotionDoneRaw(j, &val);
        tot_res &= val;
    }
    *flag = tot_res;
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setRefSpeedRaw(int j, double sp)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    int index = j;
    _ref_speeds[index] = sp;
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setRefSpeedsRaw(const double* spds)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    for (int j = 0, index = 0; j < m_njoints; j++, index++)
    {
        _ref_speeds[index] = spds[index];
    }
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setRefAccelerationRaw(int j, double acc)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command

    if (acc > 1e6)
    {
        _ref_accs[j] = 1e6;
    }
    else if (acc < -1e6)
    {
        _ref_accs[j] = -1e6;
    }
    else
    {
        _ref_accs[j] = acc;
    }

    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setRefAccelerationsRaw(const double* accs)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command
    for (int j = 0, index = 0; j < m_njoints; j++, index++)
    {
        if (accs[j] > 1e6)
        {
            _ref_accs[index] = 1e6;
        }
        else if (accs[j] < -1e6)
        {
            _ref_accs[index] = -1e6;
        }
        else
        {
            _ref_accs[index] = accs[j];
        }
    }
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefSpeedRaw(int j, double* spd)
{
    *spd = _ref_speeds[j];
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefSpeedsRaw(double* spds)
{
    memcpy(spds, _ref_speeds, sizeof(double) * m_njoints);
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefAccelerationRaw(int j, double* acc)
{
    *acc = _ref_accs[j];
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefAccelerationsRaw(double* accs)
{
    memcpy(accs, _ref_accs, sizeof(double) * m_njoints);
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::stopRaw(int j)
{
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::stopRaw()
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < m_njoints; j++)
    {
        ret &= stopRaw(j);
    }
    return ret;
}
///////////// END Position Control INTERFACE  //////////////////

yarp::dev::yarp_ret_value FakeLaserWithMotor::getControlModeRaw(int j, int* v)
{
    *v = _controlModes[j];
    return yarp_ret_value_ok;
}

// IControl Mode 2
yarp::dev::yarp_ret_value FakeLaserWithMotor::getControlModesRaw(int* v)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < m_njoints; j++)
    {
        ret = ret && getControlModeRaw(j, &v[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getControlModesRaw(const int n_joint, const int* joints, int* modes)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && getControlModeRaw(joints[j], &modes[j]);
    }
    return ret;
}



yarp::dev::yarp_ret_value FakeLaserWithMotor::setControlModeRaw(const int j, const int _mode)
{
    if (_mode == VOCAB_CM_FORCE_IDLE)
    {
        _controlModes[j] = VOCAB_CM_IDLE;
    }
    else
    {
        _controlModes[j] = _mode;
    }
    return yarp_ret_value_ok;
}


yarp::dev::yarp_ret_value FakeLaserWithMotor::setControlModesRaw(const int n_joint, const int* joints, int* modes)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < n_joint; i++)
    {
        ret &= setControlModeRaw(joints[i], modes[i]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setControlModesRaw(int* modes)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < m_njoints; i++)
    {
        ret &= setControlModeRaw(i, modes[i]);
    }
    return ret;
}

bool FakeLaserWithMotor::alloc(int nj)
{
    _controlModes = allocAndCheck<int>(nj);
    _encoders = allocAndCheck<double>(nj);
    _axisName = new std::string[nj];
    _jointType = new JointTypeEnum[nj];

    // Reserve space for data stored locally. values are initialized to 0
    _posCtrl_references = allocAndCheck<double>(nj);
    _command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);

    //resizeBuffers();

    return true;
}

bool FakeLaserWithMotor::dealloc()
{
    checkAndDestroy(_axisName);
    checkAndDestroy(_jointType);
    checkAndDestroy(_controlModes);
    checkAndDestroy(_encoders);
    checkAndDestroy(_posCtrl_references);
    checkAndDestroy(_ref_speeds);
    checkAndDestroy(_command_speeds);
    checkAndDestroy(_ref_accs);
    return true;
}


yarp::dev::yarp_ret_value FakeLaserWithMotor::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED();
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setEncodersRaw(const double* vals)
{
    return NOT_YET_IMPLEMENTED();
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::resetEncoderRaw(int j)
{
    return NOT_YET_IMPLEMENTED();
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::resetEncodersRaw()
{
    return NOT_YET_IMPLEMENTED();
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncoderRaw(int j, double* value)
{
    yarp_ret_value ret = yarp_ret_value_ok;

    *value = _encoders[j];

    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncodersRaw(double* encs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < m_njoints; j++)
    {
        bool ok = getEncoderRaw(j, &encs[j]);
        ret = ret && ok;

    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncoderSpeedRaw(int j, double* sp)
{
    // To avoid returning uninitialized memory, we set the encoder speed to 0
    *sp = 0.0;
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncoderSpeedsRaw(double* spds)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < m_njoints; j++)
    {
        ret &= getEncoderSpeedRaw(j, &spds[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncoderAccelerationRaw(int j, double* acc)
{
    // To avoid returning uninitialized memory, we set the encoder acc to 0
    *acc = 0.0;

    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getEncoderAccelerationsRaw(double* accs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < m_njoints; j++)
    {
        ret &= getEncoderAccelerationRaw(j, &accs[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setRefAccelerationsRaw(const int n_joint, const int* joints, const double* accs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && setRefAccelerationRaw(joints[j], accs[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefSpeedsRaw(const int n_joint, const int* joints, double* spds)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && getRefSpeedRaw(joints[j], &spds[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefAccelerationsRaw(const int n_joint, const int* joints, double* accs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && getRefAccelerationRaw(joints[j], &accs[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::stopRaw(const int n_joint, const int* joints)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && stopRaw(joints[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::positionMoveRaw(const int n_joint, const int* joints, const double* refs)
{
    for (int j = 0; j < n_joint; j++)
    {
        yCDebug(FAKE_LASER_MOTORS, "j: %d; ref %f;\n", joints[j], refs[j]); fflush(stdout);
    }

    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && positionMoveRaw(joints[j], refs[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::relativeMoveRaw(const int n_joint, const int* joints, const double* deltas)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && relativeMoveRaw(joints[j], deltas[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::checkMotionDoneRaw(const int n_joint, const int* joints, bool* flag)
{
    bool ret = true;
    bool val = true;
    bool tot_val = true;

    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && checkMotionDoneRaw(joints[j], &val);
        tot_val &= val;
    }
    *flag = tot_val;
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::setRefSpeedsRaw(const int n_joint, const int* joints, const double* spds)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int j = 0; j < n_joint; j++)
    {
        ret = ret && setRefSpeedRaw(joints[j], spds[j]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getTargetPositionRaw(int axis, double* ref)
{
    int mode = 0;
    getControlModeRaw(axis, &mode);
    if ((mode != VOCAB_CM_POSITION) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_POS))
    {
        yCWarning(FAKE_LASER_MOTORS) << "getTargetPosition: Joint " << axis << " is not in POSITION mode, therefore the value returned by " <<
            "this call is for reference only and may not reflect the actual behaviour of the motor/firmware.";
    }
    *ref = _posCtrl_references[axis];
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getTargetPositionsRaw(double* refs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < m_njoints; i++) {
        ret &= getTargetPositionRaw(i, &refs[i]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getTargetPositionsRaw(int nj, const int* jnts, double* refs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < nj; i++)
    {
        ret &= getTargetPositionRaw(jnts[i], &refs[i]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::velocityMoveRaw(int j, double sp)
{
    int mode = 0;
    getControlModeRaw(j, &mode);
    if ((mode != VOCAB_CM_VELOCITY) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_VEL) &&
        (mode != VOCAB_CM_IDLE))
    {
        yCError(FAKE_LASER_MOTORS) << "velocityMoveRaw: skipping command because board " << " joint " << j << " is not in VOCAB_CM_VELOCITY mode";
    }
    _command_speeds[j] = sp;
    //last_velocity_command[j] = yarp::os::Time::now();
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::velocityMoveRaw(const double* sp)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < m_njoints; i++) {
        ret &= velocityMoveRaw(i, sp[i]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::velocityMoveRaw(const int n_joint, const int* joints, const double* spds)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < n_joint; i++)
    {
        ret &= velocityMoveRaw(joints[i], spds[i]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefVelocityRaw(int axis, double* ref)
{
    *ref = _command_speeds[axis];
    return yarp_ret_value_ok;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefVelocitiesRaw(double* refs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < m_njoints; i++)
    {
        ret &= getRefVelocityRaw(i, &refs[i]);
    }
    return ret;
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getRefVelocitiesRaw(int nj, const int* jnts, double* refs)
{
    yarp_ret_value ret = yarp_ret_value_ok;
    for (int i = 0; i < nj; i++)
    {
        ret &= getRefVelocityRaw(jnts[i], &refs[i]);
    }
    return ret;
}


yarp::dev::yarp_ret_value FakeLaserWithMotor::getAxisNameRaw(int axis, std::string& name)
{
    if (axis >= 0 && axis < m_njoints)
    {
        name = _axisName[axis];
        return yarp_ret_value_ok;
    }
    else
    {
        name = "ERROR";
        return false;
    }
}

yarp::dev::yarp_ret_value FakeLaserWithMotor::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis >= 0 && axis < m_njoints)
    {
        type = _jointType[axis];
        return true;
    }
    else
    {
        return false;
    }
}
