/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IPOSITIONCONTROL_H
#define YARP_DEV_IPOSITIONCONTROL_H

#include <yarp/dev/api.h>
#include <yarp/os/Vocab.h>
#include <yarp/dev/ReturnValue.h>

namespace yarp::dev {
class IPositionControlRaw;
class IPositionControl;
}

/**
 * @ingroup dev_iface_motor_raw
 *
 * Interface for a generic control board device implementing position control in encoder
 * coordinates.
 */
class YARP_dev_API yarp::dev::IPositionControlRaw
{
public:
    /**
     * Destructor.
     */
    virtual ~IPositionControlRaw() {}

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax storage to return param
     * @return true/false.
     */
    virtual yarp::dev::yarp_ret_value getAxes(int *ax) = 0;

    /** Set new reference point for a single axis.
     * @param j joint number
     * @param ref specifies the new ref point
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value positionMoveRaw(int j, double ref)=0;

    /** Set new reference point for all axes.
     * @param refs array, new reference points.
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value positionMoveRaw(const double *refs)=0;

    /** Set relative position. The command is relative to the
     * current position of the axis.
     * @param j joint axis number
     * @param delta relative command
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value relativeMoveRaw(int j, double delta)=0;

    /** Set relative position, all joints.
     * @param deltas pointer to the relative commands
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value relativeMoveRaw(const double *deltas)=0;

    /** Check if the current trajectory is terminated. Non blocking.
     * @param j is the axis number
     * @param flag is a pointer to return value
     * @return true/false on network communication (value you actually want
        is stored in *flag)
     */
    virtual yarp::dev::yarp_ret_value checkMotionDoneRaw(int j, bool *flag)=0;

    /** Check if the current trajectory is terminated. Non blocking.
     * @param flag is a pointer to return value ("and" of all joints)
     * @return true/false on network communication (value you actually want
        is stored in *flag)
     */
    virtual yarp::dev::yarp_ret_value checkMotionDoneRaw(bool *flag)=0;

    /** Set reference speed for a joint, this is the speed used during the
     * interpolation of the trajectory.
     * @param j joint number
     * @param sp speed value
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefSpeedRaw(int j, double sp)=0;

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array of speed values.
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefSpeedsRaw(const double *spds)=0;

    /** Set reference acceleration for a joint. This value is used during the
     * trajectory generation.
     * @param j joint number
     * @param acc acceleration value
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefAccelerationRaw(int j, double acc)=0;

    /** Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param accs pointer to the array of acceleration values
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefAccelerationsRaw(const double *accs)=0;

    /** Get reference speed for a joint. Returns the speed used to
     * generate the trajectory profile.
     * @param j joint number
     * @param ref pointer to storage for the return value
     * @return true/false on success or failure
     */
    virtual yarp::dev::yarp_ret_value getRefSpeedRaw(int j, double *ref)=0;

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array that will store the speed values.
     */
    virtual yarp::dev::yarp_ret_value getRefSpeedsRaw(double *spds)=0;

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param j joint number
     * @param acc pointer to storage for the return value
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getRefAccelerationRaw(int j, double *acc)=0;

    /** Get reference acceleration of all joints. These are the values used during the
     * interpolation of the trajectory.
     * @param accs pointer to the array that will store the acceleration values.
     * @return true/false on success or failure
     */
    virtual yarp::dev::yarp_ret_value getRefAccelerationsRaw(double *accs)=0;

    /** Stop motion, single joint
     * @param j joint number
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value stopRaw(int j)=0;

    /** Stop motion, multiple joints
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value stopRaw()=0;

    /** Set new reference point for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param refs   pointer to the array specifies the new reference points
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value positionMoveRaw(const int n_joint, const int *joints, const double *refs)=0;

    /** Set relative position for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param deltas pointer to the array of relative commands
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)=0;

    /** Check if the current trajectory is terminated. Non blocking.
     * @param joints pointer to the array of joint numbers
     * @param flag true if the trajectory is terminated, false otherwise
     *        (a single value which is the 'and' of all joints')
     * @return true/false if network communication went well.
     */
    virtual yarp::dev::yarp_ret_value checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)=0;

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array with speed values.
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)=0;

    /** Set reference acceleration on all joints. This is the value that is
     * used during the generation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array with acceleration values
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)=0;

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array that will store the speed values.
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)=0;

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array that will store the acceleration values
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)=0;

    /** Stop motion for subset of joints
     * @param joints pointer to the array of joint numbers
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value stopRaw(const int n_joint, const int *joints)=0;

    /** Get the last position reference for the specified axis.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getTargetPositionRaw(const int joint, double *ref) { return yarp::dev::NOT_YET_IMPLEMENTED(); }

    /** Get the last position reference for all axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getTargetPositionsRaw(double *refs) { return yarp::dev::NOT_YET_IMPLEMENTED(); }

    /** Get the last position reference for the specified group of axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) { return yarp::dev::NOT_YET_IMPLEMENTED(); }
};

/**
 * @ingroup dev_iface_motor
 *
 * Interface for a generic control board device implementing position control.
 */
class YARP_dev_API yarp::dev::IPositionControl
{
public:
    /**
     * Destructor.
     */
    virtual ~IPositionControl() {}

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax pointer to storage
     * @return true/false.
     */
    virtual yarp::dev::yarp_ret_value getAxes(int *ax) = 0;

    /** Set new reference point for a single axis.
     * @param j joint number
     * @param ref specifies the new ref point
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value positionMove(int j, double ref)=0;

    /** Set new reference point for all axes.
     * @param refs array, new reference points.
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value positionMove(const double *refs)=0;

    /** Set relative position. The command is relative to the
     * current position of the axis.
     * @param j joint axis number
     * @param delta relative command
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value relativeMove(int j, double delta)=0;

    /** Set relative position, all joints.
     * @param deltas pointer to the relative commands
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value relativeMove(const double *deltas)=0;

    /** Check if the current trajectory is terminated. Non blocking.
     * @param j is the axis number
     * @param flag is a pointer to return value
     * @return true/false on network communication (value you actually want
        is stored in *flag)
     */
    virtual yarp::dev::yarp_ret_value checkMotionDone(int j, bool *flag)=0;

    /** Check if the current trajectory is terminated. Non blocking.
     * @param flag is a pointer to return value ("and" of all joints)
     * @return true/false on network communication (value you actually want
        is stored in *flag)
     */
    virtual yarp::dev::yarp_ret_value checkMotionDone(bool *flag)=0;

    /** Set reference speed for a joint, this is the speed used during the
     * interpolation of the trajectory.
     * @param j joint number
     * @param sp speed value
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefSpeed(int j, double sp)=0;

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array of speed values.
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefSpeeds(const double *spds)=0;

    /** Set reference acceleration for a joint. This value is used during the
     * trajectory generation.
     * @param j joint number
     * @param acc acceleration value
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefAcceleration(int j, double acc)=0;

    /** Set reference acceleration on all joints. This is the value that is
     * used during the generation of the trajectory.
     * @param accs pointer to the array of acceleration values
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefAccelerations(const double *accs)=0;

    /** Get reference speed for a joint. Returns the speed used to
     * generate the trajectory profile.
     * @param j joint number
     * @param ref pointer to storage for the return value
     * @return true/false on success or failure
     */
    virtual yarp::dev::yarp_ret_value getRefSpeed(int j, double *ref)=0;

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array that will store the speed values.
     */
    virtual yarp::dev::yarp_ret_value getRefSpeeds(double *spds)=0;

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param j joint number
     * @param acc pointer to storage for the return value
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getRefAcceleration(int j, double *acc)=0;

    /** Get reference acceleration of all joints. These are the values used during the
     * interpolation of the trajectory.
     * @param accs pointer to the array that will store the acceleration values.
     * @return true/false on success or failure
     */
    virtual yarp::dev::yarp_ret_value getRefAccelerations(double *accs)=0;

    /** Stop motion, single joint
     * @param j joint number
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value stop(int j)=0;

    /** Stop motion, multiple joints
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value stop()=0;

    /** Set new reference point for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param refs   pointer to the array specifying the new reference points
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value positionMove(const int n_joint, const int *joints, const double *refs)=0;

    /** Set relative position for a subset of joints.
     * @param joints pointer to the array of joint numbers
     * @param deltas pointer to the array of relative commands
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value relativeMove(const int n_joint, const int *joints, const double *deltas)=0;

    /** Check if the current trajectory is terminated. Non blocking.
     * @param joints pointer to the array of joint numbers
     * @param flag  pointer to return value (logical "and" of all set of joints)
     * @return true/false if network communication went well.
     */
    virtual yarp::dev::yarp_ret_value checkMotionDone(const int n_joint, const int *joints, bool *flag)=0;

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array with speed values.
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefSpeeds(const int n_joint, const int *joints, const double *spds)=0;

    /** Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array with acceleration values
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value setRefAccelerations(const int n_joint, const int *joints, const double *accs)=0;

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param joints pointer to the array of joint numbers
     * @param spds   pointer to the array that will store the speed values.
     * @return true/false upon success/failure
     */
    virtual yarp::dev::yarp_ret_value getRefSpeeds(const int n_joint, const int *joints, double *spds)=0;

    /** Get reference acceleration for a joint. Returns the acceleration used to
     * generate the trajectory profile.
     * @param joints pointer to the array of joint numbers
     * @param accs   pointer to the array that will store the acceleration values
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getRefAccelerations(const int n_joint, const int *joints, double *accs)=0;

    /** Stop motion for subset of joints
     * @param joints pointer to the array of joint numbers
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value stop(const int n_joint, const int *joints)=0;

        /** Get the last position reference for the specified axis.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getTargetPosition(const int joint, double *ref) { return yarp::dev::NOT_YET_IMPLEMENTED(); }

    /** Get the last position reference for all axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getTargetPositions(double *refs) { return yarp::dev::NOT_YET_IMPLEMENTED(); }

    /** Get the last position reference for the specified group of axes.
     *  This is the dual of PositionMove and shall return only values sent using
     *  IPositionControl interface.
     *  If other interfaces like IPositionDirect are implemented by the device, this call
     *  must ignore their values, i.e. this call must never return a reference sent using
     *  IPositionDirect::SetPosition
     * @param ref last reference sent using PositionMove functions
     * @return true/false on success/failure
     */
    virtual yarp::dev::yarp_ret_value getTargetPositions(const int n_joint, const int *joints, double *refs) { return yarp::dev::NOT_YET_IMPLEMENTED(); }
};

constexpr yarp::conf::vocab32_t VOCAB_POSITION_MOVE_GROUP    = yarp::os::createVocab32('p','o','s','g');
constexpr yarp::conf::vocab32_t VOCAB_RELATIVE_MOVE_GROUP    = yarp::os::createVocab32('r','e','l','g');
constexpr yarp::conf::vocab32_t VOCAB_MOTION_DONE_GROUP      = yarp::os::createVocab32('d','o','n','g');
constexpr yarp::conf::vocab32_t VOCAB_REF_SPEED_GROUP        = yarp::os::createVocab32('v','e','l','g');
constexpr yarp::conf::vocab32_t VOCAB_REF_ACCELERATION_GROUP = yarp::os::createVocab32('a','c','c','g');
constexpr yarp::conf::vocab32_t VOCAB_STOP_GROUP             = yarp::os::createVocab32('s','t','o','g');

#endif // YARP_DEV_IPOSITIONCONTROL_H
