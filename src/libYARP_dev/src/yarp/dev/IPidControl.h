/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_PIDCONTROL_H
#define YARP_DEV_PIDCONTROL_H

#include <yarp/os/Vocab.h>
#include <yarp/dev/api.h>
#include <yarp/dev/GenericVocabs.h>
#include <yarp/dev/PidEnums.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/ReturnValue.h>

namespace yarp::dev {
class IPidControlRaw;
class IPidControl;
}


/**
 * @ingroup dev_iface_motor_raw
 *
 * Interface for a generic control board device implementing a PID controller.
 */
class YARP_dev_API yarp::dev::IPidControlRaw
{
public:
    /**
     * Destructor.
     */
    virtual ~IPidControlRaw() {}

    //////////////////////////////////////
    /** Set new pid value for a joint axis.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param pid new pid value
    * @return true/false on success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidRaw(const PidControlTypeEnum& pidtype, int j, const Pid &pid) = 0;

    /** Set new pid value on multiple axes.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param pids pointer to a vector of pids
    * @return true/false upon success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids) = 0;

    /** Set the controller reference for a given axis.
    * Warning this method can result in very large torques
    * and should be used carefully. If you do not understand
    * this warning you should avoid using this method.
    * Have a look at other interfaces (e.g. position control).
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param ref new reference point
    * @return true/false upon success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double ref) = 0;

    /** Set the controller reference, multiple axes.
    * Warning this method can result in very large torques
    * and should be used carefully. If you do not understand
    * this warning you should avoid using this method.
    * Have a look at other interfaces (e.g. position control).
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param refs pointer to the vector that contains the new reference points.
    * @return true/false upon success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidReferencesRaw(const PidControlTypeEnum& pidtype, const double *refs) = 0;

    /** Set the error limit for the controller on a specific joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param limit limit value
    * @return true/false on success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double limit) = 0;

    /** Get the error limit for the controller on all joints.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param limits pointer to the vector with the new limits
    * @return true/false on success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limits) = 0;

    /** Get the current error for a joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param err pointer to the storage for the return value
    * @return true/false on success failure
    */
    virtual yarp::dev::yarp_ret_value getPidErrorRaw(const PidControlTypeEnum& pidtype, int j, double *err) = 0;

    /** Get the error of all joints.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param errs pointer to the vector that will store the errors
    */
    virtual yarp::dev::yarp_ret_value getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs) = 0;

    /** Get the output of the controller (e.g. pwm value)
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param out pointer to storage for return value
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPidOutputRaw(const PidControlTypeEnum& pidtype, int j, double *out) = 0;

    /** Get the output of the controllers (e.g. pwm value)
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param outs pinter to the vector that will store the output values
    */
    virtual yarp::dev::yarp_ret_value getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs) = 0;

    /** Get current pid value for a specific joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param pid pointer to storage for the return value.
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPidRaw(const PidControlTypeEnum& pidtype, int j, Pid *pid) = 0;

    /** Get current pid value for a specific joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param pids vector that will store the values of the pids.
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPidsRaw(const PidControlTypeEnum& pidtype, Pid *pids) = 0;

    /** Get the current reference of the pid controller for a specific joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param ref pointer to storage for return value
    * @return reference value
    */
    virtual yarp::dev::yarp_ret_value getPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double *ref) = 0;

    /** Get the current reference of all pid controllers.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param refs vector that will store the output.
    */
    virtual yarp::dev::yarp_ret_value getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *refs) = 0;

    /** Get the error limit for the controller on a specific joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param limit pointer to storage
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double *limit) = 0;

    /** Get the error limit for all controllers
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param limits pointer to the array that will store the output
    * @return success or failure
    */
    virtual yarp::dev::yarp_ret_value getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *limits) = 0;

    /** Reset the controller of a given joint, usually sets the current status
    * of the joint as the reference value for the PID, and resets the integrator.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value resetPidRaw(const PidControlTypeEnum& pidtype, int j) = 0;

    /** Disable the pid computation for a joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value disablePidRaw(const PidControlTypeEnum& pidtype, int j) = 0;

    /** Enable the pid computation for a joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value enablePidRaw(const PidControlTypeEnum& pidtype, int j) = 0;

    /** Set an offset value on the ourput of pid controller
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param v the offset to be added to the output of the pid controller
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value setPidOffsetRaw(const PidControlTypeEnum& pidtype, int j, double v) = 0;

   /** Get the current status (enabled/disabled) of the pid controller
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param enabled the current status of the pid controller.
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool* enabled) = 0;
};

/**
 * @ingroup dev_iface_motor
 *
 * Interface for a generic control board device implementing a PID controller,
 * with scaled arguments.
 */
class YARP_dev_API yarp::dev::IPidControl
{
public:
    /**
     * Destructor.
     */
    virtual ~IPidControl() {}

    /////////////////////////////////////////////////////////////////
    /** Set new pid value for a joint axis.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param pid new pid value
    * @return true/false on success/failure
    */
    virtual yarp::dev::yarp_ret_value setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid) = 0;

    /** Set new pid value on multiple axes.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param pids pointer to a vector of pids
    * @return true/false upon success/failure
    */
    virtual yarp::dev::yarp_ret_value setPids(const PidControlTypeEnum& pidtype, const Pid *pids) = 0;

    /** Set the controller reference for a given axis.
    * Warning this method can result in very large torques
    * and should be used carefully. If you do not understand
    * this warning you should avoid using this method.
    * Have a look at other interfaces (e.g. position control).
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param ref new reference point
    * @return true/false upon success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) = 0;

    /** Set the controller reference, multiple axes.
    * Warning this method can result in very large torques
    * and should be used carefully. If you do not understand
    * this warning you should avoid using this method.
    * Have a look at other interfaces (e.g. position control).
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param refs pointer to the vector that contains the new reference points.
    * @return true/false upon success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidReferences(const PidControlTypeEnum& pidtype, const double *refs) = 0;

    /** Set the error limit for the controller on a specifi joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param limit limit value
    * @return true/false on success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) = 0;

    /** Get the error limit for the controller on all joints.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param limits pointer to the vector with the new limits
    * @return true/false on success/failure
    */
    virtual yarp::dev::yarp_ret_value setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits) = 0;

    /** Get the current error for a joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param err pointer to the storage for the return value
    * @return true/false on success failure
    */
    virtual yarp::dev::yarp_ret_value getPidError(const PidControlTypeEnum& pidtype, int j, double *err) = 0;

    /** Get the error of all joints.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param errs pointer to the vector that will store the errors
    */
    virtual yarp::dev::yarp_ret_value getPidErrors(const PidControlTypeEnum& pidtype, double *errs) = 0;

    /** Get the output of the controller (e.g. pwm value)
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param out pointer to storage for return value
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out) = 0;

    /** Get the output of the controllers (e.g. pwm value)
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param outs pinter to the vector that will store the output values
    */
    virtual yarp::dev::yarp_ret_value getPidOutputs(const PidControlTypeEnum& pidtype, double *outs) = 0;

    /** Get current pid value for a specific joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param pid pointer to storage for the return value.
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid) = 0;

    /** Get current pid value for a specific joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param pids vector that will store the values of the pids.
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPids(const PidControlTypeEnum& pidtype, Pid *pids) = 0;

    /** Get the current reference of the pid controller for a specific joint.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param ref pointer to storage for return value
    * @return reference value
    */
    virtual yarp::dev::yarp_ret_value getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref) = 0;

    /** Get the current reference of all pid controllers.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param refs vector that will store the output.
    */
    virtual yarp::dev::yarp_ret_value getPidReferences(const PidControlTypeEnum& pidtype, double *refs) = 0;

    /** Get the error limit for the controller on a specific joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param limit pointer to storage
    * @return success/failure
    */
    virtual yarp::dev::yarp_ret_value getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit) = 0;

    /** Get the error limit for all controllers
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param limits pointer to the array that will store the output
    * @return success or failure
    */
    virtual yarp::dev::yarp_ret_value getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits) = 0;

    /** Reset the controller of a given joint, usually sets the current status of the joint
    * as the reference value for the PID, and resets the integrator.
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value resetPid(const PidControlTypeEnum& pidtype, int j) = 0;

    /** Disable the pid computation for a joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value disablePid(const PidControlTypeEnum& pidtype, int j) = 0;

    /** Enable the pid computation for a joint
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value enablePid(const PidControlTypeEnum& pidtype, int j) = 0;

    /** Set offset value for a given controller
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param v the offset to be added to the output of the pid controller
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) = 0;

    /** Get the current status (enabled/disabled) of the pid
    * @param pidtype the id of the pid that will be affected by the command (e.g. position, velocity etc)
    * @param j joint number
    * @param enabled the current status of the pid controller.
    * @return true on success, false on failure.
    */
    virtual yarp::dev::yarp_ret_value isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) = 0;
};

// interface IPositionControl gets
constexpr yarp::conf::vocab32_t VOCAB_PID     = yarp::os::createVocab32('p','i','d');
constexpr yarp::conf::vocab32_t VOCAB_PIDS    = yarp::os::createVocab32('p','i','d','s');


#endif // YARP_DEV_CONTROLBOARDINTERFACES_H
