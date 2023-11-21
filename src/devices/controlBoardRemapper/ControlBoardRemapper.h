/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H
#define YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H

#include <yarp/os/Network.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/ReturnValue.h>

#include <string>
#include <vector>

#include "ControlBoardRemapperHelpers.h"

#ifdef MSVC
    #pragma warning(disable:4355)
#endif

/**
 * @ingroup dev_impl_remappers
 *
 * @brief `controlboardremapper` : device that takes a list of axes from multiple controlboards and expose them as a single controlboard.
 *
 * | YARP device name |
 * |:-----------------:|
 * | `controlboardremapper` |
 *
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | axesNames     |      -         | vector of strings  | -      |   -           | Yes     | Ordered list of the axes that are part of the remapped device. |  |
 *
 * The axes are then mapped to the wrapped controlboard in the attachAll method, using the
 * values returned by the getAxisName method of the controlboard. If different axes
 * in two attached controlboard have the same name, the behaviour of this device is undefined.
 *
 * Configuration file using .ini format.
 *
 * \code{.unparsed}
 *  device controlboardremapper
 *  axesNames (joint1 joint2 joint3)
 *
 * ...
 * \endcode
 *
 * For compatibility with the controlBoard_nws_yarp, the
 * networks keyword can also be used to select the desired joints.
 * For more information on the syntax of the networks, see the
 * controlBoard_nws_yarp class.
 *
 * \code{.unparsed}
 *  networks (net_larm net_lhand)
 *  joints 16
 *  net_larm    0 3  0 3
 *  net_lhand   4 6  0 2
 * \endcode
 *
 */

class ControlBoardRemapper :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IPidControl,
        public yarp::dev::IPositionControl,
        public yarp::dev::IPositionDirect,
        public yarp::dev::IVelocityControl,
        public yarp::dev::IPWMControl,
        public yarp::dev::ICurrentControl,
        public yarp::dev::IEncodersTimed,
        public yarp::dev::IMotor,
        public yarp::dev::IMotorEncoders,
        public yarp::dev::IAmplifierControl,
        public yarp::dev::IControlLimits,
        public yarp::dev::IRemoteCalibrator,
        public yarp::dev::IControlCalibration,
        public yarp::dev::ITorqueControl,
        public yarp::dev::IImpedanceControl,
        public yarp::dev::IControlMode,
        public yarp::dev::IMultipleWrapper,
        public yarp::dev::IAxisInfo,
        public yarp::dev::IPreciselyTimed,
        public yarp::dev::IInteractionMode,
        public yarp::dev::IRemoteVariables,
        public yarp::dev::IJointFault {
private:
    std::vector<std::string> axesNames;
    RemappedControlBoards remappedControlBoards;

    /** number of axes controlled by this controlboard */
    int controlledJoints{0};

    /** Verbosity of the class */
    bool _verb{false};

    // to open ports and print more detailed debug messages
    std::string partName;

    // Buffer data used to simplify implementation of multi joint methods
    ControlBoardRemapperBuffers buffers;

    // Buffer data used for full controlboard methods
    ControlBoardSubControlBoardAxesDecomposition allJointsBuffers;

    // Buffer data for multiple arbitrary joint methods
    ControlBoardArbitraryAxesDecomposition selectedJointsBuffers;

    /**
     * Set the number of controlled axes, resizing appropriately
     * all the necessary buffers.
     */
    void setNrOfControlledAxes(const size_t nrOfControlledAxes);

    /**
     * If the class was configured using the networks format,
     * call this method to update the vector containing the
     * axesName .
     */
    bool updateAxesName();

    /**
     * Configure buffers used by the device
     */
    void configureBuffers();

    // Parse device options
    bool parseOptions(yarp::os::Property &prop);

    bool usingAxesNamesForAttachAll{false};
    bool usingNetworksForAttachAll{false};

    /***
     * Parse device options if networks option is passed
     *
     * This will fill the axesNames and controlledJoints attributes, while it
     * leaves empty the remappedDevices attribute that will be then
     * filled only at the attachAll method.
     */
    bool parseAxesNames(const yarp::os::Property &prop);

    /***
     * Parse device options if networks option is passed
     *
     * This will fill the remappedDevices and controlledJoints attributes, while it
     * leaves empty the axesNames attribute that will be then
     * filled only at the attachAll method.
     */
    bool parseNetworks(const yarp::os::Property &prop);

    /**
     * attachAll if the networks option is used for configuration.
     */
    bool attachAllUsingNetworks(const yarp::dev::PolyDriverList &l);

    /**
     * attachAll if the axesNames option is used for configuration.
     */
    bool attachAllUsingAxesNames(const yarp::dev::PolyDriverList &l);

    /**
     * Helper for setting the same control mode in all the axes
     * of the controlboard.
     */
    bool setControlModeAllAxes(const int cm);


public:
    ControlBoardRemapper() = default;
    ControlBoardRemapper(const ControlBoardRemapper&) = delete;
    ControlBoardRemapper(ControlBoardRemapper&&) = delete;
    ControlBoardRemapper& operator=(const ControlBoardRemapper&) = delete;
    ControlBoardRemapper& operator=(ControlBoardRemapper&&) = delete;
    ~ControlBoardRemapper() override = default;

    /**
    * Return the value of the verbose flag.
    * @return the verbose flag.
    */
    bool verbose() const { return _verb; }

    /**
    * Close the device driver by deallocating all resources and closing ports.
    * @return true if successful or false otherwise.
    */
    bool close() override;


    /**
    * Open the device driver.
    * @param prop is a Searchable object which contains the parameters.
    * Allowed parameters are described in the class documentation.
    */
    bool open(yarp::os::Searchable &prop) override;

    bool detachAll() override;

    bool attachAll(const yarp::dev::PolyDriverList &l) override;

    /* IPidControl */
    yarp::dev::yarp_ret_value setPid(const yarp::dev::PidControlTypeEnum& pidtype,int j, const yarp::dev::Pid &p) override;
    yarp::dev::yarp_ret_value setPids(const yarp::dev::PidControlTypeEnum& pidtype,const yarp::dev::Pid *ps) override;
    yarp::dev::yarp_ret_value setPidReference(const yarp::dev::PidControlTypeEnum& pidtype,int j, double ref) override;
    yarp::dev::yarp_ret_value setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype,const double *refs) override;
    yarp::dev::yarp_ret_value setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype,int j, double limit) override;
    yarp::dev::yarp_ret_value setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype,const double *limits) override;
    yarp::dev::yarp_ret_value getPidError(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *err) override;
    yarp::dev::yarp_ret_value getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype,double *errs) override;
    yarp::dev::yarp_ret_value getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *out) override;
    yarp::dev::yarp_ret_value getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype,double *outs) override;
    yarp::dev::yarp_ret_value setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype,int j, double v) override;
    yarp::dev::yarp_ret_value getPid(const yarp::dev::PidControlTypeEnum& pidtype,int j, yarp::dev::Pid *p) override;
    yarp::dev::yarp_ret_value getPids(const yarp::dev::PidControlTypeEnum& pidtype,yarp::dev::Pid *pids) override;
    yarp::dev::yarp_ret_value getPidReference(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *ref) override;
    yarp::dev::yarp_ret_value getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype,double *refs) override;
    yarp::dev::yarp_ret_value getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *limit) override;
    yarp::dev::yarp_ret_value getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype,double *limits) override;
    yarp::dev::yarp_ret_value resetPid(const yarp::dev::PidControlTypeEnum& pidtype,int j) override;
    yarp::dev::yarp_ret_value disablePid(const yarp::dev::PidControlTypeEnum& pidtype,int j) override;
    yarp::dev::yarp_ret_value enablePid(const yarp::dev::PidControlTypeEnum& pidtype,int j) override;
    yarp::dev::yarp_ret_value isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled) override;

    /* IPositionControl */
    yarp::dev::yarp_ret_value getAxes(int *ax) override;
    yarp::dev::yarp_ret_value positionMove(int j, double ref) override;
    yarp::dev::yarp_ret_value positionMove(const double *refs) override;
    yarp::dev::yarp_ret_value positionMove(const int n_joints, const int *joints, const double *refs) override;
    yarp::dev::yarp_ret_value getTargetPosition(const int joint, double *ref) override;
    yarp::dev::yarp_ret_value getTargetPositions(double *refs) override;
    yarp::dev::yarp_ret_value getTargetPositions(const int n_joint, const int *joints, double *refs) override;
    yarp::dev::yarp_ret_value relativeMove(int j, double delta) override;
    yarp::dev::yarp_ret_value relativeMove(const double *deltas) override;
    yarp::dev::yarp_ret_value relativeMove(const int n_joints, const int *joints, const double *deltas) override;
    yarp::dev::yarp_ret_value checkMotionDone(int j, bool *flag) override;
    yarp::dev::yarp_ret_value checkMotionDone(bool *flag) override;
    yarp::dev::yarp_ret_value checkMotionDone(const int n_joints, const int *joints, bool *flags) override;
    yarp::dev::yarp_ret_value setRefSpeed(int j, double sp) override;
    yarp::dev::yarp_ret_value setRefSpeeds(const double *spds) override;
    yarp::dev::yarp_ret_value setRefSpeeds(const int n_joints, const int *joints, const double *spds) override;
    yarp::dev::yarp_ret_value setRefAcceleration(int j, double acc) override;
    yarp::dev::yarp_ret_value setRefAccelerations(const double *accs) override;
    yarp::dev::yarp_ret_value setRefAccelerations(const int n_joints, const int *joints, const double *accs) override;
    yarp::dev::yarp_ret_value getRefSpeed(int j, double *ref) override;
    yarp::dev::yarp_ret_value getRefSpeeds(double *spds) override;
    yarp::dev::yarp_ret_value getRefSpeeds(const int n_joints, const int *joints, double *spds) override;
    yarp::dev::yarp_ret_value getRefAcceleration(int j, double *acc) override;
    yarp::dev::yarp_ret_value getRefAccelerations(double *accs) override;
    yarp::dev::yarp_ret_value getRefAccelerations(const int n_joints, const int *joints, double *accs) override;
    yarp::dev::yarp_ret_value stop(int j) override;
    yarp::dev::yarp_ret_value stop() override;
    yarp::dev::yarp_ret_value stop(const int n_joints, const int *joints) override;

    /* IJointFault */

    yarp::dev::yarp_ret_value getLastJointFault(int j, int& fault, std::string& message) override;

    /* IVelocityControl */
    yarp::dev::yarp_ret_value velocityMove(int j, double v) override;
    yarp::dev::yarp_ret_value velocityMove(const double *v) override;

    /* IEncoders */
    yarp::dev::yarp_ret_value resetEncoder(int j) override;
    yarp::dev::yarp_ret_value resetEncoders() override;
    yarp::dev::yarp_ret_value setEncoder(int j, double val) override;
    yarp::dev::yarp_ret_value setEncoders(const double *vals) override;
    yarp::dev::yarp_ret_value getEncoder(int j, double *v) override;
    yarp::dev::yarp_ret_value getEncoders(double *encs) override;
    yarp::dev::yarp_ret_value getEncodersTimed(double *encs, double *t) override;
    yarp::dev::yarp_ret_value getEncoderTimed(int j, double *v, double *t) override;
    yarp::dev::yarp_ret_value getEncoderSpeed(int j, double *sp) override;
    yarp::dev::yarp_ret_value getEncoderSpeeds(double *spds) override;
    yarp::dev::yarp_ret_value getEncoderAcceleration(int j, double *acc) override;
    yarp::dev::yarp_ret_value getEncoderAccelerations(double *accs) override;

    /* IMotorEncoders */
    yarp::dev::yarp_ret_value getNumberOfMotorEncoders(int *num) override;
    yarp::dev::yarp_ret_value resetMotorEncoder(int m) override;
    yarp::dev::yarp_ret_value resetMotorEncoders() override;
    yarp::dev::yarp_ret_value setMotorEncoderCountsPerRevolution(int m, const double cpr) override;
    yarp::dev::yarp_ret_value getMotorEncoderCountsPerRevolution(int m, double *cpr) override;
    yarp::dev::yarp_ret_value setMotorEncoder(int m, const double val) override;
    yarp::dev::yarp_ret_value setMotorEncoders(const double *vals) override;
    yarp::dev::yarp_ret_value getMotorEncoder(int m, double *v) override;
    yarp::dev::yarp_ret_value getMotorEncoders(double *encs) override;
    yarp::dev::yarp_ret_value getMotorEncodersTimed(double *encs, double *t) override;
    yarp::dev::yarp_ret_value getMotorEncoderTimed(int m, double *v, double *t) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeed(int m, double *sp) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeeds(double *spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAcceleration(int m, double *acc) override;
    yarp::dev::yarp_ret_value getMotorEncoderAccelerations(double *accs) override;

    /* IAmplifierControl */
    yarp::dev::yarp_ret_value enableAmp(int j) override;
    yarp::dev::yarp_ret_value disableAmp(int j) override;
    yarp::dev::yarp_ret_value getAmpStatus(int *st) override;
    yarp::dev::yarp_ret_value getAmpStatus(int j, int *v) override;
    yarp::dev::yarp_ret_value setMaxCurrent(int j, double v) override;
    yarp::dev::yarp_ret_value getMaxCurrent(int j, double *v) override;
    yarp::dev::yarp_ret_value getNominalCurrent(int m, double *val) override;
    yarp::dev::yarp_ret_value setNominalCurrent(int m, const double val) override;
    yarp::dev::yarp_ret_value getPeakCurrent(int m, double *val) override;
    yarp::dev::yarp_ret_value setPeakCurrent(int m, const double val) override;
    yarp::dev::yarp_ret_value getPWM(int m, double *val) override;
    yarp::dev::yarp_ret_value getPWMLimit(int m, double *val) override;
    yarp::dev::yarp_ret_value setPWMLimit(int m, const double val) override;
    yarp::dev::yarp_ret_value getPowerSupplyVoltage(int m, double *val) override;

    /* IControlLimits */
    yarp::dev::yarp_ret_value setLimits(int j, double min, double max) override;
    yarp::dev::yarp_ret_value getLimits(int j, double *min, double *max) override;
    yarp::dev::yarp_ret_value setVelLimits(int j, double min, double max) override;
    yarp::dev::yarp_ret_value getVelLimits(int j, double *min, double *max) override;

    /* IRemoteVariables */
    yarp::dev::yarp_ret_value getRemoteVariable(std::string key, yarp::os::Bottle &val) override;
    yarp::dev::yarp_ret_value setRemoteVariable(std::string key, const yarp::os::Bottle &val) override;
    yarp::dev::yarp_ret_value getRemoteVariablesList(yarp::os::Bottle *listOfKeys) override;

    /* IRemoteCalibrator */
    yarp::dev::yarp_ret_value isCalibratorDevicePresent(bool *isCalib) override;
    yarp::dev::IRemoteCalibrator *getCalibratorDevice() override;
    yarp::dev::yarp_ret_value calibrateSingleJoint(int j) override;
    yarp::dev::yarp_ret_value calibrateWholePart() override;
    yarp::dev::yarp_ret_value homingSingleJoint(int j) override;
    yarp::dev::yarp_ret_value homingWholePart() override;
    yarp::dev::yarp_ret_value parkSingleJoint(int j, bool _wait = true) override;
    yarp::dev::yarp_ret_value parkWholePart() override;
    yarp::dev::yarp_ret_value quitCalibrate() override;
    yarp::dev::yarp_ret_value quitPark() override;

    /* IControlCalibration */
    yarp::dev::yarp_ret_value calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3) override;
    yarp::dev::yarp_ret_value setCalibrationParameters(int j, const yarp::dev::CalibrationParameters &params) override;
    yarp::dev::yarp_ret_value calibrationDone(int j) override;
    yarp::dev::yarp_ret_value abortPark() override;
    yarp::dev::yarp_ret_value abortCalibration() override;

    /* IMotor */
    yarp::dev::yarp_ret_value getNumberOfMotors(int *num) override;
    yarp::dev::yarp_ret_value getTemperature(int m, double *val) override;
    yarp::dev::yarp_ret_value getTemperatures(double *vals) override;
    yarp::dev::yarp_ret_value getTemperatureLimit(int m, double *val) override;
    yarp::dev::yarp_ret_value setTemperatureLimit(int m, const double val) override;
    yarp::dev::yarp_ret_value getGearboxRatio(int m, double *val) override;
    yarp::dev::yarp_ret_value setGearboxRatio(int m, const double val) override;

    /* IAxisInfo */
    yarp::dev::yarp_ret_value getAxisName(int j, std::string &name) override;
    yarp::dev::yarp_ret_value getJointType(int j, yarp::dev::JointTypeEnum &type) override;
    yarp::dev::yarp_ret_value getRefTorques(double *refs) override;
    yarp::dev::yarp_ret_value getRefTorque(int j, double *t) override;
    yarp::dev::yarp_ret_value setRefTorques(const double *t) override;
    yarp::dev::yarp_ret_value setRefTorque(int j, double t) override;
    yarp::dev::yarp_ret_value setRefTorques(const int n_joint, const int *joints, const double *t) override;
    yarp::dev::yarp_ret_value getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters *params) override;
    yarp::dev::yarp_ret_value setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;
    yarp::dev::yarp_ret_value setImpedance(int j, double stiff, double damp) override;
    yarp::dev::yarp_ret_value setImpedanceOffset(int j, double offset) override;
    yarp::dev::yarp_ret_value getTorque(int j, double *t) override;
    yarp::dev::yarp_ret_value getTorques(double *t) override;
    yarp::dev::yarp_ret_value getTorqueRange(int j, double *min, double *max) override;
    yarp::dev::yarp_ret_value getTorqueRanges(double *min, double *max) override;
    yarp::dev::yarp_ret_value getImpedance(int j, double *stiff, double *damp) override;
    yarp::dev::yarp_ret_value getImpedanceOffset(int j, double *offset) override;
    yarp::dev::yarp_ret_value getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;
    yarp::dev::yarp_ret_value getControlMode(int j, int *mode) override;
    yarp::dev::yarp_ret_value getControlModes(int *modes) override;

    // IControlMode interface
    yarp::dev::yarp_ret_value getControlModes(const int n_joint, const int *joints, int *modes) override;
    yarp::dev::yarp_ret_value setControlMode(const int j, const int mode) override;
    yarp::dev::yarp_ret_value setControlModes(const int n_joints, const int *joints, int *modes) override;
    yarp::dev::yarp_ret_value setControlModes(int *modes) override;
    yarp::dev::yarp_ret_value setPosition(int j, double ref) override;
    yarp::dev::yarp_ret_value setPositions(const int n_joints, const int *joints, const double *dpos) override;
    yarp::dev::yarp_ret_value setPositions(const double *refs) override;
    yarp::dev::yarp_ret_value getRefPosition(const int joint, double *ref) override;
    yarp::dev::yarp_ret_value getRefPositions(double *refs) override;
    yarp::dev::yarp_ret_value getRefPositions(const int n_joint, const int *joints, double *refs) override;
    yarp::os::Stamp getLastInputStamp() override;

    // IVelocityControl interface
    yarp::dev::yarp_ret_value velocityMove(const int n_joints, const int *joints, const double *spds) override;
    yarp::dev::yarp_ret_value getRefVelocity(const int joint, double *vel) override;
    yarp::dev::yarp_ret_value getRefVelocities(double *vels) override;
    yarp::dev::yarp_ret_value getRefVelocities(const int n_joint, const int *joints, double *vels) override;
    yarp::dev::yarp_ret_value getInteractionMode(int j, yarp::dev::InteractionModeEnum *mode) override;
    yarp::dev::yarp_ret_value getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum *modes) override;
    yarp::dev::yarp_ret_value getInteractionModes(yarp::dev::InteractionModeEnum *modes) override;
    yarp::dev::yarp_ret_value setInteractionMode(int j, yarp::dev::InteractionModeEnum mode) override;
    yarp::dev::yarp_ret_value setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum *modes) override;
    yarp::dev::yarp_ret_value setInteractionModes(yarp::dev::InteractionModeEnum *modes) override;

    // IPWMControl
    yarp::dev::yarp_ret_value setRefDutyCycle(int m, double ref) override;
    yarp::dev::yarp_ret_value setRefDutyCycles(const double *refs) override;
    yarp::dev::yarp_ret_value getRefDutyCycle(int m, double *ref) override;
    yarp::dev::yarp_ret_value getRefDutyCycles(double *refs) override;
    yarp::dev::yarp_ret_value getDutyCycle(int m, double *val) override;
    yarp::dev::yarp_ret_value getDutyCycles(double *vals) override;

    // ICurrentControl
    yarp::dev::yarp_ret_value getCurrent(int m, double *curr) override;
    yarp::dev::yarp_ret_value getCurrents(double *currs) override;
    yarp::dev::yarp_ret_value getCurrentRange(int m, double *min, double *max) override;
    yarp::dev::yarp_ret_value getCurrentRanges(double *min, double *max) override;
    yarp::dev::yarp_ret_value setRefCurrents(const double *currs) override;
    yarp::dev::yarp_ret_value setRefCurrent(int m, double curr) override;
    yarp::dev::yarp_ret_value setRefCurrents(const int n_motor, const int *motors, const double *currs) override;
    yarp::dev::yarp_ret_value getRefCurrents(double *currs) override;
    yarp::dev::yarp_ret_value getRefCurrent(int m, double *curr) override;
};

#endif // YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H
