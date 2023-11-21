/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_REMOTECONTROLBOARD_REMOTECONTROLBOARD_H
#define YARP_DEV_REMOTECONTROLBOARD_REMOTECONTROLBOARD_H

#include <yarp/sig/Vector.h>

#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IMotor.h>
#include <yarp/dev/IAmplifierControl.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IControlCalibration.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IImpedanceControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IRemoteCalibrator.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IJointFault.h>
#include <yarp/dev/ControlBoardHelpers.h>

#include "stateExtendedReader.h"

struct ProtocolVersion
{
    int major{0};
    int minor{0};
    int tweak{0};
};

class DiagnosticThread;


/**
* @ingroup dev_impl_network_clients
*
* \brief `remote_controlboard`: The client side of the control board, connects to a remote controlboard using the YARP network.
*
* This device communicates using the YARP ports opened the controlBoard_nws_yarp device
* to use a device exposing controlboard method even from a different process (or even computer)
* from the one that opened the controlboard device.
*
*  Parameters required by this device are:
* | Parameter name | SubParameter   | Type    | Units | Default Value | Required     | Description                                    | Notes |
* |:--------------:|:--------------:|:-------:|:-----:|:-------------:|:-----------: |:----------------------------------------------:|:-----:|
* | remote         |       -        | string  | -     |   -           | Yes          | Prefix of the port to which to connect.        |       |
* | local          |       -        | string  | -     |   -           | Yes          | Port prefix of the port opened by this device. |       |
* | writeStrict    |       -        | string  | -     | See note      | No           |                                                |       |
* | carrier        |       -        | string  | -     | udp           | No           | Specify the carrier used for reading state and sending references. This option does not change the carrier used by rpc, that is hardcoded to tcp. |       |
*
*/
class RemoteControlBoard :
        public yarp::dev::IPidControl,
        public yarp::dev::IPositionControl,
        public yarp::dev::IVelocityControl,
        public yarp::dev::IEncodersTimed,
        public yarp::dev::IMotorEncoders,
        public yarp::dev::IMotor,
        public yarp::dev::IAmplifierControl,
        public yarp::dev::IControlLimits,
        public yarp::dev::IAxisInfo,
        public yarp::dev::IPreciselyTimed,
        public yarp::dev::IControlCalibration,
        public yarp::dev::ITorqueControl,
        public yarp::dev::IImpedanceControl,
        public yarp::dev::IControlMode,
        public yarp::dev::DeviceDriver,
        public yarp::dev::IPositionDirect,
        public yarp::dev::IInteractionMode,
        public yarp::dev::IRemoteCalibrator,
        public yarp::dev::IRemoteVariables,
        public yarp::dev::IPWMControl,
        public yarp::dev::ICurrentControl,
        public yarp::dev::IJointFault
{
protected:
    yarp::os::Port rpc_p;
    yarp::os::Port command_p;
    DiagnosticThread *diagnosticThread{nullptr};

    yarp::os::PortReaderBuffer<yarp::sig::Vector> state_buffer;
    yarp::os::PortWriterBuffer<CommandMessage> command_buffer;
    bool writeStrict_singleJoint{true};
    bool writeStrict_moreJoints{false};

    // Buffer associated to the extendedOutputStatePort port; in this case we will use the type generated
    // from the YARP .thrift file
//  yarp::os::PortReaderBuffer<jointData>           extendedInputState_buffer;  // Buffer storing new data
    StateExtendedInputPort                          extendedIntputStatePort;  // Buffered port storing new data
    std::mutex extendedPortMutex;
    yarp::dev::impl::jointData last_singleJoint;     // tmp to store last received data for a particular joint
//    yarp::os::Port extendedIntputStatePort;         // Port /stateExt:i reading the state of the joints
    yarp::dev::impl::jointData last_wholePart;         // tmp to store last received data for whole part

    std::string remote;
    std::string local;
    mutable Stamp lastStamp;  //this is shared among all calls that read encoders
    size_t nj{0};
    bool njIsKnown{false};

    ProtocolVersion protocolVersion;

    // Check for number of joints, if needed.
    // This is to allow for delayed connection to the remote control board.
    bool isLive();

    bool checkProtocolVersion(bool ignore);

    //Helper methods used to set/get a value from the remote peer.
    yarp::dev::yarp_ret_value send1V(int v);
    yarp::dev::yarp_ret_value send2V(int v1, int v2);
    yarp::dev::yarp_ret_value send2V1I(int v1, int v2, int axis);
    yarp::dev::yarp_ret_value send1V1I(int v, int axis);
    yarp::dev::yarp_ret_value send3V1I(int v1, int v2, int v3, int j);
    yarp::dev::yarp_ret_value set1V(int code);
    yarp::dev::yarp_ret_value set1V2D(int code, double v);
    yarp::dev::yarp_ret_value set1V1I(int code, int v);
    yarp::dev::yarp_ret_value get1V1D(int code, double& v) const;
    yarp::dev::yarp_ret_value get1V1I(int code, int& v) const;
    yarp::dev::yarp_ret_value set1V1I1D(int code, int j, double val);
    yarp::dev::yarp_ret_value set1V1I2D(int code, int j, double val1, double val2);
    yarp::dev::yarp_ret_value set1VDA(int v, const double *val);
    yarp::dev::yarp_ret_value set2V1DA(int v1, int v2, const double *val);
    yarp::dev::yarp_ret_value set2V2DA(int v1, int v2, const double *val1, const double *val2);
    yarp::dev::yarp_ret_value set1V1I1IA1DA(int v, const int len, const int *val1, const double *val2);
    yarp::dev::yarp_ret_value set2V1I1D(int v1, int v2, int axis, double val);
    yarp::dev::yarp_ret_value setValWithPidType(int voc, PidControlTypeEnum type, int axis, double val);
    yarp::dev::yarp_ret_value setValWithPidType(int voc, PidControlTypeEnum type, const double* val_arr);
    yarp::dev::yarp_ret_value getValWithPidType(int voc, PidControlTypeEnum type, int j, double *val);
    yarp::dev::yarp_ret_value getValWithPidType(int voc, PidControlTypeEnum type, double *val);
    yarp::dev::yarp_ret_value set2V1I(int v1, int v2, int axis);
    yarp::dev::yarp_ret_value get1V1I1D(int v, int j, double *val);
    yarp::dev::yarp_ret_value get1V1I1I(int v, int j, int *val);
    yarp::dev::yarp_ret_value get2V1I1D(int v1, int v2, int j, double *val);
    yarp::dev::yarp_ret_value get2V1I2D(int v1, int v2, int j, double *val1, double *val2);
    yarp::dev::yarp_ret_value get1V1I2D(int code, int axis, double *v1, double *v2);
    yarp::dev::yarp_ret_value get1V1I1B(int v, int j, bool &val);
    yarp::dev::yarp_ret_value get1V1I1IA1B(int v,  const int len, const int *val1, bool &retVal);
    yarp::dev::yarp_ret_value get2V1I1IA1DA(int v1, int v2, const int n_joints, const int *joints, double *retVals, std::string functionName = "");
    yarp::dev::yarp_ret_value get1V1B(int v, bool &val);
    yarp::dev::yarp_ret_value get1VIA(int v, int *val);
    yarp::dev::yarp_ret_value get1VDA(int v, double *val);
    yarp::dev::yarp_ret_value get1V1DA(int v1, double *val);
    yarp::dev::yarp_ret_value get2V1DA(int v1, int v2, double *val);
    yarp::dev::yarp_ret_value get2V2DA(int v1, int v2, double *val1, double *val2);
    yarp::dev::yarp_ret_value get1V1I1S(int code, int j, std::string &name);
    yarp::dev::yarp_ret_value get1V1I1IA1DA(int v, const int len, const int *val1, double *val2);

public:
    RemoteControlBoard() = default;
    RemoteControlBoard(const RemoteControlBoard&) = delete;
    RemoteControlBoard(RemoteControlBoard&&) = delete;
    RemoteControlBoard& operator=(const RemoteControlBoard&) = delete;
    RemoteControlBoard& operator=(RemoteControlBoard&&) = delete;
    ~RemoteControlBoard() override = default;

    bool open(Searchable& config) override;

    /**
     * Close the device driver and stop the port connections.
     * @return true/false on success/failure.
     */
    bool close() override;

    yarp_ret_value getAxes(int *ax) override;

    // IPidControl
    yarp_ret_value setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
    yarp_ret_value setPids(const PidControlTypeEnum& pidtype, const Pid *pids) override;
    yarp_ret_value setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) override;
    yarp_ret_value setPidReferences(const PidControlTypeEnum& pidtype, const double *refs) override;
    yarp_ret_value setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) override;
    yarp_ret_value setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits) override;
    yarp_ret_value getPidError(const PidControlTypeEnum& pidtype, int j, double *err) override;
    yarp_ret_value getPidErrors(const PidControlTypeEnum& pidtype, double *errs) override;
    yarp_ret_value getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
    yarp_ret_value getPids(const PidControlTypeEnum& pidtype, Pid *pids) override;
    yarp_ret_value getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref) override;
    yarp_ret_value getPidReferences(const PidControlTypeEnum& pidtype, double *refs) override;
    yarp_ret_value getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit) override;
    yarp_ret_value getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits) override;
    yarp_ret_value resetPid(const PidControlTypeEnum& pidtype, int j) override;
    yarp_ret_value disablePid(const PidControlTypeEnum& pidtype, int j) override;
    yarp_ret_value enablePid(const PidControlTypeEnum& pidtype, int j) override;
    yarp_ret_value isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;
    yarp_ret_value getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out) override;
    yarp_ret_value getPidOutputs(const PidControlTypeEnum& pidtype, double *outs) override;
    yarp_ret_value setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) override;

    // IEncoder
    yarp_ret_value resetEncoder(int j) override;
    yarp_ret_value resetEncoders() override;
    yarp_ret_value setEncoder(int j, double val) override;
    yarp_ret_value setEncoders(const double *vals) override;
    yarp_ret_value getEncoder(int j, double *v) override;
    yarp_ret_value getEncoderTimed(int j, double *v, double *t) override;
    yarp_ret_value getEncoders(double *encs) override;
    yarp_ret_value getEncodersTimed(double *encs, double *ts) override;
    yarp_ret_value getEncoderSpeed(int j, double *sp) override;
    yarp_ret_value getEncoderSpeeds(double *spds) override;
    yarp_ret_value getEncoderAcceleration(int j, double *acc) override;
    yarp_ret_value getEncoderAccelerations(double *accs) override;

    // IRemoteVariable
    yarp_ret_value getRemoteVariable(std::string key, yarp::os::Bottle& val) override;
    yarp_ret_value setRemoteVariable(std::string key, const yarp::os::Bottle& val) override;
    yarp_ret_value getRemoteVariablesList(yarp::os::Bottle* listOfKeys) override;

    // IMotor
    yarp_ret_value getNumberOfMotors(int *num) override;
    yarp::dev::yarp_ret_value getTemperature(int m, double* val) override;
    yarp::dev::yarp_ret_value getTemperatures(double *vals) override;
    yarp_ret_value getTemperatureLimit (int m, double* val) override;
    yarp_ret_value setTemperatureLimit (int m, const double val) override;
    yarp_ret_value getGearboxRatio(int m, double* val) override;
    yarp_ret_value setGearboxRatio(int m, const double val) override;

    // IMotorEncoder
    yarp_ret_value resetMotorEncoder(int j) override;
    yarp_ret_value resetMotorEncoders() override;
    yarp_ret_value setMotorEncoder(int j, const double val) override;
    yarp_ret_value setMotorEncoderCountsPerRevolution(int m, const double cpr) override;
    yarp_ret_value getMotorEncoderCountsPerRevolution(int m, double *cpr) override;
    yarp_ret_value setMotorEncoders(const double *vals) override;
    yarp_ret_value getMotorEncoder(int j, double *v) override;
    yarp_ret_value getMotorEncoderTimed(int j, double *v, double *t) override;
    yarp_ret_value getMotorEncoders(double *encs) override;
    yarp_ret_value getMotorEncodersTimed(double *encs, double *ts) override;
    yarp_ret_value getMotorEncoderSpeed(int j, double *sp) override;
    yarp_ret_value getMotorEncoderSpeeds(double *spds) override;
    yarp_ret_value getMotorEncoderAcceleration(int j, double *acc) override;
    yarp_ret_value getMotorEncoderAccelerations(double *accs) override;
    yarp_ret_value getNumberOfMotorEncoders(int *num) override;

    // IPreciselyTimed
    Stamp getLastInputStamp() override;

    // IPositionControl
    yarp_ret_value positionMove(int j, double ref) override;
    yarp_ret_value positionMove(const int n_joint, const int *joints, const double *refs) override;
    yarp_ret_value positionMove(const double *refs) override;
    yarp_ret_value getTargetPosition(const int joint, double *ref) override;
    yarp_ret_value getTargetPositions(double *refs) override;
    yarp_ret_value getTargetPositions(const int n_joint, const int *joints, double *refs) override;
    yarp_ret_value relativeMove(int j, double delta) override;
    yarp_ret_value relativeMove(const int n_joint, const int *joints, const double *refs) override;
    yarp_ret_value relativeMove(const double *deltas) override;
    yarp_ret_value checkMotionDone(int j, bool *flag) override;
    yarp_ret_value checkMotionDone(const int n_joint, const int *joints, bool *flag) override;
    yarp_ret_value checkMotionDone(bool *flag) override;
    yarp_ret_value setRefSpeed(int j, double sp) override;
    yarp_ret_value setRefSpeeds(const int n_joint, const int *joints, const double *spds) override;
    yarp_ret_value setRefSpeeds(const double *spds) override;
    yarp_ret_value setRefAcceleration(int j, double acc) override;
    yarp_ret_value setRefAccelerations(const int n_joint, const int *joints, const double *accs) override;
    yarp_ret_value setRefAccelerations(const double *accs) override;
    yarp_ret_value getRefSpeed(int j, double *ref) override;
    yarp_ret_value getRefSpeeds(const int n_joint, const int *joints, double *spds) override;
    yarp_ret_value getRefSpeeds(double *spds) override;
    yarp_ret_value getRefAcceleration(int j, double *acc) override;
    yarp_ret_value getRefAccelerations(const int n_joint, const int *joints, double *accs) override;
    yarp_ret_value getRefAccelerations(double *accs) override;
    yarp_ret_value stop(int j) override;
    yarp_ret_value stop(const int len, const int *val1) override;
    yarp_ret_value stop() override;

    // IJointFault
    yarp_ret_value getLastJointFault(int j, int& fault, std::string& message) override;

    // IVelocityControl
    yarp_ret_value velocityMove(int j, double v) override;
    yarp_ret_value velocityMove(const double *v) override;

    // IAmplifierControl
    yarp_ret_value enableAmp(int j) override;
    yarp_ret_value disableAmp(int j) override;
    yarp_ret_value getAmpStatus(int *st) override;
    yarp_ret_value getAmpStatus(int j, int *st) override;
    yarp_ret_value setMaxCurrent(int j, double v) override;
    yarp_ret_value getMaxCurrent(int j, double *v) override;
    yarp_ret_value getNominalCurrent(int m, double *val) override;
    yarp_ret_value setNominalCurrent(int m, const double val) override;
    yarp_ret_value getPeakCurrent(int m, double *val) override;
    yarp_ret_value setPeakCurrent(int m, const double val) override;
    yarp_ret_value getPWM(int m, double* val) override;
    yarp_ret_value getPWMLimit(int m, double* val) override;
    yarp_ret_value setPWMLimit(int m, const double val) override;
    yarp_ret_value getPowerSupplyVoltage(int m, double* val) override;

    // IControlLimits
    yarp_ret_value setLimits(int axis, double min, double max) override;
    yarp_ret_value getLimits(int axis, double *min, double *max) override;
    yarp_ret_value setVelLimits(int axis, double min, double max) override;
    yarp_ret_value getVelLimits(int axis, double *min, double *max) override;

    // IAxisInfo
    yarp_ret_value getAxisName(int j, std::string& name) override;
    yarp_ret_value getJointType(int j, yarp::dev::JointTypeEnum& type) override;

    // IControlCalibration
    yarp_ret_value calibrateRobot() override;
    yarp_ret_value abortCalibration() override;
    yarp_ret_value abortPark() override;
    yarp_ret_value park(bool wait=true) override;
    yarp_ret_value calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3) override;
    yarp_ret_value setCalibrationParameters(int j, const CalibrationParameters& params) override;
    yarp_ret_value calibrationDone(int j) override;

    // ITorqueControl
    yarp_ret_value getRefTorque(int j, double *t) override;
    yarp_ret_value getRefTorques(double *t) override;
    yarp_ret_value setRefTorques(const double *t) override;
    yarp_ret_value setRefTorque(int j, double v) override;
    yarp_ret_value setRefTorques(const int n_joint, const int *joints, const double *t) override;
    yarp_ret_value setMotorTorqueParams(int j, const MotorTorqueParameters params) override;
    yarp_ret_value getMotorTorqueParams(int j, MotorTorqueParameters *params) override;
    yarp_ret_value getTorque(int j, double *t) override;
    yarp_ret_value getTorques(double *t) override;
    yarp_ret_value getTorqueRange(int j, double *min, double* max) override;
    yarp_ret_value getTorqueRanges(double *min, double *max) override;

    // IImpedanceControl
    yarp_ret_value getImpedance(int j, double *stiffness, double *damping) override;
    yarp_ret_value getImpedanceOffset(int j, double *offset) override;
    yarp_ret_value setImpedance(int j, double stiffness, double damping) override;
    yarp_ret_value setImpedanceOffset(int j, double offset) override;
    yarp_ret_value getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;

    // IControlMode
    yarp_ret_value getControlMode(int j, int *mode) override;
    yarp_ret_value getControlModes(int *modes) override;
    yarp_ret_value getControlModes(const int n_joint, const int *joints, int *modes) override;
    yarp_ret_value setControlMode(const int j, const int mode) override;
    yarp_ret_value setControlModes(const int n_joint, const int *joints, int *modes) override;
    yarp_ret_value setControlModes(int *modes) override;

    // IPositionDirect
    yarp_ret_value setPosition(int j, double ref) override;
    yarp_ret_value setPositions(const int n_joint, const int *joints, const double *refs) override;
    yarp_ret_value setPositions(const double *refs) override;
    yarp_ret_value getRefPosition(const int joint, double* ref) override;
    yarp_ret_value getRefPositions(double* refs) override;
    yarp_ret_value getRefPositions(const int n_joint, const int* joints, double* refs) override;

    // IVelocityControl
    yarp_ret_value velocityMove(const int n_joint, const int *joints, const double *spds) override;
    yarp_ret_value getRefVelocity(const int joint, double* vel) override;
    yarp_ret_value getRefVelocities(double* vels) override;
    yarp_ret_value getRefVelocities(const int n_joint, const int* joints, double* vels) override;

    // IInteractionMode
    yarp_ret_value getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    yarp_ret_value getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    yarp_ret_value getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    yarp_ret_value setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    yarp_ret_value setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    yarp_ret_value setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    // IRemoteCalibrator
    yarp_ret_value isCalibratorDevicePresent(bool *isCalib) override;
    yarp_ret_value calibrateSingleJoint(int j) override;
    yarp_ret_value calibrateWholePart() override;
    yarp_ret_value homingSingleJoint(int j) override;
    yarp_ret_value homingWholePart() override;
    yarp_ret_value parkSingleJoint(int j, bool _wait=true) override;
    yarp_ret_value parkWholePart() override;
    yarp_ret_value quitCalibrate() override;
    yarp_ret_value quitPark() override;

    // ICurrentControl
    yarp_ret_value getRefCurrents(double *t) override;
    yarp_ret_value getRefCurrent(int j, double *t) override;
    yarp_ret_value setRefCurrents(const double *refs) override;
    yarp_ret_value setRefCurrent(int j, double ref) override;
    yarp_ret_value setRefCurrents(const int n_joint, const int *joints, const double *refs) override;
    yarp_ret_value getCurrents(double *vals) override;
    yarp_ret_value getCurrent(int j, double *val) override;
    yarp_ret_value getCurrentRange(int j, double *min, double *max) override;
    yarp_ret_value getCurrentRanges(double *min, double *max) override;

    // IPWMControl
    yarp_ret_value setRefDutyCycle(int j, double v) override;
    yarp_ret_value setRefDutyCycles(const double *v) override;
    yarp_ret_value getRefDutyCycle(int j, double *ref) override;
    yarp_ret_value getRefDutyCycles(double *refs) override;
    yarp_ret_value getDutyCycle(int j, double *out) override;
    yarp_ret_value getDutyCycles(double *outs) override;
};



#endif // YARP_DEV_REMOTECONTROLBOARD_REMOTECONTROLBOARD_H
