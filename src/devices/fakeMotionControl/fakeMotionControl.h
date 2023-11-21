/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEVICE_FAKE_MOTIONCONTROL
#define YARP_DEVICE_FAKE_MOTIONCONTROL

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/ImplementVirtualAnalogSensor.h>
#include <yarp/dev/ImplementPositionControl.h>
#include <yarp/dev/ImplementVelocityControl.h>
#include <yarp/dev/ImplementJointFault.h>

#include <mutex>


struct ImpedanceLimits
{
    double min_stiff;
    double max_stiff;
    double min_damp;
    double max_damp;
    double param_a;
    double param_b;
    double param_c;

public:
    ImpedanceLimits()
    {
        min_stiff=0;
        max_stiff=0;
        min_damp=0;
        max_damp=0;
        param_a=0;
        param_b=0;
        param_c=0;
    }

    double get_min_stiff()
    {
        return min_stiff;
    }
    double get_max_stiff()
    {
        return max_stiff;
    }
    double get_min_damp()
    {
        return min_damp;
    }
    double get_max_damp()
    {
        return max_damp;
    }
};

struct ImpedanceParameters
{
    double stiffness;
    double damping;
    ImpedanceLimits limits;
    ImpedanceParameters() {stiffness=0; damping=0;}
};

/**
 * @ingroup dev_impl_fake dev_impl_motor
 *
 * \brief `fakeMotionControl`: Documentation to be added
 *
 * The aim of this device is to mimic the expected behavior of a
 * real motion control device to help testing the high level software.
 *
 * This device is implementing last version of interfaces and it is compatible
 * with controlBoard_nws_yarp device.
 *
 * WIP - it is very basic now, not all interfaces are implemented yet.
 */
class FakeMotionControl :
        public yarp::dev::DeviceDriver,
//        public yarp::dev::DeviceResponder,
        public yarp::os::PeriodicThread,
        public yarp::dev::IPidControlRaw,
        public yarp::dev::IControlCalibrationRaw,
        public yarp::dev::IAmplifierControlRaw,
        public yarp::dev::IEncodersTimedRaw,
        public yarp::dev::IMotorEncodersRaw,
        public yarp::dev::IMotorRaw,
        public yarp::dev::IPositionControlRaw,
        public yarp::dev::IVelocityControlRaw,
        public yarp::dev::IControlModeRaw,
        public yarp::dev::IControlLimitsRaw,
        public yarp::dev::IPositionDirectRaw,
        public yarp::dev::ITorqueControlRaw,
        public yarp::dev::ICurrentControlRaw,
        public yarp::dev::IPWMControlRaw,
        public yarp::dev::IImpedanceControlRaw,
        public yarp::dev::IInteractionModeRaw,
        public yarp::dev::IAxisInfoRaw,
        public yarp::dev::IVirtualAnalogSensorRaw, //*
        public yarp::dev::IJointFaultRaw,
        public yarp::dev::ImplementControlCalibration,
        public yarp::dev::ImplementAmplifierControl,
        public yarp::dev::ImplementPidControl,
        public yarp::dev::ImplementEncodersTimed,
        public yarp::dev::ImplementPositionControl,
        public yarp::dev::ImplementVelocityControl,
        public yarp::dev::ImplementControlMode,
        public yarp::dev::ImplementImpedanceControl,
        public yarp::dev::ImplementJointFault,
        public yarp::dev::ImplementMotorEncoders,
        public yarp::dev::ImplementTorqueControl,
        public yarp::dev::ImplementControlLimits,
        public yarp::dev::ImplementPositionDirect,
        public yarp::dev::ImplementInteractionMode,
        public yarp::dev::ImplementCurrentControl,
        public yarp::dev::ImplementPWMControl,
        public yarp::dev::ImplementMotor,
        public yarp::dev::ImplementAxisInfo,
        public yarp::dev::ImplementVirtualAnalogSensor //*
{
private:
    enum VerboseLevel
    {
        MUTE                = 0,    // only errors that prevent device from working
        QUIET               = 1,    // adds errors that can cause malfunctioning
        DEFAULT             = 2,    // adds warnings // DEFAULT // show noisy messages about back-compatible changes
        CHATTY              = 3,    // adds info messages
        VERBOSE             = 4,    // adds debug messages
        VERY_VERBOSE        = 5,    // adds trace of events (shows thread running and catch if they get stuck)
        VERY_VERY_VERBOSE   = 6     // adds messages printed every cycle, so too much verbose for usage, only for deep debugging
    };

    std::recursive_mutex _mutex;
    int  _njoints;
    int *_axisMap;                              /** axis remapping lookup-table */
    double *_angleToEncoder;                    /** angle to iCubDegrees conversion factors */
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */
    double *_ampsToSensor;
    double *_dutycycleToPWM;
    float *_DEPRECATED_encoderconversionfactor;            /** iCubDegrees to encoder conversion factors */
    float *_DEPRECATED_encoderconversionoffset;            /** iCubDegrees offset */
//     uint8_t *_jointEncoderType;                 /** joint encoder type*/
    int    *_jointEncoderRes;                   /** joint encoder resolution */
    int    *_rotorEncoderRes;                   /** rotor encoder resolution */
//     uint8_t *_rotorEncoderType;                  /** rotor encoder type*/
    double *_gearbox;                           /** the gearbox ratio */
    bool   *_hasHallSensor;                     /** */
    bool   *_hasTempSensor;                     /** */
    bool   *_hasRotorEncoder;                   /** */
    bool   *_hasRotorEncoderIndex;              /** */
    int    *_rotorIndexOffset;                  /** */
    int    *_motorPoles;                        /** */
    double *_rotorlimits_max;                   /** */
    double *_rotorlimits_min;                   /** */
    yarp::dev::Pid *_ppids;                                /** initial position gains */
    yarp::dev::Pid *_tpids;                                /** initial torque gains */
    yarp::dev::Pid *_cpids;                                /** initial current gains */
    yarp::dev::Pid *_vpids;                                /** initial velocity gains */
    bool *_ppids_ena;
    bool *_tpids_ena;
    bool *_cpids_ena;
    bool *_vpids_ena;
    double *_ppids_lim;
    double *_tpids_lim;
    double *_cpids_lim;
    double *_vpids_lim;
    double *_ppids_ref;
    double *_tpids_ref;
    double *_cpids_ref;
    double *_vpids_ref;

    std::string *_axisName;                          /** axis name */
    yarp::dev::JointTypeEnum *_jointType;                          /** axis type */
//     ImpedanceLimits     *_impedance_limits;     /** impedance limits */
    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_kinematic_mj;                      /** the kinematic coupling matrix from joints space to motor space */
    //double *_currentLimits;                     /** current limits */
//     MotorCurrentLimits *_currentLimits;
    double *_maxJntCmdVelocity;                 /** max joint commanded velocity */
    double *_maxMotorVelocity;                  /** max motor velocity */
    int *_velocityShifts;                       /** velocity shifts */
    int *_velocityTimeout;                      /** velocity shifts */
    double *_kbemf;                             /** back-emf compensation parameter */
    double *_ktau;                              /** motor torque constant */
    int *_kbemf_scale;                          /** back-emf compensation parameter */
    int *_ktau_scale;                           /** motor torque constant */
    double *_viscousPos;                        /** viscous pos friction  */
    double *_viscousNeg;                        /** viscous neg friction  */
    double *_coulombPos;                        /** coulomb up friction  */
    double *_coulombNeg;                        /** coulomb neg friction */
    double *_velocityThres;                     /** velocity threshold for torque control */
    int * _filterType;                          /** the filter type (int value) used by the force control algorithm */
    int *_torqueSensorId;                       /** Id of associated Joint Torque Sensor */
    int *_torqueSensorChan;                     /** Channel of associated Joint Torque Sensor */
    double *_maxTorque;                         /** Max torque of a joint */
    double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */
    bool  *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */
    double *_last_position_move_time;           /** time stamp for last received position move command*/
    double *_motorPwmLimits;                    /** motors PWM limits*/
    double *_torques;                           /** joint torques */

//     ImpedanceParameters *_impedance_params;     /** impedance parameters */

    bool        verbosewhenok;
    bool        useRawEncoderData;
    bool        _pwmIsLimited;                         /** set to true if pwm is limited */
    bool        _torqueControlEnabled;                 /** set to true if the torque control parameters are successfully loaded. If false, boards cannot switch in torque mode */

    enum       torqueControlUnitsType {T_MACHINE_UNITS=0, T_METRIC_UNITS=1};
    torqueControlUnitsType _torqueControlUnits;

    enum       positionControlUnitsType {P_MACHINE_UNITS=0, P_METRIC_UNITS=1};
    positionControlUnitsType _positionControlUnits;

    // internal stuff
    bool    velocity_watchdog_enabled = false; //false for testing purposes. On the real robot is true.
    bool    openloop_watchdog_enabled = false; //false for testing purposes. On the real robot is true.
    int     *_controlModes = nullptr;
    int     *_hwfault_code = nullptr;
    std::string  *_hwfault_message = nullptr;
    int     *_interactMode = nullptr;
    bool    *_enabledAmp = nullptr;           // Middle step toward a full enabled motor controller. Amp (pwm) plus Pid enable command must be sent in order to get the joint into an active state.
    bool    *_enabledPid = nullptr;           // Depends on enabledAmp. When both are set, the joint exits the idle mode and goes into position mode. If one of them is disabled, it falls to idle.
    bool    *_calibrated = nullptr;           // Flag to know if the calibrate function has been called for the joint
    double  *_posCtrl_references = nullptr;   // used for position control.
    double  *_posDir_references = nullptr;    // used for position Direct control.
    double  *_ref_speeds = nullptr;           // used for position control.
    double  *_command_speeds = nullptr;       // used for velocity control.
    double  *_ref_accs = nullptr;             // for velocity control, in position min jerk eq is used.
    double  *_ref_torques = nullptr;          // for torque control.
    double  *_ref_currents = nullptr;
    yarp::sig::Vector       current, nominalCurrent, maxCurrent, peakCurrent;
    yarp::sig::Vector       pwm, pwmLimit, refpwm, supplyVoltage,last_velocity_command, last_pwm_command;
    yarp::sig::Vector pos, dpos, vel, speed, acc, loc, amp;
    double prev_time;
    bool opened;

    // debugging
    VerboseLevel verbose;
public:

    FakeMotionControl();
    ~FakeMotionControl();

  // Device Driver
    bool open(yarp::os::Searchable &par) override;
    bool close() override;
    bool fromConfig(yarp::os::Searchable &config);

    virtual bool initialised();

    /**
     * Allocated buffers.
     */
    bool alloc(int njoints);

    /**
     * Resize previously allocated buffers.
     */
    void resizeBuffers();

    bool threadInit() override;
    void threadRelease() override;

    /////////   PID INTERFACE   /////////
    yarp::dev::yarp_ret_value setPidRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, const yarp::dev::Pid &pid) override;
    yarp::dev::yarp_ret_value setPidsRaw(const yarp::dev::PidControlTypeEnum& pidtype,const yarp::dev::Pid *pids) override;
    yarp::dev::yarp_ret_value setPidReferenceRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double ref) override;
    yarp::dev::yarp_ret_value setPidReferencesRaw(const yarp::dev::PidControlTypeEnum& pidtype,const double *refs) override;
    yarp::dev::yarp_ret_value setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double limit) override;
    yarp::dev::yarp_ret_value setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum& pidtype,const double *limits) override;
    yarp::dev::yarp_ret_value getPidErrorRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *err) override;
    yarp::dev::yarp_ret_value getPidErrorsRaw(const yarp::dev::PidControlTypeEnum& pidtype, double *errs) override;
    yarp::dev::yarp_ret_value getPidOutputRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *out) override;
    yarp::dev::yarp_ret_value getPidOutputsRaw(const yarp::dev::PidControlTypeEnum& pidtype,double *outs) override;
    yarp::dev::yarp_ret_value getPidRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, yarp::dev::Pid *pid) override;
    yarp::dev::yarp_ret_value getPidsRaw(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid *pids) override;
    yarp::dev::yarp_ret_value getPidReferenceRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *ref) override;
    yarp::dev::yarp_ret_value getPidReferencesRaw(const yarp::dev::PidControlTypeEnum& pidtype,double *refs) override;
    yarp::dev::yarp_ret_value getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double *limit) override;
    yarp::dev::yarp_ret_value getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum& pidtype,double *limits) override;
    yarp::dev::yarp_ret_value resetPidRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j) override;
    yarp::dev::yarp_ret_value disablePidRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j) override;
    yarp::dev::yarp_ret_value enablePidRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j) override;
    yarp::dev::yarp_ret_value setPidOffsetRaw(const yarp::dev::PidControlTypeEnum& pidtype,int j, double v) override;
    yarp::dev::yarp_ret_value isPidEnabledRaw(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled) override;

    // POSITION CONTROL INTERFACE RAW
    yarp::dev::yarp_ret_value getAxes(int *ax) override;
    yarp::dev::yarp_ret_value positionMoveRaw(int j, double ref) override;
    yarp::dev::yarp_ret_value positionMoveRaw(const double *refs) override;
    yarp::dev::yarp_ret_value relativeMoveRaw(int j, double delta) override;
    yarp::dev::yarp_ret_value relativeMoveRaw(const double *deltas) override;
    yarp::dev::yarp_ret_value checkMotionDoneRaw(bool *flag) override;
    yarp::dev::yarp_ret_value checkMotionDoneRaw(int j, bool *flag) override;
    yarp::dev::yarp_ret_value setRefSpeedRaw(int j, double sp) override;
    yarp::dev::yarp_ret_value setRefSpeedsRaw(const double *spds) override;
    yarp::dev::yarp_ret_value setRefAccelerationRaw(int j, double acc) override;
    yarp::dev::yarp_ret_value setRefAccelerationsRaw(const double *accs) override;
    yarp::dev::yarp_ret_value getRefSpeedRaw(int j, double *ref) override;
    yarp::dev::yarp_ret_value getRefSpeedsRaw(double *spds) override;
    yarp::dev::yarp_ret_value getRefAccelerationRaw(int j, double *acc) override;
    yarp::dev::yarp_ret_value getRefAccelerationsRaw(double *accs) override;
    yarp::dev::yarp_ret_value stopRaw(int j) override;
    yarp::dev::yarp_ret_value stopRaw() override;

    // Position Control2 Interface
    yarp::dev::yarp_ret_value positionMoveRaw(const int n_joint, const int *joints, const double *refs) override;
    yarp::dev::yarp_ret_value relativeMoveRaw(const int n_joint, const int *joints, const double *deltas) override;
    yarp::dev::yarp_ret_value checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags) override;
    yarp::dev::yarp_ret_value setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds) override;
    yarp::dev::yarp_ret_value setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs) override;
    yarp::dev::yarp_ret_value getRefSpeedsRaw(const int n_joint, const int *joints, double *spds) override;
    yarp::dev::yarp_ret_value getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs) override;
    yarp::dev::yarp_ret_value stopRaw(const int n_joint, const int *joints) override;
    yarp::dev::yarp_ret_value getTargetPositionRaw(const int joint, double *ref) override;
    yarp::dev::yarp_ret_value getTargetPositionsRaw(double *refs) override;
    yarp::dev::yarp_ret_value getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    //  Velocity control interface raw
    yarp::dev::yarp_ret_value velocityMoveRaw(int j, double sp) override;
    yarp::dev::yarp_ret_value velocityMoveRaw(const double *sp) override;

    // IJointFault
    yarp::dev::yarp_ret_value getLastJointFaultRaw(int j, int& fault, std::string& message) override;

    // calibration2raw
    yarp::dev::yarp_ret_value setCalibrationParametersRaw(int axis, const yarp::dev::CalibrationParameters& params) override;
    yarp::dev::yarp_ret_value calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override;
    yarp::dev::yarp_ret_value calibrationDoneRaw(int j) override;


    /////////////////////////////// END Position Control INTERFACE

    // ControlMode
    yarp::dev::yarp_ret_value getControlModeRaw(int j, int *v) override;
    yarp::dev::yarp_ret_value getControlModesRaw(int *v) override;

    // ControlMode 2
    yarp::dev::yarp_ret_value getControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    yarp::dev::yarp_ret_value setControlModeRaw(const int j, const int mode) override;
    yarp::dev::yarp_ret_value setControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    yarp::dev::yarp_ret_value setControlModesRaw(int *modes) override;

    //////////////////////// BEGIN EncoderInterface
    yarp::dev::yarp_ret_value resetEncoderRaw(int j) override;
    yarp::dev::yarp_ret_value resetEncodersRaw() override;
    yarp::dev::yarp_ret_value setEncoderRaw(int j, double val) override;
    yarp::dev::yarp_ret_value setEncodersRaw(const double *vals) override;
    yarp::dev::yarp_ret_value getEncoderRaw(int j, double *v) override;
    yarp::dev::yarp_ret_value getEncodersRaw(double *encs) override;
    yarp::dev::yarp_ret_value getEncoderSpeedRaw(int j, double *sp) override;
    yarp::dev::yarp_ret_value getEncoderSpeedsRaw(double *spds) override;
    yarp::dev::yarp_ret_value getEncoderAccelerationRaw(int j, double *spds) override;
    yarp::dev::yarp_ret_value getEncoderAccelerationsRaw(double *accs) override;
    ///////////////////////// END Encoder Interface

    yarp::dev::yarp_ret_value getEncodersTimedRaw(double *encs, double *stamps) override;
    yarp::dev::yarp_ret_value getEncoderTimedRaw(int j, double *encs, double *stamp) override;

    //////////////////////// BEGIN MotorEncoderInterface
    yarp::dev::yarp_ret_value getNumberOfMotorEncodersRaw(int * num) override;
    yarp::dev::yarp_ret_value resetMotorEncoderRaw(int m) override;
    yarp::dev::yarp_ret_value resetMotorEncodersRaw() override;
    yarp::dev::yarp_ret_value setMotorEncoderRaw(int m, const double val) override;
    yarp::dev::yarp_ret_value setMotorEncodersRaw(const double *vals) override;
    yarp::dev::yarp_ret_value getMotorEncoderRaw(int m, double *v) override;
    yarp::dev::yarp_ret_value getMotorEncodersRaw(double *encs) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeedRaw(int m, double *sp) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeedsRaw(double *spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAccelerationRaw(int m, double *spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAccelerationsRaw(double *accs) override;
    yarp::dev::yarp_ret_value getMotorEncodersTimedRaw(double *encs, double *stamps) override;
    yarp::dev::yarp_ret_value getMotorEncoderTimedRaw(int m, double *encs, double *stamp) override;
    yarp::dev::yarp_ret_value getMotorEncoderCountsPerRevolutionRaw(int m, double *v) override;
    yarp::dev::yarp_ret_value setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN IAxisInfo Interface
    yarp::dev::yarp_ret_value getAxisNameRaw(int axis, std::string& name) override;
    yarp::dev::yarp_ret_value getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type) override;
    ///////////////////////// END IAxisInfo Interface

    //Internal use, not exposed by YARP (yet)
    virtual bool getRotorEncoderResolutionRaw(int m, double &rotres);
    virtual bool getJointEncoderResolutionRaw(int m, double &jntres);
    virtual bool getJointEncoderTypeRaw(int j, int &type);
    virtual bool getRotorEncoderTypeRaw(int j, int &type);
    virtual bool getKinematicMJRaw(int j, double &rotres);
    virtual bool getHasTempSensorsRaw(int j, int& ret);
    virtual bool getHasHallSensorRaw(int j, int& ret);
    virtual bool getHasRotorEncoderRaw(int j, int& ret);
    virtual bool getHasRotorEncoderIndexRaw(int j, int& ret);
    virtual bool getMotorPolesRaw(int j, int& poles);
    virtual bool getRotorIndexOffsetRaw(int j, double& rotorOffset);
    virtual bool getTorqueControlFilterType(int j, int& type);

    ////// Amplifier interface
    yarp::dev::yarp_ret_value enableAmpRaw(int j) override;
    yarp::dev::yarp_ret_value disableAmpRaw(int j) override;
    yarp::dev::yarp_ret_value getCurrentsRaw(double *vals) override;
    yarp::dev::yarp_ret_value getCurrentRaw(int j, double *val) override;
    yarp::dev::yarp_ret_value getNominalCurrentRaw(int m, double *val) override;
    yarp::dev::yarp_ret_value setNominalCurrentRaw(int m, const double val) override;
    yarp::dev::yarp_ret_value setMaxCurrentRaw(int j, double val) override;
    yarp::dev::yarp_ret_value getMaxCurrentRaw(int j, double *val) override;
    yarp::dev::yarp_ret_value getPeakCurrentRaw(int m, double *val) override;
    yarp::dev::yarp_ret_value setPeakCurrentRaw(int m, const double val) override;
    yarp::dev::yarp_ret_value getAmpStatusRaw(int *st) override;
    yarp::dev::yarp_ret_value getAmpStatusRaw(int j, int *st) override;
    yarp::dev::yarp_ret_value getPWMRaw(int j, double* val) override;
    yarp::dev::yarp_ret_value getPWMLimitRaw(int j, double* val) override;
    yarp::dev::yarp_ret_value setPWMLimitRaw(int j, const double val) override;
    yarp::dev::yarp_ret_value getPowerSupplyVoltageRaw(int j, double* val) override;
    /////////////// END AMPLIFIER INTERFACE

    // Limits
    yarp::dev::yarp_ret_value setLimitsRaw(int axis, double min, double max) override;
    yarp::dev::yarp_ret_value getLimitsRaw(int axis, double *min, double *max) override;
    // Limits 2
    yarp::dev::yarp_ret_value setVelLimitsRaw(int axis, double min, double max) override;
    yarp::dev::yarp_ret_value getVelLimitsRaw(int axis, double *min, double *max) override;

    // Torque control
    yarp::dev::yarp_ret_value getTorqueRaw(int j, double *t) override;
    yarp::dev::yarp_ret_value getTorquesRaw(double *t) override;
    yarp::dev::yarp_ret_value getTorqueRangeRaw(int j, double *min, double *max) override;
    yarp::dev::yarp_ret_value getTorqueRangesRaw(double *min, double *max) override;
    yarp::dev::yarp_ret_value setRefTorquesRaw(const double *t) override;
    yarp::dev::yarp_ret_value setRefTorqueRaw(int j, double t) override;
    yarp::dev::yarp_ret_value setRefTorquesRaw(const int n_joint, const int *joints, const double *t) override;
    yarp::dev::yarp_ret_value getRefTorquesRaw(double *t) override;
    yarp::dev::yarp_ret_value getRefTorqueRaw(int j, double *t) override;
    yarp::dev::yarp_ret_value getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters *params) override;
    yarp::dev::yarp_ret_value setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override;
//     int32_t getRefSpeedInTbl(uint8_t boardNum, int j, eOmeas_position_t pos) override;

    // IVelocityControl interface
    yarp::dev::yarp_ret_value velocityMoveRaw(const int n_joint, const int *joints, const double *spds) override;
    yarp::dev::yarp_ret_value getRefVelocityRaw(const int joint, double *ref) override;
    yarp::dev::yarp_ret_value getRefVelocitiesRaw(double *refs) override;
    yarp::dev::yarp_ret_value getRefVelocitiesRaw(const int n_joint, const int *joints, double *refs) override;

    // Impedance interface
    yarp::dev::yarp_ret_value getImpedanceRaw(int j, double *stiffness, double *damping) override;
    yarp::dev::yarp_ret_value setImpedanceRaw(int j, double stiffness, double damping) override;
    yarp::dev::yarp_ret_value setImpedanceOffsetRaw(int j, double offset) override;
    yarp::dev::yarp_ret_value getImpedanceOffsetRaw(int j, double *offset) override;
    yarp::dev::yarp_ret_value getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;

    // PositionDirect Interface
    yarp::dev::yarp_ret_value setPositionRaw(int j, double ref) override;
    yarp::dev::yarp_ret_value setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    yarp::dev::yarp_ret_value setPositionsRaw(const double *refs) override;
    yarp::dev::yarp_ret_value getRefPositionRaw(const int joint, double *ref) override;
    yarp::dev::yarp_ret_value getRefPositionsRaw(double *refs) override;
    yarp::dev::yarp_ret_value getRefPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    // InteractionMode interface
    yarp::dev::yarp_ret_value getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode) override;
    yarp::dev::yarp_ret_value getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    yarp::dev::yarp_ret_value getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;
    yarp::dev::yarp_ret_value setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode) override;
    yarp::dev::yarp_ret_value setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    yarp::dev::yarp_ret_value setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;

    // IMotor interface
    yarp::dev::yarp_ret_value getNumberOfMotorsRaw(int * num) override;
    yarp::dev::yarp_ret_value getTemperatureRaw(int m, double* val) override;
    yarp::dev::yarp_ret_value getTemperaturesRaw(double *vals) override;
    yarp::dev::yarp_ret_value getTemperatureLimitRaw(int m, double *temp) override;
    yarp::dev::yarp_ret_value setTemperatureLimitRaw(int m, const double temp) override;
    yarp::dev::yarp_ret_value getGearboxRatioRaw(int m, double* gearbox) override;
    yarp::dev::yarp_ret_value setGearboxRatioRaw(int m, const double val) override;

    // PWM interface
    yarp::dev::yarp_ret_value setRefDutyCycleRaw(int j, double v) override;
    yarp::dev::yarp_ret_value setRefDutyCyclesRaw(const double *v) override;
    yarp::dev::yarp_ret_value getRefDutyCycleRaw(int j, double *v) override;
    yarp::dev::yarp_ret_value getRefDutyCyclesRaw(double *v) override;
    yarp::dev::yarp_ret_value getDutyCycleRaw(int j, double *v) override;
    yarp::dev::yarp_ret_value getDutyCyclesRaw(double *v) override;

    // Current interface
    //yarp::dev::yarp_ret_value getAxes(int *ax) override;
    //yarp::dev::yarp_ret_value getCurrentRaw(int j, double *t) override;
    //yarp::dev::yarp_ret_value getCurrentsRaw(double *t) override;
    yarp::dev::yarp_ret_value getCurrentRangeRaw(int j, double *min, double *max) override;
    yarp::dev::yarp_ret_value getCurrentRangesRaw(double *min, double *max) override;
    yarp::dev::yarp_ret_value setRefCurrentsRaw(const double *t) override;
    yarp::dev::yarp_ret_value setRefCurrentRaw(int j, double t) override;
    yarp::dev::yarp_ret_value setRefCurrentsRaw(const int n_joint, const int *joints, const double *t) override;
    yarp::dev::yarp_ret_value getRefCurrentsRaw(double *t) override;
    yarp::dev::yarp_ret_value getRefCurrentRaw(int j, double *t) override;

    yarp::dev::VAS_status getVirtualAnalogSensorStatusRaw(int ch) override;
    int getVirtualAnalogSensorChannelsRaw() override;
    yarp::dev::yarp_ret_value updateVirtualAnalogSensorMeasureRaw(yarp::sig::Vector &measure) override;
    yarp::dev::yarp_ret_value updateVirtualAnalogSensorMeasureRaw(int ch, double &measure) override;

    void run() override;
private:
    void cleanup();
    bool dealloc();
    bool parsePositionPidsGroup(yarp::os::Bottle& pidsGroup, yarp::dev::Pid myPid[]);
    bool parseTorquePidsGroup(yarp::os::Bottle& pidsGroup, yarp::dev::Pid myPid[], double kbemf[], double ktau[], int filterType[], double viscousPos[], double viscousNeg[], double coulombPos[], double coulombNeg[], double velocityThres[]);

    bool parseImpedanceGroup_NewFormat(yarp::os::Bottle& pidsGroup, ImpedanceParameters vals[]);

    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size);
};

#endif  // YARP_DEVICE_FAKE_MOTIONCONTROL
