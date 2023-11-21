/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEVICE_FAKE_MOTIONCONTROLMICRO
#define YARP_DEVICE_FAKE_MOTIONCONTROLMICRO

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ImplementJointFault.h>

#include <mutex>

/**
 * @ingroup dev_impl_fake dev_impl_motor
 *
 * \brief `fakeMotionControlMicro`: Documentation to be added
 *
 * The aim of this device is to mimic the expected behavior of a
 * real motion control device to help testing the high level software.
 *
 * This device is implementing last version of interfaces and it is compatible
 * with controlBoard_nws_yarp device.
 *
 * WIP - it is very basic now, not all interfaces are implemented yet.
 */
class FakeMotionControlMicro :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::IEncodersTimedRaw,
        public yarp::dev::IMotorEncodersRaw,
        public yarp::dev::IAxisInfoRaw,
        public yarp::dev::IJointFaultRaw,
        public yarp::dev::ImplementJointFault,
        public yarp::dev::ImplementAxisInfo,
        public yarp::dev::ImplementEncodersTimed,
        public yarp::dev::ImplementMotorEncoders
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

    std::string *_axisName;                     /** axis name */
    yarp::dev::JointTypeEnum *_jointType;       /** axis type */

    bool        verbosewhenok;
    bool        useRawEncoderData;


    // internal stuff
    int     *_controlModes = nullptr;
    int     *_hwfault_code = nullptr;
    std::string  *_hwfault_message = nullptr;
    yarp::sig::Vector pos, dpos, vel, speed, acc, loc, amp;
    double prev_time;
    bool opened;

    // debugging
    VerboseLevel verbose;
public:

    FakeMotionControlMicro();
    ~FakeMotionControlMicro();

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

    // IJointFault
    yarp::dev::yarp_ret_value getLastJointFaultRaw(int j, int& fault, std::string& message) override;

    //////////////////////// BEGIN MotorEncoder Interface
    yarp::dev::yarp_ret_value getNumberOfMotorEncodersRaw(int* num) override;
    yarp::dev::yarp_ret_value resetMotorEncoderRaw(int j) override;
    yarp::dev::yarp_ret_value resetMotorEncodersRaw() override;
    yarp::dev::yarp_ret_value setMotorEncoderRaw(int j, double val) override;
    yarp::dev::yarp_ret_value setMotorEncodersRaw(const double* vals) override;
    yarp::dev::yarp_ret_value getMotorEncoderRaw(int j, double* v) override;
    yarp::dev::yarp_ret_value getMotorEncodersRaw(double* encs) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeedRaw(int j, double* sp) override;
    yarp::dev::yarp_ret_value getMotorEncoderSpeedsRaw(double* spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAccelerationRaw(int j, double* spds) override;
    yarp::dev::yarp_ret_value getMotorEncoderAccelerationsRaw(double* accs) override;
    yarp::dev::yarp_ret_value getMotorEncodersTimedRaw(double* encs, double* stamps) override;
    yarp::dev::yarp_ret_value getMotorEncoderTimedRaw(int m, double* encs, double* stamp) override;
    yarp::dev::yarp_ret_value getMotorEncoderCountsPerRevolutionRaw(int m, double* v) override;
    yarp::dev::yarp_ret_value setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN EncoderInterface
    yarp::dev::yarp_ret_value getAxes(int* ax) override;
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

    //////////////////////// BEGIN EncoderTimed Interface
    yarp::dev::yarp_ret_value getEncodersTimedRaw(double *encs, double *stamps) override;
    yarp::dev::yarp_ret_value getEncoderTimedRaw(int j, double *encs, double *stamp) override;
    ///////////////////////// END EncoderTimed Interface

    //////////////////////// BEGIN IAxisInfo Interface
    yarp::dev::yarp_ret_value getAxisNameRaw(int axis, std::string& name) override;
    yarp::dev::yarp_ret_value getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type) override;
    ///////////////////////// END IAxisInfo Interface

    void run() override;
private:
    void cleanup();
    bool dealloc();

    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size);
};

#endif  // YARP_DEVICE_FAKE_MOTIONCONTROLMICRO
