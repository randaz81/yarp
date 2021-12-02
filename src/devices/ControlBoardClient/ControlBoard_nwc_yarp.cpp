/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ControlBoard_nwc_yarp.h"
#include "ControlBoard_nwc_yarp_LogComponent.h"
#include "stateExtendedReader.h"

#include <cstring>

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/QosStyle.h>


#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/dev/IPreciselyTimed.h>

#include <iterator>
#include <vector>
#include <mutex>


using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

namespace {

constexpr int PROTOCOL_VERSION_MAJOR = 1;
constexpr int PROTOCOL_VERSION_MINOR = 9;
constexpr int PROTOCOL_VERSION_TWEAK = 0;

constexpr double DIAGNOSTIC_THREAD_PERIOD = 1.000;

inline bool getTimeStamp(Bottle &bot, Stamp &st)
{
    if (bot.get(3).asVocab32()==VOCAB_TIMESTAMP)
    {
        //yup! we have a timestamp
        int fr=bot.get(4).asInt32();
        double ts=bot.get(5).asFloat64();
        st=Stamp(fr,ts);
        return true;
    }
    return false;
}

} // namespace


class DiagnosticThread :
        public PeriodicThread
{
    StateExtendedInputPort *owner{nullptr};
    std::string ownerName;

public:
    using PeriodicThread::PeriodicThread;

    void setOwner(StateExtendedInputPort *o)
    {
        owner = o;
        ownerName = owner->getName();
    }

    void run() override
    {
        if (owner != nullptr)
        {
            if (owner->getIterations() > 100)
            {
                int it;
                double av;
                double max;
                double min;
                owner->getEstFrequency(it, av, min, max);
                owner->resetStat();
                yCDebug(CONTROLBOARD_NWC_YARP,
                        "%s: %d msgs av:%.2lf min:%.2lf max:%.2lf [ms]",
                        ownerName.c_str(),
                        it,
                        av,
                        min,
                        max);
            }

        }
    }
};


bool ControlBoard_nwc_yarp::isLive()
{
    if (!njIsKnown) {
        int axes = 0;
        bool ok = get1V1I(VOCAB_AXES, axes);
        if (axes >= 0 && ok) {
            nj = axes;
            njIsKnown = true;
        }
    }
    return njIsKnown;
}


bool ControlBoard_nwc_yarp::checkProtocolVersion(bool ignore)
{
    bool error=false;
    // verify protocol
    Bottle cmd, reply;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PROTOCOL_VERSION);
    rpc_p.write(cmd, reply);

    // check size and format of messages, expected [prot] int int int [ok]
    if (reply.size() != 5) {
        error = true;
    }

    if (reply.get(0).asVocab32() != VOCAB_PROTOCOL_VERSION) {
        error = true;
    }

    if (!error)
    {
        protocolVersion.major=reply.get(1).asInt32();
        protocolVersion.minor=reply.get(2).asInt32();
        protocolVersion.tweak=reply.get(3).asInt32();

        //verify protocol
        if (protocolVersion.major != PROTOCOL_VERSION_MAJOR) {
            error = true;
        }

        if (protocolVersion.minor != PROTOCOL_VERSION_MINOR) {
            error = true;
        }
    }

    if (!error) {
        return true;
    }

    // protocol did not match
    yCError(CONTROLBOARD_NWC_YARP,
            "Expecting protocol %d %d %d, but the device we are connecting to has protocol version %d %d %d",
            PROTOCOL_VERSION_MAJOR,
            PROTOCOL_VERSION_MINOR,
            PROTOCOL_VERSION_TWEAK,
            protocolVersion.major,
            protocolVersion.minor,
            protocolVersion.tweak);

    bool ret;
    if (ignore)
    {
        yCWarning(CONTROLBOARD_NWC_YARP, "Ignoring error but please update YARP or the remotecontrolboard implementation");
        ret = true;
    }
    else
    {
        yCError(CONTROLBOARD_NWC_YARP, "Please update YARP or the remotecontrolboard implementation");
        ret = false;
    }

    return ret;
}

bool ControlBoard_nwc_yarp::open(Searchable& config)
{
    remote = config.find("remote").asString();
    local = config.find("local").asString();

    if (config.check("timeout"))
    {
        extendedIntputStatePort.setTimeout(config.find("timeout").asFloat64());
    }
    // check the Qos perefernces if available (local and remote)
    yarp::os::QosStyle localQos;
    if (config.check("local_qos")) {
        Bottle& qos = config.findGroup("local_qos");
        if (qos.check("thread_priority")) {
            localQos.setThreadPriority(qos.find("thread_priority").asInt32());
        }
        if (qos.check("thread_policy")) {
            localQos.setThreadPolicy(qos.find("thread_policy").asInt32());
        }
        if (qos.check("packet_priority")) {
            localQos.setPacketPriority(qos.find("packet_priority").asString());
        }
    }

    yarp::os::QosStyle remoteQos;
    if (config.check("remote_qos")) {
        Bottle& qos = config.findGroup("remote_qos");
        if (qos.check("thread_priority")) {
            remoteQos.setThreadPriority(qos.find("thread_priority").asInt32());
        }
        if (qos.check("thread_policy")) {
            remoteQos.setThreadPolicy(qos.find("thread_policy").asInt32());
        }
        if (qos.check("packet_priority")) {
            remoteQos.setPacketPriority(qos.find("packet_priority").asString());
        }
    }

    bool writeStrict_isFound = config.check("writeStrict");
    if(writeStrict_isFound)
    {
        Value &gotStrictVal = config.find("writeStrict");
        if(gotStrictVal.asString() == "on")
        {
            writeStrict_singleJoint = true;
            writeStrict_moreJoints  = true;
            yCInfo(CONTROLBOARD_NWC_YARP, "RemoteControlBoard is ENABLING the writeStrict option for all commands");
        }
        else if(gotStrictVal.asString() == "off")
        {
            writeStrict_singleJoint = false;
            writeStrict_moreJoints  = false;
            yCInfo(CONTROLBOARD_NWC_YARP, "RemoteControlBoard is DISABLING the writeStrict opition for all commands");
        } else {
            yCError(CONTROLBOARD_NWC_YARP, "Found writeStrict opition with wrong value. Accepted options are 'on' or 'off'");
        }
    }

    if (local=="") {
        yCError(CONTROLBOARD_NWC_YARP, "Problem connecting to remote controlboard, 'local' port prefix not given");
        return false;
    }

    if (remote=="") {
        yCError(CONTROLBOARD_NWC_YARP, "Problem connecting to remote controlboard, 'remote' port name not given");
        return false;
    }

    std::string carrier =
        config.check("carrier",
        Value("udp"),
        "default carrier for streaming robot state").asString();

    bool portProblem = false;
    if (local != "") {
        std::string s1 = local;
        s1 += "/rpc:o";
        if (!rpc_p.open(s1)) { portProblem = true; }
        s1 = local;
        s1 += "/command:o";
        if (!command_p.open(s1)) { portProblem = true; }
        s1 = local;
        s1 += "/stateExt:i";
        if (!extendedIntputStatePort.open(s1)) { portProblem = true; }
        if (!portProblem)
        {
            extendedIntputStatePort.useCallback();
        }
    }

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    if (!m_iControlModeMsgsRPC.yarp().attachAsClient(rpc_p))
    {
        yCError(CONTROLBOARD_NWC_YARP, "Error! Cannot attach the port as a client");
        return false;
    }
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    bool connectionProblem = false;
    if (remote != "" && !portProblem)
    {
        std::string s1 = remote;
        s1 += "/rpc:i";
        std::string s2 = local;
        s2 += "/rpc:o";
        bool ok = false;
        // RPC port needs to be tcp, therefore no carrier option is added here
        // ok=Network::connect(s2.c_str(), s1.c_str());         //This doesn't take into consideration possible YARP_PORT_PREFIX on local ports
        // ok=Network::connect(rpc_p.getName(), s1.c_str());    //This should work also with YARP_PORT_PREFIX because getting back the name of the port will return the modified name
        ok=rpc_p.addOutput(s1);                         //This works because we are manipulating only remote side and let yarp handle the local side
        if (!ok) {
            yCError(CONTROLBOARD_NWC_YARP, "Problem connecting to %s, is the remote device available?", s1.c_str());
            connectionProblem = true;
        }

        s1 = remote;
        s1 += "/command:i";
        s2 = local;
        s2 += "/command:o";
        //ok = Network::connect(s2.c_str(), s1.c_str(), carrier);
        // ok=Network::connect(command_p.getName(), s1.c_str(), carrier); //doesn't take into consideration possible YARP_PORT_PREFIX on local ports
        ok = command_p.addOutput(s1, carrier);
        if (!ok) {
            yCError(CONTROLBOARD_NWC_YARP, "Problem connecting to %s, is the remote device available?", s1.c_str());
            connectionProblem = true;
        }
        // set the QoS preferences for the 'command' port
        if (config.check("local_qos") || config.check("remote_qos")) {
            NetworkBase::setConnectionQos(command_p.getName(), s1, localQos, remoteQos, false);
        }

        s1 = remote;
        s1 += "/stateExt:o";
        s2 = local;
        s2 += "/stateExt:i";
        // not checking return value for now since it is wip (different machines can have different compilation flags
        ok = Network::connect(s1, extendedIntputStatePort.getName(), carrier);
        if (ok)
        {
            // set the QoS preferences for the 'state' port
            if (config.check("local_qos") || config.check("remote_qos")) {
                NetworkBase::setConnectionQos(s1, extendedIntputStatePort.getName(), remoteQos, localQos, false);
            }
        }
        else
        {
            yCError(CONTROLBOARD_NWC_YARP, "Problem connecting to %s, is the remote device available?", s1.c_str());
            connectionProblem = true;
        }
    }

    if (connectionProblem||portProblem) {

        rpc_p.close();
        command_p.close();
        extendedIntputStatePort.close();
        return false;
    }

    state_buffer.setStrict(false);
    command_buffer.attach(command_p);

    if (!checkProtocolVersion(config.check("ignoreProtocolCheck")))
    {
        yCError(CONTROLBOARD_NWC_YARP) << "checkProtocolVersion failed";
        command_buffer.detach();
        rpc_p.close();
        command_p.close();
        extendedIntputStatePort.close();
        return false;
    }

    if (!isLive()) {
        if (remote!="") {
            yCError(CONTROLBOARD_NWC_YARP, "Problems with obtaining the number of controlled axes");
            command_buffer.detach();
            rpc_p.close();
            command_p.close();
            extendedIntputStatePort.close();
            return false;
        }
    }

    if (config.check("diagnostic"))
    {
        diagnosticThread = new DiagnosticThread(DIAGNOSTIC_THREAD_PERIOD);
        diagnosticThread->setOwner(&extendedIntputStatePort);
        diagnosticThread->start();
    } else {
        diagnosticThread = nullptr;
    }

    // allocate memory for helper struct
    // single joint
    last_singleJoint.jointPosition.resize(1);
    last_singleJoint.jointVelocity.resize(1);
    last_singleJoint.jointAcceleration.resize(1);
    last_singleJoint.motorPosition.resize(1);
    last_singleJoint.motorVelocity.resize(1);
    last_singleJoint.motorAcceleration.resize(1);
    last_singleJoint.torque.resize(1);
    last_singleJoint.pwmDutycycle.resize(1);
    last_singleJoint.current.resize(1);
    last_singleJoint.controlMode.resize(1);
    last_singleJoint.interactionMode.resize(1);

    // whole part  (safe here because we already got the nj
    last_wholePart.jointPosition.resize(nj);
    last_wholePart.jointVelocity.resize(nj);
    last_wholePart.jointAcceleration.resize(nj);
    last_wholePart.motorPosition.resize(nj);
    last_wholePart.motorVelocity.resize(nj);
    last_wholePart.motorAcceleration.resize(nj);
    last_wholePart.torque.resize(nj);
    last_wholePart.current.resize(nj);
    last_wholePart.pwmDutycycle.resize(nj);
    last_wholePart.controlMode.resize(nj);
    last_wholePart.interactionMode.resize(nj);
    return true;
}

bool ControlBoard_nwc_yarp::close()
{
    if (diagnosticThread!=nullptr)
    {
        diagnosticThread->stop();
        delete diagnosticThread;
    }

    rpc_p.interrupt();
    command_p.interrupt();
    extendedIntputStatePort.interrupt();

    rpc_p.close();
    command_p.close();
    extendedIntputStatePort.close();
    return true;
}

// BEGIN Helpers functions

bool ControlBoard_nwc_yarp::send1V(int v)
{
    Bottle cmd, response;
    cmd.addVocab32(v);
    bool ok=rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::send2V(int v1, int v2)
{
    Bottle cmd, response;
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    bool ok=rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::send2V1I(int v1, int v2, int axis)
{
    Bottle cmd, response;
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addInt32(axis);
    bool ok=rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::send1V1I(int v, int axis)
{
    Bottle cmd, response;
    cmd.addVocab32(v);
    cmd.addInt32(axis);
    bool ok=rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::send3V1I(int v1, int v2, int v3, int j)
{
    Bottle cmd, response;
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addVocab32(v3);
    cmd.addInt32(j);
    bool ok=rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::set1V(int code)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(code);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set1V2D(int code, double v)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(code);
    cmd.addFloat64(v);

    bool ok = rpc_p.write(cmd, response);

    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set1V1I(int code, int v)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(code);
    cmd.addInt32(v);

    bool ok = rpc_p.write(cmd, response);

    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::get1V1D(int code, double& v) const
{
    Bottle cmd;
    Bottle response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(code);

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asFloat64();

        getTimeStamp(response, lastStamp);
        return true;
    }

    return false;
}

bool ControlBoard_nwc_yarp::get1V1I(int code, int& v) const
{
    Bottle cmd;
    Bottle response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(code);

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asInt32();

        getTimeStamp(response, lastStamp);
        return true;
    }

    return false;
}

bool ControlBoard_nwc_yarp::set1V1I1D(int code, int j, double val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(code);
    cmd.addInt32(j);
    cmd.addFloat64(val);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set1V1I2D(int code, int j, double val1, double val2)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(code);
    cmd.addInt32(j);
    cmd.addFloat64(val1);
    cmd.addFloat64(val2);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set1VDA(int v, const double *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(v);
    Bottle& l = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        l.addFloat64(val[i]);
    }
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set2V1DA(int v1, int v2, const double *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    Bottle& l = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        l.addFloat64(val[i]);
    }
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set2V2DA(int v1, int v2, const double *val1, const double *val2)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    Bottle& l1 = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        l1.addFloat64(val1[i]);
    }
    Bottle& l2 = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        l2.addFloat64(val2[i]);
    }
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set1V1I1IA1DA(int v, const int len, const int *val1, const double *val2)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(v);
    cmd.addInt32(len);
    int i;
    Bottle& l1 = cmd.addList();
    for (i = 0; i < len; i++) {
        l1.addInt32(val1[i]);
    }
    Bottle& l2 = cmd.addList();
    for (i = 0; i < len; i++) {
        l2.addFloat64(val2[i]);
    }
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::set2V1I1D(int v1, int v2, int axis, double val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addInt32(axis);
    cmd.addFloat64(val);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setValWithPidType(int voc, PidControlTypeEnum type, int axis, double val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(voc);
    cmd.addVocab32(type);
    cmd.addInt32(axis);
    cmd.addFloat64(val);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setValWithPidType(int voc, PidControlTypeEnum type, const double* val_arr)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(voc);
    cmd.addVocab32(type);
    Bottle& l = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        l.addFloat64(val_arr[i]);
    }
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::getValWithPidType(int voc, PidControlTypeEnum type, int j, double *val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(voc);
    cmd.addVocab32(type);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response))
    {
        *val = response.get(2).asFloat64();
        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::getValWithPidType(int voc, PidControlTypeEnum type, double *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(voc);
    cmd.addVocab32(type);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response))
    {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        yCAssert(CONTROLBOARD_NWC_YARP, nj == l.size());
        for (size_t i = 0; i < nj; i++) {
            val[i] = l.get(i).asFloat64();
        }
        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::set2V1I(int v1, int v2, int axis)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addInt32(axis);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::get1V1I1D(int v, int j, double *val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        // ok
        *val = response.get(2).asFloat64();

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1I1I(int v, int j, int *val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        // ok
        *val = response.get(2).asInt32();

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get2V1I1D(int v1, int v2, int j, double *val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        // ok
        *val = response.get(2).asFloat64();

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get2V1I2D(int v1, int v2, int j, double *val1, double *val2)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        // ok
        *val1 = response.get(2).asFloat64();
        *val2 = response.get(3).asFloat64();

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1I2D(int code, int axis, double *v1, double *v2)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(code);
    cmd.addInt32(axis);

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        *v1 = response.get(2).asFloat64();
        *v2 = response.get(3).asFloat64();
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1I1B(int v, int j, bool &val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        val = (response.get(2).asInt32()!=0);
        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1I1IA1B(int v,  const int len, const int *val1, bool &retVal)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    cmd.addInt32(len);
    Bottle& l1 = cmd.addList();
    for (int i = 0; i < len; i++) {
        l1.addInt32(val1[i]);
    }

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        retVal = (response.get(2).asInt32()!=0);
        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get2V1I1IA1DA(int v1, int v2, const int n_joints, const int *joints, double *retVals, std::string functionName)
{
    Bottle cmd, response;
    if (!isLive()) {
        return false;
    }

    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    cmd.addInt32(n_joints);

    Bottle& l1 = cmd.addList();
    for (int i = 0; i < n_joints; i++) {
        l1.addInt32(joints[i]);
    }

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response))
    {
        int i;
        Bottle& list = *(response.get(0).asList());
        yCAssert(CONTROLBOARD_NWC_YARP, list.size() >= (size_t) n_joints)

        if (list.size() != (size_t )n_joints)
        {
            yCError(CONTROLBOARD_NWC_YARP,
                    "%s length of response does not match: expected %d, received %zu\n ",
                    functionName.c_str(),
                    n_joints ,
                    list.size() );
            return false;
        }
        else
        {
            for (i = 0; i < n_joints; i++)
            {
                retVals[i] = (double) list.get(i).asFloat64();
            }
            return true;
        }
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1B(int v, bool &val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        val = (response.get(2).asInt32()!=0);
        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1VIA(int v, int *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        yCAssert(CONTROLBOARD_NWC_YARP, nj == l.size());
        for (size_t i = 0; i < nj; i++) {
            val[i] = l.get(i).asInt32();
        }

        getTimeStamp(response, lastStamp);

        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1VDA(int v, double *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        yCAssert(CONTROLBOARD_NWC_YARP, nj == l.size());
        for (size_t i = 0; i < nj; i++) {
            val[i] = l.get(i).asFloat64();
        }

        getTimeStamp(response, lastStamp);

        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1DA(int v1, double *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v1);
    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        yCAssert(CONTROLBOARD_NWC_YARP, nj == l.size());
        for (size_t i = 0; i < nj; i++) {
            val[i] = l.get(i).asFloat64();
        }

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get2V1DA(int v1, int v2, double *val)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        yCAssert(CONTROLBOARD_NWC_YARP, nj == l.size());
        for (size_t i = 0; i < nj; i++) {
            val[i] = l.get(i).asFloat64();
        }

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get2V2DA(int v1, int v2, double *val1, double *val2)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v1);
    cmd.addVocab32(v2);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp1 = response.get(2).asList();
        if (lp1 == nullptr) {
            return false;
        }
        Bottle& l1 = *lp1;
        Bottle* lp2 = response.get(3).asList();
        if (lp2 == nullptr) {
            return false;
        }
        Bottle& l2 = *lp2;

        size_t nj1 = l1.size();
        size_t nj2 = l2.size();
       // yCAssert(CONTROLBOARD_NWC_YARP, nj == nj1);
       // yCAssert(CONTROLBOARD_NWC_YARP, nj == nj2);

        for (size_t i = 0; i < nj1; i++) {
            val1[i] = l1.get(i).asFloat64();
        }
        for (size_t i = 0; i < nj2; i++) {
            val2[i] = l2.get(i).asFloat64();
        }

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::get1V1I1S(int code, int j, std::string &name)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(code);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        name = response.get(2).asString();
        return true;
    }
    return false;
}


bool ControlBoard_nwc_yarp::get1V1I1IA1DA(int v, const int len, const int *val1, double *val2)
{
    if (!isLive()) {
        return false;
    }

    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(v);
    cmd.addInt32(len);
    Bottle &l1 = cmd.addList();
    for (int i = 0; i < len; i++) {
        l1.addInt32(val1[i]);
    }

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        Bottle* lp2 = response.get(2).asList();
        if (lp2 == nullptr) {
            return false;
        }
        Bottle& l2 = *lp2;

        size_t nj2 = l2.size();
        if(nj2 != (unsigned)len)
        {
            yCError(CONTROLBOARD_NWC_YARP, "received an answer with an unexpected number of entries!");
            return false;
        }
        for (size_t i = 0; i < nj2; i++) {
            val2[i] = l2.get(i).asFloat64();
        }

        getTimeStamp(response, lastStamp);
        return true;
    }
    return false;
}

// END Helpers functions

bool ControlBoard_nwc_yarp::getAxes(int *ax)
{
    return get1V1I(VOCAB_AXES, *ax);
}

// BEGIN IPidControl

bool ControlBoard_nwc_yarp::setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(pidtype);
    cmd.addInt32(j);
    Bottle& l = cmd.addList();
    l.addFloat64(pid.kp);
    l.addFloat64(pid.kd);
    l.addFloat64(pid.ki);
    l.addFloat64(pid.max_int);
    l.addFloat64(pid.max_output);
    l.addFloat64(pid.offset);
    l.addFloat64(pid.scale);
    l.addFloat64(pid.stiction_up_val);
    l.addFloat64(pid.stiction_down_val);
    l.addFloat64(pid.kff);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setPids(const PidControlTypeEnum& pidtype, const Pid *pids)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_PIDS);
    cmd.addVocab32(pidtype);
    Bottle& l = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        Bottle& m = l.addList();
        m.addFloat64(pids[i].kp);
        m.addFloat64(pids[i].kd);
        m.addFloat64(pids[i].ki);
        m.addFloat64(pids[i].max_int);
        m.addFloat64(pids[i].max_output);
        m.addFloat64(pids[i].offset);
        m.addFloat64(pids[i].scale);
        m.addFloat64(pids[i].stiction_up_val);
        m.addFloat64(pids[i].stiction_down_val);
        m.addFloat64(pids[i].kff);
    }

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
{
    return setValWithPidType(VOCAB_REF, pidtype, j, ref);
}

bool ControlBoard_nwc_yarp::setPidReferences(const PidControlTypeEnum& pidtype, const double *refs)
{
    return setValWithPidType(VOCAB_REFS, pidtype, refs);
}

bool ControlBoard_nwc_yarp::setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit)
{
    return setValWithPidType(VOCAB_LIM, pidtype, j, limit);
}

bool ControlBoard_nwc_yarp::setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits)
{
    return setValWithPidType(VOCAB_LIMS, pidtype, limits);
}

bool ControlBoard_nwc_yarp::getPidError(const PidControlTypeEnum& pidtype, int j, double *err)
{
    return getValWithPidType(VOCAB_ERR, pidtype, j, err);
}

bool ControlBoard_nwc_yarp::getPidErrors(const PidControlTypeEnum& pidtype, double *errs)
{
    return getValWithPidType(VOCAB_ERRS, pidtype, errs);
}

bool ControlBoard_nwc_yarp::getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(pidtype);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        pid->kp = l.get(0).asFloat64();
        pid->kd = l.get(1).asFloat64();
        pid->ki = l.get(2).asFloat64();
        pid->max_int = l.get(3).asFloat64();
        pid->max_output = l.get(4).asFloat64();
        pid->offset = l.get(5).asFloat64();
        pid->scale = l.get(6).asFloat64();
        pid->stiction_up_val = l.get(7).asFloat64();
        pid->stiction_down_val = l.get(8).asFloat64();
        pid->kff = l.get(9).asFloat64();
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::getPids(const PidControlTypeEnum& pidtype, Pid *pids)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_PIDS);
    cmd.addVocab32(pidtype);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response))
    {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        yCAssert(CONTROLBOARD_NWC_YARP, nj == l.size());
        for (size_t i = 0; i < nj; i++)
        {
            Bottle* mp = l.get(i).asList();
            if (mp == nullptr) {
                return false;
            }
            pids[i].kp = mp->get(0).asFloat64();
            pids[i].kd = mp->get(1).asFloat64();
            pids[i].ki = mp->get(2).asFloat64();
            pids[i].max_int = mp->get(3).asFloat64();
            pids[i].max_output = mp->get(4).asFloat64();
            pids[i].offset = mp->get(5).asFloat64();
            pids[i].scale = mp->get(6).asFloat64();
            pids[i].stiction_up_val = mp->get(7).asFloat64();
            pids[i].stiction_down_val = mp->get(8).asFloat64();
            pids[i].kff = mp->get(9).asFloat64();
        }
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref)
{
    return getValWithPidType(VOCAB_REF, pidtype, j, ref);
}

bool ControlBoard_nwc_yarp::getPidReferences(const PidControlTypeEnum& pidtype, double *refs)
{
    return getValWithPidType(VOCAB_REFS, pidtype, refs);
}

bool ControlBoard_nwc_yarp::getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit)
{
    return getValWithPidType(VOCAB_LIM, pidtype, j, limit);
}

bool ControlBoard_nwc_yarp::getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits)
{
    return getValWithPidType(VOCAB_LIMS, pidtype, limits);
}

bool ControlBoard_nwc_yarp::resetPid(const PidControlTypeEnum& pidtype, int j)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_RESET);
    cmd.addVocab32(pidtype);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::disablePid(const PidControlTypeEnum& pidtype, int j)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_DISABLE);
    cmd.addVocab32(pidtype);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::enablePid(const PidControlTypeEnum& pidtype, int j)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_ENABLE);
    cmd.addVocab32(pidtype);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PID);
    cmd.addVocab32(VOCAB_ENABLE);
    cmd.addVocab32(pidtype);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response))
    {
        *enabled = response.get(2).asBool();
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out)
{
    return getValWithPidType(VOCAB_OUTPUT, pidtype, j, out);
}

bool ControlBoard_nwc_yarp::getPidOutputs(const PidControlTypeEnum& pidtype, double *outs)
{
    return getValWithPidType(VOCAB_OUTPUTS, pidtype, outs);
}

bool ControlBoard_nwc_yarp::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
{
    return setValWithPidType(VOCAB_OFFSET, pidtype, j, v);
}

// END IPidControl

// BEGIN IEncoder

bool ControlBoard_nwc_yarp::resetEncoder(int j)
{
    return set1V1I(VOCAB_E_RESET, j);
}

bool ControlBoard_nwc_yarp::resetEncoders()
{
    return set1V(VOCAB_E_RESETS);
}

bool ControlBoard_nwc_yarp::setEncoder(int j, double val)
{
    return set1V1I1D(VOCAB_ENCODER, j, val);
}

bool ControlBoard_nwc_yarp::setEncoders(const double *vals)
{
    return set1VDA(VOCAB_ENCODERS, vals);
}

bool ControlBoard_nwc_yarp::getEncoder(int j, double *v)
{
    double localArrivalTime = 0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER, v, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getEncoderTimed(int j, double *v, double *t)
{
    double localArrivalTime = 0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER, v, lastStamp, localArrivalTime);
    *t=lastStamp.getTime();
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getEncoders(double *encs)
{
    double localArrivalTime = 0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_ENCODERS, encs, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();

    return ret;
}

bool ControlBoard_nwc_yarp::getEncodersTimed(double *encs, double *ts)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_ENCODERS, encs, lastStamp, localArrivalTime);
    std::fill_n(ts, nj, lastStamp.getTime());
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getEncoderSpeed(int j, double *sp)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER_SPEED, sp, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getEncoderSpeeds(double *spds)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_ENCODER_SPEEDS, spds, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getEncoderAcceleration(int j, double *acc)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER_ACCELERATION, acc, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getEncoderAccelerations(double *accs)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_ENCODER_ACCELERATIONS, accs, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

// END IEncoder

// BEGIN IRemoteVariable

bool ControlBoard_nwc_yarp::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_REMOTE_VARIABILE_INTERFACE);
    cmd.addVocab32(VOCAB_VARIABLE);
    cmd.addString(key);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response))
    {
        val = *(response.get(2).asList());
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_REMOTE_VARIABILE_INTERFACE);
    cmd.addVocab32(VOCAB_VARIABLE);
    cmd.addString(key);
    cmd.append(val);
    //std::string s = cmd.toString();
    bool ok = rpc_p.write(cmd, response);

    return CHECK_FAIL(ok, response);
}


bool ControlBoard_nwc_yarp::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_REMOTE_VARIABILE_INTERFACE);
    cmd.addVocab32(VOCAB_LIST_VARIABLES);
    bool ok = rpc_p.write(cmd, response);
    //std::string s = response.toString();
    if (CHECK_FAIL(ok, response))
    {
        *listOfKeys = *(response.get(2).asList());
        //std::string s = listOfKeys->toString();
        return true;
    }
    return false;
}

// END IRemoteVariable

// BEGIN IMotor

bool ControlBoard_nwc_yarp::getNumberOfMotors(int *num)
{
    return get1V1I(VOCAB_MOTORS_NUMBER, *num);
}

bool ControlBoard_nwc_yarp::getTemperature      (int m, double* val)
{
    return get1V1I1D(VOCAB_TEMPERATURE, m, val);
}

bool ControlBoard_nwc_yarp::getTemperatures     (double *vals)
{
    return get1VDA(VOCAB_TEMPERATURES, vals);
}

bool ControlBoard_nwc_yarp::getTemperatureLimit (int m, double* val)
{
    return get1V1I1D(VOCAB_TEMPERATURE_LIMIT, m, val);
}

bool ControlBoard_nwc_yarp::setTemperatureLimit (int m, const double val)
{
    return set1V1I1D(VOCAB_TEMPERATURE_LIMIT, m, val);
}

bool ControlBoard_nwc_yarp::getGearboxRatio(int m, double* val)
{
    return get1V1I1D(VOCAB_GEARBOX_RATIO, m, val);
}

bool ControlBoard_nwc_yarp::setGearboxRatio(int m, const double val)
{
    return set1V1I1D(VOCAB_GEARBOX_RATIO, m, val);
}

// END IMotor

// BEGIN IMotorEncoder

bool ControlBoard_nwc_yarp::resetMotorEncoder(int j)
{
    return set1V1I(VOCAB_MOTOR_E_RESET, j);
}

bool ControlBoard_nwc_yarp::resetMotorEncoders()
{
    return set1V(VOCAB_MOTOR_E_RESETS);
}

bool ControlBoard_nwc_yarp::setMotorEncoder(int j, const double val)
{
    return set1V1I1D(VOCAB_MOTOR_ENCODER, j, val);
}

bool ControlBoard_nwc_yarp::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    return set1V1I1D(VOCAB_MOTOR_CPR, m, cpr);
}

bool ControlBoard_nwc_yarp::getMotorEncoderCountsPerRevolution(int m, double *cpr)
{
     return get1V1I1D(VOCAB_MOTOR_CPR, m, cpr);
}

bool ControlBoard_nwc_yarp::setMotorEncoders(const double *vals)
{
    return set1VDA(VOCAB_MOTOR_ENCODERS, vals);
}

bool ControlBoard_nwc_yarp::getMotorEncoder(int j, double *v)
{
    double localArrivalTime = 0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_MOTOR_ENCODER, v, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncoderTimed(int j, double *v, double *t)
{
    double localArrivalTime = 0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_MOTOR_ENCODER, v, lastStamp, localArrivalTime);
    *t=lastStamp.getTime();
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncoders(double *encs)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_MOTOR_ENCODERS, encs, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();

    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncodersTimed(double *encs, double *ts)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_MOTOR_ENCODERS, encs, lastStamp, localArrivalTime);
    std::fill_n(ts, nj, lastStamp.getTime());
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncoderSpeed(int j, double *sp)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_MOTOR_ENCODER_SPEED, sp, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncoderSpeeds(double *spds)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_MOTOR_ENCODER_SPEEDS, spds, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncoderAcceleration(int j, double *acc)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_MOTOR_ENCODER_ACCELERATION, acc, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getMotorEncoderAccelerations(double *accs)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_MOTOR_ENCODER_SPEEDS, accs, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getNumberOfMotorEncoders(int *num)
{
    return get1V1I(VOCAB_MOTOR_ENCODER_NUMBER, *num);
}

// END IMotorEncoder

// BEGIN IPreciselyTimed

/**
 * Get the time stamp for the last read data
 * @return last time stamp.
 */
Stamp ControlBoard_nwc_yarp::getLastInputStamp()
{
    Stamp ret;
//    mutex.lock();
    ret = lastStamp;
//    mutex.unlock();
    return ret;
}

// END IPreciselyTimed

// BEGIN IPositionControl

bool ControlBoard_nwc_yarp::positionMove(int j, double ref)
{
    return set1V1I1D(VOCAB_POSITION_MOVE, j, ref);
}

bool ControlBoard_nwc_yarp::positionMove(const int n_joint, const int *joints, const double *refs)
{
    return set1V1I1IA1DA(VOCAB_POSITION_MOVE_GROUP, n_joint, joints, refs);
}

bool ControlBoard_nwc_yarp::positionMove(const double *refs)
{
    return set1VDA(VOCAB_POSITION_MOVES, refs);
}

bool ControlBoard_nwc_yarp::getTargetPosition(const int joint, double *ref)
{
    return get1V1I1D(VOCAB_POSITION_MOVE, joint, ref);
}

bool ControlBoard_nwc_yarp::getTargetPositions(double *refs)
{
    return get1V1DA(VOCAB_POSITION_MOVES, refs);
}

bool ControlBoard_nwc_yarp::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    return get1V1I1IA1DA(VOCAB_POSITION_MOVE_GROUP, n_joint, joints, refs);
}

bool ControlBoard_nwc_yarp::relativeMove(int j, double delta)
{
    return set1V1I1D(VOCAB_RELATIVE_MOVE, j, delta);
}

bool ControlBoard_nwc_yarp::relativeMove(const int n_joint, const int *joints, const double *refs)
{
    return set1V1I1IA1DA(VOCAB_RELATIVE_MOVE_GROUP, n_joint, joints, refs);
}

bool ControlBoard_nwc_yarp::relativeMove(const double *deltas)
{
    return set1VDA(VOCAB_RELATIVE_MOVES, deltas);
}

bool ControlBoard_nwc_yarp::checkMotionDone(int j, bool *flag)
{
    return get1V1I1B(VOCAB_MOTION_DONE, j, *flag);
}

bool ControlBoard_nwc_yarp::checkMotionDone(const int n_joint, const int *joints, bool *flag)
{
    return get1V1I1IA1B(VOCAB_MOTION_DONE_GROUP, n_joint, joints, *flag);
}

bool ControlBoard_nwc_yarp::checkMotionDone(bool *flag)
{
    return get1V1B(VOCAB_MOTION_DONES, *flag);
}

bool ControlBoard_nwc_yarp::setRefSpeed(int j, double sp)
{
    return set1V1I1D(VOCAB_REF_SPEED, j, sp);
}

bool ControlBoard_nwc_yarp::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    return set1V1I1IA1DA(VOCAB_REF_SPEED_GROUP, n_joint, joints, spds);
}

bool ControlBoard_nwc_yarp::setRefSpeeds(const double *spds)
{
    return set1VDA(VOCAB_REF_SPEEDS, spds);
}

bool ControlBoard_nwc_yarp::setRefAcceleration(int j, double acc)
{
    return set1V1I1D(VOCAB_REF_ACCELERATION, j, acc);
}

bool ControlBoard_nwc_yarp::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    return set1V1I1IA1DA(VOCAB_REF_ACCELERATION_GROUP, n_joint, joints, accs);
}

bool ControlBoard_nwc_yarp::setRefAccelerations(const double *accs)
{
    return set1VDA(VOCAB_REF_ACCELERATIONS, accs);
}

bool ControlBoard_nwc_yarp::getRefSpeed(int j, double *ref)
{
    return get1V1I1D(VOCAB_REF_SPEED, j, ref);
}

bool ControlBoard_nwc_yarp::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    return get1V1I1IA1DA(VOCAB_REF_SPEED_GROUP, n_joint, joints, spds);
}

bool ControlBoard_nwc_yarp::getRefSpeeds(double *spds)
{
    return get1VDA(VOCAB_REF_SPEEDS, spds);
}

bool ControlBoard_nwc_yarp::getRefAcceleration(int j, double *acc)
{
    return get1V1I1D(VOCAB_REF_ACCELERATION, j, acc);
}

bool ControlBoard_nwc_yarp::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    return get1V1I1IA1DA(VOCAB_REF_ACCELERATION_GROUP, n_joint, joints, accs);
}

bool ControlBoard_nwc_yarp::getRefAccelerations(double *accs)
{
    return get1VDA(VOCAB_REF_ACCELERATIONS, accs);
}

bool ControlBoard_nwc_yarp::stop(int j)
{
    return set1V1I(VOCAB_STOP, j);
}

bool ControlBoard_nwc_yarp::stop(const int len, const int *val1)
{
    if (!isLive()) {
        return false;
    }
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_STOP_GROUP);
    cmd.addInt32(len);
    int i;
    Bottle& l1 = cmd.addList();
    for (i = 0; i < len; i++) {
        l1.addInt32(val1[i]);
    }

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::stop()
{
    return set1V(VOCAB_STOPS);
}

// END IPositionControl

// BEGIN IJoint Fault
bool ControlBoard_nwc_yarp::getLastJointFault(int j, int& fault, std::string& message)
{
    Bottle cmd, response;

    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_IJOINTFAULT);
    cmd.addVocab32(VOCAB_JF_GET_JOINTFAULT);
    cmd.addInt32(j);

    bool ok = rpc_p.write(cmd, response);

    std::string ss = response.toString();

    if (CHECK_FAIL(ok, response))
    {
        fault = response.get(1).asInt32();
        message = response.get(2).asString();
        return true;
    }
    return false;
}
// END IJointFault

// BEGIN IVelocityControl

bool ControlBoard_nwc_yarp::velocityMove(int j, double v)
{
 //   return set1V1I1D(VOCAB_VELOCITY_MOVE, j, v);
 if (!isLive()) {
     return false;
 }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_VELOCITY_MOVE);
    c.head.addInt32(j);
    c.body.resize(1);
    memcpy(&(c.body[0]), &v, sizeof(double));
    command_buffer.write(writeStrict_singleJoint);
    return true;
}

bool ControlBoard_nwc_yarp::velocityMove(const double *v)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_VELOCITY_MOVES);
    c.body.resize(nj);
    memcpy(&(c.body[0]), v, sizeof(double)*nj);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

// END IVelocityControl

// BEGIN IAmplifierControl

bool ControlBoard_nwc_yarp::enableAmp(int j)
{
    return set1V1I(VOCAB_AMP_ENABLE, j);
}

bool ControlBoard_nwc_yarp::disableAmp(int j)
{
    return set1V1I(VOCAB_AMP_DISABLE, j);
}

bool ControlBoard_nwc_yarp::getAmpStatus(int *st)
{
    return get1VIA(VOCAB_AMP_STATUS, st);
}

bool ControlBoard_nwc_yarp::getAmpStatus(int j, int *st)
{
    return get1V1I1I(VOCAB_AMP_STATUS_SINGLE, j, st);
}

bool ControlBoard_nwc_yarp::setMaxCurrent(int j, double v)
{
    return set1V1I1D(VOCAB_AMP_MAXCURRENT, j, v);
}

bool ControlBoard_nwc_yarp::getMaxCurrent(int j, double *v)
{
    return get1V1I1D(VOCAB_AMP_MAXCURRENT, j, v);
}

bool ControlBoard_nwc_yarp::getNominalCurrent(int m, double *val)
{
    return get1V1I1D(VOCAB_AMP_NOMINAL_CURRENT, m, val);
}

bool ControlBoard_nwc_yarp::setNominalCurrent(int m, const double val)
{
    return set1V1I1D(VOCAB_AMP_NOMINAL_CURRENT, m, val);
}

bool ControlBoard_nwc_yarp::getPeakCurrent(int m, double *val)
{
    return get1V1I1D(VOCAB_AMP_PEAK_CURRENT, m, val);
}

bool ControlBoard_nwc_yarp::setPeakCurrent(int m, const double val)
{
    return set1V1I1D(VOCAB_AMP_PEAK_CURRENT, m, val);
}

bool ControlBoard_nwc_yarp::getPWM(int m, double* val)
{
    double localArrivalTime = 0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(m, VOCAB_PWMCONTROL_PWM_OUTPUT, val, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getPWMLimit(int m, double* val)
{
    return get1V1I1D(VOCAB_AMP_PWM_LIMIT, m, val);
}

bool ControlBoard_nwc_yarp::setPWMLimit(int m, const double val)
{
    return set1V1I1D(VOCAB_AMP_PWM_LIMIT, m, val);
}

bool ControlBoard_nwc_yarp::getPowerSupplyVoltage(int m, double* val)
{
    return get1V1I1D(VOCAB_AMP_VOLTAGE_SUPPLY, m, val);
}

// END IAmplifierControl

// BEGIN IControlLimits

bool ControlBoard_nwc_yarp::setLimits(int axis, double min, double max)
{
    return set1V1I2D(VOCAB_LIMITS, axis, min, max);
}

bool ControlBoard_nwc_yarp::getLimits(int axis, double *min, double *max)
{
    return get1V1I2D(VOCAB_LIMITS, axis, min, max);
}

bool ControlBoard_nwc_yarp::setVelLimits(int axis, double min, double max)
{
    return set1V1I2D(VOCAB_VEL_LIMITS, axis, min, max);
}

bool ControlBoard_nwc_yarp::getVelLimits(int axis, double *min, double *max)
{
    return get1V1I2D(VOCAB_VEL_LIMITS, axis, min, max);
}

// END IControlLimits

// BEGIN IAxisInfo

bool ControlBoard_nwc_yarp::getAxisName(int j, std::string& name)
{
    return get1V1I1S(VOCAB_INFO_NAME, j, name);
}

bool ControlBoard_nwc_yarp::getJointType(int j, yarp::dev::JointTypeEnum& type)
{
    return get1V1I1I(VOCAB_INFO_TYPE, j, (int*)&type);
}

// END IAxisInfo

// BEGIN IControlCalibration
bool ControlBoard_nwc_yarp::calibrateRobot()
{
    return send1V(VOCAB_CALIBRATE);
}

bool ControlBoard_nwc_yarp::abortCalibration()
{
    return send1V(VOCAB_ABORTCALIB);
}

bool ControlBoard_nwc_yarp::abortPark()
{
    return send1V(VOCAB_ABORTPARK);
}

bool ControlBoard_nwc_yarp::park(bool wait)
{
    return send1V(VOCAB_PARK);
}

bool ControlBoard_nwc_yarp::calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3)
{
    Bottle cmd, response;

    cmd.addVocab32(VOCAB_CALIBRATE_JOINT);
    cmd.addInt32(j);
    cmd.addInt32(ui);
    cmd.addFloat64(v1);
    cmd.addFloat64(v2);
    cmd.addFloat64(v3);

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::setCalibrationParameters(int j, const CalibrationParameters& params)
{
    Bottle cmd, response;

    cmd.addVocab32(VOCAB_CALIBRATE_JOINT_PARAMS);
    cmd.addInt32(j);
    cmd.addInt32(params.type);
    cmd.addFloat64(params.param1);
    cmd.addFloat64(params.param2);
    cmd.addFloat64(params.param3);
    cmd.addFloat64(params.param4);

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response)) {
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::calibrationDone(int j)
{
    return send1V1I(VOCAB_CALIBRATE_DONE, j);
}

// END IControlCalibration

// BEGIN ITorqueControl

bool ControlBoard_nwc_yarp::getRefTorque(int j, double *t)
{
    return get2V1I1D(VOCAB_TORQUE, VOCAB_REF, j, t);
}

bool ControlBoard_nwc_yarp::getRefTorques(double *t)
{
    return get2V1DA(VOCAB_TORQUE, VOCAB_REFS, t);
}

bool ControlBoard_nwc_yarp::setRefTorques(const double *t)
{
    //Now we use streaming instead of rpc
    //return set2V1DA(VOCAB_TORQUE, VOCAB_REFS, t);
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_TORQUES_DIRECTS);

    c.body.resize(nj);

    memcpy(c.body.data(), t, sizeof(double) * nj);

    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::setRefTorque(int j, double v)
{
    //return set2V1I1D(VOCAB_TORQUE, VOCAB_REF, j, v);
    // use the streaming port!
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    // in streaming port only SET command can be sent, so it is implicit
    c.head.addVocab32(VOCAB_TORQUES_DIRECT);
    c.head.addInt32(j);

    c.body.clear();
    c.body.resize(1);
    c.body[0] = v;
    command_buffer.write(writeStrict_singleJoint);
    return true;
}

bool ControlBoard_nwc_yarp::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    //return set2V1I1D(VOCAB_TORQUE, VOCAB_REF, j, v);
    // use the streaming port!
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    // in streaming port only SET command can be sent, so it is implicit
    c.head.addVocab32(VOCAB_TORQUES_DIRECT_GROUP);
    c.head.addInt32(n_joint);
    Bottle &jointList = c.head.addList();
    for (int i = 0; i < n_joint; i++) {
        jointList.addInt32(joints[i]);
    }
    c.body.resize(n_joint);
    memcpy(&(c.body[0]), t, sizeof(double)*n_joint);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::setMotorTorqueParams(int j, const MotorTorqueParameters params)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_TORQUE);
    cmd.addVocab32(VOCAB_MOTOR_PARAMS);
    cmd.addInt32(j);
    Bottle& b = cmd.addList();
    b.addFloat64(params.bemf);
    b.addFloat64(params.bemf_scale);
    b.addFloat64(params.ktau);
    b.addFloat64(params.ktau_scale);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::getMotorTorqueParams(int j, MotorTorqueParameters *params)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_TORQUE);
    cmd.addVocab32(VOCAB_MOTOR_PARAMS);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        if (l.size() != 4)
        {
            yCError(CONTROLBOARD_NWC_YARP, "getMotorTorqueParams return value not understood, size!=4");
            return false;
        }
        params->bemf        = l.get(0).asFloat64();
        params->bemf_scale  = l.get(1).asFloat64();
        params->ktau        = l.get(2).asFloat64();
        params->ktau_scale  = l.get(3).asFloat64();
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::getTorque(int j, double *t)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_TRQ, t, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getTorques(double *t)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_TRQS, t, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getTorqueRange(int j, double *min, double* max)
{
    return get2V1I2D(VOCAB_TORQUE, VOCAB_RANGE, j, min, max);
}

bool ControlBoard_nwc_yarp::getTorqueRanges(double *min, double *max)
{
    return get2V2DA(VOCAB_TORQUE, VOCAB_RANGES, min, max);
}

// END ITorqueControl

// BEGIN IImpedanceControl

bool ControlBoard_nwc_yarp::getImpedance(int j, double *stiffness, double *damping)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_IMPEDANCE);
    cmd.addVocab32(VOCAB_IMP_PARAM);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        *stiffness = l.get(0).asFloat64();
        *damping   = l.get(1).asFloat64();
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::getImpedanceOffset(int j, double *offset)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_IMPEDANCE);
    cmd.addVocab32(VOCAB_IMP_OFFSET);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        *offset    = l.get(0).asFloat64();
        return true;
    }
    return false;
}

bool ControlBoard_nwc_yarp::setImpedance(int j, double stiffness, double damping)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_IMPEDANCE);
    cmd.addVocab32(VOCAB_IMP_PARAM);
    cmd.addInt32(j);

    Bottle& b = cmd.addList();
    b.addFloat64(stiffness);
    b.addFloat64(damping);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setImpedanceOffset(int j, double offset)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_IMPEDANCE);
    cmd.addVocab32(VOCAB_IMP_OFFSET);
    cmd.addInt32(j);

    Bottle& b = cmd.addList();
    b.addFloat64(offset);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_IMPEDANCE);
    cmd.addVocab32(VOCAB_LIMITS);
    cmd.addInt32(j);
    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response)) {
        Bottle* lp = response.get(2).asList();
        if (lp == nullptr) {
            return false;
        }
        Bottle& l = *lp;
        *min_stiff    = l.get(0).asFloat64();
        *max_stiff    = l.get(1).asFloat64();
        *min_damp     = l.get(2).asFloat64();
        *max_damp     = l.get(3).asFloat64();
        return true;
    }
    return false;
}

// END IImpedanceControl

// BEGIN IControlMode

bool ControlBoard_nwc_yarp::getControlMode(int j, int *mode)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_CM_CONTROL_MODE, mode, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getControlModes(int *modes)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_CM_CONTROL_MODES, modes, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getControlModes(const int n_joint, const int *joints, int *modes)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_CM_CONTROL_MODES, last_wholePart.controlMode.data(), lastStamp, localArrivalTime);
    if(ret)
    {
        for (int i = 0; i < n_joint; i++) {
            modes[i] = last_wholePart.controlMode[joints[i]];
        }
    } else {
        ret = false;
    }

    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::setControlMode(const int j, const int mode)
{
    if (!isLive()) {
        return false;
    }

    auto ret = m_iControlModeMsgsRPC.setControlMode_singlej(j,mode);
    return ret.retvalue;
}

bool ControlBoard_nwc_yarp::setControlModes(const int n_joint, const int *joints, int *modes)
{
    if (!isLive()) {
        return false;
    }

    std::vector<int32_t> o_modes  (n_joint);
    std::vector<int16_t> o_joints (n_joint);
    for (size_t i = 0; i < n_joint; i++) { o_modes[i] =  modes[i]; }
    for (size_t i = 0; i < n_joint; i++) { o_joints[i] = joints[i]; }
    auto ret = m_iControlModeMsgsRPC.setControlModes_somej(o_joints, o_modes);
    return ret.retvalue;
}

bool ControlBoard_nwc_yarp::setControlModes(int *modes)
{
    if (!isLive()) {
        return false;
    }

    std::vector<int32_t> o_modes(nj);
    for (size_t i = 0; i < nj; i++) {o_modes[i]=modes[i];}
    auto ret = m_iControlModeMsgsRPC.setControlModes_allj(o_modes);
    return ret.retvalue;
}

// END IControlMode

// BEGIN IPositionDirect

bool ControlBoard_nwc_yarp::setPosition(int j, double ref)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_POSITION_DIRECT);
    c.head.addInt32(j);
    c.body.resize(1);
    memcpy(&(c.body[0]), &ref, sizeof(double));
    command_buffer.write(writeStrict_singleJoint);
    return true;
}

bool ControlBoard_nwc_yarp::setPositions(const int n_joint, const int *joints, const double *refs)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_POSITION_DIRECT_GROUP);
    c.head.addInt32(n_joint);
    Bottle &jointList = c.head.addList();
    for (int i = 0; i < n_joint; i++) {
        jointList.addInt32(joints[i]);
    }
    c.body.resize(n_joint);
    memcpy(&(c.body[0]), refs, sizeof(double)*n_joint);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::setPositions(const double *refs)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_POSITION_DIRECTS);
    c.body.resize(nj);
    memcpy(&(c.body[0]), refs, sizeof(double)*nj);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::getRefPosition(const int joint, double* ref)
{
    return get1V1I1D(VOCAB_POSITION_DIRECT, joint, ref);
}

bool ControlBoard_nwc_yarp::getRefPositions(double* refs)
{
    return get1V1DA(VOCAB_POSITION_DIRECTS, refs);
}

bool ControlBoard_nwc_yarp::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    return get1V1I1IA1DA(VOCAB_POSITION_DIRECT_GROUP, n_joint, joints, refs);
}

// END IPositionDirect

// BEGIN IVelocityControl

bool ControlBoard_nwc_yarp::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    // streaming port
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_VELOCITY_MOVE_GROUP);
    c.head.addInt32(n_joint);
    Bottle &jointList = c.head.addList();
    for (int i = 0; i < n_joint; i++) {
        jointList.addInt32(joints[i]);
    }
    c.body.resize(n_joint);
    memcpy(&(c.body[0]), spds, sizeof(double)*n_joint);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::getRefVelocity(const int joint, double* vel)
{
    return get1V1I1D(VOCAB_VELOCITY_MOVE, joint, vel);
}

bool ControlBoard_nwc_yarp::getRefVelocities(double* vels)
{
    return get1VDA(VOCAB_VELOCITY_MOVES, vels);
}

bool ControlBoard_nwc_yarp::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    return get1V1I1IA1DA(VOCAB_VELOCITY_MOVE_GROUP, n_joint, joints, vels);
}

// END IVelocityControl

// BEGIN IInteractionMode

bool ControlBoard_nwc_yarp::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(axis, VOCAB_INTERACTION_MODE, (int*) mode, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    double localArrivalTime=0.0;

    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_INTERACTION_MODES, last_wholePart.interactionMode.data(), lastStamp, localArrivalTime);
    if(ret)
    {
        for (int i = 0; i < n_joints; i++) {
            modes[i] = (yarp::dev::InteractionModeEnum)last_wholePart.interactionMode[joints[i]];
        }
    } else {
        ret = false;
    }

    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_INTERACTION_MODES, (int*) modes, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    Bottle cmd, response;
    if (!isLive()) {
        return false;
    }

    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_INTERFACE_INTERACTION_MODE);
    cmd.addVocab32(VOCAB_INTERACTION_MODE);
    cmd.addInt32(axis);
    cmd.addVocab32(mode);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    Bottle cmd, response;
    if (!isLive()) {
        return false;
    }

    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_INTERFACE_INTERACTION_MODE);
    cmd.addVocab32(VOCAB_INTERACTION_MODE_GROUP);
    cmd.addInt32(n_joints);

    Bottle& l1 = cmd.addList();
    for (int i = 0; i < n_joints; i++) {
        l1.addInt32(joints[i]);
    }

    Bottle& l2 = cmd.addList();
    for (int i = 0; i < n_joints; i++)
    {
        l2.addVocab32(modes[i]);
    }
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    Bottle cmd, response;
    if (!isLive()) {
        return false;
    }

    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_INTERFACE_INTERACTION_MODE);
    cmd.addVocab32(VOCAB_INTERACTION_MODES);

    Bottle& l1 = cmd.addList();
    for (size_t i = 0; i < nj; i++) {
        l1.addVocab32(modes[i]);
    }

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

// END IInteractionMode

// BEGIN IRemoteCalibrator

bool ControlBoard_nwc_yarp::isCalibratorDevicePresent(bool *isCalib)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_REMOTE_CALIBRATOR_INTERFACE);
    cmd.addVocab32(VOCAB_IS_CALIBRATOR_PRESENT);
    bool ok = rpc_p.write(cmd, response);
    if(ok) {
        *isCalib = response.get(2).asInt32()!=0;
    } else {
        *isCalib = false;
    }
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::calibrateSingleJoint(int j)
{
    return set2V1I(VOCAB_REMOTE_CALIBRATOR_INTERFACE, VOCAB_CALIBRATE_SINGLE_JOINT, j);
}

bool ControlBoard_nwc_yarp::calibrateWholePart()
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_REMOTE_CALIBRATOR_INTERFACE);
    cmd.addVocab32(VOCAB_CALIBRATE_WHOLE_PART);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::homingSingleJoint(int j)
{
    return set2V1I(VOCAB_REMOTE_CALIBRATOR_INTERFACE, VOCAB_HOMING_SINGLE_JOINT, j);
}

bool ControlBoard_nwc_yarp::homingWholePart()
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_REMOTE_CALIBRATOR_INTERFACE);
    cmd.addVocab32(VOCAB_HOMING_WHOLE_PART);
    bool ok = rpc_p.write(cmd, response);
    yCDebug(CONTROLBOARD_NWC_YARP) << "Sent homing whole part message";
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::parkSingleJoint(int j, bool _wait)
{
    return set2V1I(VOCAB_REMOTE_CALIBRATOR_INTERFACE, VOCAB_PARK_SINGLE_JOINT, j);
}

bool ControlBoard_nwc_yarp::parkWholePart()
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_REMOTE_CALIBRATOR_INTERFACE);
    cmd.addVocab32(VOCAB_PARK_WHOLE_PART);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::quitCalibrate()
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_REMOTE_CALIBRATOR_INTERFACE);
    cmd.addVocab32(VOCAB_QUIT_CALIBRATE);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool ControlBoard_nwc_yarp::quitPark()
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_SET);
    cmd.addVocab32(VOCAB_REMOTE_CALIBRATOR_INTERFACE);
    cmd.addVocab32(VOCAB_QUIT_PARK);
    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

// END IRemoteCalibrator

// BEGIN ICurrentControl

bool ControlBoard_nwc_yarp::getRefCurrents(double *t)
{
    return get2V1DA(VOCAB_CURRENTCONTROL_INTERFACE, VOCAB_CURRENT_REFS, t);
}

bool ControlBoard_nwc_yarp::getRefCurrent(int j, double *t)
{
    return get2V1I1D(VOCAB_CURRENTCONTROL_INTERFACE, VOCAB_CURRENT_REF, j, t);
}

bool ControlBoard_nwc_yarp::setRefCurrents(const double *refs)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_CURRENTCONTROL_INTERFACE);
    c.head.addVocab32(VOCAB_CURRENT_REFS);
    c.body.resize(nj);
    memcpy(&(c.body[0]), refs, sizeof(double)*nj);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::setRefCurrent(int j, double ref)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_CURRENTCONTROL_INTERFACE);
    c.head.addVocab32(VOCAB_CURRENT_REF);
    c.head.addInt32(j);
    c.body.resize(1);
    memcpy(&(c.body[0]), &ref, sizeof(double));
    command_buffer.write(writeStrict_singleJoint);
    return true;
}

bool ControlBoard_nwc_yarp::setRefCurrents(const int n_joint, const int *joints, const double *refs)
{
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_CURRENTCONTROL_INTERFACE);
    c.head.addVocab32(VOCAB_CURRENT_REF_GROUP);
    c.head.addInt32(n_joint);
    Bottle &jointList = c.head.addList();
    for (int i = 0; i < n_joint; i++) {
        jointList.addInt32(joints[i]);
    }
    c.body.resize(n_joint);
    memcpy(&(c.body[0]), refs, sizeof(double)*n_joint);
    command_buffer.write(writeStrict_moreJoints);
    return true;
}

bool ControlBoard_nwc_yarp::getCurrents(double *vals)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_AMP_CURRENTS, vals, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getCurrent(int j, double *val)
{
    double localArrivalTime=0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_AMP_CURRENT, val, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getCurrentRange(int j, double *min, double *max)
{
    return get2V1I2D(VOCAB_CURRENTCONTROL_INTERFACE, VOCAB_CURRENT_RANGE, j, min, max);
}

bool ControlBoard_nwc_yarp::getCurrentRanges(double *min, double *max)
{
    return get2V2DA(VOCAB_CURRENTCONTROL_INTERFACE, VOCAB_CURRENT_RANGES, min, max);
}

// END ICurrentControl

// BEGIN IPWMControl
bool ControlBoard_nwc_yarp::setRefDutyCycle(int j, double v)
{
    // using the streaming port
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    // in streaming port only SET command can be sent, so it is implicit
    c.head.addVocab32(VOCAB_PWMCONTROL_INTERFACE);
    c.head.addVocab32(VOCAB_PWMCONTROL_REF_PWM);
    c.head.addInt32(j);

    c.body.clear();
    c.body.resize(1);
    c.body[0] = v;
    command_buffer.write(writeStrict_singleJoint);
    return true;
}

bool ControlBoard_nwc_yarp::setRefDutyCycles(const double *v)
{
    // using the streaming port
    if (!isLive()) {
        return false;
    }
    CommandMessage& c = command_buffer.get();
    c.head.clear();
    c.head.addVocab32(VOCAB_PWMCONTROL_INTERFACE);
    c.head.addVocab32(VOCAB_PWMCONTROL_REF_PWMS);

    c.body.resize(nj);

    memcpy(&(c.body[0]), v, sizeof(double)*nj);

    command_buffer.write(writeStrict_moreJoints);

    return true;
}

bool ControlBoard_nwc_yarp::getRefDutyCycle(int j, double *ref)
{
    Bottle cmd, response;
    cmd.addVocab32(VOCAB_GET);
    cmd.addVocab32(VOCAB_PWMCONTROL_INTERFACE);
    cmd.addVocab32(VOCAB_PWMCONTROL_REF_PWM);
    cmd.addInt32(j);
    response.clear();

    bool ok = rpc_p.write(cmd, response);

    if (CHECK_FAIL(ok, response))
    {
        // ok
        *ref = response.get(2).asFloat64();

        getTimeStamp(response, lastStamp);
        return true;
    } else {
        return false;
    }
}

bool ControlBoard_nwc_yarp::getRefDutyCycles(double *refs)
{
    return get2V1DA(VOCAB_PWMCONTROL_INTERFACE, VOCAB_PWMCONTROL_REF_PWMS, refs);
}

bool ControlBoard_nwc_yarp::getDutyCycle(int j, double *out)
{
    double localArrivalTime = 0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastSingle(j, VOCAB_PWMCONTROL_PWM_OUTPUT, out, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}

bool ControlBoard_nwc_yarp::getDutyCycles(double *outs)
{
    double localArrivalTime = 0.0;
    extendedPortMutex.lock();
    bool ret = extendedIntputStatePort.getLastVector(VOCAB_PWMCONTROL_PWM_OUTPUTS, outs, lastStamp, localArrivalTime);
    extendedPortMutex.unlock();
    return ret;
}
// END IPWMControl
