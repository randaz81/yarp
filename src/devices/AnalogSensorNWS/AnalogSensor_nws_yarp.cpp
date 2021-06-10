/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "AnalogSensor_nws_yarp.h"
#include <sstream>
#include <iostream>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace std;


YARP_LOG_COMPONENT(ANALOGSENSOR_NWS_YARPP, "yarp.devices.AnalogSensor_nws_yarp")

/**
  * Handler of the rpc port related to an analog sensor.
  * Manage the calibration command received on the rpc port.
 **/
class AnalogServerHandler :
        public yarp::os::PortReader
{
    yarp::dev::IAnalogSensor* is;   // analog sensor to calibrate, when required
    yarp::os::Port rpcPort;         // rpc port related to the analog sensor

public:
    AnalogServerHandler(const char* n);
    ~AnalogServerHandler();

    void setInterface(yarp::dev::IAnalogSensor *is);

    bool _handleIAnalog(yarp::os::Bottle &cmd, yarp::os::Bottle &reply);

    bool read(yarp::os::ConnectionReader& connection) override;
};


/**
  * A yarp port that output data read from an analog sensor.
  * It contains information about which data of the analog sensor are sent
  * on the port, i.e. an offset and a length.
  */
class AnalogPortEntry
{
public:
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    std::string port_name;      // the complete name of the port
    int offset;                 // an offset, the port is mapped starting from this channel
    int length;                 // length of the output vector of the port (-1 for max length)
    AnalogPortEntry();
    AnalogPortEntry(const AnalogPortEntry &alt);
    AnalogPortEntry &operator =(const AnalogPortEntry &alt);
};


/**
  * Handler of the rpc port related to an analog sensor.
  * Manage the calibration command received on the rpc port.
  **/

AnalogServerHandler::AnalogServerHandler(const char* n) : is(nullptr)
{
    rpcPort.open(n);
    rpcPort.setReader(*this);
}

AnalogServerHandler::~AnalogServerHandler()
{
    rpcPort.close();
    is = nullptr;
}

void AnalogServerHandler::setInterface(yarp::dev::IAnalogSensor *is)
{
    this->is = is;
}

bool AnalogServerHandler::_handleIAnalog(yarp::os::Bottle &cmd, yarp::os::Bottle &reply)
{
    if (is==nullptr)
      return false;

    const size_t msgsize=cmd.size();
    int ret=IAnalogSensor::AS_ERROR;

    int code=cmd.get(1).asVocab();
    switch (code)
    {
      case VOCAB_CALIBRATE:
          if (msgsize==2)
          {   ret=is->calibrateSensor();  }
          else if (msgsize>2)
          {
              size_t offset=2;
              Vector v(msgsize-offset);
              for (unsigned int i=0; i<v.size(); i++)
              {
                 v[i]=cmd.get(i+offset).asFloat64();
              }
              ret=is->calibrateSensor(v);
          }
      break;
      case VOCAB_CALIBRATE_CHANNEL:
          if (msgsize==3)
          {
              int ch=cmd.get(2).asInt32();
              ret=is->calibrateChannel(ch);
          }
          else if (msgsize==4)
          {
              int ch=cmd.get(2).asInt32();
              double v=cmd.get(3).asFloat64();
              ret=is->calibrateChannel(ch, v);
          }
      break;
      default:
          return false;
    }

    reply.addInt32(ret);
    return true;
}

bool AnalogServerHandler::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok=in.read(connection);
    if (!ok) return false;

    // parse in, prepare out
    int code = in.get(0).asVocab();
    bool ret=false;
    if (code==VOCAB_IANALOG)
    {
        ret=_handleIAnalog(in, out);
    }

    if (!ret)
    {
        out.clear();
        out.addVocab(VOCAB_FAILED);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender!=nullptr) {
        out.write(*returnToSender);
    }
    return true;
}


/**
  * A yarp port that output data read from an analog sensor.
  * It contains information about which data of the analog sensor are sent
  * on the port, i.e. an offset and a length.
  */

AnalogPortEntry::AnalogPortEntry() :
    offset(0),
    length(0)
{}

AnalogPortEntry::AnalogPortEntry(const AnalogPortEntry &alt)
{
    this->length = alt.length;
    this->offset = alt.offset;
    this->port_name = alt.port_name;
}

AnalogPortEntry &AnalogPortEntry::operator =(const AnalogPortEntry &alt)
{
    this->length = alt.length;
    this->offset = alt.offset;
    this->port_name = alt.port_name;
    return *this;
}

 // closing anonymous namespace


/**
  * It reads the data from an analog sensor and sends them on one or more ports.
  * It creates one rpc port and its related handler for every output port.
  */

bool AnalogSensor_nws_yarp::createPort(const char* name, double per)
{
    analogSensor_p=nullptr;
    analogPorts.resize(1);
    analogPorts[0].offset = 0;
    analogPorts[0].length = -1; // max length
    analogPorts[0].port_name = std::string(name);
    setHandlers();
    setPeriod(per);
    return true;
}

bool AnalogSensor_nws_yarp::createPorts(const std::vector<AnalogPortEntry>& _analogPorts, double per)
{
    analogSensor_p=nullptr;
    this->analogPorts=_analogPorts;
    setHandlers();
    setPeriod(per);
    return true;
}

AnalogSensor_nws_yarp::AnalogSensor_nws_yarp() :
        PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}

AnalogSensor_nws_yarp::~AnalogSensor_nws_yarp()
{
    threadRelease();
    close();
    _period = DEFAULT_THREAD_PERIOD;
    analogSensor_p = nullptr;
}

void AnalogSensor_nws_yarp::setHandlers()
{
    for(auto& analogPort : analogPorts)
    {
        std::string rpcPortName = analogPort.port_name;
        rpcPortName += "/rpc:i";
        auto* ash = new AnalogServerHandler(rpcPortName.c_str());
        handlers.push_back(ash);
    }
}

void AnalogSensor_nws_yarp::removeHandlers()
{
    for(auto& handler : handlers)
    {
        if (handler != nullptr)
        {
            delete handler;
            handler = nullptr;
        }
    }
    handlers.clear();
}

bool AnalogSensor_nws_yarp::openAndAttachSubDevice(Searchable &prop)
{
    Property p;
    subDeviceOwned = new PolyDriver;
    p.fromString(prop.toString());

//     p.setMonitor(prop.getMonitor(), "subdevice"); // pass on any monitoring
    p.unput("device");
    p.put("device", prop.find("subdevice").asString());  // subdevice was already checked before

    // if errors occurred during open, quit here.
    yCDebug(ANALOGSENSOR_NWS_YARPP, "opening AnalogSensor_nws_yarp subdevice...");
    subDeviceOwned->open(p);

    if (!subDeviceOwned->isValid())
    {
        yCError(ANALOGSENSOR_NWS_YARPP, "opening AnalogSensor_nws_yarp subdevice... FAILED\n");
        return false;
    }

    subDeviceOwned->view(analogSensor_p);

    if (analogSensor_p == nullptr)
    {
        yCError(ANALOGSENSOR_NWS_YARPP, "Opening IAnalogSensor interface of AnalogSensor_nws_yarp subdevice... FAILED\n");
        return false;
    }

    int chNum = analogSensor_p->getChannels();

    if (chNum <= 0)
    {
        yCError(ANALOGSENSOR_NWS_YARPP, "Calling analog sensor has invalid channels number %d.\n", chNum);
        return false;
    }

    attach(analogSensor_p);
    PeriodicThread::setPeriod(_period);
    return PeriodicThread::start();
}


bool AnalogSensor_nws_yarp::openDeferredAttach(yarp::os::Searchable &prop)
{
    // nothing to do here?
    if( (subDeviceOwned != nullptr) || (ownDevices == true) )
        yCError(ANALOGSENSOR_NWS_YARPP) << "AnalogSensor_nws_yarp: something wrong with the initialization.";
    return true;
}


/**
  * Specify which analog sensor this thread has to read from.
  */

bool AnalogSensor_nws_yarp::attachAll(const PolyDriverList &analog2attach)
{
    //check if we already instantiated a subdevice previously
    if (ownDevices)
        return false;

    if (analog2attach.size() != 1)
    {
        yCError(ANALOGSENSOR_NWS_YARPP, "cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach=analog2attach[0]->poly;

    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(analogSensor_p);
    }

    if(nullptr == analogSensor_p)
    {
        yCError(ANALOGSENSOR_NWS_YARPP, "subdevice passed to attach method is invalid");
        return false;
    }
    attach(analogSensor_p);
    PeriodicThread::setPeriod(_period);
    return PeriodicThread::start();
}

bool AnalogSensor_nws_yarp::detachAll()
{
    //check if we already instantiated a subdevice previously
    if (ownDevices)
        return false;

    analogSensor_p = nullptr;
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        if(handlers[i] != nullptr)
            handlers[i]->setInterface(analogSensor_p);
    }
    return true;
}

void AnalogSensor_nws_yarp::attach(yarp::dev::IAnalogSensor *s)
{
    analogSensor_p=s;
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        handlers[i]->setInterface(analogSensor_p);
    }
    //Resize vector of read data to avoid further allocation of memory
    //as long as the number of channels does not change
    lastDataRead.resize((size_t)analogSensor_p->getChannels(),0.0);
}

void AnalogSensor_nws_yarp::detach()
{
    // Set interface to NULL
    analogSensor_p = nullptr;
    for(unsigned int i=0; i<analogPorts.size(); i++)
    {
        handlers[i]->setInterface(analogSensor_p);
    }
}

bool AnalogSensor_nws_yarp::threadInit()
{
    for(auto& analogPort : analogPorts)
    {
        // open data port
        if (!analogPort.port.open(analogPort.port_name))
           {
               yCError(ANALOGSENSOR_NWS_YARPP, "failed to open port %s", analogPort.port_name.c_str());
               return false;
           }
    }
    return true;
}

void AnalogSensor_nws_yarp::setId(const std::string &id)
{
    sensorId=id;
}

std::string AnalogSensor_nws_yarp::getId()
{
    return sensorId;
}

bool AnalogSensor_nws_yarp::open(yarp::os::Searchable &config)
{
    Property params;
    params.fromString(config.toString());
    yCTrace(ANALOGSENSOR_NWS_YARPP) << "params are: " << config.toString();

    if (!config.check("period"))
    {
        yCError(ANALOGSENSOR_NWS_YARPP) << "missing 'period' parameter. Check you configuration file\n";
        return false;
    }
    else
    {
        _period = config.find("period").asFloat64();
    }

    if (config.check("deviceId"))
    {
        yCError(ANALOGSENSOR_NWS_YARPP) << "the parameter 'deviceId' has been removed, please use parameter 'name' instead. \n"
            << "e.g. In the FT wrapper configuration files of your robot, replace '<param name=""deviceId""> left_arm </param>' \n"
            << "with '/icub/left_arm/analog:o' ";
        return false;
    }

    if (!config.check("name"))
    {
        yCError(ANALOGSENSOR_NWS_YARPP) << "missing 'name' parameter. Check you configuration file; it must be like:\n"
                    "   name:         full name of the port, like /robotName/deviceId/sensorType:o";
        return false;
    }
    else
    {
        streamingPortName  = config.find("name").asString();
        setId("AnalogServer");
    }

    if(!initialize_YARP(config) )
    {
        yCError(ANALOGSENSOR_NWS_YARPP) << sensorId << "Error initializing YARP ports";
        return false;
    }

    // check if we need to create subdevice or if they are
    // passed later on through attachAll()
    if(config.check("subdevice"))
    {
        ownDevices=true;
        if(! openAndAttachSubDevice(config))
        {
            yCError(ANALOGSENSOR_NWS_YARPP, "error while opening subdevice\n");
            return false;
        }
    }
    else
    {
        ownDevices=false;
        if(!openDeferredAttach(config))
            return false;
    }

    return true;
}

bool AnalogSensor_nws_yarp::initialize_YARP(yarp::os::Searchable &params)
{
    // Create the list of ports
    // port names are optional, do not check for correctness.
    if(!params.check("ports"))
    {
        // if there is no "ports" section open only 1 port and use name as is.
        if (Network::exists(streamingPortName + "/rpc:i") || Network::exists(streamingPortName))
        {
            yCError(ANALOGSENSOR_NWS_YARPP) << "unable to open the analog server, address conflict";
            return false;
        }
        createPort((streamingPortName ).c_str(), _period);
        // since createPort always return true, check the port is really been opened is done here
        if(! Network::exists(streamingPortName + "/rpc:i"))
            return false;
    }
    else
    {
        Bottle *ports=params.find("ports").asList();

        Value &deviceChannels =  params.find("channels");
        if (deviceChannels.isNull())
        {
            yCError(ANALOGSENSOR_NWS_YARPP, "'channels' parameters was not found in config file.");
            return false;
        }

        int nports=ports->size();
        int sumOfChannels = 0;
        std::vector<AnalogPortEntry> tmpPorts;
        tmpPorts.resize(nports);

        for(size_t k=0; k<ports->size(); k++)
        {
            Bottle parameters=params.findGroup(ports->get(k).asString());

            if (parameters.size()!=5)
            {
                yCError(ANALOGSENSOR_NWS_YARPP) << "check port parameters in part description, I was expecting "
                            << ports->get(k).asString().c_str() << " followed by four integers";
                yCError(ANALOGSENSOR_NWS_YARPP) << " your param is " << parameters.toString();
                return false;
            }

            if (Network::exists(streamingPortName + "/" + string(ports->get(k).asString()) + "/rpc:i")
                || Network::exists(streamingPortName + "/" + string(ports->get(k).asString())))
            {
                yCError(ANALOGSENSOR_NWS_YARPP) << "unable to open the analog server, address conflict";
                return false;
            }
            int wBase=parameters.get(1).asInt32();
            int wTop=parameters.get(2).asInt32();
            int base=parameters.get(3).asInt32();
            int top=parameters.get(4).asInt32();

            yCDebug(ANALOGSENSOR_NWS_YARPP) << "--> " << wBase << " " << wTop << " " << base << " " << top;

            //check consistency
            if(wTop-wBase != top-base){
                yCError(ANALOGSENSOR_NWS_YARPP) << "numbers of mapped channels do not match, check "
                            << ports->get(k).asString().c_str() << " port parameters in part description";
                return false;
            }
            int portChannels = top-base+1;

            tmpPorts[k].length = portChannels;
            tmpPorts[k].offset = wBase;
            yCDebug(ANALOGSENSOR_NWS_YARPP) << "opening port " << ports->get(k).asString().c_str();
            tmpPorts[k].port_name = streamingPortName+ "/" + string(ports->get(k).asString());

            sumOfChannels+=portChannels;
        }
        createPorts(tmpPorts, _period);

        if (sumOfChannels!=deviceChannels.asInt32())
        {
            yCError(ANALOGSENSOR_NWS_YARPP) << "Total number of mapped channels does not correspond to total channels";
            return false;
        }
    }
    return true;
}

void AnalogSensor_nws_yarp::threadRelease()
{
    for(auto& analogPort : analogPorts)
    {
        analogPort.port.interrupt();
        analogPort.port.close();
    }
}

void AnalogSensor_nws_yarp::run()
{
    int first, last, ret;

    if (analogSensor_p!=nullptr)
    {
        ret=analogSensor_p->read(lastDataRead);

        if (ret==yarp::dev::IAnalogSensor::AS_OK)
        {
            if (lastDataRead.size()>0)
            {
                lastStateStamp.update();
                // send the data on the port(s), splitting them as specified in the config file
                for(auto& analogPort : analogPorts)
                {
                    yarp::sig::Vector &pv = analogPort.port.prepare();
                    first = analogPort.offset;
                    if(analogPort.length == -1)   // read the max length available
                        last = lastDataRead.size()-1;
                    else
                        last = analogPort.offset + analogPort.length - 1;

                    // check vector limit
                    if(last>=(int)lastDataRead.size()){
                        yCError(ANALOGSENSOR_NWS_YARPP, )<<"error while sending analog sensor output on port "<< analogPort.port_name
                                <<" Vector size expected to be at least "<<last<<" whereas it is "<< lastDataRead.size();
                        continue;
                    }
                    pv = lastDataRead.subVector(first, last);

                    analogPort.port.setEnvelope(lastStateStamp);
                    analogPort.port.write();
                }
            }
            else
            {
                yCError(ANALOGSENSOR_NWS_YARPP, "%s: vector size non valid: %lu", sensorId.c_str(), static_cast<unsigned long> (lastDataRead.size()));
            }
        }
        else
        {
            switch(ret)
            {
                case IAnalogSensor::AS_OVF:
                    yCError(ANALOGSENSOR_NWS_YARPP, "%s: Sensor returned overflow error (code %d).", sensorId.c_str(), ret);
                    break;
                case IAnalogSensor::AS_TIMEOUT:
                    yCError(ANALOGSENSOR_NWS_YARPP, " %s: Sensor returned timeout error (code %d).", sensorId.c_str(), ret);
                    break;
                case IAnalogSensor::AS_ERROR:
                default:
                    yCError(ANALOGSENSOR_NWS_YARPP, "%s: Sensor returned error with code %d.", sensorId.c_str(), ret);
                    break;
            }
        }
    }
}

bool AnalogSensor_nws_yarp::close()
{
    yCTrace(ANALOGSENSOR_NWS_YARPP, "Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detachAll();
    removeHandlers();

    if(subDeviceOwned)
    {
        subDeviceOwned->close();
        delete subDeviceOwned;
        subDeviceOwned = nullptr;
    }

    return true;
}
