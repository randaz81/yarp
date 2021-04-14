/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "FrameTransform_nwc_yarp.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <mutex>

/*! \file FRAMETRANSFORM_NWC_YARP.cpp */

//example: yarpdev --device frameTransform_nwc_yarp --local /transformClient --remote /transformServer

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORM_NWC_YARP, "yarp.device.frameTransform_nwc_yarp")
}

//------------------------------------------------------------------------------------------------------------------------------
bool FrameTransform_nwc_yarp::read(yarp::os::ConnectionReader& connection)
{
    std::lock_guard<std::mutex> lock (m_rpc_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    std::string request = in.get(0).asString();
    if (request == "help")
    {
        out.addVocab(Vocab::encode("many"));
        out.addString("'get_transform <src> <dst>: print the transform from <src> to <dst>");
        out.addString("'list_frames: print all the available reference frames");
        out.addString("'list_ports: print all the opened ports for transform broadcasting");
        out.addString("'publish_transform <src> <dst> <portname> <format>: opens a port to publish transform from src to dst");
        out.addString("'unpublish_transform <portname>: closes a previously opened port to publish a transform");
        out.addString("'unpublish_all <portname>: closes a all previously opened ports to publish a transform");
        out.addString("'is_connected'");
        out.addString("'reconnect'");
    }
    else if (request == "is_connected")
    {
        if (isConnectedWithServer())
        {
            out.addString("yes");
        }
        else
        {
            out.addString("no");
        }
    }
    else if (request == "reconnect")
    {
        if (reconnectWithServer())
        {
            out.addString("successful");
        }
        else
        {
            out.addString("unsuccessful");
        }
    }
    else if (request == "list_frames")
    {
        std::vector<std::string> v;
        this->getAllFrameIds(v);
        out.addVocab(Vocab::encode("many"));
        out.addString("List of available reference frames:");
        int count = 0;
        for (auto& vec : v)
        {
            count++;
            std::string str = std::to_string(count) + "- " + vec;
            out.addString(str.c_str());
        }
    }
    else if (request == "get_transform")
    {
        std::string src = in.get(1).asString();
        std::string dst = in.get(2).asString();
        out.addVocab(Vocab::encode("many"));
        yarp::sig::Matrix m;
        this->getTransform(src, dst, m);
        out.addString("Transform from " + src + " to " + dst + " is: ");
        out.addString(m.toString());
    }
    else if (request == "list_ports")
    {
        out.addVocab(Vocab::encode("many"));
        if (m_array_of_ports.size()==0)
        {
            out.addString("No ports are currently active");
        }
        for (auto& m_array_of_port : m_array_of_ports)
        {
            if (m_array_of_port)
            {
                std::string  s = m_array_of_port->port.getName() + " "+ m_array_of_port->transform_src +  " -> " + m_array_of_port->transform_dst;
                out.addString(s);
            }
        }
    }
    else if (request == "publish_transform")
    {
        out.addVocab(Vocab::encode("many"));
        std::string src  = in.get(1).asString();
        std::string dst  = in.get(2).asString();
        std::string port_name = in.get(3).asString();
        std::string format = "matrix";
        if (in.size() > 4)
            {format= in.get(4).asString();}
        if (port_name[0]=='/')  port_name.erase(port_name.begin());
        std::string full_port_name = m_local_name + "/" + port_name;
        bool ret = true;
        for (auto& m_array_of_port : m_array_of_ports)
        {
            if (m_array_of_port && m_array_of_port->port.getName() == full_port_name)
            {
                ret = false;
                break;
            }
        }
        if (this->frameExists(src)==false)
        {
            out.addString("Requested src frame " + src + " does not exists.");
            yCWarning(FRAMETRANSFORM_NWC_YARP, "Requested src frame %s does not exists.", src.c_str());
        }
        if (this->frameExists(dst)==false)
        {
            out.addString("Requested dst frame " + dst + " does not exists.");
            yCWarning(FRAMETRANSFORM_NWC_YARP, "Requested fst frame %s does not exists.", dst.c_str());
        }
        if (ret == true)
        {
            auto* b = new broadcast_port_t;
            b->transform_src = src;
            b->transform_dst = dst;
            b->format = format;
            bool pret = b->port.open(full_port_name);
            if (pret)
            {
                out.addString("Operation complete. Port " + full_port_name + " opened.");
                m_array_of_ports.push_back(b);
                if (m_array_of_ports.size()==1) this->start();
            }
            else
            {
                delete b;
                out.addString("Operation failed. Unable to open port " + full_port_name + ".");
            }
        }
        else
        {
            out.addString("unable to perform operation");
        }
    }
    else if (request == "unpublish_all")
    {
        for (auto& m_array_of_port : m_array_of_ports)
        {
            m_array_of_port->port.close();
            delete m_array_of_port;
            m_array_of_port=nullptr;
        }
        m_array_of_ports.clear();
        if (m_array_of_ports.size()==0) this->askToStop();
        out.addString("Operation complete");
    }
    else if (request == "unpublish_transform")
    {
        bool ret = false;
        std::string port_name = in.get(1).asString();
        if (port_name[0]=='/')  port_name.erase(port_name.begin());
        std::string full_port_name = m_local_name + "/" + port_name;
        for (auto it = m_array_of_ports.begin(); it != m_array_of_ports.end(); it++)
        {
            if ((*it)->port.getName() == port_name)
            {
                (*it)->port.close();
                delete (*it);
                (*it)=nullptr;
                 m_array_of_ports.erase(it);
                 ret = true;
                 break;
            }
        }
        if (ret)
        {
            out.addString("Port " + full_port_name + " has been closed.");
        }
        else
        {
            out.addString("Port " + full_port_name + " was not found.");
        }
        if (m_array_of_ports.size()==0) this->askToStop();
    }
    else
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "Invalid vocab received in FRAMETRANSFORM_NWC_YARP");
        out.clear();
        out.addVocab(VOCAB_ERR);
        out.addString("Invalid command name");
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }
    else
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "Invalid return to sender";
    }
    return true;
}

bool FrameTransform_nwc_yarp::open(yarp::os::Searchable &config)
{
    m_local_name.clear();
    m_remote_name.clear();

    m_local_name  = config.find("local").asString();
    m_remote_name = config.find("remote").asString();
    m_streaming_connection_type = "udp";

    if (m_local_name == "")
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "open(): Invalid local name");
        return false;
    }
    if (m_remote_name == "")
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "open(): Invalid remote name");
        return false;
    }

    if (config.check("period"))
    {
        m_period = config.find("period").asInt32() / 1000.0;
    }
    else
    {
        m_period = 0.010;
        yCWarning(FRAMETRANSFORM_NWC_YARP, "Using default period of %f s" , m_period);
    }

    m_local_rpcServer = m_local_name + "/rpc:o";
    m_local_rpcUser = m_local_name + "/rpc:i";
    m_remote_rpc = m_remote_name + "/rpc";
    m_remote_streaming_name = m_remote_name + "/transforms:o";
    m_local_streaming_name = m_local_name + "/transforms:i";

    if (!m_rpc_InterfaceToUser.open(m_local_rpcUser))
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "open(): Could not open rpc port %s, check network", m_local_rpcUser.c_str());
        return false;
    }

    if (!m_rpc_InterfaceToServer.open(m_local_rpcServer))
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "open(): Could not open rpc port %s, check network", m_local_rpcServer.c_str());
        return false;
    }

    bool ok = Network::connect(m_remote_streaming_name.c_str(), m_local_streaming_name.c_str(), m_streaming_connection_type.c_str());
    if (!ok)
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "open(): Could not connect to %s", m_remote_streaming_name.c_str());
        return false;
    }

    ok = Network::connect(m_local_rpcServer, m_remote_rpc);
    if (!ok)
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "open(): Could not connect to %s", m_remote_rpc.c_str());
        return false;
    }


    m_rpc_InterfaceToUser.setReader(*this);
    return true;
}

bool FrameTransform_nwc_yarp::close()
{
    m_rpc_InterfaceToServer.close();
    m_rpc_InterfaceToUser.close();
    return true;
}

bool FrameTransform_nwc_yarp::allFramesAsString(std::string &all_frames)
{
}

bool FrameTransform_nwc_yarp::canTransform(const std::string &target_frame, const std::string &source_frame)
{
}

bool FrameTransform_nwc_yarp::clear()
{
}

bool FrameTransform_nwc_yarp::frameExists(const std::string &frame_id)
{
}

bool FrameTransform_nwc_yarp::getAllFrameIds(std::vector< std::string > &ids)
{
}

bool FrameTransform_nwc_yarp::getParent(const std::string &frame_id, std::string &parent_frame_id)
{
}

bool FrameTransform_nwc_yarp::getTransform(const std::string& target_frame_id, const std::string& source_frame_id, yarp::sig::Matrix& transform)
{
}

bool FrameTransform_nwc_yarp::setTransform(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Matrix& transform)
{
    if(target_frame_id == source_frame_id)
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Invalid transform detected.\n" \
                    "\t Source frame and target frame are both equal to " << source_frame_id;
        return false;
    }

    yarp::os::Bottle b;
    yarp::os::Bottle resp;
    FrameTransform   tf;

    if (!tf.fromMatrix(transform))
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Wrong matrix format, it has to be 4 by 4";
        return false;
    }

    b.addVocab(VOCAB_ITRANSFORM);
    b.addVocab(VOCAB_TRANSFORM_SET);
    b.addString(source_frame_id);
    b.addString(target_frame_id);
    b.addFloat64(1000.0); //transform lifetime
    b.addFloat64(tf.translation.tX);
    b.addFloat64(tf.translation.tY);
    b.addFloat64(tf.translation.tZ);
    b.addFloat64(tf.rotation.w());
    b.addFloat64(tf.rotation.x());
    b.addFloat64(tf.rotation.y());
    b.addFloat64(tf.rotation.z());
    bool ret = m_rpc_InterfaceToServer.write(b, resp);
    if (ret)
    {
        if (resp.get(0).asVocab() != VOCAB_OK)
        {
            yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Received error from server on creating frame between " + source_frame_id + " and " + target_frame_id;
            return false;
        }
    }
    else
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Error on writing on rpc port";
        return false;
    }
    return true;
}

bool FrameTransform_nwc_yarp::setTransformStatic(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Matrix &transform)
{
    if(target_frame_id == source_frame_id)
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransformStatic(): Invalid transform detected.\n" \
                    "\t Source frame and target frame are both equal to " << source_frame_id;
        return false;
    }

    if (canTransform(target_frame_id, source_frame_id))
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Such static transform already exist, directly or by chaining transforms";
        return false;
    }

    yarp::os::Bottle b;
    yarp::os::Bottle resp;
    FrameTransform   tf;

    if (!tf.fromMatrix(transform))
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Wrong matrix format, it has to be 4 by 4";
        return false;
    }

    b.addVocab(VOCAB_ITRANSFORM);
    b.addVocab(VOCAB_TRANSFORM_SET);
    b.addString(source_frame_id);
    b.addString(target_frame_id);
    b.addFloat64(-1);
    b.addFloat64(tf.translation.tX);
    b.addFloat64(tf.translation.tY);
    b.addFloat64(tf.translation.tZ);
    b.addFloat64(tf.rotation.w());
    b.addFloat64(tf.rotation.x());
    b.addFloat64(tf.rotation.y());
    b.addFloat64(tf.rotation.z());
    bool ret = m_rpc_InterfaceToServer.write(b, resp);
    if (ret)
    {
        if (resp.get(0).asVocab() != VOCAB_OK)
        {
            yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Received error from server on creating frame between " + source_frame_id + " and " + target_frame_id;
            return false;
        }
    }
    else
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "setTransform(): Error on writing on rpc port";
        return false;
    }
    return true;
}

bool FrameTransform_nwc_yarp::deleteTransform(const std::string &target_frame_id, const std::string &source_frame_id)
{
    yarp::os::Bottle b;
    yarp::os::Bottle resp;
    b.addVocab(VOCAB_ITRANSFORM);
    b.addVocab(VOCAB_TRANSFORM_DELETE);
    b.addString(target_frame_id);
    b.addString(source_frame_id);
    bool ret = m_rpc_InterfaceToServer.write(b, resp);
    if (ret)
    {
        if (resp.get(0).asVocab()!=VOCAB_OK)
        {
            yCError(FRAMETRANSFORM_NWC_YARP) << "Received error from server on deleting frame between "+source_frame_id+" and "+target_frame_id;
            return false;
        }
    }
    else
    {
        yCError(FRAMETRANSFORM_NWC_YARP) << "deleteFrame(): Error on writing on rpc port";
        return false;
    }
    return true;
}

bool FrameTransform_nwc_yarp::transformPoint(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_point, yarp::sig::Vector &transformed_point)
{
}

bool FrameTransform_nwc_yarp::transformPose(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_pose, yarp::sig::Vector &transformed_pose)
{
}

bool FrameTransform_nwc_yarp::transformQuaternion(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::math::Quaternion &input_quaternion, yarp::math::Quaternion &transformed_quaternion)
{
}

bool FrameTransform_nwc_yarp::waitForTransform(const std::string &target_frame_id, const std::string &source_frame_id, const double &timeout)
{
    //loop until canTransform == true or timeout expires
    double start = yarp::os::SystemClock::nowSystem();
    while (!canTransform(target_frame_id, source_frame_id))
    {
        if (yarp::os::SystemClock::nowSystem() - start > timeout)
        {
            yCError(FRAMETRANSFORM_NWC_YARP) << "waitForTransform(): timeout expired";
            return false;
        }
        yarp::os::SystemClock::delaySystem(0.001);
    }
    return true;
}

FrameTransform_nwc_yarp::FrameTransform_nwc_yarp() : PeriodicThread(0.01),
    m_period(0.01)
{
}

FrameTransform_nwc_yarp::~FrameTransform_nwc_yarp() = default;

bool     FrameTransform_nwc_yarp::threadInit()
{
    yCTrace(FRAMETRANSFORM_NWC_YARP, "Thread started");
    return true;
}

void     FrameTransform_nwc_yarp::threadRelease()
{
    yCTrace(FRAMETRANSFORM_NWC_YARP, "Thread stopped");
}

void     FrameTransform_nwc_yarp::run()
{
    std::lock_guard<std::mutex> lock (m_rpc_mutex);
    if (m_array_of_ports.size()==0)
    {
        return;
    }

    for (auto& m_array_of_port : m_array_of_ports)
    {
        if (m_array_of_port)
        {
            std::string src = m_array_of_port->transform_src;
            std::string dst = m_array_of_port->transform_dst;
            yarp::sig::Matrix m;
            this->getTransform(src, dst, m);
            if (m_array_of_port->format == "matrix")
            {
                m_array_of_port->port.write(m);
            }
            else
            {
                yCError(FRAMETRANSFORM_NWC_YARP) << "Unknown format requested: " << m_array_of_port->format;
            }
        }
    }
}

bool     FrameTransform_nwc_yarp::isConnectedWithServer()
{
    bool ok1 = Network::isConnected(m_local_rpcServer.c_str(), m_remote_rpc.c_str());
    if (!ok1) yCInfo(FRAMETRANSFORM_NWC_YARP) << m_local_rpcServer << "is not connected to: " << m_remote_rpc;

    bool ok2 = Network::isConnected(m_remote_streaming_name.c_str(), m_local_streaming_name.c_str(),m_streaming_connection_type.c_str());
    if (!ok2) yCInfo(FRAMETRANSFORM_NWC_YARP) << m_remote_streaming_name << "is not connected to: " << m_local_streaming_name;

    return ok1 && ok2;
}

bool     FrameTransform_nwc_yarp::reconnectWithServer()
{
    bool ok = Network::connect(m_remote_streaming_name.c_str(), m_local_streaming_name.c_str(), m_streaming_connection_type.c_str());
    if (!ok)
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "reconnectWithServer(): Could not connect to %s", m_remote_streaming_name.c_str());
        return false;
    }

    ok = Network::connect(m_local_rpcServer, m_remote_rpc);
    if (!ok)
    {
        yCError(FRAMETRANSFORM_NWC_YARP, "reconnectWithServer(): Could not connect to %s", m_remote_rpc.c_str());
        return false;
    }
    return true;
}
