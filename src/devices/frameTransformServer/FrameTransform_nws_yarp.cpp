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

// example: yarpdev --device frameTransform_nws_yarp

#define _USE_MATH_DEFINES
#include <cmath>

#include "FrameTransform_nws_yarp.h"
#include <sstream>
#include <limits>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <mutex>
#include <cstdlib>

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::os;
using namespace std;


namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORMNWSYARP, "yarp.device.frameTransform_nws_yarp")
}

/**
  * FrameTransform_nws_yarp
  */

FrameTransform_nws_yarp::FrameTransform_nws_yarp() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_period = DEFAULT_THREAD_PERIOD;
    m_yarp_static_transform_storage = nullptr;
    m_yarp_timed_transform_storage = nullptr;
    m_ros_static_transform_storage = nullptr;
    m_ros_timed_transform_storage = nullptr;
}

FrameTransform_nws_yarp::~FrameTransform_nws_yarp()
{
    threadRelease();
}

bool FrameTransform_nws_yarp::read(yarp::os::ConnectionReader& connection)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    string request = in.get(0).asString();

    // parse in, prepare out
    int code = in.get(0).asVocab();
    bool ret = false;
    if (code == VOCAB_ITRANSFORM)
    {
        int cmd = in.get(1).asVocab();
        if (cmd == VOCAB_TRANSFORM_SET)
        {
            if (in.size() != 12)
            {
                yCError(FRAMETRANSFORMNWSYARP) << "read(): Protocol error";
                out.clear();
                out.addVocab(VOCAB_FAILED);
            }
            else
            {
                FrameTransform t;
                t.src_frame_id = in.get(2).asString();
                t.dst_frame_id = in.get(3).asString();
                double duration = in.get(4).asFloat64();
                t.translation.tX = in.get(5).asFloat64();
                t.translation.tY = in.get(6).asFloat64();
                t.translation.tZ = in.get(7).asFloat64();
                t.rotation.w() = in.get(8).asFloat64();
                t.rotation.x() = in.get(9).asFloat64();
                t.rotation.y() = in.get(10).asFloat64();
                t.rotation.z() = in.get(11).asFloat64();
                t.timestamp = yarp::os::Time::now();

                if (duration > 0)
                {
                    ret = m_iTf->set_transform(t);
                }
                else
                {
                    ret = m_iTf->set_transformStatic(t);
                }

                if (ret == true)
                {
                    out.clear();
                    out.addVocab(VOCAB_OK);
                }
                else
                {
                    out.clear();
                    out.addVocab(VOCAB_FAILED);
                    yCError(FRAMETRANSFORMNWSYARP) << "read(): Something strange happened";
                }
            }
        }
        else if (cmd == VOCAB_TRANSFORM_DELETE_ALL)
        {
            m_iTf->clear();
            out.clear();
            out.addVocab(VOCAB_OK);
        }
        else if (cmd == VOCAB_TRANSFORM_DELETE)
        {
            string frame1 = in.get(2).asString();
            string frame2 = in.get(3).asString();
            bool ret1 = m_iTf->delete_transform(frame1, frame2);
            if (ret1 == true)
            {
                out.clear();
                out.addVocab(VOCAB_OK);
            }
            else
            {
                bool ret2 = m_iTf->delete_transform(frame1, frame2);
                if (ret2 == true)
                {
                    out.clear();
                    out.addVocab(VOCAB_OK);
                }
            }

        }
        else
        {
            yCError(FRAMETRANSFORMNWSYARP, "Invalid vocab received");
            out.clear();
            out.addVocab(VOCAB_ERR);
        }
    }
    else if(request == "help")
    {
        out.addVocab(Vocab::encode("many"));
        out.addString("'delete_all': delete all transforms");
        out.addString("'set_static_transform_rad <src> <dst> <x> <y> <z> <roll> <pitch> <yaw>': create a static transform (angles in radians)");
        out.addString("'set_static_transform_deg <src> <dst> <x> <y> <z> <roll> <pitch> <yaw>': create a static transform (angles in degrees)");
        out.addString("'delete_static_transform <src> <dst>': delete a static transform");
        out.addString("'generate_view <option>': generate a frames.pdf file showing the transform tree diagram.");
        out.addString("     The following values are valid for option (default=none)");
        out.addString("    'show_rpy': show rotation as rpy angles");
        out.addString("    'show_quaterion:'show rotation as a quaternion");
        out.addString("    'show_matrix:'show rotation as a 3x3 rotation matrix");
    }
    else if (request == "set_static_transform_rad" ||
             request == "set_static_transform_deg")
    {
        FrameTransform t;
        t.src_frame_id = in.get(1).asString();
        t.dst_frame_id = in.get(2).asString();
        t.translation.tX = in.get(3).asFloat64();
        t.translation.tY = in.get(4).asFloat64();
        t.translation.tZ = in.get(5).asFloat64();
        if (request == "set_static_transform_rad")
            { t.rotFromRPY(in.get(6).asFloat64(), in.get(7).asFloat64(), in.get(8).asFloat64());}
        else if (request == "set_static_transform_deg")
            { t.rotFromRPY(in.get(6).asFloat64() * 180 / M_PI, in.get(7).asFloat64() * 180 / M_PI, in.get(8).asFloat64() * 180 / M_PI);}
        t.timestamp = yarp::os::Time::now();
        ret = m_iTf->set_transformStatic(t);
        if (ret == true)
        {
            yCInfo(FRAMETRANSFORMNWSYARP) << "set_static_transform done";
            out.addString("set_static_transform done");
        }
        else
        {
            yCError(FRAMETRANSFORMNWSYARP) << "read(): something strange happened";
        }
    }
    else if(request == "delete_all")
    {
        m_iTf->clear();
        yCInfo(FRAMETRANSFORMNWSYARP) << "delete_all done";
        out.addString("delete_all done");
    }
    else if (request == "generate_view")
    {
        m_show_transforms_in_diagram  = do_not_show;
        if      (in.get(1).asString() == "show_quaternion") m_show_transforms_in_diagram = show_quaternion;
        else if (in.get(1).asString() == "show_matrix") m_show_transforms_in_diagram = show_matrix;
        else if (in.get(1).asString() == "show_rpy") m_show_transforms_in_diagram = show_rpy;
        generate_view();
        out.addString("ok");
    }
    else if (request == "delete_static_transform")
    {
        std::string src = in.get(1).asString();
        std::string dst = in.get(2).asString();
        m_iTf->deleteTransform(src, dst);
        out.addString("delete_static_transform done");
    }
    else
    {
        yCError(FRAMETRANSFORMNWSYARP, "Invalid vocab received");
        out.clear();
        out.addVocab(VOCAB_ERR);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }
    else
    {
        yCError(FRAMETRANSFORMNWSYARP) << "Invalid return to sender";
    }
    return true;
}

bool FrameTransform_nws_yarp::threadInit()
{
    //open rpc port
    if (!m_rpcPort.open(m_rpcPortName))
    {
        yCError(FRAMETRANSFORMNWSYARP, "Failed to open port %s", m_rpcPortName.c_str());
        return false;
    }
    m_rpcPort.setReader(*this);

    // open data port
    if (!m_streamingPort.open(m_streamingPortName))
    {
        yCError(FRAMETRANSFORMNWSYARP, "Failed to open port %s", m_streamingPortName.c_str());
        return false;
    }

    m_yarp_static_transform_storage = new Transforms_server_storage();
    m_yarp_timed_transform_storage = new Transforms_server_storage();

    m_ros_static_transform_storage = new Transforms_server_storage();
    m_ros_timed_transform_storage = new Transforms_server_storage();

    yCInfo(FRAMETRANSFORMNWSYARP) << "Transform server started";
    return true;
}

bool FrameTransform_nws_yarp::open(yarp::os::Searchable &config)
{
    Property params;
    params.fromString(config.toString());

    if (!config.check("period"))
    {
        m_period = 0.01;
    }
    else
    {
        m_period = config.find("period").asInt32() / 1000.0;
        yCInfo(FRAMETRANSFORMNWSYARP) << "Thread period set to:" << m_period;
    }

    std::string name;
    if (!config.check("name"))
    {
        name = "frameTransform_nws_yarp";
    }
    else
    {
        name = config.find("name").asString();
    }
    m_streamingPortName =  "/"+ name + "/transforms:o";
    m_rpcPortName = "/" + name + "/rpc";

    this->start();

    yarp::os::Time::delay(0.5);

    return true;
}

void FrameTransform_nws_yarp::threadRelease()
{
    m_streamingPort.interrupt();
    m_streamingPort.close();
    m_rpcPort.interrupt();
    m_rpcPort.close();
}

void FrameTransform_nws_yarp::run()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (true)
    {
        double current_time = yarp::os::Time::now();

        //yarp streaming port
        m_lastStateStamp.update();
        std::vector <yarp::math::FrameTransform> yarp_static_transform_storage;
        std::vector <yarp::math::FrameTransform> yarp_timed_transform_storage;
        m_iTf->getAllTransforms(yarp_timed_transform_storage);
        m_iTf->getAllStaticTransforms(yarp_static_transform_storage);
        size_t    tfVecSize_static_yarp = yarp_static_transform_storage.size();
        size_t    tfVecSize_timed_yarp = yarp_timed_transform_storage.size();

        size_t    tfVecSize_static_ros  = m_ros_static_transform_storage->size();
        size_t    tfVecSize_timed_ros = m_ros_timed_transform_storage->size();

        yarp::os::Bottle& b = m_streamingPort.prepare();
        b.clear();

        for (size_t i = 0; i < tfVecSize_static_yarp; i++)
        {
            yarp::os::Bottle& transform = b.addList();
            transform.addString((*m_yarp_static_transform_storage)[i].src_frame_id);
            transform.addString((*m_yarp_static_transform_storage)[i].dst_frame_id);
            transform.addFloat64((*m_yarp_static_transform_storage)[i].timestamp);

            transform.addFloat64((*m_yarp_static_transform_storage)[i].translation.tX);
            transform.addFloat64((*m_yarp_static_transform_storage)[i].translation.tY);
            transform.addFloat64((*m_yarp_static_transform_storage)[i].translation.tZ);

            transform.addFloat64((*m_yarp_static_transform_storage)[i].rotation.w());
            transform.addFloat64((*m_yarp_static_transform_storage)[i].rotation.x());
            transform.addFloat64((*m_yarp_static_transform_storage)[i].rotation.y());
            transform.addFloat64((*m_yarp_static_transform_storage)[i].rotation.z());
        }
        for (size_t i = 0; i < tfVecSize_timed_yarp; i++)
        {
            yarp::os::Bottle& transform = b.addList();
            transform.addString((*m_yarp_timed_transform_storage)[i].src_frame_id);
            transform.addString((*m_yarp_timed_transform_storage)[i].dst_frame_id);
            transform.addFloat64((*m_yarp_timed_transform_storage)[i].timestamp);

            transform.addFloat64((*m_yarp_timed_transform_storage)[i].translation.tX);
            transform.addFloat64((*m_yarp_timed_transform_storage)[i].translation.tY);
            transform.addFloat64((*m_yarp_timed_transform_storage)[i].translation.tZ);

            transform.addFloat64((*m_yarp_timed_transform_storage)[i].rotation.w());
            transform.addFloat64((*m_yarp_timed_transform_storage)[i].rotation.x());
            transform.addFloat64((*m_yarp_timed_transform_storage)[i].rotation.y());
            transform.addFloat64((*m_yarp_timed_transform_storage)[i].rotation.z());
        }
        for (size_t i = 0; i < tfVecSize_timed_ros; i++)
        {
            yarp::os::Bottle& transform = b.addList();
            transform.addString((*m_ros_timed_transform_storage)[i].src_frame_id);
            transform.addString((*m_ros_timed_transform_storage)[i].dst_frame_id);
            transform.addFloat64((*m_ros_timed_transform_storage)[i].timestamp);

            transform.addFloat64((*m_ros_timed_transform_storage)[i].translation.tX);
            transform.addFloat64((*m_ros_timed_transform_storage)[i].translation.tY);
            transform.addFloat64((*m_ros_timed_transform_storage)[i].translation.tZ);

            transform.addFloat64((*m_ros_timed_transform_storage)[i].rotation.w());
            transform.addFloat64((*m_ros_timed_transform_storage)[i].rotation.x());
            transform.addFloat64((*m_ros_timed_transform_storage)[i].rotation.y());
            transform.addFloat64((*m_ros_timed_transform_storage)[i].rotation.z());
        }
        for (size_t i = 0; i < tfVecSize_static_ros; i++)
        {
            yarp::os::Bottle& transform = b.addList();
            transform.addString((*m_ros_static_transform_storage)[i].src_frame_id);
            transform.addString((*m_ros_static_transform_storage)[i].dst_frame_id);
            transform.addFloat64((*m_ros_static_transform_storage)[i].timestamp);

            transform.addFloat64((*m_ros_static_transform_storage)[i].translation.tX);
            transform.addFloat64((*m_ros_static_transform_storage)[i].translation.tY);
            transform.addFloat64((*m_ros_static_transform_storage)[i].translation.tZ);

            transform.addFloat64((*m_ros_static_transform_storage)[i].rotation.w());
            transform.addFloat64((*m_ros_static_transform_storage)[i].rotation.x());
            transform.addFloat64((*m_ros_static_transform_storage)[i].rotation.y());
            transform.addFloat64((*m_ros_static_transform_storage)[i].rotation.z());
        }

        m_streamingPort.setEnvelope(m_lastStateStamp);
        m_streamingPort.write();
    }
    else
    {
        yCError(FRAMETRANSFORMNWSYARP, "Returned error");
    }
}

bool FrameTransform_nws_yarp::close()
{
    yCTrace(FRAMETRANSFORMNWSYARP, "Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    return true;
}
