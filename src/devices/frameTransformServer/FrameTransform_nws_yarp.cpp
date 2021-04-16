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
}

FrameTransform_nws_yarp::~FrameTransform_nws_yarp()
{
    threadRelease();
}

/*
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
                    ret = m_iTf->setTransform(t);
                }
                else
                {
                    ret = m_iTf->setTransformStatic(t);
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
            bool ret1 = m_iTf->deleteTransform(frame1, frame2);
            if (ret1 == true)
            {
                out.clear();
                out.addVocab(VOCAB_OK);
            }
            else
            {
                bool ret2 = m_iTf->deleteTransform(frame1, frame2);
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
        out.addString("No commands available");
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

*/

bool FrameTransform_nws_yarp::threadInit()
{

    //attach the thrift interface
    if (this->yarp().attachAsServer(m_rpcPort))
    {
        yCError(FRAMETRANSFORMNWSYARP, "Failure in attaching RPC port to thrift RPC interface.");
        return false;
    }

    //open rpc port
    if (!m_rpcPort.open(m_rpcPortName))
    {
        yCError(FRAMETRANSFORMNWSYARP, "Failed to open port %s", m_rpcPortName.c_str());
        return false;
    }

    // open data port
    if (!m_streamingPort.open(m_streamingPortName))
    {
        yCError(FRAMETRANSFORMNWSYARP, "Failed to open port %s", m_streamingPortName.c_str());
        return false;
    }

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

    //publish all static and timed transforms on a yarp streaming port
    if (true)
    {
        double current_time = yarp::os::Time::now();

        //yarp streaming port
        m_lastStateStamp.update();
        std::vector <yarp::math::FrameTransform> timed_transform_storage;
        std::vector <yarp::math::FrameTransform> static_transform_storage;
        m_iTf->getAllTransforms(timed_transform_storage);
        m_iTf->getAllStaticTransforms(static_transform_storage);
        size_t    tfVecSize_static_yarp = timed_transform_storage.size();
        size_t    tfVecSize_static = static_transform_storage.size();

        yarp::os::Bottle& b = m_streamingPort.prepare();
        b.clear();

        for (size_t i = 0; i < tfVecSize_static_yarp; i++)
        {
            yarp::os::Bottle& transform = b.addList();
            transform.addString(timed_transform_storage[i].src_frame_id);
            transform.addString(timed_transform_storage[i].dst_frame_id);
            transform.addFloat64(timed_transform_storage[i].timestamp);

            transform.addFloat64(timed_transform_storage[i].translation.tX);
            transform.addFloat64(timed_transform_storage[i].translation.tY);
            transform.addFloat64(timed_transform_storage[i].translation.tZ);

            transform.addFloat64(timed_transform_storage[i].rotation.w());
            transform.addFloat64(timed_transform_storage[i].rotation.x());
            transform.addFloat64(timed_transform_storage[i].rotation.y());
            transform.addFloat64(timed_transform_storage[i].rotation.z());
        }

        for (size_t i = 0; i < tfVecSize_static; i++)
        {
            yarp::os::Bottle& transform = b.addList();
            transform.addString(static_transform_storage[i].src_frame_id);
            transform.addString(static_transform_storage[i].dst_frame_id);
            transform.addFloat64(static_transform_storage[i].timestamp);

            transform.addFloat64(static_transform_storage[i].translation.tX);
            transform.addFloat64(static_transform_storage[i].translation.tY);
            transform.addFloat64(static_transform_storage[i].translation.tZ);

            transform.addFloat64(static_transform_storage[i].rotation.w());
            transform.addFloat64(static_transform_storage[i].rotation.x());
            transform.addFloat64(static_transform_storage[i].rotation.y());
            transform.addFloat64(static_transform_storage[i].rotation.z());
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

//Thrift RPC interface
bool FrameTransform_nws_yarp::getTranformTest(const std::int32_t x)
{
    return true;
}
bool FrameTransform_nws_yarp::setTranformTest(const std::int32_t x)
{
    return true;
}

return_allFramesAsString FrameTransform_nws_yarp::allFramesAsString()
{
    return_allFramesAsString ret;
    string allframes;
    bool b = m_iTf->allFramesAsString(allframes);
    ret.all_frames=allframes;
    ret.retvalue =b;
    return ret;
}

return_canTransform FrameTransform_nws_yarp::canTransform(const std::string& target_frame, const std::string& source_frame)
{
    return_canTransform ret;
    bool b = m_iTf->canTransform(target_frame, source_frame);
    ret.retvalue = b;
    return ret;
}

return_clear FrameTransform_nws_yarp::clear()
{
    return_clear ret;
    bool b = m_iTf->clear();
    ret.retvalue = b;
    return ret;
}

return_frameExists FrameTransform_nws_yarp::frameExists(const std::string& frame_id)
{
    return_frameExists ret;
    bool b = m_iTf->frameExists(frame_id);
    ret.retvalue = b;
    return ret;
}

return_getAllFrameIds FrameTransform_nws_yarp::getAllFrameIds()
{
    return_getAllFrameIds ret;
    std::vector <string> frame_ids;
    bool b = m_iTf->getAllFrameIds(frame_ids);
    ret.ids=frame_ids;
    ret.retvalue = b;
    return ret;
}

return_getParent FrameTransform_nws_yarp::getParent(const std::string& frame_id)
{
    return_getParent ret;
    string parent_id;
    bool b = m_iTf->getParent(frame_id, parent_id);
    ret.parent_frame_id = parent_id;
    ret.retvalue = b;
    return ret;
}

return_getTransform FrameTransform_nws_yarp::getTransform(const std::string& target_frame_id, const std::string& source_frame_id)
{
    return_getTransform ret;
    yarp::sig::Matrix matrix;
    bool b = m_iTf->getTransform(target_frame_id, source_frame_id, matrix);
    ret.transform = matrix;
    ret.retvalue = b;
    return ret;
}

return_setTransform FrameTransform_nws_yarp::setTransform(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Matrix& transform)
{
    return_setTransform ret;
    bool b = m_iTf->setTransform(target_frame_id, source_frame_id, transform);
    ret.retvalue = b;
    return ret;
}

return_setTransformStatic FrameTransform_nws_yarp::setTransformStatic(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Matrix& transform)
{
    return_setTransformStatic ret;
    bool b = m_iTf->setTransformStatic(target_frame_id, source_frame_id, transform);
    ret.retvalue = b;
    return ret;
}

return_deleteTransform FrameTransform_nws_yarp::deleteTransform(const std::string& target_frame_id, const std::string& source_frame_id)
{
    return_deleteTransform ret;
    bool b = m_iTf->deleteTransform(target_frame_id, source_frame_id);
    ret.retvalue = b;
    return ret;
}

return_transformPoint FrameTransform_nws_yarp::transformPoint(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Vector& input_point)
{
    return_transformPoint ret;
    yarp::sig::Vector output_point;
    bool b = m_iTf->transformPoint(target_frame_id, source_frame_id, input_point, output_point);
    ret.transformed_point= output_point;
    ret.retvalue = b;
    return ret;
}

return_transformPose FrameTransform_nws_yarp::transformPose(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Vector& input_pose)
{
    return_transformPose ret;
    yarp::sig::Vector output_pose;
    bool b = m_iTf->transformPose(target_frame_id, source_frame_id, input_pose, output_pose);
    ret.transformed_pose = output_pose;
    ret.retvalue = b;
    return ret;
}

return_transformQuaternion FrameTransform_nws_yarp::transformQuaternion(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::math::Quaternion& input_quaternion)
{
    return_transformQuaternion ret;
    yarp::math::Quaternion output_quaternion;
    bool b = m_iTf->transformQuaternion(target_frame_id, source_frame_id, input_quaternion, output_quaternion);
    ret.transformed_quaternion = output_quaternion;
    ret.retvalue = b;
    return ret;
}

return_waitForTransform FrameTransform_nws_yarp::waitForTransform(const std::string& target_frame_id, const std::string& source_frame_id, const double timeout)
{
    return_waitForTransform ret;
    bool b = m_iTf->waitForTransform(target_frame_id, source_frame_id, timeout);
    ret.retvalue = b;
    return ret;
}

return_getAllTransforms FrameTransform_nws_yarp::getAllTransforms()
{
    return_getAllTransforms ret;
    std::vector<FrameTransform> list;
    bool b = m_iTf->getAllTransforms(list);
    ret.transforms_list = list;
    ret.retvalue = b;
    return ret;
}

return_getAllStaticTransforms FrameTransform_nws_yarp::getAllStaticTransforms()
{
    return_getAllStaticTransforms ret;
    std::vector<FrameTransform> list;
    bool b = m_iTf->getAllStaticTransforms(list);
    ret.transforms_list = list;
    ret.retvalue = b;
    return ret;
}

return_setTransform2 FrameTransform_nws_yarp::setTransform2(const yarp::math::FrameTransform& transform)
{
    return_setTransform2 ret;
    bool b = m_iTf->setTransform(transform);
    ret.retvalue = b;
    return ret;
}

return_setTransformStatic2 FrameTransform_nws_yarp::setTransformStatic2(const yarp::math::FrameTransform& static_transform)
{
    return_setTransformStatic2 ret;
    bool b = m_iTf->setTransformStatic(static_transform);
    ret.retvalue = b;
    return ret;
}
