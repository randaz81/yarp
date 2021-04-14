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

// example: yarpdev --device transformServer --ROS::enable_ros_publisher 0 --ROS::enable_ros_subscriber 0

#define _USE_MATH_DEFINES
#include <cmath>

#include "FrameTransform_nws_ros.h"
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
YARP_LOG_COMPONENT(FRAMETRANSFORMNWSROS, "yarp.device.frameTransform_nws_ros")
}

/**
  * FrameTransform_nws_ros
  */

FrameTransform_nws_ros::FrameTransform_nws_ros() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_period = DEFAULT_THREAD_PERIOD;
    m_enable_publish_ros_tf = false;
    m_enable_subscribe_ros_tf = false;
    m_yarp_static_transform_storage = nullptr;
    m_yarp_timed_transform_storage = nullptr;
    m_ros_static_transform_storage = nullptr;
    m_ros_timed_transform_storage = nullptr;
    m_rosNode = nullptr;
    m_FrameTransformTimeout = 0.200; //ms
}

FrameTransform_nws_ros::~FrameTransform_nws_ros()
{
    threadRelease();
}

bool FrameTransform_nws_ros::read(yarp::os::ConnectionReader& connection)
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
    if(request == "help")
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
        ret = m_iTf->set_transform_static(t);
        if (ret == true)
        {
            yCInfo(FRAMETRANSFORMNWSROS) << "set_static_transform done";
            out.addString("set_static_transform done");
        }
        else
        {
            yCError(FRAMETRANSFORMNWSROS) << "read(): something strange happened";
        }
    }
    else if(request == "delete_all")
    {
        m_iTf->clear();
        yCInfo(FRAMETRANSFORMNWSROS) << "delete_all done";
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
        yCError(FRAMETRANSFORMNWSROS, "Invalid vocab received");
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
        yCError(FRAMETRANSFORMNWSROS) << "Invalid return to sender";
    }
    return true;
}

bool FrameTransform_nws_ros::threadInit()
{
    //open rpc port
    if (!m_rpcPort.open(m_rpcPortName))
    {
        yCError(FRAMETRANSFORMNWSROS, "Failed to open port %s", m_rpcPortName.c_str());
        return false;
    }
    m_rpcPort.setReader(*this);

    //open ros publisher (if requested)
    if (m_enable_publish_ros_tf)
    {
        if (m_rosNode == nullptr)
        {
            m_rosNode = new yarp::os::Node(ROSNODENAME);
        }
        if (!m_rosPublisherPort_tf_timed.topic(ROSTOPICNAME_TF))
        {
            yCError(FRAMETRANSFORMNWSROS) << "Unable to publish data on " << ROSTOPICNAME_TF << " topic, check your yarp-ROS network configuration";
            return false;
        }
        if (!m_rosPublisherPort_tf_static.topic(ROSTOPICNAME_TF_STATIC))
        {
            yCError(FRAMETRANSFORMNWSROS) << "Unable to publish data on " << ROSTOPICNAME_TF_STATIC << " topic, check your yarp-ROS network configuration";
            return false;
        }
    }

    //open ros subscriber(if requested)
    if (m_enable_subscribe_ros_tf)
    {
        if (m_rosNode == nullptr)
        {
            m_rosNode = new yarp::os::Node(ROSNODENAME);
        }
        if (!m_rosSubscriberPort_tf_timed.topic(ROSTOPICNAME_TF))
        {
            yCError(FRAMETRANSFORMNWSROS) << "Unable to subscribe to " << ROSTOPICNAME_TF << " topic, check your yarp-ROS network configuration";
            return false;
        }
        m_rosSubscriberPort_tf_timed.setStrict();
        if (!m_rosSubscriberPort_tf_static.topic(ROSTOPICNAME_TF_STATIC))
        {
            yCError(FRAMETRANSFORMNWSROS) << "Unable to subscribe to " << ROSTOPICNAME_TF_STATIC << " topic, check your yarp-ROS network configuration";
            return false;
        }
        m_rosSubscriberPort_tf_static.setStrict();
    }

    yCInfo(FRAMETRANSFORMNWSROS) << "Transform server started";
    return true;
}

bool FrameTransform_nws_ros::open(yarp::os::Searchable &config)
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
        yCInfo(FRAMETRANSFORMNWSROS) << "Thread period set to:" << m_period;
    }

    std::string name;
    if (!config.check("name"))
    {
        name = "frameTransform_nws_ros";
    }
    else
    {
        name = config.find("name").asString();
    }
    m_rpcPortName = "/" + name + "/rpc";

    //ROS configuration
    if (!config.check("ROS"))
    {
        yCError(FRAMETRANSFORMNWSROS) << "Missing ROS group";
        return false;
    }
    Bottle ROS_config = config.findGroup("ROS");
    if (ROS_config.check("enable_ros_publisher") == false)
    {
        yCError(FRAMETRANSFORMNWSROS) << "Missing 'enable_ros_publisher' in ROS group";
        return false;
    }
    if (ROS_config.find("enable_ros_publisher").asInt32() == 1 || ROS_config.find("enable_ros_publisher").asString() == "true")
    {
        m_enable_publish_ros_tf = true;
        yCInfo(FRAMETRANSFORMNWSROS) << "Enabled ROS publisher";
    }
    if (ROS_config.check("enable_ros_subscriber") == false)
    {
        yCError(FRAMETRANSFORMNWSROS) << "Missing 'enable_ros_subscriber' in ROS group";
        return false;
    }
    if (ROS_config.find("enable_ros_subscriber").asInt32() == 1 || ROS_config.find("enable_ros_subscriber").asString() == "true")
    {
        m_enable_subscribe_ros_tf = true;
        yCInfo(FRAMETRANSFORMNWSROS) << "Enabled ROS subscriber";
    }

    this->start();

    yarp::os::Time::delay(0.5);

    return true;
}

void FrameTransform_nws_ros::threadRelease()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();
    if (m_enable_publish_ros_tf)
    {
        m_rosPublisherPort_tf_timed.interrupt();
        m_rosPublisherPort_tf_timed.close();
    }
    if (m_enable_subscribe_ros_tf)
    {
        m_rosSubscriberPort_tf_timed.interrupt();
        m_rosSubscriberPort_tf_timed.close();
    }
    if (m_enable_publish_ros_tf)
    {
        m_rosPublisherPort_tf_static.interrupt();
        m_rosPublisherPort_tf_static.close();
    }
    if (m_enable_subscribe_ros_tf)
    {
        m_rosSubscriberPort_tf_static.interrupt();
        m_rosSubscriberPort_tf_static.close();
    }
    if (m_rosNode)
    {
        m_rosNode->interrupt();
        delete  m_rosNode;
        m_rosNode = nullptr;
    }
}

void FrameTransform_nws_ros::run()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (true)
    {
        double current_time = yarp::os::Time::now();

        //ros subscriber
        if (m_enable_subscribe_ros_tf)
        {
            yarp::rosmsg::tf2_msgs::TFMessage* rosInData_timed = nullptr;
            do
            {
                rosInData_timed = m_rosSubscriberPort_tf_timed.read(false);
                if (rosInData_timed != nullptr)
                {
                    std::vector <yarp::rosmsg::geometry_msgs::TransformStamped> tfs = rosInData_timed->transforms;
                    size_t tfs_size = tfs.size();
                    for (size_t i = 0; i < tfs_size; i++)
                    {
                        FrameTransform t;
                        t.translation.tX = tfs[i].transform.translation.x;
                        t.translation.tY = tfs[i].transform.translation.y;
                        t.translation.tZ = tfs[i].transform.translation.z;
                        t.rotation.x() = tfs[i].transform.rotation.x;
                        t.rotation.y() = tfs[i].transform.rotation.y;
                        t.rotation.z() = tfs[i].transform.rotation.z;
                        t.rotation.w() = tfs[i].transform.rotation.w;
                        t.src_frame_id = tfs[i].header.frame_id;
                        t.dst_frame_id = tfs[i].child_frame_id;
                        //@@@ should we use yarp or ROS timestamps?
                        t.timestamp = yarp::os::Time::now();
                        //t.timestamp = tfs[i].header.stamp.sec; //@@@this needs some revising
                        m_iTf->setTransform(t);
                    }
                }
            } while (rosInData_timed != nullptr);

            yarp::rosmsg::tf2_msgs::TFMessage* rosInData_static = nullptr;
            do
            {
                rosInData_static = m_rosSubscriberPort_tf_static.read(false);
                if (rosInData_static != nullptr)
                {
                    std::vector <yarp::rosmsg::geometry_msgs::TransformStamped> tfs = rosInData_static->transforms;
                    size_t tfs_size = tfs.size();
                    for (size_t i = 0; i < tfs_size; i++)
                    {
                        FrameTransform t;
                        t.translation.tX = tfs[i].transform.translation.x;
                        t.translation.tY = tfs[i].transform.translation.y;
                        t.translation.tZ = tfs[i].transform.translation.z;
                        t.rotation.x() = tfs[i].transform.rotation.x;
                        t.rotation.y() = tfs[i].transform.rotation.y;
                        t.rotation.z() = tfs[i].transform.rotation.z;
                        t.rotation.w() = tfs[i].transform.rotation.w;
                        t.src_frame_id = tfs[i].header.frame_id;
                        t.dst_frame_id = tfs[i].child_frame_id;
                        //@@@ should we use yarp or ROS timestamps?
                        t.timestamp = yarp::os::Time::now();
                        //t.timestamp = tfs[i].header.stamp; //@@@ is this ok?
                        m_iTf->set_transform(t);
                    }
                }
            } while (rosInData_static != nullptr);
        }

        m_lastStateStamp.update();
        size_t    tfVecSize_static_yarp = m_yarp_static_transform_storage->size();
        size_t    tfVecSize_timed_yarp = m_yarp_timed_transform_storage->size();
        size_t    tfVecSize_static_ros  = m_ros_static_transform_storage->size();
        size_t    tfVecSize_timed_ros = m_ros_timed_transform_storage->size();

        //ros publisher
        if (m_enable_publish_ros_tf)
        {
            static int                        rosMsgCounter = 0;
            yarp::rosmsg::tf2_msgs::TFMessage& rosOutData_timed = m_rosPublisherPort_tf_timed.prepare();
            yarp::rosmsg::geometry_msgs::TransformStamped transform_timed;
            rosOutData_timed.transforms.clear();
            for (size_t i = 0; i < tfVecSize_timed_yarp; i++)
            {
                transform_timed.child_frame_id = (*m_yarp_timed_transform_storage)[i].dst_frame_id;
                transform_timed.header.frame_id = (*m_yarp_timed_transform_storage)[i].src_frame_id;
                transform_timed.header.seq = rosMsgCounter;
                transform_timed.header.stamp = (*m_yarp_timed_transform_storage)[i].timestamp;
                transform_timed.transform.rotation.x = (*m_yarp_timed_transform_storage)[i].rotation.x();
                transform_timed.transform.rotation.y = (*m_yarp_timed_transform_storage)[i].rotation.y();
                transform_timed.transform.rotation.z = (*m_yarp_timed_transform_storage)[i].rotation.z();
                transform_timed.transform.rotation.w = (*m_yarp_timed_transform_storage)[i].rotation.w();
                transform_timed.transform.translation.x = (*m_yarp_timed_transform_storage)[i].translation.tX;
                transform_timed.transform.translation.y = (*m_yarp_timed_transform_storage)[i].translation.tY;
                transform_timed.transform.translation.z = (*m_yarp_timed_transform_storage)[i].translation.tZ;

                rosOutData_timed.transforms.push_back(transform_timed);
            }
            m_rosPublisherPort_tf_timed.write();

            yarp::rosmsg::tf2_msgs::TFMessage& rosOutData_static = m_rosPublisherPort_tf_static.prepare();
            yarp::rosmsg::geometry_msgs::TransformStamped transform_static;
            rosOutData_static.transforms.clear();
            for (size_t i = 0; i < tfVecSize_static_yarp; i++)
            {
                transform_static.child_frame_id = (*m_yarp_static_transform_storage)[i].dst_frame_id;
                transform_static.header.frame_id = (*m_yarp_static_transform_storage)[i].src_frame_id;
                transform_static.header.seq = rosMsgCounter;
                transform_static.header.stamp = yarp::os::Time::now(); //@@@check timestamp of static transform?
                transform_static.transform.rotation.x = (*m_yarp_static_transform_storage)[i].rotation.x();
                transform_static.transform.rotation.y = (*m_yarp_static_transform_storage)[i].rotation.y();
                transform_static.transform.rotation.z = (*m_yarp_static_transform_storage)[i].rotation.z();
                transform_static.transform.rotation.w = (*m_yarp_static_transform_storage)[i].rotation.w();
                transform_static.transform.translation.x = (*m_yarp_static_transform_storage)[i].translation.tX;
                transform_static.transform.translation.y = (*m_yarp_static_transform_storage)[i].translation.tY;
                transform_static.transform.translation.z = (*m_yarp_static_transform_storage)[i].translation.tZ;

                rosOutData_static.transforms.push_back(transform_static);
            }
            m_rosPublisherPort_tf_static.write();


            rosMsgCounter++;
        }

    }
    else
    {
        yCError(FRAMETRANSFORMNWSROS, "Returned error");
    }
}

bool FrameTransform_nws_ros::close()
{
    yCTrace(FRAMETRANSFORMNWSROS, "Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    return true;
}
