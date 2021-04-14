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

#ifndef YARP_DEV_FRAMETRANSFORMNWSROS_H
#define YARP_DEV_FRAMETRANSFORMNWSROS_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <mutex>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/api.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Node.h>
#include <yarp/dev/IFrameTransform.h>

#include <yarp/math/FrameTransform.h>

#include <yarp/rosmsg/geometry_msgs/TransformStamped.h>
#include <yarp/rosmsg/tf2_msgs/TFMessage.h>


#define ROSNODENAME "/tfNode"
#define ROSTOPICNAME_TF "/tf"
#define ROSTOPICNAME_TF_STATIC "/tf_static"
#define DEFAULT_THREAD_PERIOD 0.02 //s

/**
* @ingroup dev_impl_network_wrapper @@@@@@@@@@@@@@@@@@@@@@@@@@@@
 *
 * \brief `FrameTransform_nws_ros`: Documentation to be added
 */
class FrameTransform_nws_ros :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::os::PortReader
{
public:
    FrameTransform_nws_ros();
    ~FrameTransform_nws_ros();

    bool open(yarp::os::Searchable &params) override;
    bool close() override;
    yarp::os::Bottle getOptions();

    bool threadInit() override;
    void threadRelease() override;
    void run() override;

private:
    std::mutex                   m_mutex;
    std::string                  m_rpcPortName;
    yarp::os::Stamp              m_lastStateStamp;
    double                       m_period;
    yarp::os::Node*              m_rosNode;
    bool                         m_enable_publish_ros_tf;
    bool                         m_enable_subscribe_ros_tf;

    //the interface to the attached device
    IFrameTransform*             m_iTf = nullptr;

    yarp::os::RpcServer                      m_rpcPort;
    yarp::os::Publisher<yarp::rosmsg::tf2_msgs::TFMessage> m_rosPublisherPort_tf_timed;
    yarp::os::Publisher<yarp::rosmsg::tf2_msgs::TFMessage> m_rosPublisherPort_tf_static;
    yarp::os::Subscriber<yarp::rosmsg::tf2_msgs::TFMessage> m_rosSubscriberPort_tf_timed;
    yarp::os::Subscriber<yarp::rosmsg::tf2_msgs::TFMessage> m_rosSubscriberPort_tf_static;

    bool read(yarp::os::ConnectionReader& connection) override;
};

#endif // YARP_DEV_FRAMETRANSFORMSERVER_FRAMETRANSFORMSERVER_H
