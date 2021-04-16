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

#ifndef YARP_DEV_FRAMETRANSFORM_NWC_YARP_H
#define YARP_DEV_FRAMETRANSFORM_NWC_YARP_H


#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IFrameTransformClientControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/os/PeriodicThread.h>
#include <mutex>


#define DEFAULT_THREAD_PERIOD 20 //ms
const int TRANSFORM_TIMEOUT_MS = 100; //ms
const int MAX_PORTS = 5;

/**
* @ingroup dev_impl_network_clients
*
* \brief `transformClient`: Documentation to be added
*/
class FrameTransform_nwc_yarp :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IFrameTransform,
        public yarp::dev::IFrameTransformClientControl,
        public yarp::os::PortReader,
        public yarp::os::PeriodicThread
{
protected:

    yarp::os::Port                m_rpc_InterfaceToServer;
    yarp::os::Port                m_rpc_InterfaceToUser;
    std::string         m_local_name;
    std::string         m_remote_name;

    std::string         m_local_rpcServer;
    std::string         m_local_rpcUser;
    std::string         m_remote_rpc;
    std::string         m_remote_streaming_name;
    std::string         m_local_streaming_name;

    std::string         m_streaming_connection_type;
    double                        m_period;
    std::mutex               m_rpc_mutex;
    struct broadcast_port_t
    {
        std::string format;
        yarp::os::Port port;
        std::string transform_src;
        std::string transform_dst;
    };
    std::vector<broadcast_port_t*>  m_array_of_ports;

public:

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;
    bool read(yarp::os::ConnectionReader& connection) override;

    /* IPreciselyTimed methods */
    /**
    * Get the time stamp for the last read data
    * @return last time stamp.
    */
    yarp::os::Stamp getLastInputStamp();

     bool     allFramesAsString(std::string &all_frames) override;
     bool     canTransform(const std::string &target_frame, const std::string &source_frame) override;
     bool     clear() override;
     bool     frameExists(const std::string &frame_id) override;
     bool     getAllFrameIds(std::vector< std::string > &ids) override;
     bool     getParent(const std::string &frame_id, std::string &parent_frame_id) override;
     bool     getTransform(const std::string &target_frame_id, const std::string &source_frame_id, yarp::sig::Matrix &transform) override;
     bool     setTransform(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Matrix &transform) override;
     bool     setTransformStatic(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Matrix &transform) override;
     bool     deleteTransform(const std::string &target_frame_id, const std::string &source_frame_id) override;
     bool     transformPoint(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_point, yarp::sig::Vector &transformed_point) override;
     bool     transformPose(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_pose, yarp::sig::Vector &transformed_pose) override;
     bool     transformQuaternion(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::math::Quaternion &input_quaternion, yarp::math::Quaternion &transformed_quaternion) override;
     bool     waitForTransform(const std::string &target_frame_id, const std::string &source_frame_id, const double &timeout) override;

     bool     isConnectedWithServer() override;
     bool     reconnectWithServer() override;

     FrameTransform_nwc_yarp();
    ~FrameTransform_nwc_yarp();
     bool     threadInit() override;
     void     threadRelease() override;
     void     run() override;
};

#endif // YARP_DEV_FRAMETRANSFORMCLIENT_H
