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

#ifndef YARP_DEV_FRAMETRANSFORM_NWS_YARP_H
#define YARP_DEV_FRAMETRANSFORM_NWS_YARP_H

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
#include <yarp/dev/IFrameTransform.h>
#include <yarp/math/FrameTransform.h>

#include "FrameTransformRPCTest.h"

#define DEFAULT_THREAD_PERIOD 0.02 //s

/**
* @ingroup dev_impl_network_wrapper
 *
 * \brief `transformServer`: Documentation to be added
 */
class FrameTransform_nws_yarp :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public FrameTransformRPCTest
{
public:
    FrameTransform_nws_yarp();
    ~FrameTransform_nws_yarp();

    bool open(yarp::os::Searchable &params) override;
    bool close() override;

    bool threadInit() override;
    void threadRelease() override;
    void run() override;

private:
    std::mutex                   m_mutex;
    std::string                  m_streamingPortName;
    std::string                  m_rpcPortName;
    yarp::os::Stamp              m_lastStateStamp;
    double                       m_period;

    //the interface to the attached device
    yarp::dev::IFrameTransform* m_iTf = nullptr;

    yarp::os::BufferedPort<yarp::os::Bottle> m_streamingPort;

    //Thrift RPC interface
    yarp::os::RpcServer                        m_rpcPort;
    bool getTranformTest(const std::int32_t x) override;
    bool setTranformTest(const std::int32_t x) override;
    virtual return_allFramesAsString allFramesAsString() override;
    virtual return_canTransform canTransform(const std::string& target_frame, const std::string& source_frame) override;
    virtual return_clear clear() override;
    virtual return_frameExists frameExists(const std::string& frame_id) override;
    virtual return_getAllFrameIds getAllFrameIds() override;
    virtual return_getParent getParent(const std::string& frame_id) override;
    virtual return_getTransform getTransform(const std::string& target_frame_id, const std::string& source_frame_id) override;
    virtual return_setTransform setTransform(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Matrix& transform) override;
    virtual return_setTransformStatic setTransformStatic(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Matrix& transform) override;
    virtual return_deleteTransform deleteTransform(const std::string& target_frame_id, const std::string& source_frame_id) override;
    virtual return_transformPoint transformPoint(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Vector& input_point) override;
    virtual return_transformPose transformPose(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Vector& input_pose) override;
    virtual return_transformQuaternion transformQuaternion(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::math::Quaternion& input_quaternion) override;
    virtual return_waitForTransform waitForTransform(const std::string& target_frame_id, const std::string& source_frame_id, const double timeout) override;
    virtual return_getAllTransforms getAllTransforms() override;
    virtual return_getAllStaticTransforms getAllStaticTransforms() override;
    virtual return_setTransform2 setTransform2(const yarp::math::FrameTransform& transform) override;
    virtual return_setTransformStatic2 setTransformStatic2(const yarp::math::FrameTransform& static_transform) override;
};

#endif // YARP_DEV_FRAMETRANSFORM_NWS_YARP_H
