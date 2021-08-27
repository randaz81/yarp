/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "Localization2DClient.h"
#include <yarp/dev/ILocalization2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <mutex>

/*! \file Localization2DClient.cpp */

using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::os;
using namespace yarp::sig;

namespace {
YARP_LOG_COMPONENT(LOCALIZATION2DCLIENT, "yarp.device.localization2DClient")
}

//------------------------------------------------------------------------------------------------------------------------------

bool Localization2DClient::open(yarp::os::Searchable &config)
{
    m_local_name.clear();
    m_remote_name.clear();

    m_local_name = config.find("local").asString();
    m_remote_name = config.find("remote").asString();

    if (m_local_name == "")
    {
        yCError(LOCALIZATION2DCLIENT, "open() error you have to provide a valid 'local' param");
        return false;
    }

    if (m_remote_name == "")
    {
        yCError(LOCALIZATION2DCLIENT, "open() error you have to provide valid 'remote' param");
        return false;
    }

    std::string
            local_rpc,
            remote_rpc,
            remote_streaming_name,
            local_streaming_name;

    local_rpc             = m_local_name  + "/localization/rpc";
    remote_rpc            = m_remote_name + "/rpc";
    remote_streaming_name = m_remote_name + "/stream:o";
    local_streaming_name  = m_local_name  + "/stream:i";

    if (!m_rpc_port_localization_server.open(local_rpc))
    {
        yCError(LOCALIZATION2DCLIENT, "open() error could not open rpc port %s, chlocal_streaming_nameeck network", local_rpc.c_str());
        return false;
    }

    /*
    //currently unused
    bool ok=Network::connect(remote_streaming_name.c_str(), local_streaming_name.c_str(), "tcp");
    if (!ok)
    {
        yCError(LOCALIZATION2DCLIENT, "open() error could not connect to %s", remote_streaming_name.c_str());
        return false;
    }*/

    bool ok = true;

    ok = Network::connect(local_rpc, remote_rpc);
    if (!ok)
    {
        yCError(LOCALIZATION2DCLIENT, "open() error could not connect to %s", remote_rpc.c_str());
        return false;
    }

    if (!m_RPC.yarp().attachAsClient(m_rpc_port_localization_server))
    {
        yCError(LOCALIZATION2DCLIENT, "Error! Cannot attach the port as a client");
        return false;
    }

    return true;
}

bool Localization2DClient::setInitialPose(const Map2DLocation& loc)
{
    std::lock_guard <std::mutex> lg(m_mutex);
    return m_RPC.setInitialPoseRPC1(loc);
}

bool Localization2DClient::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    if (cov.rows() != 3 || cov.cols() != 3)
    {
        yCError(LOCALIZATION2DCLIENT) << "Covariance matrix is expected to have size (3,3)";
        return false;
    }

    std::lock_guard <std::mutex> lg(m_mutex);
    return m_RPC.setInitialPoseRPC2(loc,cov);
}

bool  Localization2DClient::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    std::lock_guard <std::mutex> lg(m_mutex);
    auto ret = m_RPC.getEstimatedOdometryRPC();
    if (!ret.ret)
    {
        yCError(LOCALIZATION2DCLIENT, "Unable to set transformation");
        return false;
    }
    odom = ret.odom;
    return true;
}

bool  Localization2DClient::getCurrentPosition(Map2DLocation& loc)
{
    std::lock_guard <std::mutex> lg(m_mutex);
    auto ret = m_RPC.getCurrentPositionRPC1();
    if (!ret.ret)
    {
        yCError(LOCALIZATION2DCLIENT, "Unable to set transformation");
        return false;
    }
    loc = ret.loc;
    return true;
}

bool  Localization2DClient::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    std::lock_guard <std::mutex> lg(m_mutex);
    auto ret = m_RPC.getCurrentPositionRPC2();
    if (!ret.ret)
    {
        yCError(LOCALIZATION2DCLIENT, "Unable to set transformation");
        return false;
    }
    loc = ret.loc;
    cov = ret.cov;
    return true;
}

bool  Localization2DClient::getEstimatedPoses(std::vector<Map2DLocation>& poses)
{
    std::lock_guard <std::mutex> lg(m_mutex);
    auto ret = m_RPC.getEstimatedPosesRPC();
    if (!ret.ret)
    {
        yCError(LOCALIZATION2DCLIENT, "Unable to set transformation");
        return false;
    }
    poses = ret.poses;
    return true;
}

bool  Localization2DClient::getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status)
{
    std::lock_guard <std::mutex> lg(m_mutex);
    auto ret = m_RPC.getLocalizationStatusRPC();
    if (!ret.ret)
    {
        yCError(LOCALIZATION2DCLIENT, "Unable to set transformation");
        return false;
    }
    status = yarp::dev::Nav2D::LocalizationStatusEnum(ret.status);
    return true;
}

bool  Localization2DClient::startLocalizationService()
{
    std::lock_guard <std::mutex> lg(m_mutex);
    return m_RPC.startLocalizationServiceRPC();
}

bool  Localization2DClient::stopLocalizationService()
{
    std::lock_guard <std::mutex> lg(m_mutex);
    return m_RPC.stopLocalizationServiceRPC();
}

bool Localization2DClient::close()
{
    m_rpc_port_localization_server.close();
    return true;
}
