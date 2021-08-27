/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef YARP_DEV_LOCALIZATION2DSERVERIMPL_H
#define YARP_DEV_LOCALIZATION2DSERVERIMPL_H

#include "ILocalization2DRPC.h"
#include <yarp/dev/ILocalization2D.h>
#include <yarp/os/Stamp.h>

class Localization2DServer;

class ILocalization2DRPCd : public ILocalization2DRPC
{
    private:
    yarp::dev::Nav2D::ILocalization2D* m_iLoc = nullptr;
    std::mutex                         m_mutex;

    public:
    void setInterface(yarp::dev::Nav2D::ILocalization2D* _iloc) { m_iLoc = _iloc; }

    bool startLocalizationServiceRPC() override;
    bool stopLocalizationServiceRPC()  override;
    return_getLocalizationStatusRPC getLocalizationStatusRPC() override;
    return_getEstimatedPosesRPC getEstimatedPosesRPC() override;
    return_getCurrentPositionRPC1 getCurrentPositionRPC1() override;
    return_getCurrentPositionRPC2 getCurrentPositionRPC2() override;
    return_getEstimatedOdometryRPC getEstimatedOdometryRPC() override;
    bool setInitialPoseRPC1(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool setInitialPoseRPC2(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;

    std::mutex* getMutex() {return &m_mutex;}

    yarp::dev::OdometryData                     m_current_odometry;
    yarp::dev::Nav2D::Map2DLocation             m_current_position;
    yarp::dev::Nav2D::LocalizationStatusEnum    m_current_status = yarp::dev::Nav2D::LocalizationStatusEnum::localization_status_not_yet_localized;

    yarp::os::Stamp                         m_loc_stamp;
    yarp::os::Stamp                         m_odom_stamp;
    bool                                    m_getdata_using_periodic_thread = true;
};

#endif // YARP_DEV_LOCALIZATION2DSERVERIMPL_H
