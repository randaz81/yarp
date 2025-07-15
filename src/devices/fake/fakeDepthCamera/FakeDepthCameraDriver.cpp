/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FakeDepthCameraDriver.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

#include <algorithm>
#include <map>
#include <cmath>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

namespace {
YARP_LOG_COMPONENT(FAKEDEPTHCAMERA, "yarp.device.fakeDepthCamera")
}

FakeDepthCameraDriver::FakeDepthCameraDriver()
{
    regenerate_rgb_image();
    regenerate_depth_image();
}

FakeDepthCameraDriver::~FakeDepthCameraDriver() = default;

bool FakeDepthCameraDriver::open(Searchable& config)
{
    if (!this->parseParams(config)) {return false;}

    return true;
}

bool FakeDepthCameraDriver::close()
{
    return true;
}

ReturnValue FakeDepthCameraDriver::getCameraDescription(yarp::dev::CameraDescriptor& camera)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::hasFeature(int feature, bool& hasFeature)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::setFeature(int feature, double value)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::getFeature(int feature, double& value)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::setFeature(int feature, double value1, double value2)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::getFeature(int feature, double& value1, double& value2)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::hasOnOff(int feature, bool& HasOnOff)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::setActive(int feature, bool onoff)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::getActive(int feature, bool& isActive)
{
    //not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::hasAuto(int feature, bool& hasAuto)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::hasManual(int feature, bool& hasManual)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::hasOnePush(int feature, bool& hasOnePush)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::setMode(int feature, yarp::dev::FeatureMode mode)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::getMode(int feature, yarp::dev::FeatureMode& mode)
{
    // not yet implemented
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver::setOnePush(int feature)
{
    // not yet implemented
    return ReturnValue_ok;
}
