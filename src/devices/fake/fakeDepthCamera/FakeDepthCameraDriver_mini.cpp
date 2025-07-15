/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FakeDepthCameraDriver_mini.h"

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

FakeDepthCameraDriver_mini::FakeDepthCameraDriver_mini()
{
    regenerate_rgb_image();
    regenerate_depth_image();
}

FakeDepthCameraDriver_mini::~FakeDepthCameraDriver_mini() = default;

bool FakeDepthCameraDriver_mini::open(Searchable& config)
{
    if (!this->parseParams(config)) {return false;}

    return true;
}

bool FakeDepthCameraDriver_mini::close()
{
    return true;
}

int FakeDepthCameraDriver_mini::getRgbHeight()
{
    return m_rgb_height;
}

int FakeDepthCameraDriver_mini::getRgbWidth()
{
    return m_rgb_width;
}

ReturnValue FakeDepthCameraDriver_mini::getRgbSupportedConfigurations(std::vector<CameraConfig>& configurations)
{
    CameraConfig cfg;
    configurations.push_back(cfg);
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getRgbResolution(int& width, int& height)
{
    width = m_rgb_width;
    height = m_rgb_height;
    return ReturnValue_ok;
}

void FakeDepthCameraDriver_mini::regenerate_rgb_image()
{
    m_rgbImage.setPixelCode(VOCAB_PIXEL_RGB);
    m_rgbImage.resize(m_rgb_width, m_rgb_height);
}

void FakeDepthCameraDriver_mini::regenerate_depth_image()
{
    m_depthImage.resize(m_depth_width, m_depth_height);
}

ReturnValue FakeDepthCameraDriver_mini::setRgbResolution(int width, int height)
{
    m_rgb_width = width;
    m_rgb_height = height;
    regenerate_rgb_image();
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthResolution(int& width, int& height)
{
    width = m_depth_width;
    height = m_depth_height;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setDepthResolution(int width, int height)
{
    m_depth_width = width;
    m_depth_height = height;
    regenerate_depth_image();
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setRgbFOV(double horizontalFov, double verticalFov)
{
    m_rgb_Hfov = horizontalFov;
    m_rgb_Vfov = verticalFov;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setDepthFOV(double horizontalFov, double verticalFov)
{
    m_dep_Hfov = horizontalFov;
    m_dep_Vfov = verticalFov;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setDepthAccuracy(double in_accuracy)
{
    m_accuracy = in_accuracy;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    horizontalFov = m_rgb_Hfov;
    verticalFov   = m_rgb_Vfov;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getRgbMirroring(bool& mirror)
{
    mirror = m_rgb_mirror;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setRgbMirroring(bool mirror)
{
    m_rgb_mirror = mirror;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getRgbIntrinsicParam(Property& intrinsic)
{
    intrinsic.put("physFocalLength", 0.5);
    intrinsic.put("focalLengthX",    512);
    intrinsic.put("focalLengthY",    512);
    intrinsic.put("principalPointX", 235);
    intrinsic.put("principalPointY", 231);
    intrinsic.put("distortionModel", "plumb_bob");
    intrinsic.put("k1", 0);
    intrinsic.put("k2", 0);
    intrinsic.put("t1", 0);
    intrinsic.put("t2", 0);
    intrinsic.put("k3", 0);

    intrinsic.put("stamp", yarp::os::Time::now());
    return ReturnValue_ok;
}

int FakeDepthCameraDriver_mini::getDepthHeight()
{
    return m_depth_height;
}

int FakeDepthCameraDriver_mini::getDepthWidth()
{
    return m_depth_width;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    horizontalFov = m_dep_Hfov;
    verticalFov   = m_dep_Vfov;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthIntrinsicParam(Property& intrinsic)
{
    intrinsic.put("physFocalLength", 0.5);
    intrinsic.put("focalLengthX",    512);
    intrinsic.put("focalLengthY",    512);
    intrinsic.put("principalPointX", 235);
    intrinsic.put("principalPointY", 231);
    intrinsic.put("distortionModel", "plumb_bob");
    intrinsic.put("k1", 0);
    intrinsic.put("k2", 0);
    intrinsic.put("t1", 0);
    intrinsic.put("t2", 0);
    intrinsic.put("k3", 0);

    intrinsic.put("stamp", yarp::os::Time::now());
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthAccuracy(double& accuracy)
{
    accuracy = m_accuracy;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    nearPlane = m_dep_near;
    farPlane  = m_dep_far;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setDepthClipPlanes(double nearPlane, double farPlane)
{
    m_dep_near = nearPlane;
    m_dep_far  = farPlane;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthMirroring(bool& mirror)
{
    mirror = m_depth_mirror;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::setDepthMirroring(bool _mirror)
{
    m_depth_mirror = _mirror;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getExtrinsicParam(Matrix& extrinsic)
{
    extrinsic.resize(4, 4);
    extrinsic.zero();

    extrinsic[0][0] = 1;
    extrinsic[1][1] = 1;
    extrinsic[2][2] = 1;
    extrinsic[3][3] = 1;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getRgbImage(FlexImage& rgbImage, Stamp* timeStamp)
{
    rgbImage = m_rgbImage;
    if (timeStamp) {
        timeStamp->update(yarp::os::Time::now());
    }
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getDepthImage(ImageOf<PixelFloat>& depthImage, Stamp* timeStamp)
{
    depthImage = m_depthImage;
    if (timeStamp) {
        timeStamp->update(yarp::os::Time::now());
    }
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getImages(FlexImage& colorFrame, ImageOf<PixelFloat>& depthFrame, Stamp* colorStamp, Stamp* depthStamp)
{
    auto r1 = getRgbImage(colorFrame, colorStamp);
    auto r2 = getDepthImage(depthFrame, depthStamp);
    ReturnValue rr = r1 && r2;
    return rr;
}

ReturnValue FakeDepthCameraDriver_mini::getSensorStatus(IRGBDSensor::RGBDSensor_status& status)
{
    status = RGBD_SENSOR_OK_IN_USE;
    return ReturnValue_ok;
}

ReturnValue FakeDepthCameraDriver_mini::getLastErrorMsg(std::string& msg, Stamp* timeStamp)
{
    msg = std::string("no error");
    return ReturnValue_ok;
}
