/*
 * SPDX-FileCopyrightText: 2025-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef YARP_OPENCV_DEVICE_OPENCVWRITER_H
#define YARP_OPENCV_DEVICE_OPENCVWRITER_H

#include <yarp/os/Property.h>
#include <yarp/dev/IFrameWriterImage.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>
#include "OpenCVWriter_ParamsParser.h"

#include <opencv2/videoio.hpp>

/**
 * @ingroup dev_impl_media
 *
 * @brief `OpenCVWriter`: A device which receives images and writes a video file to disk using openCV
 *
 * Parameters required by this device are shown in class: OpenCVGrabber_ParamsParser
 */
class OpenCVWriter :
        public yarp::dev::IFrameWriterImage,
        public yarp::dev::DeviceDriver,
        public OpenCVWriter_ParamsParser
{
public:

    OpenCVWriter() {}
    virtual ~OpenCVWriter() {}


    bool open(yarp::os::Searchable & config) override;
    bool close() override;
    yarp::dev::ReturnValue putImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) override;

protected:
    cv::VideoWriter m_writer;
    bool m_isInitialized =false;
};

#endif // YARP_OPENCV_DEVICE_OPENCVWRITER_H
