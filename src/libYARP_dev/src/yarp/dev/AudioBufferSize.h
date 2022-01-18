/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_AUDIOBUFFERSIZE_H
#define YARP_DEV_AUDIOBUFFERSIZE_H

#include <yarp/os/Portable.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/PortWriter.h>
#include <yarp/dev/api.h>
#include <yarp/dev/audioBufferSizeDataStorage.h>

namespace yarp::dev {

template <typename SAMPLE>
class CircularAudioBuffer;

class YARP_dev_API AudioBufferSize :
        private audioBufferSizeDataStorage
{
    template <typename SAMPLE>
    friend class CircularAudioBuffer;

public:
    size_t getSamples() { return m_samples; }
    size_t getChannels() { return m_channels; }
    size_t getBufferElements() { return size; }
    size_t getBytes() { return size_t(m_samples * m_channels * m_depth); }

    AudioBufferSize();
    AudioBufferSize(size_t samples, size_t channels, size_t depth_in_bytes);
};

} // namespace yarp::dev

#endif // YARP_DEV_AUDIOBUFFERSIZE_H
