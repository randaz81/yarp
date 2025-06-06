/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IAUDIOGRABBERSOUND_H
#define YARP_DEV_IAUDIOGRABBERSOUND_H

#include <yarp/dev/api.h>
#include <yarp/dev/ReturnValue.h>
#include <yarp/sig/AudioBufferSize.h>
#include <yarp/sig/Sound.h>

namespace yarp::dev {

/**
 * @ingroup dev_iface_media
 *
 * Read a YARP-format sound block from a device.
 */
class YARP_dev_API IAudioGrabberSound
{
public:
    /**
     * Destructor.
     */
    virtual ~IAudioGrabberSound();

    /**
     * Get a sound from a device.
     * Examples:
     * getSound(s, 0,   100,    0.1); returns a sound whose size can vary between 0 and 100, with a maximum blocking time of 0.1
     * getSound(s, 100, 100,    0.0); returns a sound with exact size of 100. It may block forever (more specifically, until sound size is at least 100).
     * getSound(s, 100, 100000, 0.0); returns a sound with a minimum size of 0, while trying to transfer all the internal buffer. It may block forever (more specifically, until sound size is at least 100).
     * @param sound the sound to be filled
     * @param min_number_of_samples. The function will block until the driver is able to collect at least min_number_of_samples. If set to zero, the function may return empty sounds.
     * @param max_number_of_samples. The function will block until the driver is either able to collect max_number_of_samples or the timeout expires.
     * @param max_samples_timeout_s. The timeout (in seconds) to retrieve max_number_of_samples.
     * @return true upon success, false for an invalid set of parameters, such as max_number_of_samples<min_number_of_samples, etc.
     */
    virtual yarp::dev::ReturnValue getSound(yarp::sig::Sound& sound, size_t min_number_of_samples, size_t max_number_of_samples, double max_samples_timeout_s) = 0;

    /**
     * Start the recording.
     *
     * @return true/false upon success/failure
     */
    virtual yarp::dev::ReturnValue startRecording() = 0;

    /**
     * Stop the recording.
     *
     * @return true/false upon success/failure
     */
    virtual yarp::dev::ReturnValue stopRecording() = 0;

    /**
     * Check if the recording has been enabled (e.g. via startRecording()/stopRecording())
     * @param recording_enabled the status of the device
     * @return true/false upon success/failure
     */
    virtual yarp::dev::ReturnValue isRecording(bool& recording_enabled) = 0;

    virtual yarp::dev::ReturnValue getRecordingAudioBufferMaxSize(yarp::sig::AudioBufferSize& size) = 0;

    virtual yarp::dev::ReturnValue getRecordingAudioBufferCurrentSize(yarp::sig::AudioBufferSize& size) = 0;

    virtual yarp::dev::ReturnValue resetRecordingAudioBuffer() = 0;

    /**
     * Sets a software gain for the grabbed audio
     * @param gain the audio gain (1.0 is the default value)
     * @return true/false upon success/failure
     */
    virtual yarp::dev::ReturnValue setSWGain(double gain) = 0;

    /**
     * Sets the hardware gain of the grabbing device (if supported by the hardware)
     * @param gain the audio gain (1.0 is the default value)
     * @return true/false upon success/failure
     */
    virtual yarp::dev::ReturnValue setHWGain(double gain) = 0;
};

} // namespace yarp::dev

#endif // YARP_DEV_IAUDIOGRABBERSOUND_H
