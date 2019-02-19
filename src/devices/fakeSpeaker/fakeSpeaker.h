/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/dev/CircularAudioBuffer.h>
#include <yarp/sig/Sound.h>
#include <yarp/sig/SoundFile.h>

namespace yarp{
    namespace dev{
        class fakeSpeaker;
    }
}

#define DEFAULT_PERIOD 0.01   //s

/**
* \brief `fakeSpeaker` : fake device implementing the IAudioRender device interface to play sound
*
*/
class yarp::dev::fakeSpeaker :  public DeviceDriver,
                            public yarp::dev::IAudioRender,
                            public yarp::os::PeriodicThread
{
public:
    fakeSpeaker();
    ~fakeSpeaker();

    // Device Driver interface
    bool open(yarp::os::Searchable &config) override;
    bool close() override;

    virtual bool renderSound(const yarp::sig::Sound& sound)  override;
    virtual bool getPlaybackAudioBufferMaxSize(yarp::dev::audio_buffer_size& size)  override;
    virtual bool getPlaybackAudioBufferCurrentSize(yarp::dev::audio_buffer_size& size)  override;
    virtual bool resetPlaybackAudioBuffer() override;

private:
    bool threadInit() override;
    void run() override;

    bool             m_isPlaying;

    size_t m_cfg_numSamples;
    size_t m_cfg_numChannels;
    size_t m_cfg_frequency;
    size_t m_cfg_bytesPerSample;

    size_t      m_bpnt;
    circularAudioBuffer  *m_outputBuffer;
    bool        m_renderSoundImmediate;
};
