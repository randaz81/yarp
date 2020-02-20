/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
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

#ifndef RPLIDAR_H
#define RPLIDAR_H


#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ISerialDevice.h>
#include <yarp/dev/Lidar2DDeviceBase.h>

#include <mutex>
#include <string>
#include <vector>

using namespace yarp::os;
using namespace yarp::dev;

typedef unsigned char byte;

class rpLidarCircularBuffer
{
    int         maxsize;
    int         start;
    int         end;
    byte       *elems;

public:
    inline bool isFull()
    {
        return (end + 1) % maxsize == start;
    }

    inline const byte* getRawData()
    {
        return elems;
    }

    inline bool isEmpty()
    {
        return end == start;
    }

    inline bool write_elem(byte elem)
    {
        elems[end] = elem;
        end = (end + 1) % maxsize;
        if (end == start)
        {
            yError("rpLidar buffer overrun!");
            start = (start + 1) % maxsize; // full, overwrite
            return false;
        }
        return true;
    }

    inline bool write_elems(byte* elems, int size)
    {
        for (int i = 0; i < size; i++)
        {
            if (write_elem(elems[i]) == false) return false;
        }
        return true;
    }

    inline int size()
    {
        int i;
        if (end>start)
            i = end - start;
        else if (end == start)
            i = 0;
        else
            i = maxsize - start + end;
        return i;
    }

    inline bool read_elem(byte* elem)
    {
        if (end == start)
        {
            yError("rpLidar buffer underrun!");
            return false;
        }
        *elem = elems[start];
        start = (start + 1) % maxsize;
        return true;
    }

    inline void throw_away_elem()
    {
        start = (start + 1) % maxsize;
    }

    inline void throw_away_elems(int size)
    {
        start = (start + size) % maxsize;
    }

    inline byte select_elem(int offset)
    {
        return elems[(start+offset) % maxsize];
    }

    inline void select_elems(byte* elems, int size)
    {
        for (int i = 0; i < size; i++)
        {
            elems[i] = select_elem(i);
        }
    }

    inline bool read_elems(byte* elems, int size)
    {
        for (int i = 0; i < size; i++)
        {
            if (read_elem(&elems[i]) == false) return false;
        }
        return true;
    }

    inline unsigned int getMaxSize()
    {
        return maxsize;
    }

    inline void clear()
    {
        start = 0;
        end = 0;
    }

    inline unsigned int get_start()
    {
        return start;
    }

    inline unsigned int get_end()
    {
        return end;
    }

    rpLidarCircularBuffer(int bufferSize);
    ~rpLidarCircularBuffer();
};

//---------------------------------------------------------------------------------------------------------------

class RpLidar : public PeriodicThread, public yarp::dev::Lidar2DDeviceBase, public DeviceDriver
{
protected:
    PolyDriver driver;
    ISerialDevice *pSerial;

    rpLidarCircularBuffer * buffer;

    std::string info;
    Device_status device_status;

    yarp::sig::Vector laser_data;

public:
    RpLidar(double period = 0.01) : PeriodicThread(period),
        pSerial(nullptr),
        device_status(Device_status::DEVICE_OK_STANBY)
    {
        buffer = new rpLidarCircularBuffer(20000);
    }


    ~RpLidar()
    {
        if (buffer)
        {
            delete buffer;
            buffer = 0;
        }
    }

    bool open(yarp::os::Searchable& config) override;
    bool close() override;
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

public:
    //IRangefinder2D interface
    bool setDistanceRange    (double min, double max) override;
    bool setScanLimits        (double min, double max) override;
    bool setHorizontalResolution      (double step) override;
    bool setScanRate         (double rate) override;

private:
    bool  HW_getHealth();
    bool  HW_reset();
    bool  HW_start();
    bool  HW_stop();
    bool  HW_getInfo(std::string& s_info);

};

#endif
