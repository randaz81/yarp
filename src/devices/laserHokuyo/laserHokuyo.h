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

#ifndef LASERHOKUYO_THREAD_H
#define LASERHOKUYO_THREAD_H

//#include <cstdio>
#include <string>

#include <yarp/os/PeriodicThread.h>
#include <mutex>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISerialDevice.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class laserHokuyo : public PeriodicThread, public yarp::dev::Lidar2DDeviceBase, public DeviceDriver
{
protected:
    PolyDriver driver;
    ISerialDevice *pSerial;

    int cardId;
    double period;
    int start_position;
    int end_position;
    int error_codes;
    int internal_status;

    enum Laser_mode_type {FAKE_MODE=2, GD_MODE=1, MD_MODE=0};
    enum Error_code
    {
        HOKUYO_STATUS_ACQUISITION_COMPLETE =1,
        HOKUYO_STATUS_OK = 0,
        HOKUYO_STATUS_ERROR_BUSY = -1,
        HOKUYO_STATUS_ERROR_INVALID_COMMAND = -2,
        HOKUYO_STATUS_ERROR_INVALID_CHECKSUM = -3,
        HOKUYO_STATUS_ERROR_NOTHING_RECEIVED = -4,
        HOKUYO_STATUS_NOT_READY = -5
    };

    Laser_mode_type laser_mode;

    struct sensor_property_struct
    {
        std::string MODL;
        int DMIN;
        int DMAX;
        int ARES;
        int AMIN;
        int AMAX;
        int AFRT;
        int SCAN;
    } sensor_properties;

    yarp::sig::Vector laser_data;

public:
    laserHokuyo(double period = 0.02) : PeriodicThread(period),
        pSerial(nullptr),
        cardId(0),
        period(period),
        start_position(0),
        end_position(0),
        error_codes(0),
        internal_status(0),
        laser_mode(Laser_mode_type::FAKE_MODE)
    {}


    ~laserHokuyo()
    {
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
    //laser methods
    int  calculateCheckSum(const char* buffer, int size, char actual_sum);
    long decodeDataValue(const char* data, int data_byte);
    int  readData(const Laser_mode_type laser_mode, const char* text_data, const int lext_data_len, int current_line, yarp::sig::Vector& values);
};

#endif
