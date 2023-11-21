/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "yarp/dev/ImplementVirtualAnalogSensor.h"
#include <yarp/dev/ControlBoardHelper.h>
#include "yarp/sig/Vector.h"
#include <cstdio>

using namespace yarp::dev;

ImplementVirtualAnalogSensor::ImplementVirtualAnalogSensor(IVirtualAnalogSensorRaw *virt)
{
    iVASRaw = virt;
    helper=nullptr;
}

ImplementVirtualAnalogSensor::~ImplementVirtualAnalogSensor()
{
    uninitialize();
}

bool ImplementVirtualAnalogSensor::initialize(int size, const int *amap, const double *userToRaw)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, nullptr, nullptr, nullptr, nullptr, userToRaw, nullptr));
    yAssert (helper != nullptr);

    return true;
}

bool ImplementVirtualAnalogSensor::uninitialize ()
{
    if (helper!=nullptr)
    {
        delete castToMapper(helper);
        helper=nullptr;
    }
    return true;
}

yarp::dev::VAS_status ImplementVirtualAnalogSensor::getVirtualAnalogSensorStatus(int ch)
{
    if (ch >= castToMapper(helper)->axes())
    {
        return yarp::dev::VAS_status::VAS_ERROR;
    }
    else
    {
        int ch_raw = castToMapper(helper)->toHw(ch);
        return iVASRaw->getVirtualAnalogSensorStatusRaw(ch_raw);
    }
}

int  ImplementVirtualAnalogSensor::getVirtualAnalogSensorChannels()
{
    return iVASRaw->getVirtualAnalogSensorChannelsRaw();
}

yarp_ret_value ImplementVirtualAnalogSensor::updateVirtualAnalogSensorMeasure(yarp::sig::Vector &measure)
{
    yarp::sig::Vector measure_raw;
    castToMapper(helper)->voltageV2S(measure.data(), measure_raw.data());
    yarp_ret_value ret = iVASRaw->updateVirtualAnalogSensorMeasureRaw(measure_raw);
    return ret;
}

yarp_ret_value ImplementVirtualAnalogSensor::updateVirtualAnalogSensorMeasure(int j, double &measure)
{
    JOINTIDCHECK(j)
    int j_raw;
    double measure_raw;
    castToMapper(helper)->voltageV2S(measure, j, measure_raw, j_raw);
    yarp_ret_value ret = iVASRaw->updateVirtualAnalogSensorMeasureRaw(j_raw, measure_raw);
    return ret;
}
