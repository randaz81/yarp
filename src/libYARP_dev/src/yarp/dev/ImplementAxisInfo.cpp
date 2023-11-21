/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "yarp/dev/ImplementAxisInfo.h"
#include <yarp/dev/ControlBoardHelper.h>

#include <cstdio>
using namespace yarp::dev;

////////////////////////
// Encoder Interface Timed Implementation
ImplementAxisInfo::ImplementAxisInfo(yarp::dev::IAxisInfoRaw *y)
{
    iinfo=y;
    helper = nullptr;
}

ImplementAxisInfo::~ImplementAxisInfo()
{
    uninitialize();
}

bool ImplementAxisInfo::initialize(int size, const int *amap)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap));
    yAssert (helper != nullptr);

    return true;
}

/**
* Clean up internal data and memory.
* @return true if uninitialization is executed, false otherwise.
*/
bool ImplementAxisInfo::uninitialize()
{
    if (helper!=nullptr)
    {
        delete castToMapper(helper);
        helper=nullptr;
    }



    return true;
}

yarp_ret_value ImplementAxisInfo::getAxes(int* ax)
{
    yarp_ret_value ret= yarp_ret_value_ok;
    (*ax) = castToMapper(helper)->axes();
    return ret;
}

yarp_ret_value ImplementAxisInfo::getAxisName(int axis, std::string& name)
{
    int k = castToMapper(helper)->toHw(axis);
    yarp_ret_value ret = iinfo->getAxisNameRaw(k, name);
    return ret;
}

yarp_ret_value ImplementAxisInfo::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    int k = castToMapper(helper)->toHw(axis);
    yarp_ret_value ret = iinfo->getJointTypeRaw(k, type);
    return ret;
}
