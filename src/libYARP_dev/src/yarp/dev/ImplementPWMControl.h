/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMPLEMENTPWMCONTROL_H
#define YARP_DEV_IMPLEMENTPWMCONTROL_H

#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/api.h>
#include <yarp/conf/system.h>

namespace yarp::dev {
class ImplementPWMControl;
}

namespace yarp::dev::impl {

template <typename T>
class FixedSizeBuffersManager;

} // namespace yarp::dev::impl

class YARP_dev_API yarp::dev::ImplementPWMControl: public IPWMControl
{
    void *helper;
    yarp::dev::IPWMControlRaw *raw;
    yarp::dev::impl::FixedSizeBuffersManager<double> *doubleBuffManager;
public:
    bool initialize(int k, const int *amap, const double* dutyToPWM);
    bool uninitialize();
    ImplementPWMControl(IPWMControlRaw *v);
    ~ImplementPWMControl();
    yarp::dev::yarp_ret_value getNumberOfMotors(int *ax) override;
    yarp::dev::yarp_ret_value setRefDutyCycle(int j, double v) override;
    yarp::dev::yarp_ret_value setRefDutyCycles(const double *v) override;
    yarp::dev::yarp_ret_value getRefDutyCycle(int j, double *v) override;
    yarp::dev::yarp_ret_value getRefDutyCycles(double *v) override;
    yarp::dev::yarp_ret_value getDutyCycle(int j, double *v) override;
    yarp::dev::yarp_ret_value getDutyCycles(double *v) override;

};

#endif // YARP_DEV_IMPLEMENTPWMCONTROL_H
