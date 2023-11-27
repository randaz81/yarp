/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include <yarp/dev/ImplementImpedanceControl.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/os/LogStream.h>
#include <cmath>

using namespace yarp::dev;

/////////////// implement ImplementImpedanceControl
ImplementImpedanceControl::ImplementImpedanceControl(IImpedanceControlRaw *r)
{
    iImpedanceRaw=r;
    helper=nullptr;
}

bool ImplementImpedanceControl::initialize(int size, const int *amap, const double *enc, const double *zos, const double *nw)
{
    if (helper != nullptr) {
        return false;
    }

    helper=(void *)(new ControlBoardHelper(size, amap, enc, zos, nw));
    yAssert (helper != nullptr);

    return true;
}

ImplementImpedanceControl::~ImplementImpedanceControl()
{
    uninitialize();
}

bool ImplementImpedanceControl::uninitialize ()
{
    if (helper!=nullptr)
    {
        delete castToMapper(helper);
        helper=nullptr;
    }

    return true;
}

yarp_ret_value ImplementImpedanceControl::getAxes(int *axes)
{
    return iImpedanceRaw->getAxes(axes);
}

yarp_ret_value ImplementImpedanceControl::setImpedance(int j, double stiffness, double damping)
{
    JOINTIDCHECK(j)
    int k;
    double stiff;
    double damp;
    castToMapper(helper)->impN2S(stiffness,j,stiff,k);
    castToMapper(helper)->impN2S(damping,j,damp,k);
    return iImpedanceRaw->setImpedanceRaw(k, stiff, damp);
}

yarp_ret_value ImplementImpedanceControl::getImpedance(int j, double *stiffness, double *damping)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret=iImpedanceRaw->getImpedanceRaw(k, stiffness, damping);
    *stiffness = (castToMapper(helper)->impS2N(*stiffness, k));
    *damping   = (castToMapper(helper)->impS2N(*damping, k));
    //prevent negative stiffness
    *stiffness = fabs (*stiffness);
    *damping   = fabs (*damping);
    return ret;
}

yarp_ret_value ImplementImpedanceControl::setImpedanceOffset(int j, double offset)
{
    JOINTIDCHECK(j)
    int k;
    double off;
    castToMapper(helper)->trqN2S(offset,j,off,k);
    return iImpedanceRaw->setImpedanceOffsetRaw(k, off);
}

yarp_ret_value ImplementImpedanceControl::getImpedanceOffset(int j, double *offset)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    yarp_ret_value ret = iImpedanceRaw->getImpedanceOffsetRaw(k, offset);
    *offset    = (castToMapper(helper)->trqS2N(*offset,k));
    return ret;
}

yarp_ret_value ImplementImpedanceControl::getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    JOINTIDCHECK(j)
    int k;
    k=castToMapper(helper)->toHw(j);
    return iImpedanceRaw->getCurrentImpedanceLimitRaw(k, min_stiff, max_stiff, min_damp, max_damp);
}
