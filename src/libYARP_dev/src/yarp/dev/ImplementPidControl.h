/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMPLEMENTPIDCONTROL_H
#define YARP_DEV_IMPLEMENTPIDCONTROL_H

#include <yarp/dev/IPidControl.h>

namespace yarp::dev {
class ImplementPidControl;
}

namespace yarp::dev::impl {

template <typename T>
class FixedSizeBuffersManager;

} // namespace yarp::dev::impl

class YARP_dev_API yarp::dev::ImplementPidControl : public IPidControl
{
protected:
    IPidControlRaw *iPid;
    void *helper;
    yarp::dev::impl::FixedSizeBuffersManager<double> *doubleBuffManager;
    yarp::dev::impl::FixedSizeBuffersManager<yarp::dev::Pid> *pidBuffManager;

    /**
    * Initialize the internal data and alloc memory.
    * @param size is the number of controlled axes the driver deals with.
    * @param amap is a lookup table mapping axes onto physical drivers.
    * @return true if initialized succeeded, false if it wasn't executed, or assert.
    */
    bool initialize(int size, const int *amap, const double *enc, const double *zos, const double* newtons, const double* amps, const double* dutys);

    /**
    * Clean up internal data and memory.
    * @return true if uninitialization is executed, false otherwise.
    */
    bool uninitialize();

    bool setConversionUnits(const PidControlTypeEnum& pidtype, const PidFeedbackUnitsEnum fbk_conv_units, const PidOutputUnitsEnum out_conv_units);

public:
    /* Constructor.
    * @param y is the pointer to the class instance inheriting from this
    *  implementation.
    */
    ImplementPidControl(yarp::dev::IPidControlRaw *y);

    /* Destructor.
    */
    virtual ~ImplementPidControl();

    yarp::dev::yarp_ret_value setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
    yarp::dev::yarp_ret_value setPids(const PidControlTypeEnum& pidtype, const Pid *pids) override;
    yarp::dev::yarp_ret_value setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) override;
    yarp::dev::yarp_ret_value setPidReferences(const PidControlTypeEnum& pidtype, const double *refs) override;
    yarp::dev::yarp_ret_value setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) override;
    yarp::dev::yarp_ret_value setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits) override;
    yarp::dev::yarp_ret_value getPidError(const PidControlTypeEnum& pidtype, int j, double *err) override;
    yarp::dev::yarp_ret_value getPidErrors(const PidControlTypeEnum& pidtype, double *errs) override;
    yarp::dev::yarp_ret_value getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out) override;
    yarp::dev::yarp_ret_value getPidOutputs(const PidControlTypeEnum& pidtype, double *outs) override;
    yarp::dev::yarp_ret_value getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
    yarp::dev::yarp_ret_value getPids(const PidControlTypeEnum& pidtype, Pid *pids) override;
    yarp::dev::yarp_ret_value getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref) override;
    yarp::dev::yarp_ret_value getPidReferences(const PidControlTypeEnum& pidtype, double *refs) override;
    yarp::dev::yarp_ret_value getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *ref) override;
    yarp::dev::yarp_ret_value getPidErrorLimits(const PidControlTypeEnum& pidtype, double *refs) override;
    yarp::dev::yarp_ret_value resetPid(const PidControlTypeEnum& pidtype, int j) override;
    yarp::dev::yarp_ret_value enablePid(const PidControlTypeEnum& pidtype, int j) override;
    yarp::dev::yarp_ret_value disablePid(const PidControlTypeEnum& pidtype, int j) override;
    yarp::dev::yarp_ret_value setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) override;
    yarp::dev::yarp_ret_value isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;
    void b();
};

#endif // YARP_DEV_IMPLEMENTPIDCONTROL_H
