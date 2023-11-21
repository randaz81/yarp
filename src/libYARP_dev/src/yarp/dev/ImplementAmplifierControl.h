/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMPLEMENTAMPLIFIERCONTROL_H
#define YARP_DEV_IMPLEMENTAMPLIFIERCONTROL_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/api.h>

namespace yarp::dev {
class ImplementAmplifierControl;
}

class YARP_dev_API yarp::dev::ImplementAmplifierControl : public IAmplifierControl
{
protected:
    IAmplifierControlRaw *iAmplifier;
    void *helper;
    double *dTemp;
    int *iTemp;

    /**
     * Initialize the internal data and alloc memory.
     * @param size is the number of controlled axes the driver deals with.
     * @param amap is a lookup table mapping axes onto physical drivers.
     * @param enc is an array containing the encoder to angles conversion factors.
     * @param zos is an array containing the zeros of the encoders.
     * @return true if initialized succeeded, false if it wasn't executed, or assert.
     */
    bool initialize (int size, const int *amap, const double *enc, const double *zos, const double *ampereFactor=NULL, const double *voltFactor=NULL);

    /**
     * Clean up internal data and memory.
     * @return true if uninitialization is executed, false otherwise.
     */
    bool uninitialize ();

public:
    /* Constructor.
     * @param y is the pointer to the class instance inheriting from this
     *  implementation.
     */
    ImplementAmplifierControl(yarp::dev::IAmplifierControlRaw  *y);

    /**
     * Destructor. Perform uninitialize if needed.
     */
    virtual ~ImplementAmplifierControl();

    yarp::dev::yarp_ret_value enableAmp(int j) override;
    yarp::dev::yarp_ret_value disableAmp(int j) override;
    yarp::dev::yarp_ret_value getAmpStatus(int *st) override;
    yarp::dev::yarp_ret_value getAmpStatus(int j, int *st) override;
    yarp::dev::yarp_ret_value getCurrents(double *vals) override;
    yarp::dev::yarp_ret_value getCurrent(int j, double *val) override;
    yarp::dev::yarp_ret_value setMaxCurrent(int j, double v) override;
    yarp::dev::yarp_ret_value getMaxCurrent(int j, double *v) override;
    yarp::dev::yarp_ret_value getNominalCurrent(int m, double *val) override;
    yarp::dev::yarp_ret_value setNominalCurrent(int m, const double val) override;
    yarp::dev::yarp_ret_value getPeakCurrent(int m, double *val) override;
    yarp::dev::yarp_ret_value setPeakCurrent(int m, const double val) override;
    yarp::dev::yarp_ret_value getPWM(int j, double* val) override;
    yarp::dev::yarp_ret_value getPWMLimit(int j, double* val) override;
    yarp::dev::yarp_ret_value setPWMLimit(int j, const double val) override;
    yarp::dev::yarp_ret_value getPowerSupplyVoltage(int j, double* val) override;
};


#endif // YARP_DEV_IMPLEMENTAMPLIFIERCONTROL_H
