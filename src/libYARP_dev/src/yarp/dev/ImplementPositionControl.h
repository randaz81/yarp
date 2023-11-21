/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IPOSITIONCONTROLIMPL_H
#define YARP_DEV_IPOSITIONCONTROLIMPL_H


#include <yarp/dev/IPositionControl.h>

namespace yarp::dev {
class ImplementPositionControl;
class StubImplPositionControlRaw;
}

namespace yarp::dev::impl {

template <typename T>
class FixedSizeBuffersManager;

} // namespace yarp::dev::impl

/**
 * Default implementation of the IPositionControl interface. This class can
 * be used to easily provide an implementation of IPositionControl.
 *
 */
class YARP_dev_API yarp::dev::ImplementPositionControl : public IPositionControl
{
protected:
    IPositionControlRaw *iPosition;
    void    *helper;
    yarp::dev::impl::FixedSizeBuffersManager<int> *intBuffManager;
    yarp::dev::impl::FixedSizeBuffersManager<double> *doubleBuffManager;
    yarp::dev::impl::FixedSizeBuffersManager<bool> *boolBuffManager;

    /**
     * Initialize the internal data and alloc memory.
     * @param size is the number of controlled axes the driver deals with.
     * @param amap is a lookup table mapping axes onto physical drivers.
     * @param enc is an array containing the encoder to angles conversion factors.
     * @param zos is an array containing the zeros of the encoders.
     *  respect to the control/output values of the driver.
     * @return true if initialized succeeded, false if it wasn't executed, or assert.
     */
    bool initialize (int size, const int *amap, const double *enc, const double *zos);

    /**
     * Clean up internal data and memory.
     * @return true if uninitialization is executed, false otherwise.
     */
    bool uninitialize ();

public:
    /**
     * Constructor.
     * @param y is the pointer to the class instance inheriting from this
     *  implementation.
     */
    ImplementPositionControl(yarp::dev::IPositionControlRaw *y);

    /**
     * Destructor. Perform uninitialize if needed.
     */
    virtual ~ImplementPositionControl();


    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @return the number of controlled axes.
     */
    yarp::dev::yarp_ret_value getAxes(int *axis) override;

    yarp::dev::yarp_ret_value positionMove(int j, double ref) override;
    yarp::dev::yarp_ret_value positionMove(const int n_joint, const int *joints, const double *refs) override;
    yarp::dev::yarp_ret_value positionMove(const double *refs) override;
    yarp::dev::yarp_ret_value relativeMove(int j, double delta) override;
    yarp::dev::yarp_ret_value relativeMove(const int n_joint, const int *joints, const double *deltas) override;
    yarp::dev::yarp_ret_value relativeMove(const double *deltas) override;
    yarp::dev::yarp_ret_value checkMotionDone(bool *flag) override;
    yarp::dev::yarp_ret_value checkMotionDone(const int n_joint, const int *joints, bool *flags) override;
    yarp::dev::yarp_ret_value checkMotionDone(int j, bool *flag) override;
    yarp::dev::yarp_ret_value setRefSpeed(int j, double sp) override;
    yarp::dev::yarp_ret_value setRefSpeeds(const int n_joint, const int *joints, const double *spds) override;
    yarp::dev::yarp_ret_value setRefSpeeds(const double *spds) override;
    yarp::dev::yarp_ret_value setRefAcceleration(int j, double acc) override;
    yarp::dev::yarp_ret_value setRefAccelerations(const int n_joint, const int *joints, const double *accs) override;
    yarp::dev::yarp_ret_value setRefAccelerations(const double *accs) override;
    yarp::dev::yarp_ret_value getRefSpeed(int j, double *ref) override;
    yarp::dev::yarp_ret_value getRefSpeeds(const int n_joint, const int *joints, double *spds) override;
    yarp::dev::yarp_ret_value getRefSpeeds(double *spds) override;
    yarp::dev::yarp_ret_value getRefAcceleration(int j, double *acc) override;
    yarp::dev::yarp_ret_value getRefAccelerations(const int n_joint, const int *joints, double *accs) override;
    yarp::dev::yarp_ret_value getRefAccelerations(double *accs) override;
    yarp::dev::yarp_ret_value stop(int j) override;
    yarp::dev::yarp_ret_value stop(const int n_joint, const int *joints) override;
    yarp::dev::yarp_ret_value stop() override;
    yarp::dev::yarp_ret_value getTargetPosition(const int joint, double *ref) override;
    yarp::dev::yarp_ret_value getTargetPositions(double *refs) override;
    yarp::dev::yarp_ret_value getTargetPositions(const int n_joint, const int *joints, double *refs) override;
};

/**
 * Stub implementation of IPositionControlRaw interface.
 * Inherit from this class if you want a stub implementation
 * of methods in IPositionControlRaw. This class allows to
 * gradually implement an interface; you just have to implement
 * functions that are useful for the underlying device.
 * Another way to see this class is as a means to convert
 * compile time errors in runtime errors.
 *
 * If you use this class please be aware that the device
 * you are wrapping might not function properly because you
 * missed to implement useful functionalities.
 *
 */
class YARP_dev_API yarp::dev::StubImplPositionControlRaw: public IPositionControlRaw
{
public:
    virtual ~StubImplPositionControlRaw(){}

    yarp::dev::yarp_ret_value getAxes(int *ax) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value positionMoveRaw(int j, double ref) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value positionMoveRaw(const int n_joint, const int *joints, const double *refs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value positionMoveRaw(const double *refs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value relativeMoveRaw(int j, double delta) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value relativeMoveRaw(const int n_joint, const int *joints, const double *refs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value relativeMoveRaw(const double *deltas) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value checkMotionDoneRaw(int j, bool *flag) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value checkMotionDoneRaw(bool *flag) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setRefSpeedRaw(int j, double sp) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setRefSpeedsRaw(const double *spds) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setRefAccelerationRaw(int j, double acc) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value setRefAccelerationsRaw(const double *accs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getRefSpeedRaw(int j, double *ref) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getRefSpeedsRaw(const int n_joint, const int *joints, double *spds) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getRefSpeedsRaw(double *spds) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getRefAccelerationRaw(int j, double *acc) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getRefAccelerationsRaw(double *accs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value stopRaw(int j) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value stopRaw(const int n_joint, const int *joints) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value stopRaw() override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getTargetPositionRaw(const int joint, double *ref) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getTargetPositionsRaw(double *refs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}

    yarp::dev::yarp_ret_value getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) override
    {return yarp::dev::NOT_YET_IMPLEMENTED();}
};

#endif // YARP_DEV_IPOSITIONCONTROLIMPL_H
