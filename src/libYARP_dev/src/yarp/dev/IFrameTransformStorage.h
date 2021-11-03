/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IFRAMETRANSFORM_STORAGE_H
#define YARP_DEV_IFRAMETRANSFORM_STORAGE_H

#include <string>
#include <vector>

#include <yarp/dev/api.h>
#include <yarp/os/Vocab.h>
#include <yarp/math/FrameTransform.h>

namespace yarp {
    namespace dev {
        class IFrameTransformStorageSet;
        class IFrameTransformStorageGet;
        class IFrameTransformStorageUtils;
        class IFrameTransformGet_nwc_yarp_control;
      }
}

class FrameTransformContainer;

/**
 * @ingroup dev_iface_transform
 *
 * Transform Interface.
 */
class YARP_dev_API yarp::dev::IFrameTransformStorageSet
{
public:
    virtual ~IFrameTransformStorageSet();

    /**
    * Save some frame transforms in a storage.
    * @param transforms the list of transforms to be stored
    * @return true/false
    */
    virtual bool setTransforms(const std::vector<yarp::math::FrameTransform>& transforms) = 0;

    /**
    * Save a frame transform in a storage.
    * @param transforms the transform to be stored
    * @return true/false
    */
    virtual bool setTransform(const yarp::math::FrameTransform& transform) = 0;

    /**
    * Delete all transforms in a storage.
    * @return true/false
    */
    virtual bool clearAll() = 0;

    /**
    * Delete a single transform in the storage.
    * @param src the source of frame transform to delete
    * @param dst the destination of frame transform to delete
    * @return true/false
    */
    virtual bool deleteTransform(std::string src, std::string dst) = 0;
};

/**
 * @ingroup dev_iface_transform
 *
 * Transform Interface.
 */
class YARP_dev_API yarp::dev::IFrameTransformStorageGet
{
public:
    virtual ~IFrameTransformStorageGet();

    /**
    * Obtains all frame transforms saved in a storage.
    * @param transforms the returned list of frame transforms
    * @return true/false
    */
    virtual bool getTransforms(std::vector<yarp::math::FrameTransform>& transforms) const = 0;
};

/**
 * @ingroup dev_iface_transform
 *
 * Transform Interface.
 */
class YARP_dev_API yarp::dev::IFrameTransformStorageUtils
{
public:
    virtual ~IFrameTransformStorageUtils();

    virtual bool size (size_t& size) const =0;

    virtual bool getInternalContainer(FrameTransformContainer*& container)  =0;

    virtual bool startStorageThread() = 0;

    virtual bool stopStorageThread() = 0;

};

class YARP_dev_API yarp::dev::IFrameTransformGet_nwc_yarp_control
{
public:
    virtual ~IFrameTransformGet_nwc_yarp_control();

    virtual bool sync_nwc() = 0;
};

#endif // YARP_DEV_IFRAMETRANSFORM_STORAGE_H
