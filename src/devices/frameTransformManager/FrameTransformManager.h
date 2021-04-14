/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
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

#ifndef YARP_DEV_FRAMETRANSFORMMANAGER_H
#define YARP_DEV_FRAMETRANSFORMMANAGER_H


#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IFrameTransformClientControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/os/PeriodicThread.h>
#include <mutex>


#define DEFAULT_THREAD_PERIOD 20 //ms
const int TRANSFORM_TIMEOUT_MS = 100; //ms

class Transforms_storage
{
private:
    std::vector <yarp::math::FrameTransform> m_transforms;
    std::mutex  m_mutex;

public:
    Transforms_storage() {}
    ~Transforms_storage() {}
    bool     set_transform(const yarp::math::FrameTransform& t);
    bool     delete_transform(int id);
    bool     delete_transform(std::string t1, std::string t2);
    inline size_t   size() { return m_transforms.size(); }
    inline yarp::math::FrameTransform& operator[]   (std::size_t idx) { return m_transforms[idx]; }
    void clear();
};

/**
* @ingroup dev_impl_network_clients@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
*
* \brief `frameTransformManager`: Documentation to be added
*/
class FrameTransformManager :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IFrameTransform,
        public yarp::os::PortReader,
        public yarp::os::PeriodicThread
{
private:
    enum ConnectionType {DISCONNECTED = 0, DIRECT, INVERSE, UNDIRECT, IDENTITY};

    FrameTransformManager::ConnectionType getConnectionType(const std::string &target_frame, const std::string &source_frame, std::string* commonAncestor);

    bool canExplicitTransform(const std::string& target_frame_id, const std::string& source_frame_id) const;
    bool getChainedTransform(const std::string &target_frame_id, const std::string &source_frame_id, yarp::sig::Matrix &transform) const;

protected:

    Transforms_storage*                         m_transform_storage;
    double                                      m_period;
    std::mutex                                  m_rpc_mutex;

public:

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;
    bool read(yarp::os::ConnectionReader& connection) override;

     bool     allFramesAsString(std::string &all_frames) override;
     bool     canTransform(const std::string &target_frame, const std::string &source_frame) override;
     bool     clear() override;
     bool     frameExists(const std::string &frame_id) override;
     bool     getAllFrameIds(std::vector< std::string > &ids) override;
     bool     getParent(const std::string &frame_id, std::string &parent_frame_id) override;
     bool     getTransform(const std::string &target_frame_id, const std::string &source_frame_id, yarp::sig::Matrix &transform) override;
     bool     setTransform(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Matrix &transform) override;
     bool     setTransformStatic(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Matrix &transform) override;
     bool     deleteTransform(const std::string &target_frame_id, const std::string &source_frame_id) override;
     bool     transformPoint(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_point, yarp::sig::Vector &transformed_point) override;
     bool     transformPose(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_pose, yarp::sig::Vector &transformed_pose) override;
     bool     transformQuaternion(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::math::Quaternion &input_quaternion, yarp::math::Quaternion &transformed_quaternion) override;
     bool     waitForTransform(const std::string &target_frame_id, const std::string &source_frame_id, const double &timeout) override;
     bool     getAllTransforms(std::vector <yarp::math::FrameTransform> transforms_list)  override;
     bool     getAllStaticTransforms(std::vector <yarp::math::FrameTransform> static_transforms_list)  override;
     bool     setTransform(const yarp::math::FrameTransform& transform) override;
     bool     setTransformStatic(const yarp::math::FrameTransform& static_transform) override;

     FrameTransformManager();
    ~FrameTransformManager();
     bool     threadInit() override;
     void     threadRelease() override;
     void     run() override;

    protected:
    enum show_transforms_in_diagram_t
    {
        do_not_show = 0,
        show_quaternion = 1,
        show_matrix = 2,
        show_rpy = 3
    };
    show_transforms_in_diagram_t  m_show_transforms_in_diagram = do_not_show;

    std::string  get_matrix_as_text(Transforms_storage* storage, int i);
    bool         generate_view();
    bool         parseInitialTf(yarp::os::Searchable& config);
    void         list_response(yarp::os::Bottle& out);
};

#endif // YARP_DEV_FRAMETRANSFORMCLIENT_H
