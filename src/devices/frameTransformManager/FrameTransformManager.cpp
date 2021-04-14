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

#include "FrameTransformManager.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <mutex>

/*! \file FrameTransformManager.cpp */

//example: yarpdev --device transformClient --local /transformClient --remote /transformServer

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORMMANAGER, "yarp.device.frameTransformManager")
}

//------------------------------------------------------------------------------------------------------------------------------

/**
  * Transforms storage
  */

bool Transforms_storage::delete_transform(int id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (id >= 0 && (size_t)id < m_transforms.size())
    {
        m_transforms.erase(m_transforms.begin() + id);
        return true;
    }
    return false;
}

bool Transforms_storage::set_transform(const FrameTransform& t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& m_transform : m_transforms)
    {
        //@@@ this linear search requires optimization!
        if (m_transform.dst_frame_id == t.dst_frame_id && m_transform.src_frame_id == t.src_frame_id)
        {
            //transform already exists, update it
            m_transform = t;
            return true;
        }
    }

    //add a new transform
    m_transforms.push_back(t);
    return true;
}

bool Transforms_storage::delete_transform(string t1, string t2)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (t1 == "*" && t2 == "*")
    {
        m_transforms.clear();
        return true;
    }
    else
        if (t1 == "*")
        {
            for (size_t i = 0; i < m_transforms.size(); )
            {
                //source frame is jolly, thus delete all frames with destination == t2
                if (m_transforms[i].dst_frame_id == t2)
                {
                    m_transforms.erase(m_transforms.begin() + i);
                    i = 0; //the erase operation invalidates the iteration, loop restart is required
                }
                else
                {
                    i++;
                }
            }
            return true;
        }
        else
            if (t2 == "*")
            {
                for (size_t i = 0; i < m_transforms.size(); )
                {
                    //destination frame is jolly, thus delete all frames with source == t1
                    if (m_transforms[i].src_frame_id == t1)
                    {
                        m_transforms.erase(m_transforms.begin() + i);
                        i = 0; //the erase operation invalidates the iteration, loop restart is required
                    }
                    else
                    {
                        i++;
                    }
                }
                return true;
            }
            else
            {
                for (size_t i = 0; i < m_transforms.size(); i++)
                {
                    if ((m_transforms[i].dst_frame_id == t1 && m_transforms[i].src_frame_id == t2) ||
                        (m_transforms[i].dst_frame_id == t2 && m_transforms[i].src_frame_id == t1))
                    {
                        m_transforms.erase(m_transforms.begin() + i);
                        return true;
                    }
                }
            }
    return false;
}

void Transforms_storage::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_transforms.clear();
}

//------------------------------------------------------------------------------------------------------------------------------
bool FrameTransformManager::read(yarp::os::ConnectionReader& connection)
{
    std::lock_guard<std::mutex> lock (m_rpc_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    std::string request = in.get(0).asString();
    if (request == "help")
    {
        out.addVocab(Vocab::encode("many"));
        out.addString("'get_transform <src> <dst>: print the transform from <src> to <dst>");
        out.addString("'list_frames: print all the available reference frames");
    }
    else if (request == "list_frames")
    {
        std::vector<std::string> v;
        this->getAllFrameIds(v);
        out.addVocab(Vocab::encode("many"));
        out.addString("List of available reference frames:");
        int count = 0;
        for (auto& vec : v)
        {
            count++;
            std::string str = std::to_string(count) + "- " + vec;
            out.addString(str.c_str());
        }
    }
    else if (request == "get_transform")
    {
        std::string src = in.get(1).asString();
        std::string dst = in.get(2).asString();
        out.addVocab(Vocab::encode("many"));
        yarp::sig::Matrix m;
        this->getTransform(src, dst, m);
        out.addString("Transform from " + src + " to " + dst + " is: ");
        out.addString(m.toString());
    }
    else
    {
        yCError(FRAMETRANSFORMMANAGER, "Invalid vocab received in FrameTransformManager");
        out.clear();
        out.addVocab(VOCAB_ERR);
        out.addString("Invalid command name");
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }
    else
    {
        yCError(FRAMETRANSFORMMANAGER) << "Invalid return to sender";
    }
    return true;
}

bool FrameTransformManager::open(yarp::os::Searchable &config)
{
    m_local_name  = config.find("local").asString();

    if (m_local_name == "")
    {
        yCError(FRAMETRANSFORMMANAGER, "open(): Invalid local name");
        return false;
    }

    if (config.check("period"))
    {
        m_period = config.find("period").asInt32() / 1000.0;
    }
    else
    {
        m_period = 0.010;
        yCWarning(FRAMETRANSFORMMANAGER, "Using default period of %f s" , m_period);
    }

    if (!m_rpc_InterfaceToUser.open(m_local_rpcUser))
    {
        yCError(FRAMETRANSFORMMANAGER, "open(): Could not open rpc port %s, check network", m_local_rpcUser.c_str());
        return false;
    }

    m_transform_storage = new Transforms_client_storage(m_local_streaming_name);

    m_rpc_InterfaceToUser.setReader(*this);
    return true;
}

bool FrameTransformManager::close()
{
    if (m_transform_storage != nullptr)
    {
        delete m_transform_storage;
        m_transform_storage = nullptr;
    }
    return true;
}

bool FrameTransformManager::allFramesAsString(std::string &all_frames)
{
    for (size_t i = 0; i < m_transform_storage->size(); i++)
    {
        all_frames += (*m_transform_storage)[i].toString() + " ";
    }
    return true;
}

FrameTransformManager::ConnectionType FrameTransformManager::getConnectionType(const std::string &target_frame, const std::string &source_frame, std::string* commonAncestor = nullptr)
{
    if (target_frame == source_frame) {return IDENTITY;}

    Transforms_client_storage& tfVec = *m_transform_storage;
    size_t                     i, j;
    std::vector<std::string>   tar2root_vec;
    std::vector<std::string>   src2root_vec;
    std::string                ancestor, child;
    child = target_frame;
    std::lock_guard<std::recursive_mutex> l(tfVec.m_mutex);
    while(getParent(child, ancestor))
    {
        if(ancestor == source_frame)
        {
            return DIRECT;
        }

        tar2root_vec.push_back(ancestor);
        child = ancestor;
    }
    child = source_frame;
    while(getParent(child, ancestor))
    {
        if(ancestor == target_frame)
        {
            return INVERSE;
        }

        src2root_vec.push_back(ancestor);
        child = ancestor;
    }

    for(i = 0; i < tar2root_vec.size(); i++)
    {
        std::string a;
        a = tar2root_vec[i];
        for(j = 0; j < src2root_vec.size(); j++)
        {
            if(a == src2root_vec[j])
            {
                if(commonAncestor)
                {
                    *commonAncestor = a;
                }
                return UNDIRECT;
            }
        }
    }

    return DISCONNECTED;
}

bool FrameTransformManager::canTransform(const std::string &target_frame, const std::string &source_frame)
{
    return getConnectionType(target_frame, source_frame) != DISCONNECTED;
}

bool FrameTransformManager::clear()
{
    m_transform_storage->clear();
    return true;
}

bool FrameTransformManager::frameExists(const std::string &frame_id)
{
    for (size_t i = 0; i < m_transform_storage->size(); i++)
    {
        if (((*m_transform_storage)[i].src_frame_id) == frame_id) { return true; }
        if (((*m_transform_storage)[i].dst_frame_id) == frame_id) { return true; }
    }
    return false;
}

bool FrameTransformManager::getAllFrameIds(std::vector< std::string > &ids)
{
    for (size_t i = 0; i < m_transform_storage->size(); i++)
    {
        bool found = false;
        for (const auto& id : ids)
        {
            if (((*m_transform_storage)[i].src_frame_id) == id) { found = true; break; }
        }
        if (found == false) ids.push_back((*m_transform_storage)[i].src_frame_id);
    }

    for (size_t i = 0; i < m_transform_storage->size(); i++)
    {
        bool found = false;
        for (const auto& id : ids)
        {
            if (((*m_transform_storage)[i].dst_frame_id) == id) { found = true; break; }
        }
        if (found == false) ids.push_back((*m_transform_storage)[i].dst_frame_id);
    }

    return true;
}

bool FrameTransformManager::getParent(const std::string &frame_id, std::string &parent_frame_id)
{
    for (size_t i = 0; i < m_transform_storage->size(); i++)
    {
        std::string par((*m_transform_storage)[i].dst_frame_id);
        if (((*m_transform_storage)[i].dst_frame_id == frame_id))
        {

            parent_frame_id = (*m_transform_storage)[i].src_frame_id;
            return true;
        }
    }
    return false;
}

bool FrameTransformManager::canExplicitTransform(const std::string& target_frame_id, const std::string& source_frame_id) const
{
    Transforms_client_storage& tfVec = *m_transform_storage;
    size_t                     i, tfVec_size;
    std::lock_guard<std::recursive_mutex>         l(tfVec.m_mutex);

    tfVec_size = tfVec.size();
    for (i = 0; i < tfVec_size; i++)
    {
        if (tfVec[i].dst_frame_id == target_frame_id && tfVec[i].src_frame_id == source_frame_id)
        {
            return true;
        }
    }
    return false;
}

bool FrameTransformManager::getChainedTransform(const std::string& target_frame_id, const std::string& source_frame_id, yarp::sig::Matrix& transform) const
{
    Transforms_client_storage& tfVec = *m_transform_storage;
    size_t                     i, tfVec_size;
    std::lock_guard<std::recursive_mutex>         l(tfVec.m_mutex);

    tfVec_size = tfVec.size();
    for (i = 0; i < tfVec_size; i++)
    {
        if (tfVec[i].dst_frame_id == target_frame_id)
        {
            if (tfVec[i].src_frame_id == source_frame_id)
            {
                transform = tfVec[i].toMatrix();
                return true;
            }
            else
            {
                yarp::sig::Matrix m;
                if (getChainedTransform(tfVec[i].src_frame_id, source_frame_id, m))
                {
                    transform = m * tfVec[i].toMatrix();
                    return true;
                }
            }
        }
    }
    return false;
}

bool FrameTransformManager::getTransform(const std::string& target_frame_id, const std::string& source_frame_id, yarp::sig::Matrix& transform)
{
    ConnectionType ct;
    std::string    ancestor;
    ct = getConnectionType(target_frame_id, source_frame_id, &ancestor);
    if (ct == DIRECT)
    {
        return getChainedTransform(target_frame_id, source_frame_id, transform);
    }
    else if (ct == INVERSE)
    {
        yarp::sig::Matrix m(4, 4);
        getChainedTransform(source_frame_id, target_frame_id, m);
        transform = yarp::math::SE3inv(m);
        return true;
    }
    else if(ct == UNDIRECT)
    {
        yarp::sig::Matrix root2tar(4, 4), root2src(4, 4);
        getChainedTransform(source_frame_id, ancestor, root2src);
        getChainedTransform(target_frame_id, ancestor, root2tar);
        transform = yarp::math::SE3inv(root2src) * root2tar;
        return true;
    }
    else if (ct == IDENTITY)
    {
        yarp::sig::Matrix tmp(4, 4); tmp.eye();
        transform = tmp;
        return true;
    }

    yCError(FRAMETRANSFORMMANAGER) << "getTransform(): Frames " << source_frame_id << " and " << target_frame_id << " are not connected";
    return false;
}

bool FrameTransformManager::setTransform(const std::string& target_frame_id, const std::string& source_frame_id, const yarp::sig::Matrix& transform)
{
    if(target_frame_id == source_frame_id)
    {
        yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Invalid transform detected.\n" \
                    "\t Source frame and target frame are both equal to " << source_frame_id;
        return false;
    }

    if (!canExplicitTransform(target_frame_id, source_frame_id) && canTransform(target_frame_id, source_frame_id))
    {
        yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Such transform already exist by chaining transforms";
        return false;
    }

    yarp::os::Bottle b;
    yarp::os::Bottle resp;
    FrameTransform   tf;

    if (!tf.fromMatrix(transform))
    {
        yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Wrong matrix format, it has to be 4 by 4";
        return false;
    }

    setccode
    yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Error on writing on rpc port";
    return true;
}

bool FrameTransformManager::setTransformStatic(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Matrix &transform)
{
    if(target_frame_id == source_frame_id)
    {
        yCError(FRAMETRANSFORMMANAGER) << "setTransformStatic(): Invalid transform detected.\n" \
                    "\t Source frame and target frame are both equal to " << source_frame_id;
        return false;
    }

    if (canTransform(target_frame_id, source_frame_id))
    {
        yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Such static transform already exist, directly or by chaining transforms";
        return false;
    }

    yarp::os::Bottle b;
    yarp::os::Bottle resp;
    FrameTransform   tf;

    if (!tf.fromMatrix(transform))
    {
        yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Wrong matrix format, it has to be 4 by 4";
        return false;
    }

    setstaticcode
    yCError(FRAMETRANSFORMMANAGER) << "setTransform(): Error on writing on rpc port";
   
    return true;
}

bool FrameTransformManager::deleteTransform(const std::string &target_frame_id, const std::string &source_frame_id)
{
    deletion code
    return true;
}

bool FrameTransformManager::transformPoint(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_point, yarp::sig::Vector &transformed_point)
{
    if (input_point.size() != 3)
    {
        yCError(FRAMETRANSFORMMANAGER) << "Only 3 dimensional vector allowed.";
        return false;
    }
    yarp::sig::Matrix m(4, 4);
    if (!getTransform(target_frame_id, source_frame_id, m))
    {
        yCError(FRAMETRANSFORMMANAGER) << "No transform found between source '" << target_frame_id << "' and target '" << source_frame_id << "'";
        return false;
    }
    yarp::sig::Vector in = input_point;
    in.push_back(1);
    transformed_point = m * in;
    transformed_point.pop_back();
    return true;
}

bool FrameTransformManager::transformPose(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::sig::Vector &input_pose, yarp::sig::Vector &transformed_pose)
{
    if (input_pose.size() != 6)
    {
        yCError(FRAMETRANSFORMMANAGER) << "Only 6 dimensional vector (3 axes + roll pith and yaw) allowed.";
        return false;
    }
    if (transformed_pose.size() != 6)
    {
        yCWarning(FRAMETRANSFORMMANAGER, "transformPose(): Performance warning: size transformed_pose should be 6, resizing.");
        transformed_pose.resize(6, 0.0);
    }
    yarp::sig::Matrix m(4, 4);
    if (!getTransform(target_frame_id, source_frame_id, m))
    {
        yCError(FRAMETRANSFORMMANAGER) << "No transform found between source '" << target_frame_id << "' and target '" << source_frame_id << "'";
        return false;
    }
    FrameTransform t;
    t.transFromVec(input_pose[0], input_pose[1], input_pose[2]);
    t.rotFromRPY(input_pose[3], input_pose[4], input_pose[5]);
    t.fromMatrix(m * t.toMatrix());
    transformed_pose[0] = t.translation.tX;
    transformed_pose[1] = t.translation.tY;
    transformed_pose[2] = t.translation.tZ;

    yarp::sig::Vector rot;
    rot = t.getRPYRot();
    transformed_pose[3] = rot[0];
    transformed_pose[4] = rot[1];
    transformed_pose[5] = rot[2];
    return true;
}

bool FrameTransformManager::transformQuaternion(const std::string &target_frame_id, const std::string &source_frame_id, const yarp::math::Quaternion &input_quaternion, yarp::math::Quaternion &transformed_quaternion)
{
    yarp::sig::Matrix m(4, 4);
    if (!getTransform(target_frame_id, source_frame_id, m))
    {
        yCError(FRAMETRANSFORMMANAGER) << "No transform found between source '" << target_frame_id << "' and target '" << source_frame_id <<"'";
        return false;
    }
    FrameTransform t;
    t.rotation=input_quaternion;
    transformed_quaternion.fromRotationMatrix(m * t.toMatrix());
    return true;
}

bool FrameTransformManager::waitForTransform(const std::string &target_frame_id, const std::string &source_frame_id, const double &timeout)
{
    //loop until canTransform == true or timeout expires
    double start = yarp::os::SystemClock::nowSystem();
    while (!canTransform(target_frame_id, source_frame_id))
    {
        if (yarp::os::SystemClock::nowSystem() - start > timeout)
        {
            yCError(FRAMETRANSFORMMANAGER) << "waitForTransform(): timeout expired";
            return false;
        }
        yarp::os::SystemClock::delaySystem(0.001);
    }
    return true;
}

FrameTransformManager::FrameTransformManager() : PeriodicThread(0.01),
    m_transform_storage(nullptr),
    m_period(0.01)
{
}

FrameTransformManager::~FrameTransformManager() = default;

bool     FrameTransformManager::threadInit()
{
    yCTrace(FRAMETRANSFORMMANAGER, "Thread started");
    return true;
}

void     FrameTransformManager::threadRelease()
{
    yCTrace(FRAMETRANSFORMMANAGER, "Thread stopped");
}

void     FrameTransformManager::run()
{
    std::lock_guard<std::mutex> lock (m_rpc_mutex);

    for (auto& m_array_of_port : m_array_of_ports)
    {
        if (m_array_of_port)
        {
            std::string src = m_array_of_port->transform_src;
            std::string dst = m_array_of_port->transform_dst;
            yarp::sig::Matrix m;
            this->getTransform(src, dst, m);
            if (m_array_of_port->format == "matrix")
            {
                m_array_of_port->port.write(m);
            }
            else
            {
                yCError(FRAMETRANSFORMMANAGER) << "Unknown format requested: " << m_array_of_port->format;
            }
        }
    }

    //timeout check for timed transforms.
    bool repeat_check;
    do
    {
        repeat_check = false;
        size_t tfVecSize_timed_yarp = m_yarp_timed_transform_storage->size();
        for (size_t i = 0; i < tfVecSize_timed_yarp; i++)
        {
            if (current_time - (*m_yarp_timed_transform_storage)[i].timestamp > m_FrameTransformTimeout)
            {
                m_yarp_timed_transform_storage->delete_transform(i);
                repeat_check = true;
                break;
            }
        }
    } while (repeat_check);

    //timeout check for ROS timed transforms.
    do
    {
        repeat_check = false;
        size_t tfVecSize_timed_ros = m_ros_timed_transform_storage->size();
        for (size_t i = 0; i < tfVecSize_timed_ros; i++)
        {
            if (current_time - (*m_ros_timed_transform_storage)[i].timestamp > m_FrameTransformTimeout)
            {
                m_ros_timed_transform_storage->delete_transform(i);
                repeat_check = true;
                break;
            }
        }
    } while (repeat_check);
}

bool FrameTransformManager::generate_view()
{
    string dot_string = "digraph G { ";
    for (size_t i = 0; i < m_ros_timed_transform_storage->size(); i++)
    {
        string edge_text = get_matrix_as_text(m_ros_timed_transform_storage, i);
        string trf_text = (*m_ros_timed_transform_storage)[i].src_frame_id + "->" +
            (*m_ros_timed_transform_storage)[i].dst_frame_id + " " +
            "[color = black]";
        dot_string += trf_text + '\n';
    }
    for (size_t i = 0; i < m_ros_static_transform_storage->size(); i++)
    {
        string edge_text = get_matrix_as_text(m_ros_static_transform_storage, i);
        string trf_text = (*m_ros_static_transform_storage)[i].src_frame_id + "->" +
            (*m_ros_static_transform_storage)[i].dst_frame_id + " " +
            "[color = black, style=dashed " + edge_text + "]";
        dot_string += trf_text + '\n';
    }
    for (size_t i = 0; i < m_yarp_timed_transform_storage->size(); i++)
    {
        string edge_text = get_matrix_as_text(m_yarp_timed_transform_storage, i);
        string trf_text = (*m_yarp_timed_transform_storage)[i].src_frame_id + "->" +
            (*m_yarp_timed_transform_storage)[i].dst_frame_id + " " +
            "[color = blue " + edge_text + "]";
        dot_string += trf_text + '\n';
    }
    for (size_t i = 0; i < m_yarp_static_transform_storage->size(); i++)
    {
        string edge_text = get_matrix_as_text(m_yarp_static_transform_storage, i);
        string trf_text = (*m_yarp_static_transform_storage)[i].src_frame_id + "->" +
            (*m_yarp_static_transform_storage)[i].dst_frame_id + " " +
            "[color = blue, style=dashed " + edge_text + "]";
        dot_string += trf_text + '\n';
    }

    string legend = "\n\
        rankdir=LR\n\
        node[shape=plaintext]\n\
        subgraph cluster_01 {\n\
          label = \"Legend\";\n\
          key[label=<<table border=\"0\" cellpadding=\"2\" cellspacing=\"0\" cellborder=\"0\">\n\
            <tr><td align=\"right\" port=\"i1\">YARP timed transform</td></tr>\n\
            <tr><td align=\"right\" port=\"i2\">YARP static transform</td></tr>\n\
            <tr><td align=\"right\" port=\"i3\">ROS timed transform</td></tr>\n\
            <tr><td align=\"right\" port=\"i4\">ROS static transform</td></tr>\n\
            </table>>]\n\
          key2[label=<<table border=\"0\" cellpadding=\"2\" cellspacing=\"0\" cellborder=\"0\">\n\
            <tr><td port = \"i1\">&nbsp;</td></tr>\n\
            <tr><td port = \"i2\">&nbsp;</td></tr>\n\
            <tr><td port = \"i3\">&nbsp;</td></tr>\n\
            <tr><td port = \"i4\">&nbsp;</td></tr>\n\
            </table>>]\n\
          key:i1:e -> key2:i1:w [color = blue]\n\
          key:i2:e -> key2:i2:w [color = blue, style=dashed]\n\
          key:i3:e -> key2:i3:w [color = black]\n\
          key:i4:e -> key2:i4:w [color = black, style=dashed]\n\
        } }";

    string command_string = "printf '" + dot_string + legend + "' | dot -Tpdf > frames.pdf";
#if defined (__linux__)
    int ret = std::system("dot -V");
    if (ret != 0)
    {
        yCError(FRAMETRANSFORMMANAGER) << "dot executable not found. Please install graphviz.";
        return false;
    }

    yCDebug(FRAMETRANSFORMMANAGER) << "Command string is:" << command_string;
    ret = std::system(command_string.c_str());
    if (ret != 0)
    {
        yCError(FRAMETRANSFORMMANAGER) << "The following command failed to execute:" << command_string;
        return false;
    }
#else
    yCError(FRAMETRANSFORMMANAGER) << "Not yet implemented. Available only Linux";
    return false;
#endif
    return true;
}

string FrameTransformManager::get_matrix_as_text(Transforms_storage* storage, int i)
{
    if (m_show_transforms_in_diagram == do_not_show)
    {
        return "";
    }
    else if (m_show_transforms_in_diagram == show_quaternion)
    {
        return string(",label=\" ") + (*storage)[i].toString(FrameTransform::display_transform_mode_t::rotation_as_quaternion) + "\"";
    }
    else if (m_show_transforms_in_diagram == show_matrix)
    {
        return string(",label=\" ") + (*storage)[i].toString(FrameTransform::display_transform_mode_t::rotation_as_matrix) + "\"";
    }
    else if (m_show_transforms_in_diagram == show_rpy)
    {
        return string(",label=\" ") + (*storage)[i].toString(FrameTransform::display_transform_mode_t::rotation_as_rpy) + "\"";
    }

    yCError(FRAMETRANSFORMMANAGER) << "get_matrix_as_text() invalid option";
    return "";
    /*
        //this is a test to use Latek display
        string s = "\\begin{ bmatrix } \
        1 & 2 & 3\\ \
        a & b & c \
        \\end{ bmatrix }";
    */
}

bool FrameTransformManager::parseInitialTf(yarp::os::Searchable& config)
{
    if (config.check("USER_TF"))
    {
        Bottle all_transforms_group = config.findGroup("USER_TF").tail();
        yCDebug(FRAMETRANSFORMMANAGER) << all_transforms_group.toString();

        for (size_t i = 0; i < all_transforms_group.size(); i++)
        {
            FrameTransform t;

            Bottle* b = all_transforms_group.get(i).asList();
            if (!b)
            {
                yCError(FRAMETRANSFORMMANAGER) << "No entries in USER_TF group";
                return false;
            }

            if (b->size() == 18)
            {
                bool   r(true);
                Matrix m(4, 4);

                for (int i = 0; i < 16; i++)
                {
                    if (!b->get(i).isFloat64())
                    {
                        yCError(FRAMETRANSFORMMANAGER) << "transformServer: element " << i << " is not a double.";
                        r = false;
                    }
                    else
                    {
                        m.data()[i] = b->get(i).asFloat64();
                    }
                }

                if (!b->get(16).isString() || !b->get(17).isString())
                {
                    r = false;
                }

                if (!r)
                {
                    yCError(FRAMETRANSFORMMANAGER) << "transformServer: param not correct.. for the 4x4 matrix mode" <<
                        "you must provide 18 parameter. the matrix, the source frame(string) and the destination frame(string)";
                    return false;
                }

                t.fromMatrix(m);
                t.src_frame_id = b->get(16).asString();
                t.dst_frame_id = b->get(17).asString();
            }
            else if (b->size() == 8 &&
                b->get(0).isFloat64() &&
                b->get(1).isFloat64() &&
                b->get(2).isFloat64() &&
                b->get(3).isFloat64() &&
                b->get(4).isFloat64() &&
                b->get(5).isFloat64() &&
                b->get(6).isString() &&
                b->get(7).isString())
            {
                t.translation.set(b->get(0).asFloat64(), b->get(1).asFloat64(), b->get(2).asFloat64());
                t.rotFromRPY(b->get(3).asFloat64(), b->get(4).asFloat64(), b->get(5).asFloat64());
                t.src_frame_id = b->get(6).asString();
                t.dst_frame_id = b->get(7).asString();
            }
            else
            {
                yCError(FRAMETRANSFORMMANAGER) << "transformServer: param not correct.. a tf requires 8 param in the format:" <<
                    "x(dbl) y(dbl) z(dbl) r(dbl) p(dbl) y(dbl) src(str) dst(str)";
                return false;
            }

            if (m_yarp_static_transform_storage->set_transform(t))
            {
                yCInfo(FRAMETRANSFORMMANAGER) << "Transform from" << t.src_frame_id << "to" << t.dst_frame_id << "successfully set";
            }
            else
            {
                yCInfo(FRAMETRANSFORMMANAGER) << "Unable to set transform from " << t.src_frame_id << "to" << t.dst_frame_id;
            }
        }
        return true;
    }
    else
    {
        yCInfo(FRAMETRANSFORMMANAGER) << "No initial tf found";
    }
    return true;
}

void FrameTransformManager::list_response(yarp::os::Bottle& out)
{
    std::vector<Transforms_server_storage*> storages;
    std::vector<string>                     storageDescription;
    storages.push_back(m_ros_timed_transform_storage);
    storageDescription.emplace_back("ros timed transforms");

    storages.push_back(m_ros_static_transform_storage);
    storageDescription.emplace_back("ros static transforms");

    storages.push_back(m_yarp_timed_transform_storage);
    storageDescription.emplace_back("yarp timed transforms");

    storages.push_back(m_yarp_static_transform_storage);
    storageDescription.emplace_back("yarp static transforms");

    if (storages[0]->size() == 0 &&
        storages[1]->size() == 0 &&
        storages[2]->size() == 0 &&
        storages[3]->size() == 0)
    {
        out.addString("no transforms found");
        return;
    }

    for (size_t s = 0; s < storages.size(); s++)
    {
        if (!storages[s])
        {
            continue;
        }

        std::string text_to_print = storageDescription[s] + std::string("(") + std::to_string(storages[s]->size()) + std::string("): ");
        out.addString(text_to_print);

        for (size_t i = 0; i < storages[s]->size(); i++)
        {
            out.addString((*storages[s])[i].toString());
        }

    }
}
