/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

struct YarpVector {
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

struct YarpMatrix {
} (
  yarp.name = "yarp::sig::Matrix"
  yarp.includefile="yarp/sig/Matrix.h"
)

struct YarpFrameTransform {
} (
  yarp.name = "yarp::math::FrameTransform"
  yarp.includefile="yarp/math/FrameTransform.h"
)

struct YarpQuaternion {
} (
  yarp.name = "yarp::math::Quaternion"
  yarp.includefile="yarp/math/Quaternion.h"
)

struct return_allFramesAsString
{
  1: bool retvalue;
  2: string all_frames;
}

struct return_canTransform
{
  1: bool retvalue;
}
struct return_clear
{
  1: bool retvalue;
}
struct return_frameExists
{
  1: bool retvalue;
}
struct return_getAllFrameIds
{
  1: bool retvalue;
  2: list<string> ids;
}
struct return_getParent
{
  1: bool retvalue;
  2:string parent_frame_id;
}

struct return_getTransform
{
  1: bool retvalue;
  2: YarpMatrix transform;
}

struct return_setTransform
{
  1: bool retvalue;
}
struct return_setTransformStatic
{
  1: bool retvalue;
}
struct return_deleteTransform
{
  1: bool retvalue;
}
struct return_transformPoint
{
  1: bool retvalue;
  2: YarpVector transformed_point;
}
struct return_transformPose
{
  1: bool retvalue;
  2: YarpVector transformed_pose;
}

struct return_transformQuaternion
{
  1: bool retvalue;
  2: YarpQuaternion transformed_quaternion;
}
struct return_waitForTransform
{
  1: bool retvalue;
}

struct return_getAllTransforms
{
  1: bool retvalue;
  2: list<YarpFrameTransform> transforms_list;
}
struct return_getAllStaticTransforms
{
  1: bool retvalue;
  2: list<YarpFrameTransform> transforms_list;
}

struct return_setTransform2
{
  1: bool retvalue;
}
struct return_setTransformStatic2
{
  1: bool retvalue;
}

service FrameTransformRPCTest
{
  /**
   * Read the sensor metadata necessary to configure the MultipleAnalogSensorsClient device.
   */
  bool getTranformTest(1:i32 x);
  bool setTranformTest(1:i32 x);

  return_allFramesAsString allFramesAsString();
  return_canTransform canTransform(1:string target_frame, 2:string source_frame);
  return_clear clear();
  return_frameExists frameExists(1:string frame_id);
  return_getAllFrameIds getAllFrameIds();
  return_getParent getParent(  1:string frame_id);
  return_getTransform getTransform(  1:string target_frame_id,   2:string source_frame_id);
  return_setTransform setTransform(  1:string target_frame_id,   2:string source_frame_id,   3:YarpMatrix transform);
  return_setTransformStatic setTransformStatic(  1:string target_frame_id,   2:string source_frame_id,   3:YarpMatrix transform);
  return_deleteTransform deleteTransform(  1:string target_frame_id,   2:string source_frame_id);
  return_transformPoint transformPoint(  1:string target_frame_id,   2:string source_frame_id,   3:YarpVector input_point);
  return_transformPose transformPose(  1:string target_frame_id,   2:string source_frame_id,   3:YarpVector input_pose);
  return_transformQuaternion transformQuaternion(  1:string target_frame_id,   2:string source_frame_id,   3:YarpQuaternion input_quaternion);
  return_waitForTransform waitForTransform(  1:string target_frame_id,   2:string source_frame_id,   3:double timeout);
  return_getAllTransforms getAllTransforms() ;
  return_getAllStaticTransforms getAllStaticTransforms();
  return_setTransform2 setTransform2(  YarpFrameTransform transform);
  return_setTransformStatic2 setTransformStatic2(  YarpFrameTransform static_transform);
}
