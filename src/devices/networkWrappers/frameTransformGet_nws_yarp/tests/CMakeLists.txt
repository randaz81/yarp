# SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

if(TARGET YARP::YARP_math)
  create_device_test(FrameTransformGet_nws_yarp YARP::YARP_math)
endif()
