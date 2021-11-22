# SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

if(NOT YARP_COMPILE_DEVICE_PLUGINS)
  return()
endif()

include(YarpChooseIDL)
yarp_choose_idl(ICMM_THRIFT IControlModeMsgs.thrift)

add_library(IControlModeMsgs OBJECT)

target_sources(IControlModeMsgs PRIVATE ${ICMM_THRIFT_GEN_FILES})

target_link_libraries(IControlModeMsgs
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
)

target_include_directories(IControlModeMsgs PUBLIC ${ICMM_THRIFT_BUILD_INTERFACE_INCLUDE_DIRS})

set_property(TARGET IControlModeMsgs PROPERTY FOLDER "Devices/Shared")
