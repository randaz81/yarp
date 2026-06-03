# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

include(YarpChooseIDL)
yarp_choose_idl(YARPRUNMSG_THRIFT YarprunMsgs.thrift)

add_library(YarprunMsgs OBJECT)

target_sources(YarprunMsgs PRIVATE ${YARPRUNMSG_THRIFT_GEN_FILES})

target_link_libraries(YarprunMsgs
  PRIVATE
    YARP::YARP_os
)

target_include_directories(YarprunMsgs PUBLIC ${YARPRUNMSG_THRIFT_BUILD_INTERFACE_INCLUDE_DIRS})

set_property(TARGET YarprunMsgs PROPERTY FOLDER "Libraries/Private")
