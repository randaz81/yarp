# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(transformClient
                    CATEGORY device
                    TYPE FrameTransformClient
                    INCLUDE FrameTransformClient.h
                    DEPENDS "TARGET YARP::YARP_math"
                    DEFAULT ON)

if(NOT SKIP_transformClient)
  yarp_add_plugin(yarp_transformClient)

  target_sources(yarp_transformClient PRIVATE FrameTransformClient.cpp
                                              FrameTransformClient.h)

  target_link_libraries(yarp_transformClient PRIVATE YARP::YARP_os
                                                     YARP::YARP_sig
                                                     YARP::YARP_dev
                                                     YARP::YARP_math)
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                      YARP_sig
                                                      YARP_dev
                                                      YARP_math)

  yarp_install(TARGETS yarp_transformClient
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_transformClient PROPERTY FOLDER "Plugins/Device")
endif()

#######################################################################################################

yarp_prepare_plugin(frameTransform_nwc_yarp
                    CATEGORY device
                    TYPE FrameTransform_nwc_yarp
                    INCLUDE FrameTransform_nwc_yarp.h
                    DEPENDS "TARGET YARP::YARP_math"
                    DEFAULT ON)

if(NOT SKIP_frameTransform_nwc_yarp)
  yarp_add_plugin(yarp_frameTransform_nwc_yarp)

  target_sources(yarp_frameTransform_nwc_yarp PRIVATE FrameTransform_nwc_yarp.cpp
                                                      FrameTransform_nwc_yarp.h)

  target_sources(yarp_frameTransform_nwc_yarp PRIVATE $<TARGET_OBJECTS:frameTransformMsgs>)
  target_include_directories(yarp_frameTransform_nwc_yarp PRIVATE $<TARGET_PROPERTY:frameTransformMsgs,INTERFACE_INCLUDE_DIRECTORIES>)

  target_link_libraries(yarp_frameTransform_nwc_yarp PRIVATE YARP::YARP_os
                                                     YARP::YARP_sig
                                                     YARP::YARP_dev
                                                     YARP::YARP_math)
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                      YARP_sig
                                                      YARP_dev
                                                      YARP_math)

  yarp_install(TARGETS yarp_frameTransform_nwc_yarp
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_frameTransform_nwc_yarp PROPERTY FOLDER "Plugins/Device")
endif()