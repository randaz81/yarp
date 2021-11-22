# SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(remote_controlboard
  CATEGORY device
  TYPE ControlBoard_nwc_yarp
  INCLUDE ControlBoard_nwc_yarp.h
  EXTRA_CONFIG
    WRAPPER=controlboardwrapper2
  DEFAULT ON
)

if(NOT SKIP_remote_controlboard)
  yarp_add_plugin(yarp_controlBoard_nwc_yarp)

  target_sources(yarp_controlBoard_nwc_yarp
    PRIVATE
      ControlBoard_nwc_yarp.cpp
      ControlBoard_nwc_yarp.h
      ControlBoard_nwc_yarp_LogComponent.cpp
      ControlBoard_nwc_yarp_LogComponent.h
      stateExtendedReader.cpp
      stateExtendedReader.h)

  target_sources(yarp_controlBoard_nwc_yarp PRIVATE $<TARGET_OBJECTS:IControlModeMsgs>)
  target_include_directories(yarp_controlBoard_nwc_yarp PRIVATE $<TARGET_PROPERTY:IControlModeMsgs,INTERFACE_INCLUDE_DIRECTORIES>)

  target_link_libraries(yarp_controlBoard_nwc_yarp
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
  )
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_sig
    YARP_dev
  )

  yarp_install(
    TARGETS yarp_controlBoard_nwc_yarp
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_controlBoard_nwc_yarp PROPERTY FOLDER "Plugins/Device/NWC")
endif()
