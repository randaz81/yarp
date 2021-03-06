# Copyright (C) 2015 Istituto Italiano di Tecnologia (IIT)
# Author: Marco Randazzo
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(laserFromDepth
                    CATEGORY device
                    TYPE LaserFromDepth
                    INCLUDE laserFromDepth.h
                    DEPENDS CREATE_DEVICE_LIBRARY_MODULES)

if(NOT SKIP_laserFromDepth)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  yarp_add_plugin(laserFromDepth laserFromDepth.h
                                 laserFromDepth.cpp)
  target_link_libraries(laserFromDepth YARP::YARP_OS
                                       YARP::YARP_sig
                                       YARP::YARP_dev)
  yarp_install(TARGETS laserFromDepth
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})
  yarp_install(FILES laserFromDepth.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set_property(TARGET laserFromDepth PROPERTY FOLDER "Plugins/Device")
endif()
