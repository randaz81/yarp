# Copyright: (C) 2011 Istituto Italiano di Tecnologia (IIT)
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname yarpdataplayer)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${templates} DESTINATION ${YARP_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${scripts} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})


