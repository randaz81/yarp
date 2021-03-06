# Copyright (C) 2015 Istituto Italiano di Tecnologia (IIT)
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

if(CREATE_YARPLASERSCANNERGUI)

  include(YarpUseQt5)
  include(YarpOSXUtilities)

  get_property(YARP_OS_INCLUDE_DIRS TARGET YARP_OS PROPERTY INCLUDE_DIRS)
  get_property(YARP_sig_INCLUDE_DIRS TARGET YARP_sig PROPERTY INCLUDE_DIRS)
  get_property(YARP_dev_INCLUDE_DIRS TARGET YARP_dev PROPERTY INCLUDE_DIRS)

  include_directories(${YARP_OS_INCLUDE_DIRS}
                      ${YARP_sig_INCLUDE_DIRS}
                      ${YARP_dev_INCLUDE_DIRS}
                      ${OpenCV_INCLUDE_DIRS})

  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

 # set(yarplaserscannergui_SRCS main.cpp gui.cpp )
 # set(yarplaserscannergui_HDRS gui.h )
 # set(yarplaserscannergui_QRC_FILES res.qrc)
 # set(yarplaserscannergui_UI_FILES gui.ui)

  set(yarplaserscannergui_SRCS main.cpp)

 # qt5_add_resources(yarplaserscannergui_QRC_GEN_SRCS ${yarplaserscannergui_QRC_FILES})
  qt5_wrap_ui(yarplaserscannergui_UI_GEN_SRCS ${yarplaserscannerguii_UI_FILES})

  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
               PREFIX "Source Files"
               FILES ${yarplaserscannergui_SRCS})
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
               PREFIX "Header Files"
               FILES ${yarplaserscannergui_HDRS})
  source_group("Resources Files" FILES ${yarplaserscannergui_QRC_FILES})
  source_group("UI Files" FILES ${yarplaserscannergui_UI_FILES})
  source_group("Generated Files" FILES ${yarpLaserScannergui_QRC_GEN_SRCS}
                                       ${yarplaserscannergui_UI_GEN_SRCS})

  add_executable(yarplaserscannergui WIN32 ${yarplaserscannergui_SRCS}
                                           ${yarplaserscannergui_HDRS}
                                           ${yarplaserscannergui_QRC_FILES}
                                           ${yarplaserscannergui_QRC_GEN_SRCS}
                                           ${yarplaserscannergui_UI_FILES}
                                           ${yarplaserscannergui_UI_GEN_SRCS})

  target_link_libraries(yarplaserscannergui YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_sig
                                            YARP::YARP_dev
                                            Qt5::Widgets
                                            Qt5::Gui)

  target_link_libraries(yarplaserscannergui ${OpenCV_LIBRARIES})

  install(TARGETS yarplaserscannergui COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

  set_property(TARGET yarplaserscannergui PROPERTY FOLDER "Graphical User Interfaces")

  yarp_osx_duplicate_and_add_bundle(TARGET yarplaserscannergui
                                    INSTALL_COMPONENT utilities
                                    INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}")

endif()
