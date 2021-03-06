# Copyright (C) 2012 Istituto Italiano di Tecnologia (IIT)
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


# SQLite
project(YARP_priv_sqlite)

set(sqlite_SRCS
    sqlite/shell.c
    sqlite/sqlite3.c
)

set(sqlite_HDRS
    sqlite/sqlite3ext.h
    sqlite/sqlite3.h
)

if(MSVC)
    set_source_files_properties(sqlite/sqlite3.c PROPERTIES COMPILE_FLAGS "/wd4996")
endif()

add_definitions(-DSQLITE_OMIT_LOAD_EXTENSION)

add_library(YARP_priv_sqlite STATIC ${sqlite_SRCS})

set_property(TARGET YARP_priv_sqlite PROPERTY FOLDER "Libraries/External")

if(UNIX)
    target_link_libraries(YARP_priv_sqlite PRIVATE pthread)
endif(UNIX)

set(SQLite_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/sqlite PARENT_SCOPE)
set(SQLite_LIBRARIES YARP_priv_sqlite PARENT_SCOPE)
set(SQLite_DEFINITIONS "" PARENT_SCOPE)

install(TARGETS YARP_priv_sqlite
        EXPORT YARP
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

set_property(GLOBAL APPEND PROPERTY YARP_LIBS YARP_priv_sqlite)
