/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _USE_MATH_DEFINES

#include <yarp/dev/Nav2D/MapGrid2DDataSerializer.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/ImageFile.h>
#include <algorithm>
#include <cmath>

#if defined (YARP_HAS_ZLIB)
#include <zlib.h>
#endif


using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::sig;
using namespace yarp::os;

bool MapGrid2DDataSerializer::read(yarp::os::ConnectionReader& connection)
{
    // auto-convert text mode interaction
    connection.convertTextMode();

    connection.expectInt32();
    connection.expectInt32();

    connection.expectInt32();
    mStorage->m_width = connection.expectInt32();
    connection.expectInt32();
    mStorage->m_height = connection.expectInt32();
    connection.expectInt32();
    double x0 = connection.expectFloat64();
    connection.expectInt32();
    double y0 = connection.expectFloat64();
    connection.expectInt32();
    double theta0 = connection.expectFloat64();
    mStorage->m_origin.setOrigin(x0,y0,theta0);
    connection.expectInt32();
    mStorage->m_resolution = connection.expectFloat64();
    connection.expectInt32();
    int siz = connection.expectInt32();
    char buff[255]; memset(buff, 0, 255);
    connection.expectBlock((char*)buff, siz);
    mStorage->m_map_name = buff;
    connection.expectInt32();
    mStorage->m_compressed_data_over_network = connection.expectInt8();
    mStorage->m_map_occupancy.resize(mStorage->m_width, mStorage->m_height);
    mStorage->m_map_flags.resize(mStorage->m_width, mStorage->m_height);

    if (mStorage->m_compressed_data_over_network)
    {
#if defined (YARP_HAS_ZLIB)
        bool ok = true;
        {
            connection.expectInt32();
            size_t sizeCompressed = connection.expectInt32();
            unsigned char* dataCompressed = new unsigned char[sizeCompressed];
            ok &= connection.expectBlock((char*)dataCompressed, sizeCompressed);
            size_t sizeUncompressed = mStorage->m_map_occupancy.getRawImageSize();
            unsigned char* dataUncompressed = mStorage->m_map_occupancy.getRawImage();
            int z_result = uncompress((Bytef*)dataUncompressed, (uLongf*)&sizeUncompressed, (const Bytef*) dataCompressed, sizeCompressed);
            YARP_UNUSED(z_result);
            delete[] dataCompressed;
        }
        {
            connection.expectInt32();
            size_t sizeCompressed = connection.expectInt32();
            unsigned char* dataCompressed = new unsigned char[sizeCompressed];
            ok &= connection.expectBlock((char*)dataCompressed, sizeCompressed);
            size_t sizeUncompressed = mStorage->m_map_flags.getRawImageSize();
            unsigned char* dataUncompressed = mStorage->m_map_flags.getRawImage();
            int z_result = uncompress((Bytef*)dataUncompressed, (uLongf*)&sizeUncompressed, (const Bytef*)dataCompressed, sizeCompressed);
            YARP_UNUSED(z_result);
            delete[] dataCompressed;
        }
#endif
    }
    else
    {
        bool ok = true;
        {
            connection.expectInt32();
            size_t memsize = connection.expectInt32();
            if (memsize != mStorage->m_map_occupancy.getRawImageSize()) { return false; }
            unsigned char* mem = mStorage->m_map_occupancy.getRawImage();
            ok &= connection.expectBlock((char*)mem, memsize);
        }
        {
            connection.expectInt32();
            size_t memsize = connection.expectInt32();
            if (memsize != mStorage->m_map_flags.getRawImageSize()) { return false; }
            unsigned char* mem = mStorage->m_map_flags.getRawImage();
            ok &= connection.expectBlock((char*)mem, memsize);
        }
        if (!ok) {
            return false;
        }
    }
    return !connection.isError();
}

bool MapGrid2DDataSerializer::write(yarp::os::ConnectionWriter& connection) const
{
    connection.appendInt32(BOTTLE_TAG_LIST);
    connection.appendInt32(10);
    connection.appendInt32(BOTTLE_TAG_INT32);
    connection.appendInt32(mStorage->m_width);
    connection.appendInt32(BOTTLE_TAG_INT32);
    connection.appendInt32(mStorage->m_height);
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->m_origin.get_x());
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->m_origin.get_y());
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->m_origin.get_theta());
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(mStorage->m_resolution);
    connection.appendInt32(BOTTLE_TAG_STRING);
    connection.appendString(mStorage->m_map_name);
    connection.appendInt32(BOTTLE_TAG_INT8);
    connection.appendInt8(mStorage->m_compressed_data_over_network);

    if (mStorage->m_compressed_data_over_network)
    {
#if defined (YARP_HAS_ZLIB)
        {
            size_t      sizeUncompressed = mStorage->m_map_occupancy.getRawImageSize();
            unsigned char* dataUncompressed = mStorage->m_map_occupancy.getRawImage();
            size_t      sizeCompressed = compressBound(sizeUncompressed);
            unsigned char* dataCompressed = new unsigned char[sizeCompressed];
            int z_result = compress((Bytef*)dataCompressed, (uLongf*)&sizeCompressed, (Bytef*)dataUncompressed, sizeUncompressed);
            YARP_UNUSED(z_result);
            connection.appendInt32(BOTTLE_TAG_BLOB);
            connection.appendInt32(sizeCompressed);
            connection.appendBlock((char*)dataCompressed, sizeCompressed);
            delete [] dataCompressed;
        }
        {
            size_t      sizeUncompressed = mStorage->m_map_flags.getRawImageSize();
            unsigned char* dataUncompressed = mStorage->m_map_flags.getRawImage();
            size_t      sizeCompressed = compressBound(sizeUncompressed);
            unsigned char* dataCompressed = new unsigned char[sizeCompressed];
            int z_result = compress((Bytef*)dataCompressed, (uLongf*)&sizeCompressed, (Bytef*)dataUncompressed, sizeUncompressed);
            YARP_UNUSED(z_result);
            connection.appendInt32(BOTTLE_TAG_BLOB);
            connection.appendInt32(sizeCompressed);
            connection.appendBlock((char*)dataCompressed, sizeCompressed);
            delete[] dataCompressed;
        }
#endif
    }
    else
    {
        {
            unsigned char* mem     = mStorage->m_map_occupancy.getRawImage();
            int memsize = mStorage->m_map_occupancy.getRawImageSize();
            connection.appendInt32(BOTTLE_TAG_BLOB);
            connection.appendInt32(memsize);
            connection.appendExternalBlock((char*)mem, memsize);
        }
        {
            unsigned char* mem     = mStorage->m_map_flags.getRawImage();
            int memsize = mStorage->m_map_flags.getRawImageSize();
            connection.appendInt32(BOTTLE_TAG_BLOB);
            connection.appendInt32(memsize);
            connection.appendExternalBlock((char*)mem, memsize);
        }
    }

    connection.convertTextMode();
    return !connection.isError();
}
