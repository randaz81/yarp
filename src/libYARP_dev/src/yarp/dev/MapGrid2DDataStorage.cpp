/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _USE_MATH_DEFINES

#include <yarp/dev/MapGrid2DDataStorage.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/ImageFile.h>
#include <algorithm>
#include <fstream>
#include <cmath>

#if defined (YARP_HAS_ZLIB)
#include <zlib.h>
#endif


using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif

//helper functions
static std::string extractPathFromFile(std::string full_filename)
{
    size_t found;
    found = full_filename.find_last_of('/');
    if (found != std::string::npos) {
        return full_filename.substr(0, found) + "/";
    }
    found = full_filename.find_last_of('\\');
    if (found != std::string::npos) {
        return full_filename.substr(0, found) + "\\";
    }
    return full_filename;
}

static std::string extractExtensionFromFile(std::string full_filename)
{
    int start = full_filename.length() - 3;
    return full_filename.substr(start, 3);
}

MapGrid2DDataStorage::MapGrid2DDataStorage()
{
    m_resolution = 1.0; //each pixel corresponds to 1 m
    m_width = 2;
    m_height = 2;
    m_map_occupancy.setQuantum(1); //we do not want extra padding in map images
    m_map_flags.setQuantum(1);
    m_map_occupancy.resize(m_width, m_height);
    m_map_flags.resize(m_width, m_height);
    m_occupied_thresh = 0.80;
    m_free_thresh = 0.20;
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            m_map_occupancy.safePixel(x, y) = 0;
            m_map_flags.safePixel(x, y) = MapGrid2DDataStorage::map_flags::MAP_CELL_FREE;
        }
    }
#if defined (YARP_HAS_ZLIB)
    m_compressed_data_over_network = true;
#else
    m_compressed_data_over_network = false;
#endif
}

MapGrid2DDataStorage::~MapGrid2DDataStorage() = default;

bool MapGrid2DDataStorage::isNotFree(XYCell cell) const
{
    if (isInsideMap(cell))
    {
        if (m_map_occupancy.safePixel(cell.x, cell.y) != 0) {
            return true;
        }
        if (m_map_flags.safePixel(cell.x, cell.y) == MapGrid2DDataStorage::map_flags::MAP_CELL_KEEP_OUT) {
            return true;
        }
        if (m_map_flags.safePixel(cell.x, cell.y) == MapGrid2DDataStorage::map_flags::MAP_CELL_TEMPORARY_OBSTACLE) {
            return true;
        }
        if (m_map_flags.safePixel(cell.x, cell.y) == MapGrid2DDataStorage::map_flags::MAP_CELL_ENLARGED_OBSTACLE) {
            return true;
        }
    }
    return false;
}

bool MapGrid2DDataStorage::isFree(XYCell cell) const
{
    if (isInsideMap(cell))
    {
        if (m_map_occupancy.safePixel(cell.x, cell.y) == 0 && m_map_flags.safePixel(cell.x, cell.y) == MapGrid2DDataStorage::map_flags::MAP_CELL_FREE) {
            return true;
        }
    }
    return false;
}

bool MapGrid2DDataStorage::isKeepOut(XYCell cell) const
{
    if (isInsideMap(cell))
    {
        if (m_map_flags.safePixel(cell.x, cell.y) == MapGrid2DDataStorage::map_flags::MAP_CELL_KEEP_OUT) {
            return true;
        }
    }
    return false;
}

bool MapGrid2DDataStorage::isWall(XYCell cell) const
{
    if (isInsideMap(cell))
    {
       // if (m_map_occupancy.safePixel(cell.x, cell.y) == 0)
       //     return true;
       if (m_map_flags.safePixel(cell.x, cell.y) == MapGrid2DDataStorage::map_flags::MAP_CELL_WALL) {
           return true;
       }
    }
    return false;
}

size_t MapGrid2DDataStorage::height() const
{
    return m_height;
}

size_t MapGrid2DDataStorage::width() const
{
    return m_width;
}

bool MapGrid2DDataStorage::getMapImage(yarp::sig::ImageOf<PixelRgb>& image) const
{
    image.setQuantum(1);
    image.resize(m_width, m_height);
    image.zero();
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            image.safePixel(x, y) = CellFlagDataToPixel(m_map_flags.safePixel(x, y));
        }
    }
    return true;
}

bool MapGrid2DDataStorage::setMapImage(yarp::sig::ImageOf<PixelRgb>& image)
{
    if (image.width() != m_width ||
        image.height() != m_height)
    {
        yError() << "The size of given image does not correspond to the current map. Use method setSize() first.";
        return false;
    }
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            m_map_flags.safePixel(x, y) = PixelToCellFlagData(image.safePixel(x, y));
        }
    }
    return true;
}

void MapGrid2DDataStorage::enlargeCell(XYCell cell)
{
    size_t i = cell.x;
    size_t j = cell.y;
    size_t il = cell.x > 1 ? cell.x - 1 : 0;
    size_t ir = cell.x + 1 < (m_width)-1 ? cell.x + 1 : (m_width)-1;
    size_t ju = cell.y > 1 ? cell.y - 1 : 0;
    size_t jd = cell.y + 1 < (m_height)-1 ? cell.y + 1 : (m_height)-1;

    if (m_map_flags.pixel(il, j) == MAP_CELL_FREE) {
        m_map_flags.pixel(il, j) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(ir, j) == MAP_CELL_FREE) {
        m_map_flags.pixel(ir, j) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(i, ju) == MAP_CELL_FREE) {
        m_map_flags.pixel(i, ju) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(i, jd) == MAP_CELL_FREE) {
        m_map_flags.pixel(i, jd) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(il, ju) == MAP_CELL_FREE) {
        m_map_flags.pixel(il, ju) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(il, jd) == MAP_CELL_FREE) {
        m_map_flags.pixel(il, jd) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(ir, ju) == MAP_CELL_FREE) {
        m_map_flags.pixel(ir, ju) = MAP_CELL_ENLARGED_OBSTACLE;
    }
    if (m_map_flags.pixel(ir, jd) == MAP_CELL_FREE) {
        m_map_flags.pixel(ir, jd) = MAP_CELL_ENLARGED_OBSTACLE;
    }
}

MapGrid2DDataStorage::CellFlagData MapGrid2DDataStorage::PixelToCellFlagData(const yarp::sig::PixelRgb& pixin) const
{
    if (pixin.r == 0 && pixin.g == 0 && pixin.b == 0) {
        return MAP_CELL_WALL;
    } else if (pixin.r == 205 && pixin.g == 205 && pixin.b == 205) {
        return MAP_CELL_UNKNOWN;
    } else if (pixin.r == 254 && pixin.g == 254 && pixin.b == 254) {
        return MAP_CELL_FREE;
    } else if (pixin.r == 255 && pixin.g == 0 && pixin.b == 0) {
        return MAP_CELL_KEEP_OUT;
    }
    return  MAP_CELL_UNKNOWN;
}

yarp::sig::PixelRgb MapGrid2DDataStorage::CellFlagDataToPixel(const MapGrid2DDataStorage::CellFlagData& cellin) const
{
    yarp::sig::PixelRgb pixout_flg;
    if (cellin == MAP_CELL_WALL) { pixout_flg.r = 0; pixout_flg.g = 0; pixout_flg.b = 0;}
    else if (cellin == MAP_CELL_UNKNOWN) { pixout_flg.r = 205; pixout_flg.g = 205; pixout_flg.b = 205; }
    else if (cellin == MAP_CELL_FREE) { pixout_flg.r = 254; pixout_flg.g = 254; pixout_flg.b = 254; }
    else if (cellin == MAP_CELL_KEEP_OUT) { pixout_flg.r = 255; pixout_flg.g = 0; pixout_flg.b = 0; }
    else if (cellin == MAP_CELL_ENLARGED_OBSTACLE) { pixout_flg.r = 255; pixout_flg.g = 200; pixout_flg.b = 0; }
    else if (cellin == MAP_CELL_TEMPORARY_OBSTACLE) { pixout_flg.r = 100; pixout_flg.g = 100; pixout_flg.b = 200; }
    else
    {
        //invalid
        pixout_flg.r = 200; pixout_flg.g = 0; pixout_flg.b = 200;
    }
    return pixout_flg;
}

yarp::sig::PixelMono MapGrid2DDataStorage::CellOccupancyDataToPixel(const MapGrid2DDataStorage::CellOccupancyData& cellin) const
{
    //convert from value to image

    //255 (-1) stands for unknown
    if (cellin == 255)
    {
        return 205;
    }
    //values in the range 0-100 are converted in the range 0-254
    else if (cellin != (unsigned char)(-1) && cellin <=100)
    {
        return (254 - (cellin * 254 / 100));
    }
    else
    {
        //invalid values are in the range 100-255.
        //return invalid value 205
        return 205;
    }
}

MapGrid2DDataStorage::CellOccupancyData MapGrid2DDataStorage::PixelToCellOccupancyData(const yarp::sig::PixelMono& pixin) const
{
    //convert from image to value

    //205 is a special code, used for unknown
    if (pixin == 205)
    {
        return 255;
    }
    //values in the range 0-254 are converted in the range 0-100
    else if (pixin != (unsigned char)(-1))
    {
        auto occ = (unsigned char)((254 - pixin) / 254.0);
        return occ * 100;
    }
    else
    {
        //255 is an invalid value
        return 255;
    }
}

bool MapGrid2DDataStorage::setOrigin(double x, double y, double theta)
{
    //the given x and y are referred to the bottom left corner, pointing outwards.
    //To check if it is inside the map, I have to convert it to a cell with x and y referred to the upper left corner, pointing inwards
    if (m_resolution<=0)
    {
        yWarning() << "MapGrid2DDataStorage::setOrigin() requested is not inside map!";
        return false;
    }

    int xc = (int)(x/ m_resolution);
    int yc = (int)(y / m_resolution);

    XYCell orig(-xc, (m_height-1) + yc);
    if (isInsideMap(orig))
    {
        m_origin.setOrigin(x,y, theta);
        return true;
    }
    else
    {
        yWarning() << "MapGrid2DDataStorage::setOrigin() requested is not inside map!";
        m_origin.setOrigin(x, y, theta);
        return true;
    }
}

void MapGrid2DDataStorage::getOrigin(double& x, double& y, double& theta) const
{
    x = m_origin.get_x();
    y = m_origin.get_y();
    theta = m_origin.get_theta();
}

bool MapGrid2DDataStorage::setResolution(double resolution)
{
    if (resolution <= 0)
    {
        yError() << "MapGrid2DDataStorage::setResolution() invalid value:" << resolution;
        return false;
    }
    m_resolution = resolution;
    return true;
}

void MapGrid2DDataStorage::getResolution(double& resolution) const
{
    resolution = m_resolution;
}

bool MapGrid2DDataStorage::setMapName(std::string map_name)
{
    if (map_name != "")
    {
        m_map_name = map_name;
        return true;
    }
    yError() << "MapGrid2DDataStorage::setMapName() invalid map name";
    return false;
}

std::string MapGrid2DDataStorage::getMapName() const
{
    return m_map_name;
}

bool MapGrid2DDataStorage::setSize_in_meters(double x, double y)
{
    if (x <= 0 && y <= 0)
    {
        yError() << "MapGrid2DDataStorage::setSize() invalid size";
        return false;
    }
    if (m_resolution <= 0)
    {
        yError() << "MapGrid2DDataStorage::setSize() invalid map resolution.";
        return false;
    }
    auto w = (size_t)(x/m_resolution);
    auto h = (size_t)(y/m_resolution);
    setSize_in_cells(w,h);
    return true;
}

bool MapGrid2DDataStorage::setSize_in_cells(size_t x, size_t y)
{
    if (x == 0 && y == 0)
    {
        yError() << "MapGrid2DDataStorage::setSize() invalid size";
        return false;
    }
    m_map_occupancy.resize(x, y);
    m_map_flags.resize(x, y);
    m_map_occupancy.zero();
    m_map_flags.zero();
    m_width = x;
    m_height = y;
    return true;
}

void MapGrid2DDataStorage::getSize_in_meters(double& x, double& y) const
{
    x = m_width* m_resolution;
    y = m_height* m_resolution;
}

void MapGrid2DDataStorage::getSize_in_cells(size_t&x, size_t& y) const
{
    x = m_width;
    y = m_height;
}

bool MapGrid2DDataStorage::setMapFlag(XYCell cell, map_flags flag)
{
    if (isInsideMap(cell) == false)
    {
        yError() << "Invalid cell requested " << cell.x << " " << cell.y;
        return false;
    }
    m_map_flags.safePixel(cell.x, cell.y) = flag;
    return true;
}

bool MapGrid2DDataStorage::getMapFlag(XYCell cell, map_flags& flag) const
{
    if (isInsideMap(cell) == false)
    {
        yError() << "Invalid cell requested " << cell.x << " " << cell.y;
        return false;
    }
    flag = (MapGrid2DDataStorage::map_flags) m_map_flags.safePixel(cell.x, cell.y);
    return true;
}

bool MapGrid2DDataStorage::setOccupancyData(XYCell cell, double occupancy)
{
    if (isInsideMap(cell) == false)
    {
        yError() << "Invalid cell requested " << cell.x << " " << cell.y;
        return false;
    }
    if (occupancy <0)
    {
        m_map_occupancy.safePixel(cell.x, cell.y) = 255;
    }
    else
    {
        m_map_occupancy.safePixel(cell.x, cell.y) = (yarp::sig::PixelMono)(occupancy);
    }
    return true;
}

bool MapGrid2DDataStorage::getOccupancyData(XYCell cell, double& occupancy) const
{
    if (isInsideMap(cell) == false)
    {
        yError() << "Invalid cell requested " << cell.x << " " << cell.y;
        return false;
    }
    if (m_map_occupancy.safePixel(cell.x, cell.y)==255)
    {
      occupancy =-1;
    }
    else
    {
      occupancy = m_map_occupancy.safePixel(cell.x, cell.y);
    }
    return true;
}

bool MapGrid2DDataStorage::setOccupancyGrid(yarp::sig::ImageOf<yarp::sig::PixelMono>& image)
{
    if ((size_t) image.width() != m_width ||
        (size_t) image.height() != m_height)
    {
        yError() << "The size of given occupancy grid does not correspond to the current map. Use method setSize() first.";
        return false;
    }
    m_map_occupancy = image;
    return true;
}

bool MapGrid2DDataStorage::getOccupancyGrid(yarp::sig::ImageOf<yarp::sig::PixelMono>& image) const
{
    image = m_map_occupancy;
    return true;
}

void MapGrid2DDataStorage::clearMapTemporaryFlags()
{
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            switch (m_map_flags.safePixel(x, y))
            {
                case MapGrid2DDataStorage::map_flags::MAP_CELL_TEMPORARY_OBSTACLE:
                case MapGrid2DDataStorage::map_flags::MAP_CELL_ENLARGED_OBSTACLE:
                     m_map_flags.safePixel(x, y) = MAP_CELL_FREE;
                break;
            }
        }
    }
}

bool MapGrid2DDataStorage::enable_map_compression_over_network(bool val)
{
#if defined (YARP_HAS_ZLIB)
    m_compressed_data_over_network = val;
    return true;
#else
    yWarning() << "Zlib library not found, unable to set compression";
    return false;
#endif
}
