/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _USE_MATH_DEFINES

#include <yarp/dev/MapGrid2D.h>

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


bool MapGrid2D::isIdenticalTo(const MapGrid2D& other) const
{
    if (m_map_name != other.m_map_name) {
        return false;
    }
    if (m_origin != other.m_origin) {
        return false;
    }
    if (m_resolution != other.m_resolution) {
        return false;
    }
    if (m_width != other.width()) {
        return false;
    }
    if (m_height != other.height()) {
        return false;
    }
    for (size_t y = 0; y < m_height; y++) {
        for (size_t x = 0; x < m_width; x++) {
            if (m_map_occupancy.safePixel(x, y) != other.m_map_occupancy.safePixel(x, y)) {
                return false;
            }
        }
    }
    for (size_t y = 0; y < m_height; y++) {
        for (size_t x = 0; x < m_width; x++) {
            if (m_map_flags.safePixel(x, y) != other.m_map_flags.safePixel(x, y)) {
                return false;
            }
        }
    }
    return true;
}

MapGrid2D::MapGrid2D()
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
            m_map_flags.safePixel(x, y) = MapGrid2D::map_flags::MAP_CELL_FREE;
        }
    }
#if defined (YARP_HAS_ZLIB)
    m_compressed_data_over_network = true;
#else
    m_compressed_data_over_network = false;
#endif
}

MapGrid2D::~MapGrid2D() = default;

bool MapGrid2D::enlargeObstacles(double size)
{
    if (size <= 0)
    {
        for (size_t y = 0; y < m_height; y++)
        {
            for (size_t x = 0; x < m_width; x++)
            {
                if (this->m_map_flags.safePixel(x, y) == MapGrid2D::map_flags::MAP_CELL_ENLARGED_OBSTACLE)
                {
                    this->m_map_flags.safePixel(x, y) = MapGrid2D::map_flags::MAP_CELL_FREE;
                }
            }
        }
        return true;
    }
    auto repeat_num = (size_t)(std::ceil(size/ m_resolution));
    for (size_t repeat = 0; repeat < repeat_num; repeat++)
    {
        //contains the cells to be enlarged;
        std::vector<XYCell> list_of_cells;
        for (size_t y = 0; y < m_height; y++)
        {
            for (size_t x = 0; x < m_width; x++)
            {
                //this check could be optimized...
                if (this->m_map_flags.safePixel(x, y) == MAP_CELL_KEEP_OUT ||
                    this->m_map_flags.safePixel(x, y) == MAP_CELL_ENLARGED_OBSTACLE ||
                    this->m_map_flags.safePixel(x, y) == MAP_CELL_WALL ||
                    this->m_map_flags.safePixel(x, y) == MAP_CELL_UNKNOWN ||
                    this->m_map_flags.safePixel(x, y) == MAP_CELL_TEMPORARY_OBSTACLE)
                {
                    list_of_cells.emplace_back(x, y);
                }
            }
        }

        //process each cell of the list and enlarges it
        for (auto& list_of_cell : list_of_cells)
        {
            enlargeCell(list_of_cell);
        }
    }
    return true;
}

bool  MapGrid2D::loadFromFile(std::string map_file_with_path)
{
    Property mapfile_prop;
    std::string mapfile_path = extractPathFromFile(map_file_with_path);
    if (mapfile_prop.fromConfigFile(map_file_with_path) == false)
    {
        yError() << "Unable to open .map description file:" << map_file_with_path;
        return false;
    }

    if (mapfile_prop.check("MapName") ==false)
    {
        yError() << "Unable to find 'MapName' parameter inside:" << map_file_with_path;
        return false;
    }
    m_map_name = mapfile_prop.find("MapName").asString();

    bool YarpMapDataFound = false;
    std::string ppm_flg_filename;
    if (mapfile_prop.check("YarpMapData") == false)
    {
        yWarning() << "Unable to find 'YarpMapData' parameter inside:" << map_file_with_path;
        YarpMapDataFound = false;
    }
    else
    {
        ppm_flg_filename = mapfile_prop.find("YarpMapData").asString();
        YarpMapDataFound = true;
    }

    bool RosMapDataFound = false;
    std::string yaml_filename;
    if (mapfile_prop.check("RosMapData") == false)
    {
        yWarning() << "Unable to find 'RosMapData' parameter inside:" << map_file_with_path;
        RosMapDataFound = false;
    }
    else
    {
        yaml_filename = mapfile_prop.find("RosMapData").asString();
        RosMapDataFound = true;
    }

    m_width = -1;
    m_height = -1;
    std::string yarp_flg_filename_with_path = mapfile_path + ppm_flg_filename;
    std::string ros_yaml_filename_with_path = mapfile_path + yaml_filename;
    if (YarpMapDataFound && RosMapDataFound)
    {
        //yarp and ros
        yDebug() << "Opening files: "<< yarp_flg_filename_with_path << " and " << ros_yaml_filename_with_path;
        return this->loadMapYarpAndRos(yarp_flg_filename_with_path, ros_yaml_filename_with_path) &&
               this->parseMapParameters(mapfile_prop);
    }
    else if (!YarpMapDataFound && RosMapDataFound)
    {
        //only ros
        yDebug() << "Opening file:" << ros_yaml_filename_with_path;
        return this->loadMapROSOnly(ros_yaml_filename_with_path) &&
               this->parseMapParameters(mapfile_prop);
    }
    else if (YarpMapDataFound && !RosMapDataFound)
    {
        //only yarp
        yDebug() << "Opening file:" << yarp_flg_filename_with_path;
        return this->loadMapYarpOnly(yarp_flg_filename_with_path) &&
               this->parseMapParameters(mapfile_prop);
    }
    else
    {
        yError() << "Critical error: unable to find neither 'RosMapData' nor 'YarpMapData' inside:" << map_file_with_path;
        return false;
    }
    return true;
}

bool  MapGrid2D::crop (int left, int top, int right, int bottom)
{
    if (top < 0)
    {
        for (size_t j=0;j<height();j++){
            for (size_t i=0;i<width();i++){
                yarp::sig::PixelMono pix = m_map_occupancy.safePixel(i,j);
                if ( pix != 255)
                {
                 top = j;
                    goto topFound;
                }
            }
        }
    }
    topFound:

    if (bottom < 0)
    {
        for (int j=height()-1; j>0; j--){
            for (int i=width()-1; i>0 ;i--){
                yarp::sig::PixelMono pix = m_map_occupancy.safePixel(i,j);
                if ( pix != 255)
                {
                    bottom = j+1;
                    goto bottomFound;
                }
            }
        }
    }
    bottomFound:

    if (left<0)
    {
        for (size_t i=0;i<width();i++){
            for (size_t j=0;j<height();j++){
                yarp::sig::PixelMono pix = m_map_occupancy.safePixel(i,j);
                if ( pix != 255)
                {
                    left = i;
                    goto leftFound;
                }
           }
        }
    }
    leftFound:

    if (right<0)
    {
        for (size_t i=width()-1;i>0;i--){
            for (size_t j=0;j<height();j++){
                yarp::sig::PixelMono pix = m_map_occupancy.safePixel(i,j);
                if ( pix != 255)
                {
                    right = i;
                    goto rightFound;
                }
           }
        }
    }
    rightFound:

        if (left > (int)this->width()) {
            return false;
        }
        if (right > (int)this->width()) {
            return false;
        }
        if (top > (int)this->height()) {
            return false;
        }
        if (bottom > (int)this->height()) {
            return false;
        }

    yarp::sig::ImageOf<CellOccupancyData> new_map_occupancy;
    yarp::sig::ImageOf<CellFlagData> new_map_flags;

    new_map_occupancy.setQuantum(1);
    new_map_flags.setQuantum(1);
    new_map_occupancy.resize(right-left,bottom-top);
    new_map_flags.resize(right-left,bottom-top);

//     size_t original_width = m_map_occupancy.width();
    size_t original_height = m_map_occupancy.height();

    for (int j = top, y = 0; j < bottom; j++, y++) {
        for (int i=left, x=0; i<right; i++, x++)
        {
            new_map_occupancy.safePixel(x,y) = m_map_occupancy.safePixel(i,j);
            new_map_flags.safePixel(x,y)     = m_map_flags.safePixel(i,j);
        }
    }
    m_map_occupancy.copy(new_map_occupancy);
    m_map_flags.copy(new_map_flags);
    this->m_width=m_map_occupancy.width();
    this->m_height=m_map_occupancy.height();
    yDebug() << m_origin.get_x() << m_origin.get_y();
    double new_x0 = m_origin.get_x() +(left*m_resolution);
    double new_y0 = m_origin.get_y() +(double(original_height)-double(bottom))*m_resolution;
    m_origin.setOrigin(new_x0,new_y0, m_origin.get_theta());
    return true;
}

bool  MapGrid2D::saveToFile(std::string map_file_with_path) const
{
    std::string mapfile_path = extractPathFromFile(map_file_with_path);

    std::string yarp_filename = this->getMapName() + "_yarpflags.ppm";
    std::string yaml_filename = this->getMapName() + "_grid.yaml";
    std::string pgm_occ_filename = this->getMapName() + "_grid.pgm";

    std::ofstream map_file;
    map_file.open(map_file_with_path.c_str());
    if (!map_file.is_open())
    {
        return false;
    }
    map_file << "MapName: "<< this->getMapName() << '\n';
    map_file << "YarpMapData: "<< yarp_filename << '\n';
    map_file << "RosMapData: "<< yaml_filename << '\n';
    map_file.close();

    std::ofstream yaml_file;
    yaml_file.open(mapfile_path + yaml_filename.c_str());
    if (!yaml_file.is_open())
    {
        return false;
    }
    yaml_file << "image: " << pgm_occ_filename << '\n';
    yaml_file << "resolution: " << m_resolution << '\n';
    yaml_file << "origin: [ " << m_origin.get_x() << " " << m_origin.get_y() << " " << m_origin.get_theta() << " ]"<< '\n';
    yaml_file << "negate: 0" << '\n';
    yaml_file << "occupied_thresh: " << m_occupied_thresh << '\n';
    yaml_file << "free_thresh: " << m_free_thresh << '\n';

    yaml_file.close();

    yarp::sig::ImageOf<yarp::sig::PixelRgb> img_flg;
    yarp::sig::ImageOf<yarp::sig::PixelMono> img_occ;

    img_flg.resize(m_width, m_height);
    img_occ.resize(m_width, m_height);
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            yarp::sig::PixelMono pix_flg = m_map_flags.safePixel(x, y);
            yarp::sig::PixelMono pix_occ = m_map_occupancy.safePixel(x,y);
            img_flg.safePixel(x, y) =  CellFlagDataToPixel(pix_flg);
            img_occ.safePixel(x, y) = CellOccupancyDataToPixel(pix_occ);
        }
    }

    //std::string ppm_flg_filename = (pgm_occ_filename.substr(0, pgm_occ_filename.size() - 4)) + "_yarpflags" + ".ppm";
    std::string ppm_flg_filename = yarp_filename;
    bool ret = true;
    ret &= yarp::sig::file::write(img_occ, mapfile_path + pgm_occ_filename);
    ret &= yarp::sig::file::write(img_flg, mapfile_path + ppm_flg_filename);
    return ret;
}

bool MapGrid2D::loadROSParams(std::string ros_yaml_filename, std::string& pgm_occ_filename, double& resolution, double& orig_x, double& orig_y, double& orig_t)
{
    std::string file_string;
    std::ifstream file;
    file.open(ros_yaml_filename.c_str());
    if (!file.is_open())
    {
        yError() << "failed to open file" << ros_yaml_filename;
        return false;
    }

    std::string line;
    while (getline(file, line))
    {
        if (line.find("origin") != std::string::npos)
        {
            std::replace(line.begin(), line.end(), ',', ' ');
            std::replace(line.begin(), line.end(), '[', '(');
            std::replace(line.begin(), line.end(), ']', ')');
            /*
            auto it = line.find('[');
            if (it != std::string::npos) line.replace(it, 1, "(");
            it = line.find(']');
            if(it != std::string::npos) line.replace(it, 1, ")");*/
        }
        file_string += (line + '\n');
    }
    file.close();

    bool ret = true;
    Bottle bbb;
    bbb.fromString(file_string);
    std::string debug_s = bbb.toString();

    if (bbb.check("image:") == false) { yError() << "missing image"; ret = false; }
    pgm_occ_filename = bbb.find("image:").asString();
    //ppm_flg_filename = (pgm_occ_filename.substr(0, pgm_occ_filename.size()-4))+"_yarpflags"+".ppm";

    if (bbb.check("resolution:") == false) { yError() << "missing resolution"; ret = false; }
    resolution = bbb.find("resolution:").asFloat64();

    if (bbb.check("origin:") == false) { yError() << "missing origin"; ret = false; }
    Bottle* b = bbb.find("origin:").asList();
    if (b)
    {
        orig_x = b->get(0).asFloat64();
        orig_y = b->get(1).asFloat64();
        orig_t = b->get(2).asFloat64() * RAD2DEG;
    }

    if (bbb.check("occupied_thresh:"))
    {
        m_occupied_thresh = bbb.find("occupied_thresh:").asFloat64();
    }

    if (bbb.check("free_thresh:"))
    {
        m_free_thresh = bbb.find("free_thresh:").asFloat64();
    }

    return ret;
}

bool MapGrid2D::loadMapYarpAndRos(std::string yarp_filename, std::string ros_yaml_filename)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarp_img;
    yarp::sig::ImageOf<yarp::sig::PixelMono> ros_img;
    bool b1 = yarp::sig::file::read(yarp_img, yarp_filename);
    if (b1 == false)
    {
        yError() << "Unable to load map data" << yarp_filename;
        return false;
    }
    std::string pgm_occ_filename;
    double resolution = 0;
    double orig_x = 0;
    double orig_y = 0;
    double orig_t = 0;
    bool b2 = loadROSParams(ros_yaml_filename, pgm_occ_filename, resolution, orig_x, orig_y, orig_t);
    if (b2 == false)
    {
        yError() << "Unable to ros params from" << ros_yaml_filename;
        return false;
    }
    std::string path = extractPathFromFile(ros_yaml_filename);
    std::string extension = extractExtensionFromFile(pgm_occ_filename);
    std::string pgm_occ_filename_with_path = path + pgm_occ_filename;
    bool b3 = yarp::sig::file::read(ros_img, pgm_occ_filename_with_path);
    if (b3 == false)
    {
        yError() << "Unable to load occupancy grid file:" << pgm_occ_filename_with_path;
        return false;
    }

    if (yarp_img.width() == ros_img.width() && yarp_img.height() == ros_img.height())
    {
        //Everything ok, proceed to internal assignments
        setSize_in_cells(yarp_img.width(), yarp_img.height());
        m_resolution = resolution;
        m_origin.setOrigin(orig_x, orig_y, orig_t);

        //set YARPS stuff
        for (size_t y = 0; y < m_height; y++)
        {
            for (size_t x = 0; x < m_width; x++)
            {
                m_map_flags.safePixel(x, y) = PixelToCellFlagData(yarp_img.safePixel(x, y));
            }
        }

        //set ROS Stuff
        for (size_t y = 0; y < m_height; y++)
        {
            for (size_t x = 0; x < m_width; x++)
            {
                m_map_occupancy.safePixel(x, y) = PixelToCellOccupancyData(ros_img.safePixel(x, y));
            }
        }
    }
    else
    {
        yError() << "MapGrid2DDataStorage::loadFromFile() Size of YARP map and ROS do not match";
        return false;
    }

    return true;
}

bool MapGrid2D::loadMapROSOnly(std::string ros_yaml_filename)
{
    yarp::sig::ImageOf<yarp::sig::PixelMono> ros_img;
    std::string pgm_occ_filename;
    double resolution = 0;
    double orig_x = 0;
    double orig_y = 0;
    double orig_t = 0;
    bool b2 = loadROSParams(ros_yaml_filename, pgm_occ_filename, resolution, orig_x, orig_y, orig_t);
    if (b2 == false)
    {
        yError() << "Unable to ros params from" << ros_yaml_filename;
        return false;
    }
    std::string path = extractPathFromFile(ros_yaml_filename);
    std::string extension = extractExtensionFromFile(pgm_occ_filename);
    std::string pgm_occ_filename_with_path = path + pgm_occ_filename;
    bool b3 = yarp::sig::file::read(ros_img, pgm_occ_filename_with_path);
    if (b3 == false)
    {
        yError() << "Unable to load occupancy grid file:" << pgm_occ_filename;
        return false;
    }

    //Everything ok, proceed to internal assignments
    setSize_in_cells(ros_img.width(), ros_img.height());
    m_resolution = resolution;
    m_origin.setOrigin(orig_x, orig_y, orig_t);

    //set ROS Stuff
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            m_map_occupancy.safePixel(x, y) = PixelToCellOccupancyData(ros_img.safePixel(x, y));
        }
    }

    //generate YARP stuff from ROS Stuff
    for (size_t y = 0; y < (size_t)(m_map_occupancy.height()); y++)
    {
        for (size_t x = 0; x < (size_t)(m_map_occupancy.width()); x++)
        {
            //occup_prob is a probability, in the range 0-100 (-1 = 255 = unknown)
            CellOccupancyData occup_prob = m_map_occupancy.safePixel(x, y);
            if (occup_prob != (unsigned char)(-1) && occup_prob < 50) { m_map_flags.safePixel(x, y) = MAP_CELL_FREE; }
            else if (occup_prob >= 50 && occup_prob <= 100) { m_map_flags.safePixel(x, y) = MAP_CELL_WALL; }
            else if (occup_prob > 100) { m_map_flags.safePixel(x, y) = MAP_CELL_UNKNOWN; }
            else { yError() << "Unreachable"; }
        }
    }
    return true;
}

bool MapGrid2D::loadMapYarpOnly(std::string yarp_filename)
{
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarp_img;
    bool b1 = yarp::sig::file::read(yarp_img, yarp_filename);
    if (b1 == false)
    {
        yError() << "Unable to load map" << yarp_filename;
        return false;
    }
    //Everything ok, proceed to internal assignments
    setSize_in_cells(yarp_img.width(), yarp_img.height());
    //m_resolution = resolution;    //????
    //m_origin.x = orig_x;          //????
    //m_origin.y = orig_y;          //????
    //m_origin.theta = orig_t;      //????

    //set YARPS stuff
    for (size_t y = 0; y < m_height; y++)
    {
        for (size_t x = 0; x < m_width; x++)
        {
            m_map_flags.safePixel(x, y) = PixelToCellFlagData(yarp_img.safePixel(x, y));
        }
    }

    //generate ROS stuff from YARP Stuff
    for (size_t y = 0; y < (size_t)(m_map_flags.height()); y++)
    {
        for (size_t x = 0; x < (size_t)(m_map_flags.width()); x++)
        {
            yarp::sig::PixelMono pix_flg = m_map_flags.safePixel(x, y);

            if (pix_flg == MAP_CELL_FREE) {
                m_map_occupancy.safePixel(x, y) = 0; //@@@SET HERE
            }
            else if (pix_flg == MAP_CELL_KEEP_OUT) {
                m_map_occupancy.safePixel(x, y) = 0; //@@@SET HERE
            }
            else if (pix_flg == MAP_CELL_TEMPORARY_OBSTACLE) {
                m_map_occupancy.safePixel(x, y) = 0; //@@@SET HERE
            }
            else if (pix_flg == MAP_CELL_ENLARGED_OBSTACLE) {
                m_map_occupancy.safePixel(x, y) = 0; //@@@SET HERE
            }
            else if (pix_flg == MAP_CELL_WALL) {
                m_map_occupancy.safePixel(x, y) = 100; //@@@SET HERE
            }
            else if (pix_flg == MAP_CELL_UNKNOWN) {
                m_map_occupancy.safePixel(x, y) = 255; //@@@SET HERE
            }
            else {
                m_map_occupancy.safePixel(x, y) = 255; //@@@SET HERE
            }
        }
    }
    m_occupied_thresh = 0.80; //@@@SET HERE
    m_free_thresh = 0.20;//@@@SET HERE
    return true;
}

bool MapGrid2D::parseMapParameters(const Property& mapfile)
{
    //Map parameters.
    //these values can eventually override values previously assigned
    //(e.g. ROS values found in yaml data)
    if (mapfile.check("resolution"))
    {
        m_resolution = mapfile.find("resolution").asFloat32();
    }
    if (mapfile.check("origin"))
    {
        Bottle* b = mapfile.find("origin").asList();
        if (b)
        {
            m_origin.setOrigin(b->get(0).asFloat32(),
                b->get(1).asFloat32(),
                b->get(2).asFloat32());
        }
    }

    return true;
}

bool MapGrid2D::enable_map_compression_over_network(bool val)
{
#if defined (YARP_HAS_ZLIB)
    m_compressed_data_over_network = val;
    return true;
#else
    yWarning() << "Zlib library not found, unable to set compression";
    return false;
#endif
}
