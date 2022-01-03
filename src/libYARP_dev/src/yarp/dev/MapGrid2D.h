/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_MAPGRID2D_H
#define YARP_DEV_MAPGRID2D_H

#include <yarp/dev/api.h>

#include <yarp/dev/MapGrid2DDataStorage.h>

#include <string>

/**
* \file MapGrid2D.h contains the definition of a map type
*/
namespace yarp::dev::Nav2D {
class YARP_dev_API MapGrid2D: public yarp::dev::Nav2D::MapGrid2DDataStorage
{
    using MapGrid2DDataStorage::MapGrid2DDataStorage;

    public:
    MapGrid2D();
    MapGrid2D(const MapGrid2DDataStorage& data) : MapGrid2DDataStorage(data) {}
    virtual ~MapGrid2D();

    std::string toString() const
    {
        std::ostringstream stringStream;
        stringStream << std::string("not yet implemented");
        return stringStream.str();
    }
    
    //------------------------------utility functions-------------------------------

    /**
     * Modifies the map, cropping pixels at the boundaries.
     * @param left, top, right, bottom: the corners of the map area to keep (expressed in pixel coordinates). If the value is negative, all unknown pixels are removed until a significative pixel is found.
     * @return true if the operation is performed successfully (the input parameters are valid), false otherwise.
     */
    bool crop(int left, int top, int right, int bottom);

#if 0
    /**
     * Checks if a cell is inside the map.
     * @param cell is the cell location, referred to the top-left corner of the map.
     * @return true if cell is inside the map, false otherwise.
     */
    bool   isInsideMap(XYCell cell) const;

    /**
     * Checks if a world coordinate is inside the map.
     * @param world is the world coordinate, expressed in meters, referred to the map origin reference frame.
     * @return true if cell is inside the map, false otherwise.
     */
    bool   isInsideMap(XYWorld world) const;
#endif

    /**
     * Checks is two maps are identical.
     * @return true if all the internal data of the maps are identical, false otherwise.
     */
    bool isIdenticalTo(const MapGrid2D& otherMap) const;

    /**
     * Performs the obstacle enlargement operation. It's useful to set size to a value equal or larger to the radius of the robot bounding box.
     * In this way a navigation algorithm can easily check obstacle collision by comparing the location of the center of the robot with cell value (free/occupied etc)
     * @param size the size of the enlargement, in meters. If size>0 the requested enlargement is performed. If the function is called multiple times, the enlargement sums up.
     * If size <= 0 the enlargement stored in the map is cleaned up.
     * @return true always.
     */
    bool enlargeObstacles(double size);

    //-------------------------------file access functions-------------------------------
    /**
     * Loads a yarp map file from disk. File must have .map extension.
     * param map_filename is the full path to the map file.
     * @return true if load was successful, false otherwise.
     */
    bool loadFromFile(std::string map_filename);

    /**
     * Store a yarp map file to disk. File must have .map extension.
     * param map_filename is the full path to the map file.
     * @return true if load was successful, false otherwise.
     */
    bool saveToFile(std::string map_filename) const;

    protected:
    // internal methods to read a map from file, either in yarp or ROS format
    bool loadMapYarpOnly(std::string yarp_img_filename);
    bool loadMapROSOnly(std::string ros_yaml_filename);
    bool loadROSParams(std::string ros_yaml_filename, std::string& pgm_occ_filename, double& resolution, double& orig_x, double& orig_y, double& orig_t);
    bool loadMapYarpAndRos(std::string yarp_img_filename, std::string ros_yaml_filename);
    bool parseMapParameters(const yarp::os::Property& mapfile);

    public:
    bool enable_map_compression_over_network(bool val);
};

} // namespace yarp::dev::Nav2D

#endif // YARP_DEV_MAPGRID2D_H
