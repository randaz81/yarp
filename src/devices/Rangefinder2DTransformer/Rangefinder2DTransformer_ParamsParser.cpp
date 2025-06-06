/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (2.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Thu May 22 11:32:47 2025


#include "Rangefinder2DTransformer_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(Rangefinder2DTransformerParamsCOMPONENT, "yarp.device.Rangefinder2DTransformer")
}


Rangefinder2DTransformer_ParamsParser::Rangefinder2DTransformer_ParamsParser()
{
}


std::vector<std::string> Rangefinder2DTransformer_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("device_position_x");
    params.push_back("device_position_y");
    params.push_back("device_position_theta");
    params.push_back("laser_frame_name");
    params.push_back("robot_frame_name");
    return params;
}


bool Rangefinder2DTransformer_ParamsParser::getParamValue(const std::string& paramName, std::string& paramValue) const
{
    if (paramName =="device_position_x")
    {
        paramValue = std::to_string(m_device_position_x);
        return true;
    }
    if (paramName =="device_position_y")
    {
        paramValue = std::to_string(m_device_position_y);
        return true;
    }
    if (paramName =="device_position_theta")
    {
        paramValue = std::to_string(m_device_position_theta);
        return true;
    }
    if (paramName =="laser_frame_name")
    {
        paramValue = m_laser_frame_name;
        return true;
    }
    if (paramName =="robot_frame_name")
    {
        paramValue = m_robot_frame_name;
        return true;
    }

    yError() <<"parameter '" << paramName << "' was not found";
    return false;

}


std::string Rangefinder2DTransformer_ParamsParser::getConfiguration() const
{
    //This is a sub-optimal solution.
    //Ideally getConfiguration() should return all parameters but it is currently
    //returning only user provided parameters (excluding default values)
    //This behaviour will be fixed in the near future.
    std::string s_cfg = m_provided_configuration;
    return s_cfg;
}

bool      Rangefinder2DTransformer_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    m_provided_configuration = config.toString();
    yarp::os::Property prop_check(m_provided_configuration.c_str());
    //Parser of parameter device_position_x
    {
        if (config.check("device_position_x"))
        {
            m_device_position_x = config.find("device_position_x").asFloat64();
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'device_position_x' using value:" << m_device_position_x;
        }
        else
        {
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'device_position_x' using DEFAULT value:" << m_device_position_x;
        }
        prop_check.unput("device_position_x");
    }

    //Parser of parameter device_position_y
    {
        if (config.check("device_position_y"))
        {
            m_device_position_y = config.find("device_position_y").asFloat64();
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'device_position_y' using value:" << m_device_position_y;
        }
        else
        {
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'device_position_y' using DEFAULT value:" << m_device_position_y;
        }
        prop_check.unput("device_position_y");
    }

    //Parser of parameter device_position_theta
    {
        if (config.check("device_position_theta"))
        {
            m_device_position_theta = config.find("device_position_theta").asFloat64();
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'device_position_theta' using value:" << m_device_position_theta;
        }
        else
        {
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'device_position_theta' using DEFAULT value:" << m_device_position_theta;
        }
        prop_check.unput("device_position_theta");
    }

    //Parser of parameter laser_frame_name
    {
        if (config.check("laser_frame_name"))
        {
            m_laser_frame_name = config.find("laser_frame_name").asString();
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'laser_frame_name' using value:" << m_laser_frame_name;
        }
        else
        {
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'laser_frame_name' using DEFAULT value:" << m_laser_frame_name;
        }
        prop_check.unput("laser_frame_name");
    }

    //Parser of parameter robot_frame_name
    {
        if (config.check("robot_frame_name"))
        {
            m_robot_frame_name = config.find("robot_frame_name").asString();
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'robot_frame_name' using value:" << m_robot_frame_name;
        }
        else
        {
            yCInfo(Rangefinder2DTransformerParamsCOMPONENT) << "Parameter 'robot_frame_name' using DEFAULT value:" << m_robot_frame_name;
        }
        prop_check.unput("robot_frame_name");
    }

    /*
    //This code check if the user set some parameter which are not check by the parser
    //If the parser is set in strict mode, this will generate an error
    if (prop_check.size() > 0)
    {
        bool extra_params_found = false;
        for (auto it=prop_check.begin(); it!=prop_check.end(); it++)
        {
            if (m_parser_is_strict)
            {
                yCError(Rangefinder2DTransformerParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(Rangefinder2DTransformerParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
            }
        }

       if (m_parser_is_strict && extra_params_found)
       {
           return false;
       }
    }
    */
    return true;
}


std::string      Rangefinder2DTransformer_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: Rangefinder2DTransformer\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'device_position_x': X coordinate of the virtual lidar\n");
    doc = doc + std::string("'device_position_y': Y coordinate of the virtual lidar\n");
    doc = doc + std::string("'device_position_theta': Theta coordinate of the virtual lidar\n");
    doc = doc + std::string("'laser_frame_name': If present, open a frameTranformClient to get the robot->laser transform\n");
    doc = doc + std::string("'robot_frame_name': If present, open a frameTranformClient to get the robot->laser transform\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device rangefinder2DTransformer --device_position_x 0 --device_position_y 0 --device_position_theta 0 --laser_frame_name <optional_value> --robot_frame_name <optional_value>\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device rangefinder2DTransformer\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
