/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (2.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Thu May 22 11:32:45 2025


#include "RobotDescription_nws_yarp_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(RobotDescription_nws_yarpParamsCOMPONENT, "yarp.device.RobotDescription_nws_yarp")
}


RobotDescription_nws_yarp_ParamsParser::RobotDescription_nws_yarp_ParamsParser()
{
}


std::vector<std::string> RobotDescription_nws_yarp_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("local");
    return params;
}


bool RobotDescription_nws_yarp_ParamsParser::getParamValue(const std::string& paramName, std::string& paramValue) const
{
    if (paramName =="local")
    {
        paramValue = m_local;
        return true;
    }

    yError() <<"parameter '" << paramName << "' was not found";
    return false;

}


std::string RobotDescription_nws_yarp_ParamsParser::getConfiguration() const
{
    //This is a sub-optimal solution.
    //Ideally getConfiguration() should return all parameters but it is currently
    //returning only user provided parameters (excluding default values)
    //This behaviour will be fixed in the near future.
    std::string s_cfg = m_provided_configuration;
    return s_cfg;
}

bool      RobotDescription_nws_yarp_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(RobotDescription_nws_yarpParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    m_provided_configuration = config.toString();
    yarp::os::Property prop_check(m_provided_configuration.c_str());
    //Parser of parameter local
    {
        if (config.check("local"))
        {
            m_local = config.find("local").asString();
            yCInfo(RobotDescription_nws_yarpParamsCOMPONENT) << "Parameter 'local' using value:" << m_local;
        }
        else
        {
            yCError(RobotDescription_nws_yarpParamsCOMPONENT) << "Mandatory parameter 'local' not found!";
            yCError(RobotDescription_nws_yarpParamsCOMPONENT) << "Description of the parameter: Full port name opened by the device.";
            return false;
        }
        prop_check.unput("local");
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
                yCError(RobotDescription_nws_yarpParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(RobotDescription_nws_yarpParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
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


std::string      RobotDescription_nws_yarp_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: RobotDescription_nws_yarp\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'local': Full port name opened by the device.\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device robotDescription_nws_yarp --local /robotDescriptionServer/rpc\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device robotDescription_nws_yarp --local /robotDescriptionServer/rpc\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
