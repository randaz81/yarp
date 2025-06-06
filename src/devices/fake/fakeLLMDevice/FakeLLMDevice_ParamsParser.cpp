/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (2.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Thu May 22 11:32:32 2025


#include "FakeLLMDevice_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(FakeLLMDeviceParamsCOMPONENT, "yarp.device.FakeLLMDevice")
}


FakeLLMDevice_ParamsParser::FakeLLMDevice_ParamsParser()
{
}


std::vector<std::string> FakeLLMDevice_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("initial_prompt");
    return params;
}


bool FakeLLMDevice_ParamsParser::getParamValue(const std::string& paramName, std::string& paramValue) const
{
    if (paramName =="initial_prompt")
    {
        paramValue = m_initial_prompt;
        return true;
    }

    yError() <<"parameter '" << paramName << "' was not found";
    return false;

}


std::string FakeLLMDevice_ParamsParser::getConfiguration() const
{
    //This is a sub-optimal solution.
    //Ideally getConfiguration() should return all parameters but it is currently
    //returning only user provided parameters (excluding default values)
    //This behaviour will be fixed in the near future.
    std::string s_cfg = m_provided_configuration;
    return s_cfg;
}

bool      FakeLLMDevice_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(FakeLLMDeviceParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    m_provided_configuration = config.toString();
    yarp::os::Property prop_check(m_provided_configuration.c_str());
    //Parser of parameter initial_prompt
    {
        if (config.check("initial_prompt"))
        {
            m_initial_prompt = config.find("initial_prompt").asString();
            yCInfo(FakeLLMDeviceParamsCOMPONENT) << "Parameter 'initial_prompt' using value:" << m_initial_prompt;
        }
        else
        {
            yCInfo(FakeLLMDeviceParamsCOMPONENT) << "Parameter 'initial_prompt' using DEFAULT value:" << m_initial_prompt;
        }
        prop_check.unput("initial_prompt");
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
                yCError(FakeLLMDeviceParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(FakeLLMDeviceParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
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


std::string      FakeLLMDevice_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: FakeLLMDevice\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'initial_prompt': intial prompt\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device fakeLLMDevice --initial_prompt <optional_value>\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device fakeLLMDevice\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
