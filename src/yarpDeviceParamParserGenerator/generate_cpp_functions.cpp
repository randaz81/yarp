/*
 * SPDX-FileCopyrightText: 2024-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <vector>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "generator.h"

std::string ParamsFilesGenerator::generateFunction_getListOfParams()
{
    std::ostringstream s;
    ADD_DEBUG_COMMENT(s)
s << "\n\
std::vector<std::string> " << m_classname << "_params::getListOfParams() const\n\
{\n";

s << S_TAB1 << "std::vector<std::string> params; \n";

    ADD_DEBUG_COMMENT(s)
for (const auto& param : m_params)
{
    s << S_TAB1 << "params.push_back(\"" << param.getFullParamName() << "\");\n";
}

    ADD_DEBUG_COMMENT(s)
s << S_TAB1 << "return params;\n\
}\n\
\n\
";

    return s.str();
}

std::string ParamsFilesGenerator::generateFunction_getDocumentationOfDeviceParams()
{
    std::ostringstream s;
    ADD_DEBUG_COMMENT(s)
s << "\n\
std::string      " << m_classname << "_params::getDocumentationOfDeviceParams() const\n\
{\n";
s << S_TAB1 << "std::string doc;\n";


for (const auto& param : m_params)
{
     s << S_TAB1 << "doc = doc + std::string(\"'" << param.getFullParamName() << "': ";
     s << param.description;
     s << "\\n\");\n";
}

s << S_TAB1 << "return doc;\n";
s <<\
"}";
    return s.str();
}

void ParamsFilesGenerator::generate_section(std::ostringstream& s, std::deque<std::string> vec, size_t count, size_t siz)
{
    if (vec.size()==0) return;

    //The first iteration should take data from config...(1)
    if (count == 0)
    {
        //The last iteration should put data in sectionp...(2)
        if (count == siz-1)
        {
            s << S_TAB2 << "yarp::os::Bottle sectionp" << ";\n";
            s << S_TAB2 << "sectionp" << " = config.findGroup(\"" << vec.front() << "\");\n";
        }
        //(2)...All other iteration should put data in sectionp<count>
        else
        {
            s << S_TAB2 << "yarp::os::Bottle sectionp" << count << ";\n";
            s << S_TAB2 << "sectionp" << count << " = config.findGroup(\"" << vec.front() << "\");\n";
        }
    }
    //(1)...All other iteration should take data from previous sectionp.
    else
    {
        //The last iteration should put data in sectionp...(2)
        if (count == siz-1)
        {
            s << S_TAB2 << "yarp::os::Bottle sectionp" << ";\n";
            s << S_TAB2 << "sectionp" << " = " << "sectionp" << count - 1 << ".findGroup(\"" << vec.front() << "\");\n";
        }
        //(2)...All other iteration should put data in sectionp<count>
        else
        {
            s << S_TAB2 << "yarp::os::Bottle sectionp" << count << ";\n";
            s << S_TAB2 << "sectionp" << count << " = " << "sectionp" << count - 1 << ".findGroup(\"" << vec.front() << "\");\n";
        }
    }

    vec.pop_front();
    generate_section(s,vec, count+1, siz);
}

void ParamsFilesGenerator::generate_param(std::string origin, std::ostringstream& s, const Parameter& param)
{
    s << \
        S_TAB2 << "if ("<<origin<<".check(\"" << param.getParamOnly() << "\"))\n" << \
        S_TAB2 << "{\n";

    s << \
        S_TAB3 << "m_" << param.getFullParamVariable() << " = "<<origin<<".find(\"" << param.getParamOnly() << "\")";

    if (param.type == "string") { s << ".asString();\n"; }
    else if (param.type == "bool") { s << ".asBool();\n"; }
    else if (param.type == "double") { s << ".asFloat64();\n"; }
    else {
        yFatal("ERROR"); //error
    }

    ADD_DEBUG_COMMENT(s)
        s << \
        S_TAB3 << "yCInfo("<< m_component<< ") << \"Parameter '" << param.getFullParamName() << "' using value:\" << m_" << param.getFullParamVariable() << ";\n";


    ADD_DEBUG_COMMENT(s)

        s << \
        S_TAB2 << "}\n" << \
        S_TAB2 << "else\n" << \
        S_TAB2 << "{\n";

    if (param.required)
    {
        ADD_DEBUG_COMMENT(s)
            s << \
            S_TAB3 << "yCError(" << m_component << ") << \"Mandatory parameter '" << param.getFullParamName() << "' not found!\";\n";
        if (!param.description.empty())
        {
            s << \
            S_TAB3 << "yCError(" << m_component << ") << \"Description of the parameter: " << param.description << "\";\n";
        }
        if (!param.units.empty())
        {
            s << \
            S_TAB3 << "yCError(" << m_component << ") << \"Remember: Units for this parameter are: '" << param.units << "'\";\n";
        }
        s << \
            S_TAB3 << "return false;\n";
    }
    else
    {
        ADD_DEBUG_COMMENT(s)
            s << \
            S_TAB3 << "yCInfo(" << m_component << ") << \"Parameter '" << param.getFullParamName() << "' using DEFAULT value:\" << m_" << param.getFullParamVariable() << "; \n";
    }


    ADD_DEBUG_COMMENT(s)
        s << \
        S_TAB2 << "}\n\
";
}

std::string ParamsFilesGenerator::generateFunction_parseParams()
{
    std::ostringstream s;
    ADD_DEBUG_COMMENT(s)
s << "\n\
bool      "<< m_classname << "_params::parseParams(const yarp::os::Searchable & config)\n\
{\n";

    ADD_DEBUG_COMMENT(s)
for (const auto& param : m_params)
{
    if   (param.getListOfGroups().empty())
    {
        s << S_TAB1 << "//Parser of parameter " <<  param.getParamOnly() <<"\n";
        s << S_TAB1 << "{\n";
        generate_param("config", s, param);
        s << S_TAB1 << "}\n";
        s << "\n";
    }
    else
    {
        s << S_TAB1 << "//Parser of parameter " << param.getFullParamName() << "\n";
        s << S_TAB1 << "{\n";
        auto pg = param.getListOfGroups();
        generate_section (s,pg,0,pg.size());
        generate_param("sectionp", s, param);
        s << S_TAB1 << "}\n";
        s << "\n";
    }
}

s <<\
S_TAB1 << "return true;\n\
}\n\
\n\
";

    return s.str();
}
