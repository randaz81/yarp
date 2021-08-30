/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_OS_IDL_WIREVOCAB_H
#define YARP_OS_IDL_WIREVOCAB_H

#include <yarp/os/api.h>

#include <cstdint>
#include <string>

namespace yarp {
namespace os {
namespace idl {

class YARP_os_API WireVocab32
{
public:
    virtual ~WireVocab32() {}
    virtual int32_t fromString(const std::string& input) = 0;
    virtual std::string toString(int32_t input) const = 0;
};

#ifndef YARP_NO_DEPRECATED // Since YARP 3.6
YARP_DEPRECATED_TYPEDEF_MSG("Use yarp::dev::WireVocab32 instead") WireVocab32 WireVocab;
#endif

} // namespace idl
} // namespace os
} // namespace yarp

#endif // YARP_OS_IDL_WIREVOCAB_H
