/*
 * SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

struct yProcessInfoStruct {
    1: i32 pid=0;
    2: string tag;
    3: string status;
    4: string command;
    5: string env;
}

struct return_ps{
    1: bool result;
    2: list<yProcessInfoStruct> ps;
}

struct return_exit{
    1: bool result;
}

struct return_which{
    1: bool result;
    2: string path;
}

struct return_isRunning{
    1: bool result;
    2: bool isRunning;
}

struct yProperty {
} (
  yarp.name = "yarp::os::Property"
  yarp.includefile = "yarp/os/Property.h"
)

struct ySystemInfoSerializer {
} (
  yarp.name = "yarp::os::SystemInfoSerializer"
  yarp.includefile = "yarp/os/SystemInfoSerializer.h"
)

struct return_sysinfo{
    1: bool result;
    2: ySystemInfoSerializer sysinfo;
}

struct return_kill{
    1: bool result;
}

struct return_killall{
    1: bool result;
}

struct return_sigtermall{
    1: bool result;
}

struct return_sigterm{
    1: bool result;
}

struct return_start{
    1: bool result;
    2: i32 pid;
}

//-------------------------------------------------

service YarprunMsgs
{
    return_ps         psRPC          ();
    return_exit       exitRPC        ();
    return_which      whichRPC       (1: string keyv);
    return_isRunning  isRunningRPC   (1: string keyv);
    return_sysinfo    sysinfoRPC     ();
    return_killall    killallRPC     (1: i16 signal);
    return_kill       killRPC        (1: string keyv, 3: i16 signal);
    return_sigtermall sigtermallRPC  ();
    return_sigterm    sigtermRPC     (1: string keyv);
    return_start      startRPC       (1: yProperty command, 2: string keyv, 3:bool logenable);

}
