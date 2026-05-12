/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/run/Run.h>
#include <yarp/run/impl/RunCheckpoints.h>
#include <yarp/run/impl/PlatformStdlib.h>
#include <yarp/run/impl/PlatformUnistd.h>
#include <yarp/run/impl/PlatformSysPrctl.h>
#include <yarp/run/impl/RunProcManager.h>

#include <yarp/conf/environment.h>
#include <yarp/conf/filesystem.h>

#include <yarp/os/Network.h>
#include <yarp/os/Os.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/SystemInfo.h>
#include <yarp/os/SystemInfoSerializer.h>
#include <yarp/os/Time.h>

#include <yarp/os/impl/NameClient.h>
#include <yarp/os/impl/PlatformSignal.h>
#include <yarp/os/impl/PlatformStdio.h>

#include <cstdio>
#include <string>
#include <cstring>
#include <random>

#if defined(_WIN32)
# if !defined(WIN32_LEAN_AND_MEAN)
#  define WIN32_LEAN_AND_MEAN
# endif
# include <windows.h>
#else
# define C_MAXARGS       128     // the max number of command parameters. rational?
#endif



///////////////////////////
// OS INDEPENDENT FUNCTIONS
///////////////////////////

/////////
int yarp::run::main(int argc, char *argv[])
{
    yarp::os::Property config;
    config.fromCommand(argc, argv, false);

    // SERVER
    if (config.check("server"))
    {
        bool logged = false;
        std::string loggerportName = "/yarplogger";
        logged=config.check("log");

        if (logged)
        {
            yarp::os::Bottle botPortLogger=config.findGroup("log");

            if (botPortLogger.size()>1)
            {
                loggerportName=botPortLogger.get(1).asString();
            }
        }

        std::string serverportName=std::string(config.find("server").asString());

        RunServer server;
        return server.serverCLI(serverportName, loggerportName);
    }

    if (!yarp::os::Network::getLocalMode())
    {
        if (!yarp::os::Network::checkNetwork())
        {
            fprintf(stderr, "ERROR: no yarp network found.\n");

            return YARPRUN_ERROR;
        }
    }

    // STRESSTEST
#if 0
    if (config.check("stresstest"))
    {
        fprintf(stderr, "Yarprun stress test started.\n");
        fflush(stderr);

        int max_interval_ms=config.find("stresstest").asInt32();
        std::string tag_zero=config.find("as").asString();
        yarp::os::Bottle srv=config.findGroup("on");

        config.unput("as");
        config.unput("stresstest");

        std::string cmd;

        bool isCommand=false;

        if (config.check("cmd"))
        {
            isCommand=true;
            cmd=config.find("cmd").asString();
            config.unput("cmd");
        }

        unsigned int t=0, u=0;
        int term_cycle=0;

        char tag[256];
        char cmd_and_name[512];

        mStresstest=true;

        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_int_distribution<int> dist0maxint(0, max_interval_ms -1);

        while (mStresstest)
        {
            yarp::os::SystemClock::delaySystem(0.001*(dist0maxint(mt)));

            yarp::os::Property stresser=config;

            sprintf(tag, "%s_%u", tag_zero.c_str(), t++);
            stresser.put("as", tag);

            if (isCommand)
            {
                sprintf(cmd_and_name, "%s --name /%s", cmd.c_str(), tag);
                stresser.put("cmd", cmd_and_name);
            }

            client(stresser);

            std::uniform_int_distribution<int> dist07(0, 7);
            if (isCommand && ++term_cycle>=4)
            {
                term_cycle=0;

                int r = t - (dist07(mt));

                for (int i=u; i<r; ++i)
                {
                    sprintf(tag, "%s_%u", tag_zero.c_str(), i);

                    yarp::os::Bottle as;
                    as.addString("sigterm");
                    as.addString(tag);

                    yarp::os::Bottle term;
                    term.addList()=srv;
                    term.addList()=as;

                    sendMsg(term, srv.get(1).asString());

                    ++u;
                }
            }
        }

        return 0;
    }
#endif
    // STRESSTEST

    // HELP
    if (config.check("help"))
    {
        RunClient cli;
        cli.Help();
        return 0;
    }

    // CLIENT (config is from keyboard)
    if (config.check("stdio")
     || config.check("cmd")
     || config.check("kill")
     || config.check("sigterm")
     || config.check("sigtermall")
     || config.check("exit")
     || config.check("isrunning")
     || config.check("ps")
     || config.check("env")
     || config.check("sysinfo")
     || config.check("which"))
    {
        RunClient cli;
        int ret=cli.clientCLI(config);

        return ret;
    }

    fprintf(stderr, "ERROR: no valid options provided.\n");
    RunClient cli;
    cli.Help();

    return 0;
}



