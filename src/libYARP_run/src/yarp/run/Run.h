/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_RUN_RUN_H
#define YARP_RUN_RUN_H

#include <vector>

#include <cstring>
#include <yarp/run/api.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemInfoSerializer.h>

class ZombieHunterThread;
class YarpRunInfoVector;

/*
 * Typical YARP applications consist of several intercommunicating modules distributed on different machines.
 * If a yarprun server is running on each machine, distributed applications can be remotely launched,
 * monitored and terminated by yarprun commands.
 *
 * - To run a yarprun server on a machine:
 *      $ yarprun --server /SERVERPORT
 *
 * /SERVERPORT must be unique and identifies the remote machine.
 *
 * - The basic command to run a command/application on a remote machine is:
 *      $ yarprun --on /SERVERPORT --as TAG --cmd COMMAND [ARGLIST]
 *
 * /SERVERPORT is the name of the server that actually runs the command
 * TAG identifies the application process set, and must be unique
 * COMMAND is the application that has to be executed, followed by the optional argument list
 *
 * Some options can be added to the basic format of yarprun:
 *      $ yarprun --on /SERVERPORT1 --as TAG --cmd COMMAND [ARGLIST] --stdio /SERVERPORT2
 *
 * opens a remote shell window where the stdin, stdout and stderr of the application will be redirected.
 * /SERVERPORT2 specifies the machine where the IO shell will be executed, and can be either a remote machine or
 * be equal to /SERVERPORT1 itself.
 *
 * If --stdio is specified, there are two useful sub-options (linux only):
 * - --hold keep the stdio window open even if the command is terminated or aborted.
 * - --geometry WxH+X+Y set the stdio window size and position. Example: --geometry 320x240+80+20
 *
 * Other yarprun commands:
 *
 * - To terminate an application, the yarprun syntax is:
 *      $ yarprun --on /SERVERPORT --sigterm TAG
 *
 * - To send a signal to an application (usually SIGKILL) use:
 *      $ yarprun --on /SERVERPORT --kill TAG SIGNUM
 *
 * - To terminate all the applications managed by a yarprun server, use:
 *      $ yarprun --on /SERVERPORT --sigtermall
 *
 * - To check if an application is still running on a yarprun server, use:
 *      $ yarprun --on /SERVERPORT --isrunning TAG
 *
 * - To get a report of applications running on a yarprun server, use:
 *      $ yarprun --on /SERVERPORT --ps
 *
 * - To shutdown a yarprun server, use:
 *      $ yarprun --on /SERVERPORT --exit
 *
 */

namespace yarp::run {

/**
 * The main function of yarprun. It can be used to start a yarprun server or client.
 */
int YARP_run_API main(int argc, char *argv[]);

/**
 * \class yarp::os::Run
 * \brief yarprun provides the APIs to a client-server environment that is able to run,
 * kill and monitor applications commands on a remote machine in Windows and Linux.
 */
class YARP_run_API RunClient
{
public:
    //Default constructor/destructor
    RunClient();
    ~RunClient();

    /**
     * Executes a command on a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param command is the command to be executed by the remote server. It can include
     * an argument list and different options, in the standard yarp Property key/value mode:
     * - name COMMAND_NAME
     * - parameters ARGUMENT_LIST (optional)
     * - stdio /SERVERPORT (optional)
     * - geometry WxH+X+Y (optional)
     * - hold (optional)
     * @param keyv is the tag that will identify the running application. It must be unique in the network.
     * @return true=success false=failed.
     */
    bool start(const std::string &node, yarp::os::Property &command, const std::string &keyv);

    /**
     * Terminate an application running on a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param keyv is the tag that identifies the running application. It must be unique in the network.
     * @return true=success false=failed.
     */
    bool sigterm(const std::string &node, const std::string &keyv);

    /**
     * Terminate all applications running on a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @return true=success false=failed.
     */
    bool sigtermall(const std::string &node);

    /**
     * Send a SIGNAL to an application running on a yarprun server (Linux only).
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param keyv is the tag that identifies the running application. It must be unique in the network.
     * @param s is the SIGNAL number.
     * @return true=success false=failed.
     */
    bool kill(const std::string &node, const std::string &keyv, int s);

    /**
     * Get a report of all applications running on a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param processes is a list of applications running on the remote yarprun server.
     * @return 0=success -1=failed.
     */
    bool ps(const std::string &node, std::vector<yarp::os::SystemInfo::ProcessInfoYarpRun>& processes);

    /**
     * Get a report of system information of a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param info is the system information of the remote yarprun server.
     * @return true=success false=failed.
     */
    bool sysinfo(const std::string& node, yarp::os::SystemInfoSerializer& info);


    /**
     * Report if an application is still running on a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param keyv is the tag that identifies the application. It must be unique in the network.
     * @return true=running false=terminated.
     */
    bool isRunning(const std::string &node, const std::string &keyv);

    /**
     * Display the path of a file on a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @param keyv is the tag that identifies the application. It must be unique in the network.
     * @param path is the returned path of the file on the remote yarprun server.
     * @return true=running false=terminated.
     */
    bool which(const std::string &node, const std::string &keyv, std::string& path);

    /**
     * Exit a yarprun server.
     * @param node is the yarprun server port name. It must be unique in the network.
     * @return true=success false=failed.
     */
    bool exit(const std::string& node);

    /**
    * Display the usage of yarprun command.
    */
    void Help(const char* msg="");

    /**
    * Command Line Interface for a yarprun client.
    * It can be used to send commands to a yarprun server from the keyboard.
    */
    int clientCLI(yarp::os::Property& config);

private:
    bool ConnectToServer(yarp::os::Port& port, const std::string& node);
    void* mThriftInterface = nullptr;
};

class ServerYarprunMsgs; //forward declaration

class YARP_run_API RunServer
{
    friend class ServerYarprunMsgs;

    yarp::os::RpcServer *pServerPort=nullptr;

YARP_WARNING_PUSH
YARP_DISABLE_DLL_INTERFACE_WARNING
    bool mStresstest=false;
    bool mLogged=false;

    //this is set by the signal handler
    static inline bool mIsTerminated = false;

#if defined(_WIN32)
    YarpRunInfoVector* mProcessVector;
    YarpRunInfoVector* mStdioVector;
#else
    YarpRunInfoVector *mProcessVector;
    YarpRunInfoVector *mStdioVector;
    ZombieHunterThread *mBraveZombieHunter;
    void CleanZombie(int pid);
#define READ_FROM_PIPE 0
#define WRITE_TO_PIPE  1
#define REDIRECT_TO(from, to) yarp::run::impl::dup2(to, from)
#endif
YARP_WARNING_POP

    int executeCmdAndStdio(const yarp::os::Bottle& msg, yarp::os::Bottle& result);
    int executeCmdStdout(const yarp::os::Bottle& msg, yarp::os::Bottle& result, const std::string& loggerName);
    int executeCmd(const yarp::os::Bottle& msg, yarp::os::Bottle& result);
    int userStdio(const yarp::os::Bottle& msg, yarp::os::Bottle& result);

    inline bool IS_PARENT_OF(int pid){ return pid>0; }
    inline bool IS_NEW_PROCESS(int pid){ return !pid; }
    inline bool IS_INVALID(int pid){ return pid<0; }

YARP_SUPPRESS_DLL_INTERFACE_WARNING
    std::string mPortName;
    std::string mLoggerPortName;
    int mProcCNT = 0;

#if !defined(_WIN32)
    void cleanBeforeExec();
    void writeToPipe(int fd, std::string str);
    int readFromPipe(int fd, char* &data, int& buffsize);
#endif

    void cmdcpy(char* &dst, const char* src)
    {
        dst=new char[(strlen(src)/8+2)*16];
        strcpy(dst, src);
    }

    std::string getProcLabel(const yarp::os::Bottle& msg);

    void cmdclean(char **cmd)
    {
        while (*cmd)
        {
            delete [] *cmd++;
        }
    }

  private:
    void* mThriftInterface = nullptr;

  public:
    RunServer();
    ~RunServer();

    static void sigint_handler(int sig);
    int serverCLI(const std::string& serverportName, const std::string& loggerportName);
};

} // namespace yarp::run


#endif // YARP_RUN_RUN_H
