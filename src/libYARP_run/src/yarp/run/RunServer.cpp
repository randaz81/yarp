/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/run/Run.h>
#include <yarp/run/impl/RunCheckpoints.h>
#include <yarp/run/impl/RunProcManager.h>
#include <yarp/run/impl/PlatformStdlib.h>
#include <yarp/run/impl/PlatformUnistd.h>
#include <yarp/run/impl/PlatformSysPrctl.h>

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

#include <yarprunMsgs.h>

////// adapted from libYARP_OS: ResourceFinder.cpp
namespace fs = yarp::conf::filesystem;
constexpr auto sep = yarp::conf::environment::path_separator;
constexpr fs::value_type slash = fs::preferred_separator;

inline std::string lastError2String()
{
    int error=GetLastError();
    char buff[1024];
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, nullptr, error, 0, buff, 1024, nullptr);

    return std::string(buff);
}

void yarp::run::RunServer::sigint_handler(int sig)
{
    YARP_UNUSED(sig);
    mIsTerminated = true;
}

static yarp::os::Bottle parsePaths(const std::string& txt)
{
    yarp::os::Bottle result;
    const char *at = txt.c_str();
    int slash_tweak = 0;
    int len = 0;
    for (char ch : txt) {
        if (ch==sep) {
            result.addString(std::string(at, len-slash_tweak));
            at += len+1;
            len = 0;
            slash_tweak = 0;
            continue;
        }
        slash_tweak = (ch==slash && len>0)?1:0;
        len++;
    }
    if (len>0) {
        result.addString(std::string(at, len-slash_tweak));
    }
    return result;
}

static bool fileExists(const char *fname)
{
    FILE *fp = nullptr;
    fp = fopen(fname, "r");
    if (!fp) {
        return false;
    } else {
        fclose(fp);
        return true;
    }
}

class yarp::run::ServerYarprunMsgs : public YarprunMsgs
{
    yarp::run::RunServer* m_server = nullptr;

    public:
    ServerYarprunMsgs(yarp::run::RunServer* server)
    {
        m_server = server;
    }

    return_ps psRPC() override
    {
        std::vector<yarp::os::SystemInfo::ProcessInfoYarpRun> processes;
        return_ps result;
        yarp::os::Bottle blist = m_server->mProcessVector->PS();
        for (size_t i=0; i<blist.size(); i++)
        {
            blist.get(i).toString();
            yarp::os::SystemInfo::ProcessInfoYarpRun temp;
            yarp::os::Bottle* b = blist.get(i).asList();

            temp.pid = b->get(0).asList()->get(1).asInt32();
            temp.tag = b->get(1).asList()->get(1).asString();
            temp.status = b->get(2).asList()->get(1).asString();
            temp.command = b->get(3).asList()->get(1).asString();
            temp.env = b->get(4).asList()->get(1).asString();
            processes.push_back(temp);
        }
        return result;
    }

    return_exit exitRPC() override
    {
        return_exit result;
        result.result = true;
        m_server->pServerPort=0;
        return result;
    }

    return_which whichRPC(const std::string& keyv) override
    {
        return_which result;

        if (keyv!="")
        {
            yarp::os::Bottle possiblePaths = parsePaths(yarp::conf::environment::get_string("PATH"));
            for (int i=0; i<possiblePaths.size(); ++i)
            {
                std::string guessString=possiblePaths.get(i).asString() + std::string{slash} + keyv;
                const char* guess=guessString.c_str();
                if (fileExists (guess))
                {
                    result.path = "\"" + std::string(guess) + "\"";
                    break;
                }
            }
        }
        return result;
    }

    return_isRunning isRunningRPC (const std::string& keyv) override
    {
        return_isRunning result;
        bool b = m_server->mProcessVector->IsRunning(keyv);
        result.isRunning = b;
        return result;
    }

    return_sysinfo sysinfoRPC() override
    {
        return_sysinfo result;
        yarp::os::SystemInfoSerializer sysinfo;
        result.sysinfo = sysinfo;
        result.result = true;
        return result;
    }

    return_killall killallRPC(const std::int16_t signal) override
    {
        return_killall result;
        bool b = m_server->mProcessVector->SignalAll(signal);
        result.result = b;
        return result;
    }

    return_kill killRPC(const std::string& keyv, const std::int16_t signal) override
    {
        return_kill result;
        bool b = m_server->mProcessVector->Signal(keyv, signal);
        result.result = b;
        return result;
    }

    return_sigtermall sigtermallRPC() override
    {
        return_sigtermall result;
        bool b = m_server->mProcessVector->SignalAll(SIGTERM);
        result.result = b;
        return result;
    }

    return_sigterm sigtermRPC(const std::string& keyv) override
    {
        return_sigterm result;
        bool b = m_server->mProcessVector->Signal(keyv, SIGTERM);
        result.result = b;
        return result;
    }

    return_start startRPC(const yarp::os::Property& command, const std::string& keyv, bool logenable) override
    {
        return_start result;
        yarp::os::Bottle cmdResult;
        int cmdResultCode = 0;

        yarp::os::Bottle tempmsg;

        //This is a request from the client to log a specific process..
logenable=false;//@@@@@@@@@@@@@@@@@@@@@@@@@
m_server->mLoggerPortName = "";//@@@@@@@@@@@@@@@@@@@@@@@@@
        if (logenable)
        {
            cmdResultCode = m_server->executeCmdStdout(tempmsg, cmdResult, m_server->mLoggerPortName);
        }
        // even if the client does not specify a logger port, if the server has been started with the --log option, then it will log
        else if (m_server->mLoggerPortName != "")
        {
            cmdResultCode = m_server->executeCmdStdout(tempmsg, cmdResult, m_server->mLoggerPortName);
        }
        else
        {
            cmdResultCode = m_server->executeCmd(tempmsg, cmdResult);
        }
        //port.reply(cmdResult);
        return result;
    }
};

yarp::run::RunServer::RunServer()
{
    mThriftInterface = new  yarp::run::ServerYarprunMsgs(this);
}

yarp::run::RunServer::~RunServer()
{
    delete mThriftInterface;
    mThriftInterface=nullptr;
}

///////////////////////////
// WINDOWS SERVER
#if defined(_WIN32)
int yarp::run::RunServer::serverCLI(const std::string& serverportName, const std::string& loggerportName)
{
    mPortName = serverportName;
    mLoggerPortName = loggerportName;

    mStdioVector = new YarpRunInfoVector();
    mProcessVector = new YarpRunInfoVector();

    yarp::os::RpcServer port;

    if (!port.open(mPortName.c_str()))
    {
        yError() << "Yarprun failed to open port: " << mPortName.c_str();
        return YARPRUN_ERROR;
    }

    yarp::os::Bottle cmd, reply;
    cmd.addString("set");
    cmd.addString(port.getName());
    cmd.addString("yarprun");
    cmd.addString("true");
    yarp::os::impl::NameClient::getNameClient().send(cmd, reply);

    yInfo() << "Yarprun successfully started on port: " << mPortName.c_str();

    pServerPort=&port;

    yarp::os::impl::signal(SIGINT, sigint_handler);
    yarp::os::impl::signal(SIGTERM, sigint_handler);

    // Enabling CPU load collector on windows
    //yarp::os::impl::SystemInfo::enableCpuLoadCollector();

    ServerYarprunMsgs* mInt = (ServerYarprunMsgs*)(mThriftInterface);

    while (pServerPort)
    {
        // This stops the execution,mIsTerminated is controlled by the signal handler, so that when a SIGINT or SIGTERM is received, the server will stop accepting new commands and will proceed to clean up and exit.
        if (mIsTerminated)
        {
            yarp::os::RpcServer *pClose=pServerPort;
            pServerPort = nullptr;
            pClose->close();
            break;
        }

        bool bb = pServerPort->read(*mInt);
        continue;
    }
    //-----------------------------------------------
        /*


       // without stdio
        if (msg.check("cmd"))
        {
            yarp::os::Bottle cmdResult;

            //This is a request from the client to log a specific process..
            if (msg.check("log"))
            {
                yarp::os::Bottle botLogger=msg.findGroup("log");

                if (botLogger.size()>1)
                {
                    std::string loggerName=botLogger.get(1).asString();
                    executeCmdStdout(msg, cmdResult, loggerName);
                }
                else
                {
                    executeCmdStdout(msg, cmdResult, loggerportName);
                }
            }
            // even if the client does not specify a logger port, if the server has been started with the --log option, then it will log
            else if (loggerportName != "")
            {
                executeCmdStdout(msg, cmdResult, loggerportName);
            }
            else
            {
                executeCmd(msg, cmdResult);
            }
            port.reply(cmdResult);
            continue;
        }


        if (msg.check("killstdio"))
        {
            std::string alias(msg.find("killstdio").asString());
            mStdioVector->Signal(alias, SIGTERM);
            yarp::os::Bottle result;
            result.addString("killstdio OK");
            port.reply(result);
            continue;
        }




    }
*/

    mStdioVector->SignalAll(SIGTERM);
    mProcessVector->SignalAll(SIGTERM);
    delete mProcessVector;
    delete mStdioVector;

    yInfo() << "Yarprun server closed";
    return 0;
}

///////////////////////
#else // LINUX SERVER
///////////////////////

void yarp::run::RunServer::cleanBeforeExec()
{
    // zombie hunter stop

    //yarp::os::impl::signal(SIGPIPE, SIG_IGN);
    //yarp::os::impl::signal(SIGCHLD, SIG_DFL);
    //yarp::os::impl::signal(SIGINT, SIG_DFL);
    //yarp::os::impl::signal(SIGTERM, SIG_DFL);

    if (mProcessVector)
    {
        YarpRunInfoVector *p=mProcessVector;
        mProcessVector = nullptr;
        delete p;
    }
    if (mStdioVector)
    {
        YarpRunInfoVector *p=mStdioVector;
        mStdioVector = nullptr;
        delete p;
    }
    if (mBraveZombieHunter)
    {
        ZombieHunterThread *p=mBraveZombieHunter;
        mBraveZombieHunter = nullptr;
        p->stop();
        delete p;
    }

    //yarp::os::Network::fini();
}

void yarp::run::RunServer::writeToPipe(int fd, std::string str)
{
    int len = str.length() + 1;
    int ret;
    ret = write(fd, &len, sizeof(len));
    if (ret != sizeof(len)) {
        fprintf(stderr, "Warning: could not write string length to pipe.\n");
    }
    ret = write(fd, str.c_str(), len);
    if (ret != len) {
        fprintf(stderr, "Warning: could not write string to pipe.\n");
    }
}

int yarp::run::RunServer::readFromPipe(int fd, char* &data, int& buffsize)
{
    int len=0;
    char* buff=(char*)&len;

    for (int c=4, r=0; c>0; c-=r)
    {
        r=read(fd, buff, c);

        if (r < 1) {
            return -1;
        }

        buff+=r;
    }

    if (len <= 0) {
        return 0;
    }

    if (len>buffsize)
    {
        delete [] data;
        data=new char[buffsize=1024+(len/1024)*1024];
    }

    buff=data;

    for (int c=len, r=0; c>0; c-=r)
    {
        r=read(fd, buff, c);

        if (r < 1) {
            return -1;
        }

        buff+=r;
    }

    return len;
}

static void sigchld_handler(int sig)
{
    YARP_UNUSED(sig);
    if (yarp::run::Run::mBraveZombieHunter)
    {
        yarp::run::Run::mBraveZombieHunter->sigchldHandler();
    }
}

int yarp::run::RunServer::serverCLI()
{
    int pipe_server2manager[2];
    int pipe_manager2server[2];

    if (yarp::run::impl::pipe(pipe_server2manager))
    {
        fprintf(stderr, "Can't open pipe because %s\n", strerror(errno));
        fflush(stderr);

        return YARPRUN_ERROR;
    }

    if (yarp::run::impl::pipe(pipe_manager2server))
    {
        fprintf(stderr, "Can't open pipe because %s\n", strerror(errno));
        fflush(stderr);

        return YARPRUN_ERROR;
    }

    int pid_process_manager=yarp::run::impl::fork();

    if (IS_INVALID(pid_process_manager))
    {
        int error=errno;

        CLOSE(pipe_server2manager[WRITE_TO_PIPE]);
        CLOSE(pipe_server2manager[READ_FROM_PIPE]);
        CLOSE(pipe_manager2server[WRITE_TO_PIPE]);
        CLOSE(pipe_manager2server[READ_FROM_PIPE]);

        fprintf(stderr, "Can't fork process manager because %s\n", strerror(error));
        fflush(stderr);

        return YARPRUN_ERROR;
    }

    if (IS_PARENT_OF(pid_process_manager))
    {
        yarp::os::impl::signal(SIGPIPE, SIG_IGN);

        CLOSE(pipe_server2manager[READ_FROM_PIPE]);
        CLOSE(pipe_manager2server[WRITE_TO_PIPE]);

        yarp::os::RpcServer port;

        if (!port.open(mPortName))
        {
            yError() << "Yarprun failed to open port: " << mPortName.c_str();

            if (mPortName[0] != '/') {
                yError("Invalid port name '%s', it should start with '/'\n", mPortName.c_str());
            }
            return YARPRUN_ERROR;
        }
        yarp::os::Bottle cmd, reply;
        cmd.addString("set");
        cmd.addString(port.getName());
        cmd.addString("yarprun");
        cmd.addString("true");

        yarp::os::impl::NameClient::getNameClient().send(cmd, reply);

        yInfo() << "Yarprun successfully started on port: " << mPortName.c_str();

        pServerPort=&port;

        yarp::os::impl::signal(SIGINT, sigint_handler);
        yarp::os::impl::signal(SIGTERM, sigint_handler);

        int rsp_size=1024;
        char *rsp_str=new char[rsp_size];

        yarp::os::Bottle msg, response;

        while (pServerPort)
        {
            RUNLOG("<<<port.read(msg, true)")
            if (!port.read(msg, true)) {
                break;
            }
            RUNLOG(">>>port.read(msg, true)")

            if (!pServerPort) {
                break;
            }

            if (msg.check("sysinfo"))
            {
                yarp::os::SystemInfoSerializer sysinfo;
                port.reply(sysinfo);
                continue;
            }

            if (msg.check("which"))
            {
                std::string fileName=msg.find("which").asString();
                if (fileName!="")
                {
                    yarp::os::Bottle possiblePaths = parsePaths(yarp::conf::environment::get_string("PATH"));
                    for (size_t i=0; i<possiblePaths.size(); ++i)
                    {
                        std::string guessString=possiblePaths.get(i).asString() + slash + fileName;
                        const char* guess=guessString.c_str();
                        if (fileExists (guess))
                        {
                            fileName = guess;
                            break;
                        }
                    }
                }
                yarp::os::Value fileNameWriter(fileName);
                port.reply(fileNameWriter);
                continue;
            }

            if (msg.check("exit"))
            {
                pServerPort = nullptr;
                yarp::os::Bottle result;
                result.addString("exit OK");
                port.reply(result);
                port.close();
                break;
            }

            RUNLOG("<<<writeToPipe")
            writeToPipe(pipe_server2manager[WRITE_TO_PIPE], msg.toString());
            RUNLOG(">>>writeToPipe")

            RUNLOG("<<<readFromPipe")
            int nread=readFromPipe(pipe_manager2server[READ_FROM_PIPE], rsp_str, rsp_size);
            RUNLOG(">>>readFromPipe")

            if (nread<0)
            {
                fprintf(stderr, "ERROR: broken pipe between server and manager\n");
                fflush(stderr);
                break;
            }

            if (nread)
            {
                response.fromString(rsp_str);
                port.reply(response);
            }
        }

        //yarp::os::Network::fini();

        CLOSE(pipe_server2manager[WRITE_TO_PIPE]);
        CLOSE(pipe_manager2server[READ_FROM_PIPE]);

        delete [] rsp_str;

        return 0;
    }

    if (IS_NEW_PROCESS(pid_process_manager))
    {
        yarp::os::impl::signal(SIGPIPE, SIG_IGN);

        CLOSE(pipe_server2manager[WRITE_TO_PIPE]);
        CLOSE(pipe_manager2server[READ_FROM_PIPE]);

        //yarp::os::Network::init();

        mProcessVector=new YarpRunInfoVector;
        mStdioVector=new YarpRunInfoVector;

        mBraveZombieHunter=new ZombieHunterThread;
        mBraveZombieHunter->start();

        yarp::os::impl::signal(SIGCHLD, sigchld_handler);
        //yarp::os::impl::signal(SIGINT, SIG_IGN);
        //yarp::os::impl::signal(SIGTERM, SIG_IGN);

        int msg_size=1024;
        char *msg_str=new char[msg_size];

        yarp::os::Bottle msg;

        //while(readFromPipe(pipe_server2manager[READ_FROM_PIPE], msg_str, msg_size)>0)
        while (true)
        {
            RUNLOG("<<<readFromPipe")
            if (readFromPipe(pipe_server2manager[READ_FROM_PIPE], msg_str, msg_size) <= 0) {
                break;
            }
            RUNLOG(">>>readFromPipe")

            //printf("<<< %s >>>\n", msg_str);
            //fflush(stdout);

            msg.fromString(msg_str);

            // command with stdio management
            if (msg.check("stdio"))
            {
                std::string strOnPort=msg.find("on").asString();
                std::string strStdioPort=msg.find("stdio").asString();

                if (strOnPort==mPortName)
                {
                    std::string strUUID=mPortName+"/"+int2String(getpid())+"/"+msg.find("as").asString()+"-"+int2String(mProcCNT++);
                    yarp::os::Bottle botUUID;
                    botUUID.addString("stdiouuid");
                    botUUID.addString(strUUID.c_str());
                    msg.addList()=botUUID;

                    if (mLogged || msg.check("log"))
                    {
                        std::string strAlias=msg.find("as").asString();
                        std::string portName="/log";
                        portName+=mPortName+"/";
                        std::string command = msg.findGroup("cmd").get(1).asString();
                        command = command.substr(0, command.find(' '));
                        command = command.substr(command.find_last_of("\\/") + 1);
                        portName+=command;

                        yarp::os::Bottle botFwd;
                        botFwd.addString("forward");
                        botFwd.addString(portName.c_str());
                        if (msg.check("log"))
                        {
                            yarp::os::Bottle botLogger=msg.findGroup("log");

                            if (botLogger.size()>1)
                            {
                                botFwd.addString(botLogger.get(1).asString());
                            }
                            else
                            {
                                botFwd.addString(mLoggerPort);
                            }
                        }
                        else
                        {
                            botFwd.addString(mLoggerPort);
                        }
                        msg.addList()=botFwd;

                        yarp::os::ContactStyle style;
                        style.persistent=true;
                        yarp::os::Network::connect(portName, mLoggerPort, style);
                    }

                    yarp::os::Bottle cmdResult;
                    if (executeCmdAndStdio(msg, cmdResult)>0)
                    {
                        if (strStdioPort==mPortName)
                        {
                            yarp::os::Bottle stdioResult;
                            userStdio(msg, stdioResult);
                            cmdResult.append(stdioResult);
                        }
                        else
                        {
                            cmdResult.append(sendMsg(msg, strStdioPort));
                        }
                    }

                    RUNLOG("<<<writeToPipe")
                    writeToPipe(pipe_manager2server[WRITE_TO_PIPE], cmdResult.toString());
                    RUNLOG(">>>writeToPipe")
                }
                else
                {
                    yarp::os::Bottle stdioResult;
                    userStdio(msg, stdioResult);
                    RUNLOG("<<<writeToPipe")
                    writeToPipe(pipe_manager2server[WRITE_TO_PIPE], stdioResult.toString());
                    RUNLOG(">>>writeToPipe")
                }

                continue;
            }

            // without stdio
            if (msg.check("cmd"))
            {
                yarp::os::Bottle cmdResult;

                if (msg.check("log"))
                {
                    yarp::os::Bottle botLogger=msg.findGroup("log");

                    if (botLogger.size()>1)
                    {
                        std::string loggerName=botLogger.get(1).asString();
                        executeCmdStdout(msg, cmdResult, loggerName);
                    }
                    else
                    {
                       executeCmdStdout(msg, cmdResult, mLoggerPort);
                    }
                }
                else if (mLogged)
                {
                    executeCmdStdout(msg, cmdResult, mLoggerPort);
                }
                else
                {
                    executeCmd(msg, cmdResult);
                }

                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], cmdResult.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }

            if (msg.check("kill"))
            {
                std::string alias(msg.findGroup("kill").get(1).asString());
                int sig=msg.findGroup("kill").get(2).asInt32();
                yarp::os::Bottle result;
                result.addString(mProcessVector->Signal(alias, sig)?"kill OK":"kill FAILED");
                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], result.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }

            if (msg.check("sigterm"))
            {
                std::string alias(msg.find("sigterm").asString());
                yarp::os::Bottle result;
                result.addString(mProcessVector->Signal(alias, SIGTERM)?"sigterm OK":"sigterm FAILED");
                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], result.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }

            if (msg.check("sigtermall"))
            {
                mProcessVector->Killall(SIGTERM);
                yarp::os::Bottle result;
                result.addString("sigtermall OK");

                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], result.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }

            if (msg.check("ps"))
            {
                yarp::os::Bottle result;
                result.append(mProcessVector->PS());
                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], result.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }

            if (msg.check("isrunning"))
            {
                std::string alias(msg.find("isrunning").asString());
                yarp::os::Bottle result;
                result.addString(mProcessVector->IsRunning(alias)?"running":"not running");
                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], result.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }

            if (msg.check("killstdio"))
            {
                std::string alias(msg.find("killstdio").asString());
                mStdioVector->Signal(alias, SIGTERM);
                yarp::os::Bottle result;
                result.addString("killstdio OK");
                RUNLOG("<<<writeToPipe")
                writeToPipe(pipe_manager2server[WRITE_TO_PIPE], result.toString());
                RUNLOG(">>>writeToPipe")
                continue;
            }
        }

        mStdioVector->Killall(SIGTERM);

        mProcessVector->Killall(SIGTERM);

        if (mBraveZombieHunter)
        {
            mBraveZombieHunter->stop();
            delete mBraveZombieHunter;
            mBraveZombieHunter = nullptr;
        }

        delete mProcessVector;

        delete mStdioVector;

        //yarp::os::Network::fini();

        CLOSE(pipe_server2manager[READ_FROM_PIPE]);
        CLOSE(pipe_manager2server[WRITE_TO_PIPE]);

        delete [] msg_str;
    }

    return 0;
} // LINUX SERVER
#endif

///////////////////////////
// WINDOWS SERVER
#if defined(_WIN32)

///////////////////////
#else // LINUX SERVER
///////////////////////

static void sigchld_handler(int sig)
{
    YARP_UNUSED(sig);
    if (yarp::run::Run::mBraveZombieHunter)
    {
        yarp::run::Run::mBraveZombieHunter->sigchldHandler();
    }
}

#endif


/////////////////////////
// OS DEPENDENT FUNCTIONS
/////////////////////////

// WINDOWS

#if defined(_WIN32)

// CMD SERVER

std::string yarp::run::RunServer::getProcLabel(const yarp::os::Bottle& msg)
{
    auto ss = yarp::conf::string::split(msg.find("env").asString(), ';');
    for (const auto& s_iter : ss)
    {
        auto sss = yarp::conf::string::split(s_iter, '=');
        if (sss.size() == 2 && sss[0] == "YARP_LOG_PROCESS_LABEL")
        {
            return sss[1];
        }
    }
    return "";
}

int yarp::run::RunServer::executeCmdStdout(const  yarp::os::Bottle& msg, yarp::os::Bottle& result, const std::string& loggerName)
{
    std::string proc_label = getProcLabel(msg);

    std::string strAlias=msg.find("as").asString();
    std::string portName="/log";
    portName+=mPortName+"/";
    std::string command = msg.findGroup("cmd").get(1).asString();
    command = command.substr(0, command.find(' '));
    command = command.substr(command.find_last_of("\\/") + 1);
    portName+=command;
    if (proc_label != "") { portName += "[" + proc_label + "]"; }

    // PIPES
    SECURITY_ATTRIBUTES pipe_sec_attr;
    pipe_sec_attr.nLength=sizeof(SECURITY_ATTRIBUTES);
    pipe_sec_attr.bInheritHandle=TRUE;
    pipe_sec_attr.lpSecurityDescriptor = nullptr;
    HANDLE read_from_pipe_cmd_to_stdout, write_to_pipe_cmd_to_stdout;
    CreatePipe(&read_from_pipe_cmd_to_stdout, &write_to_pipe_cmd_to_stdout, &pipe_sec_attr, 0);

    // RUN STDOUT
    PROCESS_INFORMATION stdout_process_info;
    ZeroMemory(&stdout_process_info, sizeof(PROCESS_INFORMATION));
    STARTUPINFO stdout_startup_info;
    ZeroMemory(&stdout_startup_info, sizeof(STARTUPINFO));

    stdout_startup_info.cb=sizeof(STARTUPINFO);
    stdout_startup_info.hStdError=GetStdHandle(STD_ERROR_HANDLE);
    stdout_startup_info.hStdOutput=GetStdHandle(STD_OUTPUT_HANDLE);
    stdout_startup_info.hStdInput=read_from_pipe_cmd_to_stdout;
    stdout_startup_info.dwFlags|=STARTF_USESTDHANDLES;

    BOOL bSuccess=CreateProcess(nullptr,  // command name
                                (char*)(std::string("yarprun --log ")+loggerName+std::string(" --write ")+portName).c_str(), // command line
                                nullptr,  // process security attributes
                                nullptr,  // primary thread security attributes
                                TRUE,          // handles are inherited
                                CREATE_NEW_PROCESS_GROUP, // creation flags
                                nullptr,  // use parent's environment
                                nullptr,  // use parent's current directory
                                &stdout_startup_info,   // STARTUPINFO pointer
                                &stdout_process_info);  // receives PROCESS_INFORMATION

    if (!bSuccess)
    {
        std::string strError=std::string("ABORTED: server=")+mPortName
                                      +std::string(" alias=")+strAlias
                                      +std::string(" cmd=stdout\n")
                                      +std::string("Can't execute stdout because ")+lastError2String()
                                      +std::string("\n");

        result.addInt32(YARPRUN_ERROR);
        result.addString(strError.c_str());
        fprintf(stderr, "%s", strError.c_str());
        fflush(stderr);

        CloseHandle(write_to_pipe_cmd_to_stdout);
        CloseHandle(read_from_pipe_cmd_to_stdout);

        return YARPRUN_ERROR;
    }

    // RUN COMMAND

    PROCESS_INFORMATION cmd_process_info;
    ZeroMemory(&cmd_process_info, sizeof(PROCESS_INFORMATION));
    STARTUPINFO cmd_startup_info;
    ZeroMemory(&cmd_startup_info, sizeof(STARTUPINFO));

    cmd_startup_info.cb=sizeof(STARTUPINFO);
    cmd_startup_info.hStdError=write_to_pipe_cmd_to_stdout;
    cmd_startup_info.hStdOutput=write_to_pipe_cmd_to_stdout;
    cmd_startup_info.hStdInput=GetStdHandle(STD_INPUT_HANDLE);
    cmd_startup_info.dwFlags|=STARTF_USESTDHANDLES;

    yarp::os::Bottle botCmd=msg.findGroup("cmd").tail();

    std::string strCmd;
    for (int s=0; s<botCmd.size(); ++s)
    {
        strCmd+=botCmd.get(s).toString()+std::string(" ");
    }

    /*
     * setting environment variable for child process
     */
    TCHAR chNewEnv[32767];

    // Get a pointer to the env block.
    LPTCH chOldEnv = GetEnvironmentStrings();

    // copying parent env variables
    LPTSTR lpOld = (LPTSTR) chOldEnv;
    LPTSTR lpNew = (LPTSTR) chNewEnv;
    while (*lpOld)
    {
        lstrcpy(lpNew, lpOld);
        lpOld += lstrlen(lpOld) + 1;
        lpNew += lstrlen(lpNew) + 1;
    }

    // Set the YARP_IS_YARPRUN environment variable to 1, so that the child
    // process will now that is running inside yarprun.
    lstrcpy(lpNew, (LPTCH) "YARP_IS_YARPRUN=1");
    lpNew += lstrlen(lpNew) + 1;

    // Set the YARPRUN_IS_FORWARDING_LOG environment variable to 1, so that
    // the child process will now that yarprun is not logging the output.
    lstrcpy(lpNew, (LPTCH) "YARPRUN_IS_FORWARDING_LOG=1");
    lpNew += lstrlen(lpNew) + 1;

    // adding new env variables
    std::string cstrEnvName;
    if (msg.check("env"))
    {
        auto ss = yarp::conf::string::split(msg.find("env").asString(), ';');
        for (const auto& s : ss) {
            lstrcpy(lpNew, (LPTCH)s.c_str());
            lpNew += lstrlen(lpNew) + 1;
        }
    }

    // closing env block
    *lpNew = (TCHAR)0;

    bool bWorkdir=msg.check("workdir");
    std::string strWorkdir=bWorkdir?msg.find("workdir").asString()+"\\":"";

    bSuccess=CreateProcess(nullptr,  // command name
                           (char*)(strWorkdir+strCmd).c_str(), // command line
                           nullptr,  // process security attributes
                           nullptr,  // primary thread security attributes
                           TRUE,          // handles are inherited
                           CREATE_NEW_PROCESS_GROUP, // creation flags
                           (LPVOID) chNewEnv,        // use new environment list
                           bWorkdir?strWorkdir.c_str():nullptr, // working directory
                           &cmd_startup_info,   // STARTUPINFO pointer
                           &cmd_process_info);  // receives PROCESS_INFORMATION

    if (!bSuccess && bWorkdir)
    {
        bSuccess=CreateProcess(nullptr,  // command name
                               (char*)(strCmd.c_str()), // command line
                               nullptr,  // process security attributes
                               nullptr,  // primary thread security attributes
                               TRUE,          // handles are inherited
                               CREATE_NEW_PROCESS_GROUP, // creation flags
                               (LPVOID) chNewEnv,        // use new environment list
                               strWorkdir.c_str(), // working directory
                               &cmd_startup_info,   // STARTUPINFO pointer
                               &cmd_process_info);  // receives PROCESS_INFORMATION
    }

    // deleting old environment variable
    FreeEnvironmentStrings(chOldEnv);

    if (!bSuccess)
    {
        result.addInt32(YARPRUN_ERROR);

        DWORD nBytes;
        std::string line1=std::string("ABORTED: server=")+mPortName
                                   +std::string(" alias=")+strAlias
                                   +std::string(" cmd=")+strCmd
                                   +std::string("pid=")+int2String(cmd_process_info.dwProcessId)
                                   +std::string("\n");

        WriteFile(write_to_pipe_cmd_to_stdout, line1.c_str(), line1.length(), &nBytes, 0);

        std::string line2=std::string("Can't execute command because ")+lastError2String()+std::string("\n");
        WriteFile(write_to_pipe_cmd_to_stdout, line1.c_str(), line2.length(), &nBytes, 0);
        FlushFileBuffers(write_to_pipe_cmd_to_stdout);

        std::string out=line1+line2;
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());
        fflush(stderr);

        CloseHandle(write_to_pipe_cmd_to_stdout);
        CloseHandle(read_from_pipe_cmd_to_stdout);

        TerminateProcess(stdout_process_info.hProcess, YARPRUN_ERROR);

        CloseHandle(stdout_process_info.hProcess);

        return YARPRUN_ERROR;
    }

    FlushFileBuffers(write_to_pipe_cmd_to_stdout);

    // EVERYTHING IS ALL RIGHT
    YarpRunCmdWithStdioInfo* pInf = new YarpRunCmdWithStdioInfo(strAlias,
                                                   mPortName,
                                                   portName,
                                                   cmd_process_info.dwProcessId,
                                                   stdout_process_info.dwProcessId,
                                                   read_from_pipe_cmd_to_stdout,
                                                   write_to_pipe_cmd_to_stdout,
                                                   cmd_process_info.hProcess,
                                                   false);




    pInf->setCmd(strCmd);
    if (msg.check("env"))
    {
        pInf->setEnv(msg.find("env").asString());
    }
    mProcessVector->Add(pInf);

    result.addInt32(cmd_process_info.dwProcessId);
    std::string out=std::string("STARTED: server=")+mPortName
                             +std::string(" alias=")+strAlias
                             +std::string(" cmd=")+strCmd
                             +std::string(" pid=")+int2String(cmd_process_info.dwProcessId)
                             +std::string("\n");

    result.addString(out.c_str());
    result.addString(portName.c_str());
    fprintf(stderr, "%s", out.c_str());

    return cmd_process_info.dwProcessId;
}


int yarp::run::RunServer::executeCmd(const  yarp::os::Bottle& msg, yarp::os::Bottle& result)
{
    std::string strAlias=msg.find("as").asString().c_str();

    // RUN COMMAND
    PROCESS_INFORMATION cmd_process_info;
    ZeroMemory(&cmd_process_info, sizeof(PROCESS_INFORMATION));
    STARTUPINFO cmd_startup_info;
    ZeroMemory(&cmd_startup_info, sizeof(STARTUPINFO));

    cmd_startup_info.cb=sizeof(STARTUPINFO);

    yarp::os::Bottle botCmd=msg.findGroup("cmd").tail();

    std::string strCmd;
    for (int s=0; s<botCmd.size(); ++s)
    {
        strCmd+=botCmd.get(s).toString()+std::string(" ");
    }

    /*
     * setting environment variable for child process
     */
    TCHAR chNewEnv[32767];

    // Get a pointer to the env block.
    LPTCH chOldEnv = GetEnvironmentStrings();

    // copying parent env variables
    LPTSTR lpOld = (LPTSTR) chOldEnv;
    LPTSTR lpNew = (LPTSTR) chNewEnv;
    while (*lpOld)
    {
        lstrcpy(lpNew, lpOld);
        lpOld += lstrlen(lpOld) + 1;
        lpNew += lstrlen(lpNew) + 1;
    }

    // Set the YARP_IS_YARPRUN environment variable to 1, so that the child
    // process will know that is running inside yarprun.
    lstrcpy(lpNew, (LPTCH) "YARP_IS_YARPRUN=1");
    lpNew += lstrlen(lpNew) + 1;

    // Set the YARPRUN_IS_FORWARDING_LOG environment variable to 0, so that
    // the child process will know that yarprun is not logging the output.
    lstrcpy(lpNew, (LPTCH) "YARPRUN_IS_FORWARDING_LOG=0");
    lpNew += lstrlen(lpNew) + 1;

    // adding new env variables
    std::string cstrEnvName;
    if (msg.check("env"))
    {
        auto ss = yarp::conf::string::split(msg.find("env").asString(), ';');
        for (const auto& s : ss) {
            lstrcpy(lpNew, (LPTCH)s.c_str());
            lpNew += lstrlen(lpNew) + 1;
        }
    }

    // closing env block
    *lpNew = (TCHAR)0;

    bool bWorkdir=msg.check("workdir");
    std::string strWorkdir=bWorkdir?msg.find("workdir").asString()+"\\":"";

    BOOL bSuccess=CreateProcess(nullptr,  // command name
                                (char*)(strWorkdir+strCmd).c_str(), // command line
                                nullptr,  // process security attributes
                                nullptr,  // primary thread security attributes
                                TRUE,          // handles are inherited
                                CREATE_NEW_PROCESS_GROUP, // creation flags
                                (LPVOID) chNewEnv, // use new environment
                                bWorkdir ? strWorkdir.c_str() : nullptr, // working directory
                                &cmd_startup_info,   // STARTUPINFO pointer
                                &cmd_process_info);  // receives PROCESS_INFORMATION

    if (!bSuccess && bWorkdir)
    {
        bSuccess=CreateProcess(nullptr,  // command name
                               (char*)(strCmd.c_str()), // command line
                               nullptr,  // process security attributes
                               nullptr,  // primary thread security attributes
                               TRUE,          // handles are inherited
                               CREATE_NEW_PROCESS_GROUP, // creation flags
                               (LPVOID) chNewEnv, // use new environment
                               strWorkdir.c_str(), // working directory
                               &cmd_startup_info,   // STARTUPINFO pointer
                               &cmd_process_info);  // receives PROCESS_INFORMATION
    }

    // deleting old environment variable
    FreeEnvironmentStrings(chOldEnv);

    if (!bSuccess)
    {
        result.addInt32(YARPRUN_ERROR);

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=")+strCmd
                                 +std::string(" pid=")+int2String(cmd_process_info.dwProcessId)
                                 +std::string("\nCan't execute command because ")+lastError2String()
                                 +std::string("\n");

        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());
        fflush(stderr);

        return YARPRUN_ERROR;
    }

    // EVERYTHING IS ALL RIGHT
    YarpRunProcInfo* pInf = new YarpRunProcInfo(strAlias,
                                           mPortName,
                                           cmd_process_info.dwProcessId,
                                           cmd_process_info.hProcess,
                                           false);
    pInf->setCmd(strCmd);
    if (msg.check("env"))
        pInf->setEnv(msg.find("env").asString());

    mProcessVector->Add(pInf);

    result.addInt32(cmd_process_info.dwProcessId);
    std::string out=std::string("STARTED: server=")+mPortName
                             +std::string(" alias=")+strAlias
                             +std::string(" cmd=")+strCmd
                             +std::string(" pid=")+int2String(cmd_process_info.dwProcessId)
                             +std::string("\n");

    fprintf(stderr, "%s", out.c_str());

    return cmd_process_info.dwProcessId;
}


////////////////
#else // LINUX
////////////////
/**
 * Split a line into separate words.
 */
void splitLine(char *pLine, char **pArgs)
{
     char *pTmp = strchr(pLine, ' ');

    if (pTmp) {
        *pTmp = '\0';
        pTmp++;
        while ((*pTmp) && (*pTmp == ' ')) {
            pTmp++;
        }
        if (*pTmp == '\0') {
            pTmp = nullptr;
        }
    }
    *pArgs = pTmp;
}

/**
 * Breaks up a line into multiple arguments.
 */
void parseArguments(char *io_pLine, int *o_pArgc, char **o_pArgv)
{
    char *pNext = io_pLine;
    size_t i;
    int j;
    int quoted = 0;
    size_t len = strlen(io_pLine);

    // Protect spaces inside quotes, but lose the quotes
    for(i = 0; i < len; i++) {
        if ((!quoted) && ('"' == io_pLine[i])) {
            quoted = 1;
            io_pLine[i] = ' ';
        } else if ((quoted) && ('"' == io_pLine[i])) {
            quoted = 0;
            io_pLine[i] = ' ';
        } else if ((quoted) && (' ' == io_pLine[i])) {
            io_pLine[i] = '\1';
        }
    }

    // init
    memset(o_pArgv, 0x00, sizeof(char*) * C_MAXARGS);
    *o_pArgc = 1;
    o_pArgv[0] = io_pLine;

    while ((nullptr != pNext) && (*o_pArgc < C_MAXARGS)) {
        splitLine(pNext, &(o_pArgv[*o_pArgc]));
        pNext = o_pArgv[*o_pArgc];

        if (nullptr != o_pArgv[*o_pArgc]) {
            *o_pArgc += 1;
        }
    }

    for(j = 0; j < *o_pArgc; j++) {
        len = strlen(o_pArgv[j]);
        for(i = 0; i < len; i++) {
            if ('\1' == o_pArgv[j][i]) {
                o_pArgv[j][i] = ' ';
            }
        }
    }
}

void yarp::run::Run::CleanZombie(int pid)
{
    bool bFound=mProcessVector && mProcessVector->CleanZombie(pid);

    if (!bFound) {
        if (mStdioVector) {
            mStdioVector->CleanZombie(pid);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

int yarp::run::RunServer::executeCmdAndStdio(const yarp::os::Bottle& msg, yarp::os::Bottle& result)
{
    std::string strAlias=msg.find("as").asString();
    std::string strCmd=msg.find("cmd").asString();
    std::string strStdio=msg.find("stdio").asString();
    std::string strStdioUUID=msg.find("stdiouuid").asString();

    int  pipe_stdin_to_cmd[2];
    int ret_stdin_to_cmd=yarp::run::impl::pipe(pipe_stdin_to_cmd);

    int  pipe_cmd_to_stdout[2];
    int  ret_cmd_to_stdout=yarp::run::impl::pipe(pipe_cmd_to_stdout);

    int  pipe_child_to_parent[2];
    int  ret_child_to_parent=yarp::run::impl::pipe(pipe_child_to_parent);

    if (ret_child_to_parent!=0 || ret_cmd_to_stdout!=0 || ret_stdin_to_cmd!=0)
    {
        int error=errno;

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdout\n")
                                 +std::string("Can't create pipes ")+strerror(error)
                                 +std::string("\n");

        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    int pid_stdout=yarp::run::impl::fork();

    if (IS_INVALID(pid_stdout))
    {
        int error=errno;

        CLOSE(pipe_stdin_to_cmd[WRITE_TO_PIPE]);
        CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
        CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
        CLOSE(pipe_cmd_to_stdout[READ_FROM_PIPE]);
        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);
        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdout\n")
                                 +std::string("Can't fork stdout process because ")+strerror(error)
                                 +std::string("\n");

        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    if (IS_NEW_PROCESS(pid_stdout)) // STDOUT IMPLEMENTED HERE
    {
        REDIRECT_TO(STDIN_FILENO, pipe_cmd_to_stdout[READ_FROM_PIPE]);

        CLOSE(pipe_stdin_to_cmd[WRITE_TO_PIPE]);
        CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
        CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

        //Why removing vectors and stop threads?
        //exec* never returns and memory is claimed by the system
        //furthermore after fork() only the thread which called fork() is forked!
        //            cleanBeforeExec();

        //yarp::os::impl::signal(SIGPIPE, SIG_DFL);

        int ret = yarp::run::impl::execlp("yarprun", "yarprun", "--write", strStdioUUID.c_str(), static_cast<char*>(nullptr));

        CLOSE(pipe_cmd_to_stdout[READ_FROM_PIPE]);

        if (ret==YARPRUN_ERROR)
        {
            int error=errno;

            std::string out=std::string("ABORTED: server=")+mPortName
                                     +std::string(" alias=")+strAlias
                                     +std::string(" cmd=stdout\n")
                                     +std::string("Can't execute stdout because ")+strerror(error)
                                     +std::string("\n");

            FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
            fprintf(out_to_parent, "%s", out.c_str());
            fflush(out_to_parent);
            fclose(out_to_parent);

            fprintf(stderr, "%s", out.c_str());
        }

        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

        std::exit(ret);
    }

    if (IS_PARENT_OF(pid_stdout))
    {
        CLOSE(pipe_cmd_to_stdout[READ_FROM_PIPE]);

        fprintf(stderr, "STARTED: server=%s alias=%s cmd=stdout pid=%d\n", mPortName.c_str(), strAlias.c_str(), pid_stdout);

        int pid_stdin=yarp::run::impl::fork();

        if (IS_INVALID(pid_stdin))
        {
            int error=errno;

            CLOSE(pipe_stdin_to_cmd[WRITE_TO_PIPE]);
            CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
            CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
            CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);
            CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

            std::string out=std::string("ABORTED: server=")+mPortName
                                     +std::string(" alias=")+strAlias
                                     +std::string(" cmd=stdin\n")
                                     +std::string("Can't fork stdin process because ")+strerror(error)
                                     +std::string("\n");

            result.addInt32(YARPRUN_ERROR);
            result.addString(out.c_str());
            fprintf(stderr, "%s", out.c_str());

            SIGNAL(pid_stdout, SIGTERM);
            fprintf(stderr, "TERMINATING stdout (%d)\n", pid_stdout);

            return YARPRUN_ERROR;
        }

        if (IS_NEW_PROCESS(pid_stdin)) // STDIN IMPLEMENTED HERE
        {
            yarp::conf::environment::set_string("YARP_QUIET", "1");
            REDIRECT_TO(STDOUT_FILENO, pipe_stdin_to_cmd[WRITE_TO_PIPE]);
            REDIRECT_TO(STDERR_FILENO, pipe_stdin_to_cmd[WRITE_TO_PIPE]);

            CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
            CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
            CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

            //Why removing vectors and stop threads?
            //exec* never returns and memory is claimed by the system
            //furthermore after fork() only the thread which called fork() is forked!
            //            cleanBeforeExec();

            //yarp::os::impl::signal(SIGPIPE, SIG_DFL);

            int ret = yarp::run::impl::execlp("yarprun", "yarprun", "--read", strStdioUUID.c_str(), static_cast<char*>(nullptr));

            CLOSE(pipe_stdin_to_cmd[WRITE_TO_PIPE]);

            if (ret==YARPRUN_ERROR)
            {
                int error=errno;

                std::string out=std::string("ABORTED: server=")+mPortName
                                         +std::string(" alias=")+strAlias
                                         +std::string(" cmd=stdin\n")
                                         +std::string("Can't execute stdin because ")+strerror(error)
                                         +std::string("\n");


                FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
                fprintf(out_to_parent, "%s", out.c_str());
                fflush(out_to_parent);
                fclose(out_to_parent);
                fprintf(stderr, "%s", out.c_str());
            }

            CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

            std::exit(ret);
        }

        if (IS_PARENT_OF(pid_stdin))
        {
            // connect yarp read and write
            CLOSE(pipe_stdin_to_cmd[WRITE_TO_PIPE]);

            fprintf(stderr, "STARTED: server=%s alias=%s cmd=stdin pid=%d\n", mPortName.c_str(), strAlias.c_str(), pid_stdin);

            int pid_cmd=yarp::run::impl::fork();

            if (IS_INVALID(pid_cmd))
            {
                int error=errno;

                CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
                CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);
                CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

                std::string out=std::string("ABORTED: server=")+mPortName
                                         +std::string(" alias=")+strAlias
                                         +std::string(" cmd=")+strCmd
                                         +std::string("\nCan't fork command process because ")+strerror(error)
                                         +std::string("\n");

                result.addInt32(YARPRUN_ERROR);
                result.addString(out.c_str());
                fprintf(stderr, "%s", out.c_str());

                FILE* to_yarp_stdout=fdopen(pipe_cmd_to_stdout[WRITE_TO_PIPE], "w");
                fprintf(to_yarp_stdout, "%s", out.c_str());
                fflush(to_yarp_stdout);
                fclose(to_yarp_stdout);

                SIGNAL(pid_stdout, SIGTERM);
                fprintf(stderr, "TERMINATING stdout (%d)\n", pid_stdout);
                SIGNAL(pid_stdin, SIGTERM);
                fprintf(stderr, "TERMINATING stdin (%d)\n", pid_stdin);

                CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);

                return YARPRUN_ERROR;
            }

            if (IS_NEW_PROCESS(pid_cmd)) // RUN COMMAND HERE
            {
                CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

                char *cmd_str=new char[strCmd.length()+1];
                strcpy(cmd_str, strCmd.c_str());
                /*
                int nargs=CountArgs(cmd_str);
                char **arg_str=new char*[nargs+1];
                ParseCmd(cmd_str, arg_str);
                arg_str[nargs]=0;
                */
                int nargs = 0;
                char **arg_str = new char*[C_MAXARGS + 1];
                parseArguments(cmd_str, &nargs, arg_str);
                arg_str[nargs]=nullptr;

                setvbuf(stdout, nullptr, _IONBF, 0);

                REDIRECT_TO(STDIN_FILENO, pipe_stdin_to_cmd[READ_FROM_PIPE]);
                REDIRECT_TO(STDOUT_FILENO, pipe_cmd_to_stdout[WRITE_TO_PIPE]);
                REDIRECT_TO(STDERR_FILENO, pipe_cmd_to_stdout[WRITE_TO_PIPE]);

                // Set the YARP_IS_YARPRUN environment variable to 1, so that the child
                // process will now that is running inside yarprun.
                yarp::conf::environment::set_string("YARP_IS_YARPRUN", "1");

                // Set the YARPRUN_IS_FORWARDING_LOG environment variable to 1, so that
                // the child process will now that yarprun is not logging the output.
                yarp::conf::environment::set_string("YARPRUN_IS_FORWARDING_LOG", "1");

                if (msg.check("env"))
                {
                    auto ss = yarp::conf::string::split(msg.find("env").asString(), ';');
                    for (const auto& s : ss) {
                        char* szenv = new char[s.size()+1];
                        strcpy(szenv, s.c_str());
                        yarp::run::impl::putenv(szenv); // putenv doesn't make copy of the string
                    }
                    //delete [] szenv;
                }

                if (msg.check("workdir"))
                {
                    int ret = yarp::os::impl::chdir(msg.find("workdir").asString().c_str());

                    if (ret!=0)
                    {
                        int error=errno;

                        std::string out=std::string("ABORTED: server=")+mPortName
                                                 +std::string(" alias=")+strAlias
                                                 +std::string(" cmd=")+strCmd
                                                 +std::string("\nCan't execute command, cannot set working directory ")+strerror(error)
                                                 +std::string("\n");

                        FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
                        fprintf(out_to_parent, "%s", out.c_str());
                        fflush(out_to_parent);
                        fclose(out_to_parent);
                        fprintf(stderr, "%s", out.c_str());

                        std::exit(ret);
                    }
                }

                int ret=YARPRUN_ERROR;

                char currWorkDirBuff[1024];
                char *currWorkDir=yarp::os::impl::getcwd(currWorkDirBuff, 1024);

                if (currWorkDir)
                {
                    char **cwd_arg_str=new char*[nargs+1];
                    for (int i = 1; i < nargs; ++i) {
                        cwd_arg_str[i] = arg_str[i];
                    }
                    cwd_arg_str[nargs]=nullptr;
                    cwd_arg_str[0]=new char[strlen(currWorkDir)+strlen(arg_str[0])+16];

                    strcpy(cwd_arg_str[0], currWorkDir);
                    strcat(cwd_arg_str[0], "/");
                    strcat(cwd_arg_str[0], arg_str[0]);

                    //Why removing vectors and stop threads?
                    //exec* never returns and memory is claimed by the system
                    //furthermore after fork() only the thread which called fork() is forked!
                    //            cleanBeforeExec();

                    ret = yarp::run::impl::execvp(cwd_arg_str[0], cwd_arg_str);

                    delete [] cwd_arg_str[0];
                    delete [] cwd_arg_str;
                }

                if (ret==YARPRUN_ERROR)
                {
                    //Why removing vectors and stop threads?
                    //exec* never returns and memory is claimed by the system
                    //furthermore after fork() only the thread which called fork() is forked!
                    //            cleanBeforeExec();

                    ret = yarp::run::impl::execvp(arg_str[0], arg_str);
                }

                fflush(stdout);

                CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
                CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);

                if (ret==YARPRUN_ERROR)
                {
                    int error=errno;

                    std::string out=std::string("ABORTED: server=")+mPortName
                                             +std::string(" alias=")+strAlias
                                             +std::string(" cmd=")+strCmd
                                             +std::string("\nCan't execute command because ")+strerror(error)
                                             +std::string("\n");

                    FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
                    fprintf(out_to_parent, "%s", out.c_str());
                    fflush(out_to_parent);
                    fclose(out_to_parent);
                    fprintf(stderr, "%s", out.c_str());
                }

                delete [] cmd_str;
                delete [] arg_str;

                CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

                std::exit(ret);
            }


            if (IS_PARENT_OF(pid_cmd))
            {
                CLOSE(pipe_stdin_to_cmd[READ_FROM_PIPE]);
                CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
                CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

                auto* pInf = new YarpRunCmdWithStdioInfo(
                        strAlias,
                        mPortName,
                        strStdio,
                        pid_cmd,
                        strStdioUUID,
                        mStdioVector,
                        pid_stdin,
                        pid_stdout,
                        pipe_stdin_to_cmd[READ_FROM_PIPE],
                        pipe_stdin_to_cmd[WRITE_TO_PIPE],
                        pipe_cmd_to_stdout[READ_FROM_PIPE],
                        pipe_cmd_to_stdout[WRITE_TO_PIPE],
                        nullptr,
                        false
                    );

                pInf->setCmd(strCmd);

                if (msg.check("env"))
                {
                    pInf->setEnv(msg.find("env").asString());
                }

                mProcessVector->Add(pInf);

                yarp::os::SystemClock::delaySystem(0.01);

                FILE* in_from_child=fdopen(pipe_child_to_parent[READ_FROM_PIPE], "r");
                int flags=fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_GETFL, 0);
                fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_SETFL, flags|O_NONBLOCK);

                std::string out;

                if (in_from_child)
                {
                    char buff[1024];

                    while(true)
                    {
                        if (!fgets(buff, 1024, in_from_child) || ferror(in_from_child) || feof(in_from_child)) {
                            break;
                        }

                        out+=std::string(buff);
                    }

                    fclose(in_from_child);
                }

                if (out.length()>0)
                {
                    pid_cmd=YARPRUN_ERROR;
                }
                else
                {
                    out=std::string("STARTED: server=")+mPortName
                       +std::string(" alias=")+strAlias
                       +std::string(" cmd=")+strCmd
                       +std::string(" pid=")+int2String(pid_cmd)
                       +std::string("\n");
                }

                result.addInt32(pid_cmd);
                result.addString(out.c_str());
                result.addString(strStdioUUID.c_str());

                fprintf(stderr, "%s", out.c_str());

                CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

                return pid_cmd;
            }
        }
    }

    result.addInt32(YARPRUN_ERROR);
    result.addString("I should never reach this point!!!\n");

    return YARPRUN_ERROR;
}

int yarp::run::RunServer::executeCmdStdout(const yarp::os::Bottle& msg, yarp::os::Bottle& result, std::string& loggerName)
{
    std::string proc_label = getProcLabel(msg);

    std::string strAlias=msg.find("as").asString();
    std::string strCmd=msg.find("cmd").asString();

    std::string portName="/log";
    portName+=mPortName+"/";

    std::string command = strCmd;
    command = command.substr(0, command.find(' '));
    command = command.substr(command.find_last_of("\\/") + 1);

    portName+=command;
    if (proc_label != "") { portName += "[" + proc_label + "]"; }


    int  pipe_cmd_to_stdout[2];
    int  ret_cmd_to_stdout=yarp::run::impl::pipe(pipe_cmd_to_stdout);

    int  pipe_child_to_parent[2];
    int  ret_child_to_parent=yarp::run::impl::pipe(pipe_child_to_parent);

    if (ret_child_to_parent!=0 || ret_cmd_to_stdout!=0)
    {
        int error=errno;

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdout\n")
                                 +std::string("Can't create pipes ")+strerror(error)
                                 +std::string("\n");

        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    int pid_stdout=yarp::run::impl::fork();

    if (IS_INVALID(pid_stdout))
    {
        int error=errno;

        CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
        CLOSE(pipe_cmd_to_stdout[READ_FROM_PIPE]);
        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);
        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdout\n")
                                 +std::string("Can't fork stdout process because ")+strerror(error)
                                 +std::string("\n");

        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    if (IS_NEW_PROCESS(pid_stdout)) // STDOUT IMPLEMENTED HERE
    {
        REDIRECT_TO(STDIN_FILENO, pipe_cmd_to_stdout[READ_FROM_PIPE]);

        CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

        //Why removing vectors and stop threads?
        //exec* never returns and memory is claimed by the system
        //furthermore after fork() only the thread which called fork() is forked!
        //            cleanBeforeExec();

        //yarp::os::impl::signal(SIGPIPE, SIG_DFL);

        int ret = yarp::run::impl::execlp("yarprun", "yarprun", "--write", portName.c_str(), "--log", loggerName.c_str(), static_cast<char*>(nullptr));

        CLOSE(pipe_cmd_to_stdout[READ_FROM_PIPE]);

        if (ret==YARPRUN_ERROR)
        {
            int error=errno;

            std::string out=std::string("ABORTED: server=")+mPortName
                                     +std::string(" alias=")+strAlias
                                     +std::string(" cmd=stdout\n")
                                     +std::string("Can't execute stdout because ")+strerror(error)
                                     +std::string("\n");

            FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
            fprintf(out_to_parent, "%s", out.c_str());
            fflush(out_to_parent);
            fclose(out_to_parent);

            fprintf(stderr, "%s", out.c_str());
        }

        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

        std::exit(ret);
    }

    if (IS_PARENT_OF(pid_stdout))
    {
        CLOSE(pipe_cmd_to_stdout[READ_FROM_PIPE]);

        fprintf(stderr, "STARTED: server=%s alias=%s cmd=stdout pid=%d\n", mPortName.c_str(), strAlias.c_str(), pid_stdout);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //if (IS_PARENT_OF(pid_stdin))
        {
            int pid_cmd=yarp::run::impl::fork();

            if (IS_INVALID(pid_cmd))
            {
                int error=errno;

                CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);
                CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

                std::string out=std::string("ABORTED: server=")+mPortName
                                         +std::string(" alias=")+strAlias
                                         +std::string(" cmd=")+strCmd
                                         +std::string("\nCan't fork command process because ")+strerror(error)
                                         +std::string("\n");

                result.addInt32(YARPRUN_ERROR);
                result.addString(out.c_str());
                fprintf(stderr, "%s", out.c_str());

                FILE* to_yarp_stdout=fdopen(pipe_cmd_to_stdout[WRITE_TO_PIPE], "w");
                fprintf(to_yarp_stdout, "%s", out.c_str());
                fflush(to_yarp_stdout);
                fclose(to_yarp_stdout);

                SIGNAL(pid_stdout, SIGTERM);
                fprintf(stderr, "TERMINATING stdout (%d)\n", pid_stdout);
                CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);

                return YARPRUN_ERROR;
            }

            if (IS_NEW_PROCESS(pid_cmd)) // RUN COMMAND HERE
            {
                CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

                char *cmd_str=new char[strCmd.length()+1];
                strcpy(cmd_str, strCmd.c_str());
                /*
                int nargs=CountArgs(cmd_str);
                char **arg_str=new char*[nargs+1];
                ParseCmd(cmd_str, arg_str);
                arg_str[nargs]=0;
                */
                int nargs = 0;
                char **arg_str = new char*[C_MAXARGS + 1];
                parseArguments(cmd_str, &nargs, arg_str);
                arg_str[nargs]=nullptr;

                setvbuf(stdout, nullptr, _IONBF, 0);

                REDIRECT_TO(STDOUT_FILENO, pipe_cmd_to_stdout[WRITE_TO_PIPE]);
                REDIRECT_TO(STDERR_FILENO, pipe_cmd_to_stdout[WRITE_TO_PIPE]);

                // Set the YARP_IS_YARPRUN environment variable to 1, so that the child
                // process will now that is running inside yarprun.
                yarp::conf::environment::set_string("YARP_IS_YARPRUN", "1");

                // Set the YARPRUN_IS_FORWARDING_LOG environment variable to 1, so that
                // the child process will now that yarprun is not logging the output.
                yarp::conf::environment::set_string("YARPRUN_IS_FORWARDING_LOG", "1");

                if (msg.check("env"))
                {
                    auto ss = yarp::conf::string::split(msg.find("env").asString(), ';');
                    for (const auto& s : ss) {
                        char* szenv = new char[s.size()+1];
                        strcpy(szenv, s.c_str());
                        yarp::run::impl::putenv(szenv); // putenv doesn't make copy of the string
                    }
                    //delete [] szenv;
                }

                if (msg.check("workdir"))
                {
                    int ret = yarp::os::impl::chdir(msg.find("workdir").asString().c_str());

                    if (ret!=0)
                    {
                        int error=errno;

                        std::string out=std::string("ABORTED: server=")+mPortName
                                                 +std::string(" alias=")+strAlias
                                                 +std::string(" cmd=")+strCmd
                                                 +std::string("\nCan't execute command, cannot set working directory ")+strerror(error)
                                                 +std::string("\n");

                        FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
                        fprintf(out_to_parent, "%s", out.c_str());
                        fflush(out_to_parent);
                        fclose(out_to_parent);
                        fprintf(stderr, "%s", out.c_str());

                        std::exit(ret);
                    }
                }

                int ret=YARPRUN_ERROR;

                char currWorkDirBuff[1024];
                char *currWorkDir=getcwd(currWorkDirBuff, 1024);

                if (currWorkDir)
                {
                    char **cwd_arg_str=new char*[nargs+1];
                    for (int i = 1; i < nargs; ++i) {
                        cwd_arg_str[i] = arg_str[i];
                    }
                    cwd_arg_str[nargs]=nullptr;
                    cwd_arg_str[0]=new char[strlen(currWorkDir)+strlen(arg_str[0])+16];

                    strcpy(cwd_arg_str[0], currWorkDir);
                    strcat(cwd_arg_str[0], "/");
                    strcat(cwd_arg_str[0], arg_str[0]);

                    //Why removing vectors and stop threads?
                    //exec* never returns and memory is claimed by the system
                    //furthermore after fork() only the thread which called fork() is forked!
                    //            cleanBeforeExec();

                    ret = yarp::run::impl::execvp(cwd_arg_str[0], cwd_arg_str);

                    delete [] cwd_arg_str[0];
                    delete [] cwd_arg_str;
                }

                if (ret==YARPRUN_ERROR)
                {
                    //Why removing vectors and stop threads?
                    //exec* never returns and memory is claimed by the system
                    //furthermore after fork() only the thread which called fork() is forked!
                    //            cleanBeforeExec();

                    ret = yarp::run::impl::execvp(arg_str[0], arg_str);
                }

                fflush(stdout);

                CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);

                if (ret==YARPRUN_ERROR)
                {
                    int error=errno;

                    std::string out=std::string("ABORTED: server=")+mPortName
                                             +std::string(" alias=")+strAlias
                                             +std::string(" cmd=")+strCmd
                                             +std::string("\nCan't execute command because ")+strerror(error)
                                             +std::string("\n");

                    FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
                    fprintf(out_to_parent, "%s", out.c_str());
                    fflush(out_to_parent);
                    fclose(out_to_parent);
                    fprintf(stderr, "%s", out.c_str());
                }

                delete [] cmd_str;
                delete [] arg_str;

                CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

                std::exit(ret);
            }


            if (IS_PARENT_OF(pid_cmd))
            {
                CLOSE(pipe_cmd_to_stdout[WRITE_TO_PIPE]);
                CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

                auto* pInf = new YarpRunCmdWithStdioInfo(
                        strAlias,
                        mPortName,
                        portName,
                        pid_cmd,
                        pid_stdout,
                        pipe_cmd_to_stdout[READ_FROM_PIPE],
                        pipe_cmd_to_stdout[WRITE_TO_PIPE],
                        nullptr,
                        false
                    );

                pInf->setCmd(strCmd);

                if (msg.check("env"))
                {
                    pInf->setEnv(msg.find("env").asString());
                }

                mProcessVector->Add(pInf);

                yarp::os::SystemClock::delaySystem(0.01);

                FILE* in_from_child=fdopen(pipe_child_to_parent[READ_FROM_PIPE], "r");
                int flags=fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_GETFL, 0);
                fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_SETFL, flags|O_NONBLOCK);

                std::string out;

                if (in_from_child)
                {
                    char buff[1024];

                    while(true)
                    {
                        if (!fgets(buff, 1024, in_from_child) || ferror(in_from_child) || feof(in_from_child)) {
                            break;
                        }

                        out+=std::string(buff);
                    }

                    fclose(in_from_child);
                }

                if (out.length()>0)
                {
                    pid_cmd=YARPRUN_ERROR;
                }
                else
                {
                    out=std::string("STARTED: server=")+mPortName
                       +std::string(" alias=")+strAlias
                       +std::string(" cmd=")+strCmd
                       +std::string(" pid=")+int2String(pid_cmd)
                       +std::string("\n");
                }

                result.addInt32(pid_cmd);
                result.addString(out.c_str());

                fprintf(stderr, "%s", out.c_str());

                CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

                return pid_cmd;
            }
        }
    }

    result.addInt32(YARPRUN_ERROR);
    result.addString("I should never reach this point!!!\n");

    return YARPRUN_ERROR;
}

int yarp::run::RunServer::userStdio(const yarp::os::Bottle& msg, yarp::os::Bottle& result)
{
    std::string strAlias=msg.find("as").asString();
    std::string strUUID=msg.find("stdiouuid").asString();

    std::string strCmd;

    if (msg.check("forward"))
    {
        strCmd=std::string("/bin/bash -l -c \"yarprun --readwrite ")+strUUID
              +std::string(" --forward ")+msg.findGroup("forward").get(1).asString()+std::string(" ")+msg.findGroup("forward").get(2).asString()+std::string("\"");
    }
    else
    {
        strCmd=std::string("/bin/bash -l -c \"yarprun --readwrite ")+strUUID+"\"";
    }

    int pipe_child_to_parent[2];

    if (yarp::run::impl::pipe(pipe_child_to_parent))
    {
        int error=errno;

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdio\nCan't create pipe ")+strerror(error)
                                 +std::string("\n");

        result.clear();
        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    int c=0;
    char *command[16];
    for (auto & i : command) {
        i = nullptr;
    }

    cmdcpy(command[c++], "xterm");
    cmdcpy(command[c++], msg.check("hold")?"-hold":"+hold");

    if (msg.check("geometry"))
    {
        cmdcpy(command[c++], "-geometry");
        cmdcpy(command[c++], msg.find("geometry").asString().c_str());
    }

    cmdcpy(command[c++], "-title");
    cmdcpy(command[c++], strAlias.c_str());

    cmdcpy(command[c++], "-e");
    cmdcpy(command[c++], strCmd.c_str());

    int pid_cmd=yarp::run::impl::fork();

    if (IS_INVALID(pid_cmd))
    {
        int error=errno;

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdio\nCan't fork stdout process because ")+strerror(error)
                                 +std::string("\n");

        result.clear();
        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);
        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

        cmdclean(command);

        return YARPRUN_ERROR;
    }

    if (IS_NEW_PROCESS(pid_cmd)) // RUN COMMAND HERE
    {
        //yarp::os::impl::signal(SIGPIPE, SIG_IGN);

        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

        REDIRECT_TO(STDERR_FILENO, pipe_child_to_parent[WRITE_TO_PIPE]);

        //Why removing vectors and stop threads?
        //exec* never returns and memory is claimed by the system
        //furthermore after fork() only the thread which called fork() is forked!
        //            cleanBeforeExec();

        //yarp::os::impl::signal(SIGHUP, rwSighupHandler);

        int ret = yarp::run::impl::execvp("xterm", command);

        cmdclean(command);

        if (ret==YARPRUN_ERROR)
        {
            int error=errno;

            std::string out=std::string("ABORTED: server=")+mPortName
                                     +std::string(" alias=")+strAlias
                                     +std::string(" cmd=xterm\nCan't execute command because ")+strerror(error)
                                     +std::string("\n");

            FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");

            fprintf(out_to_parent, "%s", out.c_str());
            fflush(out_to_parent);
            fclose(out_to_parent);

            fprintf(stderr, "%s", out.c_str());
        }

        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

        std::exit(ret);
    }

    if (IS_PARENT_OF(pid_cmd))
    {
        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

        mStdioVector->Add(new YarpRunProcInfo(strAlias, mPortName, pid_cmd, nullptr, msg.check("hold")));

        result.clear();

        cmdclean(command);

        yarp::os::SystemClock::delaySystem(0.01);

        FILE* in_from_child=fdopen(pipe_child_to_parent[READ_FROM_PIPE], "r");
        int flags=fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_GETFL, 0);
        fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_SETFL, flags|O_NONBLOCK);
        std::string out;

        if (in_from_child)
        {
            char buff[1024];

            while(true)
            {
                if (!fgets(buff, 1024, in_from_child) || ferror(in_from_child) || feof(in_from_child)) {
                    break;
                }

                out+=std::string(buff);
            }

            fclose(in_from_child);
        }

        result.clear();

        //if (out.length()>0)
        if (out.substr(0, 14)=="xterm Xt error" || out.substr(0, 7)=="ABORTED")
        {
            pid_cmd=YARPRUN_ERROR;
        }
        else
        {
            out=std::string("STARTED: server=")+mPortName
               +std::string(" alias=")+strAlias
               +std::string(" cmd=xterm pid=")+int2String(pid_cmd)
               +std::string("\n");

        }

        fprintf(stderr, "%s", out.c_str());

        result.addInt32(pid_cmd);
        result.addString(out.c_str());

        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);

        return pid_cmd;
    }

    result.clear();
    result.addInt32(YARPRUN_ERROR);

    return YARPRUN_ERROR;
}

int yarp::run::RunServer::executeCmd(const yarp::os::Bottle& msg, yarp::os::Bottle& result)
{
    std::string strAlias(msg.find("as").asString());
    std::string strCmd(msg.find("cmd").toString());

    int  pipe_child_to_parent[2];
    int ret_pipe_child_to_parent=yarp::run::impl::pipe(pipe_child_to_parent);

    if (ret_pipe_child_to_parent!=0)
    {
        int error=errno;

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=stdio\nCan't create pipe ")+strerror(error)
                                 +std::string("\n");


        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    int pid_cmd=yarp::run::impl::fork();

    if (IS_INVALID(pid_cmd))
    {
        int error=errno;

        std::string out=std::string("ABORTED: server=")+mPortName
                                 +std::string(" alias=")+strAlias
                                 +std::string(" cmd=")+strCmd
                                 +std::string("\nCan't fork command process because ")+strerror(error)
                                 +std::string("\n");

        result.addInt32(YARPRUN_ERROR);
        result.addString(out.c_str());
        fprintf(stderr, "%s", out.c_str());

        return YARPRUN_ERROR;
    }

    if (IS_NEW_PROCESS(pid_cmd)) // RUN COMMAND HERE
    {
        int saved_stderr = yarp::run::impl::dup(STDERR_FILENO);
        int null_file=open("/dev/null", O_WRONLY);
        if (null_file >= 0)
        {
            REDIRECT_TO(STDOUT_FILENO, null_file);
            REDIRECT_TO(STDERR_FILENO, null_file);
            close(null_file);
        }
        char *cmd_str=new char[strCmd.length()+1];
        strcpy(cmd_str, strCmd.c_str());
        /*
        int nargs=CountArgs(cmd_str);
        char **arg_str=new char*[nargs+1];
        ParseCmd(cmd_str, arg_str);
        arg_str[nargs]=0;
        */
        int nargs = 0;
        char **arg_str = new char*[C_MAXARGS + 1];
        parseArguments(cmd_str, &nargs, arg_str);
        arg_str[nargs]=nullptr;

        // Set the YARP_IS_YARPRUN environment variable to 1, so that the child
        // process will now that is running inside yarprun.
        yarp::conf::environment::set_string("YARP_IS_YARPRUN", "1");

        // Set the YARPRUN_IS_FORWARDING_LOG environment variable to 0, so that
        // the child process will now that yarprun is not logging the output.
        yarp::conf::environment::set_string("YARPRUN_IS_FORWARDING_LOG", "0");

        if (msg.check("env"))
        {
            auto ss = yarp::conf::string::split(msg.find("env").asString(), ';');
            for (const auto& s : ss) {
                char* szenv = new char[s.size()+1];
                strcpy(szenv, s.c_str());
                yarp::run::impl::putenv(szenv); // putenv doesn't make copy of the string
            }
        }

        if (msg.check("workdir"))
        {
            int ret = yarp::os::impl::chdir(msg.find("workdir").asString().c_str());

            if (ret!=0)
            {
                int error=errno;

                std::string out=std::string("ABORTED: server=")+mPortName
                                         +std::string(" alias=")+strAlias
                                         +std::string(" cmd=")+strCmd
                                         +std::string("\nCan't execute command, cannot set working directory ")+strerror(error)
                                         +std::string("\n");

                FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
                fprintf(out_to_parent, "%s", out.c_str());
                fflush(out_to_parent);
                fclose(out_to_parent);

                REDIRECT_TO(STDERR_FILENO, saved_stderr);
                fprintf(stderr, "%s", out.c_str());
            }
        }

        int ret=YARPRUN_ERROR;

        char currWorkDirBuff[1024];
        char *currWorkDir=getcwd(currWorkDirBuff, 1024);

        if (currWorkDir)
        {
            char **cwd_arg_str=new char*[nargs+1];
            for (int i = 1; i < nargs; ++i) {
                cwd_arg_str[i] = arg_str[i];
            }
            cwd_arg_str[nargs]=nullptr;
            cwd_arg_str[0]=new char[strlen(currWorkDir)+strlen(arg_str[0])+16];


            strcpy(cwd_arg_str[0], currWorkDir);
            strcat(cwd_arg_str[0], "/");
            strcat(cwd_arg_str[0], arg_str[0]);

            //Why removing vectors and stop threads?
            //exec* never returns and memory is claimed by the system
            //furthermore after fork() only the thread which called fork() is forked!
//            cleanBeforeExec();

            ret = yarp::run::impl::execvp(cwd_arg_str[0], cwd_arg_str);

            delete [] cwd_arg_str[0];
            delete [] cwd_arg_str;
        }

        if (ret==YARPRUN_ERROR)
        {
            //Why removing vectors and stop threads?
            //exec* never returns and memory is claimed by the system
            //furthermore after fork() only the thread which called fork() is forked!
            //            cleanBeforeExec();
            ret = yarp::run::impl::execvp(arg_str[0], arg_str);
        }

        if (ret==YARPRUN_ERROR)
        {
            int error=errno;

            std::string out=std::string("ABORTED: server=")+mPortName
                                     +std::string(" alias=")+strAlias
                                     +std::string(" cmd=")+strCmd
                                     +std::string("\nCan't execute command because ")+strerror(error)
                                     +std::string("\n");

            FILE* out_to_parent=fdopen(pipe_child_to_parent[WRITE_TO_PIPE], "w");
            fprintf(out_to_parent, "%s", out.c_str());
            fflush(out_to_parent);
            fclose(out_to_parent);

            if (saved_stderr >= 0)
            {
                REDIRECT_TO(STDERR_FILENO, saved_stderr);
            }
            fprintf(stderr, "%s", out.c_str());
        }

        delete [] cmd_str;
        delete [] arg_str;

        std::exit(ret);
    }

    if (IS_PARENT_OF(pid_cmd))
    {
        auto* pInf = new YarpRunProcInfo(strAlias, mPortName, pid_cmd, nullptr, false);
        pInf->setCmd(strCmd);
        if (msg.check("env")) {
            pInf->setEnv(msg.find("env").asString());
        }
        mProcessVector->Add(pInf);
        char pidstr[16];
        sprintf(pidstr, "%d", pid_cmd);

        yarp::os::SystemClock::delaySystem(0.01);

        FILE* in_from_child=fdopen(pipe_child_to_parent[READ_FROM_PIPE], "r");
        int flags=fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_GETFL, 0);
        fcntl(pipe_child_to_parent[READ_FROM_PIPE], F_SETFL, flags|O_NONBLOCK);

        std::string out;

        if (in_from_child)
        {
            char buff[1024];

            while(true)
            {
                if (!fgets(buff, 1024, in_from_child) || ferror(in_from_child) || feof(in_from_child)) {
                    break;
                }

                out+=std::string(buff);
            }

            fclose(in_from_child);
        }

        if (out.length()>0)
        {
            pid_cmd=YARPRUN_ERROR;
        }
        else
        {
            out=std::string("STARTED: server=")+mPortName
               +std::string(" alias=")+strAlias
               +std::string(" cmd=")+strCmd
               +std::string(" pid=")+int2String(pid_cmd)
               +std::string("\n");
        }

        fprintf(stderr, "%s", out.c_str());

        result.addInt32(pid_cmd);
        result.addString(out.c_str());

        CLOSE(pipe_child_to_parent[READ_FROM_PIPE]);
        CLOSE(pipe_child_to_parent[WRITE_TO_PIPE]);

        return pid_cmd;
    }

    result.addInt32(YARPRUN_ERROR);

    return YARPRUN_ERROR;
}

#endif
