/*
 * Copyright (C) 2015 Istituto Italiano di Tecnologia (IIT)
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_IREMOTEVARIABLES_H
#define YARP_DEV_IREMOTEVARIABLES_H

#include <yarp/os/Vocab.h>

namespace yarp {
    namespace dev {
        class IRemoteVariablesRaw;
        class IRemoteVariables;
      }
}

/**
 * @ingroup dev_iface_motor
 *
 * IRemoteVariables interface.
 */
class yarp::dev::IRemoteVariablesRaw
{
public:
    /**
     * Destructor.
     */
    virtual ~IRemoteVariablesRaw() {}

    virtual bool getRemoteVariableRaw(yarp::os::ConstString key, yarp::os::Bottle& val) = 0;

    virtual bool setRemoteVariableRaw(yarp::os::ConstString key, const yarp::os::Bottle& val) = 0;

    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys) = 0;
};

/**
 * @ingroup dev_iface_motor
 *
 * IRemoteVariables interface.
 */
class YARP_dev_API yarp::dev::IRemoteVariables
{
public:
    /**
     * Destructor.
     */
    virtual ~IRemoteVariables() {}

    virtual bool getRemoteVariable(yarp::os::ConstString key, yarp::os::Bottle& val) = 0;

    virtual bool setRemoteVariable(yarp::os::ConstString key, const yarp::os::Bottle& val) = 0;

    virtual bool getRemoteVariablesList(yarp::os::Bottle* listOfKeys) = 0;
};

#define VOCAB_REMOTE_VARIABILE_INTERFACE   VOCAB4('i','v','a','r')
#define VOCAB_VARIABLE                     VOCAB4('m','v','a','r')
#define VOCAB_LIST_VARIABLES               VOCAB4('l','v','a','r')


#endif // YARP_DEV_IREMOTEVARIABLES_H
