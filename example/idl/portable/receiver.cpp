/** Copyright: (C) 2014 Istituto Italiano di Tecnologia (IIT)
* Authors: Lorenzo Natale
* CopyPolicy:  Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <SharedData.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

using namespace std;

int main()
{
   yarp::os::Network network;

   cout<<"Starting receiver\n";

   yarp::os::Port port;
   if (!port.open("/receiver"))
   {
       cerr<<"Error opening port, check your yarp network\n";
       return -1;
   }

   while(true)
   {
       SharedData d;
       port.read(d);

       //access d
       
   }
   
   return 0;
}

