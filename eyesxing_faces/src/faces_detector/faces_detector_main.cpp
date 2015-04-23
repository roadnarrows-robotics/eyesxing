////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// ROS Node:  faces_detector
//
// File:      faces_detector_main.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS faces_detector main.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <sys/types.h>
#include <unistd.h>
#include <libgen.h>
#include <string>
#include <sstream>

//
// ROS 
//
#include "ros/ros.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

//
// Node headers.
//
#include "faces_detector.h"

using namespace ::std;
using namespace eyesxing;


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Node Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error

//
// Data
//
const char *NodeName = "faces_detector";  ///< this ROS node's name


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// RoadNarrows Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Options
//
static bool_t OptsView    = false;    ///< debug view detected objects/frame

/*!
 * \brief The package information.
 *
 * For ROS nodes, RN package information is equivalent to this ROS application
 * information.
 * */
static const PkgInfo_T PkgInfo =
{
  NodeName,                       ///< package name
  "1.0.0",                        ///< package version
  "2014.05.29",                   ///< date (and time)
  "2014",                         ///< year
  NodeName,                       ///< package full name
  "Robin Knight",                 ///< authors
  "RoadNarrows LLC",              ///< owner
  "(C) 2014 RoadNarrows LLC"      ///< disclaimer
};

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  "[ROSOPTIONS]",

  // synopsis
  "The %P ROS node provides ROS interfaces to the embedded pan-tilt robotic "
  "mechanism.",
  
  // long_desc 
  "",
 
  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // --view, -v
  {
    "view",               // long_opt
    'v',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsView,            // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "View overlay of detected objects onto captured frames in window."
  },


  {NULL, }
};


/*!
 *  \brief ROS faces_detector node main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns exit code.
 */
int main(int argc, char *argv[])
{
  string  strNodeName;  // ROS-given node name
  string  strDevName;   // real device name
  double  hz = 30.0;    // ros loop hertz rate
  int     rc;           // return code

  // 
  // Initialize the node. Parse the command line arguments and environment to
  // determine ROS options such as node name, namespace and remappings.
  // This call does not contact the master. This lets you use
  // ros::master::check() and other ROS functions after calling ros::init()
  // to check on the status of the master.
  //
  ros::init(argc, argv, NodeName);

  //
  // Parse node-specific options and arguments (from librnr).
  //
  OptsGet(NodeName, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);
 
  //
  //
  // A ctrl-c interrupt will stop attempts to connect to the ROS core.
  //
  ros::NodeHandle nh(NodeName);

  // actual ROS-given node name
  strNodeName = ros::this_node::getName();

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    // add optional non-ROS unit tests here, then simply exit.
    return APP_EC_OK;
  }

  ROS_INFO("%s: Node started.", strNodeName.c_str());
  
  //
  // Create a pan-tilt node object.
  //
  EyesXingFacesDetector  detector(nh, hz);

  //
  // Advertise services.
  //
  detector.advertiseServices();

  ROS_INFO("%s: Services registered.", strNodeName.c_str());

  //
  // Advertise publishers.
  //
  detector.advertisePublishers();
  
  ROS_INFO("%s: Publishers registered.", strNodeName.c_str());
  
  //
  // Subscribed to topics.
  //
  detector.subscribeToTopics();
  
  ROS_INFO("%s: Subscribed topics registered.", strNodeName.c_str());

  //
  // Create Action Servers
  //

  // set loop rate in Hertz
  ros::Rate loop_rate(hz);

  ROS_INFO("%s: Ready.", strNodeName.c_str());

  //
  // Main loop.
  //
  while( ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // publish all advertized topics
    detector.publish();

    // view detected objects
    if( OptsView )
    {
      detector.view();
    }

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  return APP_EC_OK;
}
