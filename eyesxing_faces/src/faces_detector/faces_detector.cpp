////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// ROS Node:  faces_detector
//
// File:      faces_detector.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS faces_detector node class implementation.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"


//
// ROS generated core and eyesxing messages.
//
#include "eyesxing_msgs/Classifier.h"
#include "eyesxing_msgs/OOI.h"

//
// ROS generatated eyesxing services.
//
#include "eyesxing_faces/LoadClassifier.h"
#include "eyesxing_faces/ClearClassifiers.h"

//
// ROS generated action servers.
//

//
// RoadNarrows embedded libraries.
//

//
// Node headers.
//
#include "eyesxing_detector.h"


using namespace std;
using namespace eyesxing;
using namespace eyesxing_detector;


//------------------------------------------------------------------------------
// EyesXingFacesDetector Class
//------------------------------------------------------------------------------

hz
EyesXingFacesDetector::EyesXingFacesDetector(ros::NodeHandle &nh) : m_nh(nh)
{
}

EyesXingFacesDetector::~EyesXingFacesDetector()
{
}


//..............................................................................
// Services
//..............................................................................

void EyesXingFacesDetector::advertiseServices()
{
  string  strSvc;

  strSvc = "load_classifier";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                        &EyesXingFacesDetector::loadClassifier,
                                        &(*this));

  strSvc = "clear_classifiers";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                      &EyesXingFacesDetector::clearClassifiers,
                                      &(*this));
}

bool EyesXingFacesDetector::clearClassifiers(ClearClassifiers::Request  &req,
                                             ClearClassifiers::Response &rsp)
{
  ROS_DEBUG("clear_classifiers");

  return true;
}

bool EyesXingFacesDetector::loadClassifier(LoadClassifier::Request  &req,
                                           LoadClassifier::Response &rsp)
{
  ROS_DEBUG("load_classifier");

  //ROS_INFO("Sweep from %lf to %lf and %lf to %lf.",
  //       req.pan_min_pos, req.pan_max_pos, req.tilt_min_pos, req.tilt_max_pos);

  return true;
}


//..............................................................................
// Topic Publishers
//..............................................................................

void EyesXingFacesDetector::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "joint_states";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::JointState>(strPub, nQueueDepth);

  strPub = "joint_states_ex";
  m_publishers[strPub] =
    m_nh.advertise<pan_tilt_control::JointStateExtended>(strPub,nQueueDepth);

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);

  strPub = "robot_status_ex";
  m_publishers[strPub] =
    m_nh.advertise<pan_tilt_control::RobotStatusExtended>(strPub,nQueueDepth);
}

void EyesXingFacesDetector::publish()
{
  publishJointState();
  publishRobotStatus();
}

void EyesXingFacesDetector::publishJointState()
{
  PanTiltJointStatePoint    state;

  // get robot's extended joint state.
  m_robot.getJointState(state);
  
  // update joint state message
  updateJointStateMsg(state, m_msgJointState);

  // publish joint state messages
  m_publishers["joint_states"].publish(m_msgJointState);

  // update extended joint state message
  updateExtendedJointStateMsg(state, m_msgJointStateEx);

  // publish extened joint state messages
  m_publishers["joint_states_ex"].publish(m_msgJointStateEx);
}

void EyesXingFacesDetector::publishRobotStatus()
{
  PanTiltRobotStatus  status;

  // get robot's extended status.
  m_robot.getRobotStatus(status);

  // update robot status message
  updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  m_publishers["robot_status"].publish(m_msgRobotStatus);

  // update extended robot status message
  updateExtendedRobotStatusMsg(status, m_msgRobotStatusEx);

  // publish extened robot status message
  m_publishers["robot_status_ex"].publish(m_msgRobotStatusEx);
}

void EyesXingFacesDetector::updateJointStateMsg(PanTiltJointStatePoint &state,
                                         sensor_msgs::JointState &msg)
{
  //
  // Clear previous joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  //
  // Set joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set joint state state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    // joint state
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
  }
}

void EyesXingFacesDetector::updateExtendedJointStateMsg(PanTiltJointStatePoint &state,
                                     pan_tilt_control::JointStateExtended &msg)
{
  pan_tilt_control::OpState opstate;

  // 
  // Clear previous extended joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();
  msg.master_servo_id.clear();
  msg.slave_servo_id.clear();
  msg.odometer_pos.clear();
  msg.encoder_pos.clear();
  msg.raw_speed.clear();
  msg.op_state.clear();

  //
  // Set extended joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set extended joint state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
    msg.master_servo_id.push_back(state[n].m_nMasterServoId);
    msg.slave_servo_id.push_back(state[n].m_nSlaveServoId);
    msg.odometer_pos.push_back(state[n].m_nOdPos);
    msg.encoder_pos.push_back(state[n].m_nEncPos);
    msg.raw_speed.push_back(state[n].m_nSpeed);

    opstate.calib_state = state[n].m_eOpState;
    msg.op_state.push_back(opstate);
  }
}

void EyesXingFacesDetector::updateRobotStatusMsg(PanTiltRobotStatus &status,
                                          industrial_msgs::RobotStatus &msg)
{
  //
  // Set robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set industrial message compliant robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;

}

void EyesXingFacesDetector::updateExtendedRobotStatusMsg(
                                    PanTiltRobotStatus &status,
                                    pan_tilt_control::RobotStatusExtended &msg)
{
  ServoHealth sh;
  int         i;

  //
  // Set extended robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set pan-tilt message extended robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;
  msg.is_calibrated.val   = status.m_eIsCalibrated;

  // clear previous data
  msg.servo_health.clear();

  for(i=0; i<status.m_vecServoHealth.size(); ++i)
  {
    sh.servo_id = status.m_vecServoHealth[i].m_nServoId;
    sh.temp     = status.m_vecServoHealth[i].m_fTemperature;
    sh.voltage  = status.m_vecServoHealth[i].m_fVoltage;
    sh.alarm    = status.m_vecServoHealth[i].m_uAlarms;

    msg.servo_health.push_back(sh);
  }
}


//..............................................................................
// Subscribed Topics
//..............................................................................

void EyesXingFacesDetector::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "joint_command";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &EyesXingFacesDetector::execJointCmd,
                                          &(*this));
}

void EyesXingFacesDetector::execJointCmd(const trajectory_msgs::JointTrajectory &jt)
{
  ROS_DEBUG("Executing joint_command.");

  PanTiltJointTrajectoryPoint pt;

  // load trajectory point
  for(int j=0; j<jt.joint_names.size(); ++j)
  {
    pt.append(jt.joint_names[j],
              jt.points[0].positions[j], 
              jt.points[0].velocities[j]);
    ROS_INFO("%s: pos=%5.3f speed=%2.1f", jt.joint_names[j].c_str(), 
                                          jt.points[0].positions[j], 
                                          jt.points[0].velocities[j]);
  }

  m_robot.move(pt);
}
