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
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//
// Includes for boost libraries
//
#include <boost/bind.hpp>

//
// OpenCV
//
#include <opencv2/opencv.hpp>

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
#include "eyesxing/eyesxing.h"
#include "eyesxing/eyeOOI.h"
#include "eyesxing/eyeClassifier.h"

//
// Node headers.
//
#include "faces_detector.h"


using namespace std;
using namespace eyesxing;
using namespace eyesxing_faces;


//------------------------------------------------------------------------------
// EyesXingFacesDetector Class
//------------------------------------------------------------------------------

EyesXingFacesDetector::EyesXingFacesDetector(ros::NodeHandle &nh, double hz)
    : m_nh(nh), m_hz(hz)
{
  bool autosize = false;

  cv::namedWindow("debug", autosize ? CV_WINDOW_AUTOSIZE : 0);
  //cv::setMouseCallback("debug", &ImageNodelet::mouseCb, this);
}

EyesXingFacesDetector::~EyesXingFacesDetector()
{
  cv::destroyWindow("debug");
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void EyesXingFacesDetector::advertiseServices()
{
  string  strSvc;

  strSvc = "clear_classifiers";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                      &EyesXingFacesDetector::clearClassifiers,
                                      &(*this));

  strSvc = "load_classifier";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                        &EyesXingFacesDetector::loadClassifier,
                                        &(*this));
}

bool EyesXingFacesDetector::clearClassifiers(ClearClassifiers::Request  &req,
                                             ClearClassifiers::Response &rsp)
{
  ROS_DEBUG("clear_classifiers");

  m_pipeline.clear();

  return true;
}

bool EyesXingFacesDetector::loadClassifier(LoadClassifier::Request  &req,
                                           LoadClassifier::Response &rsp)
{
  ROS_DEBUG("load_classifier");

  DetectorPElem pelem(req.classifier.name, (SetOp)req.classifier.setop);

  m_pipeline.push_back(pelem);

  if( m_pipeline.back().load(req.classifier.filename) < 0 )
  {
    m_pipeline.pop_back();
    ROS_ERROR("Failed to load classifier:\n\t%s",
      req.classifier.filename.c_str());
    rsp.rc = (int8_t)-EYE_ECODE_NO_EXEC;
  }

  else
  {
    m_pipeline.push_back(pelem);
    ROS_INFO("Loaded classifier:\n\t%s\n\t%s\n\t%s\n",
      req.classifier.name.c_str(),
      req.classifier.filename.c_str(),
      setopstr(req.classifier.setop));
    rsp.rc = (int8_t)EYE_OK;
  }

  return true;
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void EyesXingFacesDetector::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "ooi";
  m_publishers[strPub] =
    m_nh.advertise<eyesxing_msgs::OOI>(strPub, nQueueDepth);
}

void EyesXingFacesDetector::publish()
{
  publishOOI();
}

void EyesXingFacesDetector::publishOOI()
{
  // update objects of interest message
  updateOOIMsg(m_msgOOI);

  // publish objects of interest message
  m_publishers["ooi"].publish(m_msgOOI);
}

void EyesXingFacesDetector::updateOOIMsg(eyesxing_msgs::OOI &msg)
{
  m_mutexImg.lock();

  if( m_imgLast.empty() )
  {
    m_mutexImg.unlock();
    return;
  }

  //
  // Clear previous objects of interest data
  //
  msg.id.clear();
  msg.isa.clear();
  msg.bbox.clear();
  msg.confidence.clear();

  //
  // Set header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Associate image with OoIs
  //
  msg.img_hdr     = m_msgImg->header;
  msg.img_width   = m_msgImg->width;
  msg.img_height  = m_msgImg->height;

  for(size_t i=0; i< m_oois.size(); ++i)
  {
    eyesxing_msgs::Rect r;
    cv::Rect            bbox = m_oois[i].bbox();

    r.x       = bbox.x;
    r.y       = bbox.y;
    r.width   = bbox.width;
    r.height  = bbox.height;

    msg.id.push_back(m_oois[i].id());
    msg.isa.push_back(m_oois[i].isa());
    msg.bbox.push_back(r);
    msg.confidence.push_back(m_oois[i].confidence());
  }

  m_mutexImg.unlock();
}

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void EyesXingFacesDetector::subscribeToTopics(int nQueueDepth)
{
  string  strTopic("image");
  string  strTransport("raw");

  image_transport::ImageTransport it(m_nh);
  image_transport::TransportHints hints(strTransport, ros::TransportHints(),
                                        m_nh);
  m_subImage = it.subscribe(strTopic, 1, &EyesXingFacesDetector::imageCb,
                                          &(*this), hints);
}

void EyesXingFacesDetector::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("Executing image callback.");

  m_mutexImg.lock();

  // may want to view raw bayer data, which CvBridge doesn't know about
  if( msg->encoding.find("bayer") != std::string::npos )
  {
    //ROS_INFO("bayer image");
    m_imgLast = cv::Mat(msg->height, msg->width, CV_8UC1,
                          const_cast<uint8_t*>(&msg->data[0]), msg->step);
  }

  // scale floating point images so that they display nicely
  else if( msg->encoding.find("F") != std::string::npos )
  {
    //ROS_INFO("float image");
    cv::Mat imgFloatBridge = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::Mat_<float> imgFloat = imgFloatBridge;
    float max_val = 0;
    for(int i = 0; i < imgFloat.rows; ++i)
    {
      for(int j = 0; j < imgFloat.cols; ++j)
      {
        max_val = std::max(max_val, imgFloat(i, j));
      }
    }

    if(max_val > 0)
    {
      imgFloat /= max_val;
    }
    m_imgLast = imgFloat;
  }

  // convert to OpenCV native BGR color
  else
  {
    //ROS_INFO("bgr image");
    try
    {
      m_imgLast = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Unable to convert '%s' image to bgr8: '%s'",
                             msg->encoding.c_str(), e.what());
    }
  }

  // m_imgLast may point to data owned by m_msgImg, so we hang onto it for
  // the sake of mouseCb.
  m_msgImg = msg;

  if( m_imgLast.empty() )
  {
    m_oois.clear();
    return;
  }

  vector<EyeOOI>  oois[3];
  size_t          i;

  //
  // Detect Objects of Interest by iterating over detector pipeline, executing
  // each detector's detect() function, and then performing the associated
  // setop() set operation with the upstream OoI set.
  //
  for(i=0; i<m_pipeline.size(); ++i)
  {
    oois[(i+1)%3].clear();
    m_pipeline[i].detect(m_imgLast, oois[(i+1)%3]);
    m_pipeline[i].setop(oois[i%3], oois[(i+1)%3], oois[(i+2)%3]);
  }

  m_oois = oois[(1+2)%3];

  ROS_INFO("%zu objects of interest detected.", m_oois.size());

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  m_mutexImg.unlock();
}

void EyesXingFacesDetector::view(bool bDoWait)
{
  size_t  i;

  if( !m_imgLast.empty() )
  {
    cv::Rect    bbox;
    string      strId;
    cv::Point   pt;

    for(i=0; i<m_oois.size(); ++i)
    {
      strId = m_oois[i].id();
      bbox  = m_oois[i].bbox();

      rectangle(m_imgLast, bbox, CV_RGB(0, 255, 0), 1);

      if( !strId.empty() )
      {
        pt.x = max(bbox.x - 10, 0);
        pt.y = max(bbox.y - 10, 0);
        putText(m_imgLast, strId, pt, cv::FONT_HERSHEY_PLAIN, 1.0,
                  CV_RGB(0,255,0), 2.0);
      }
    }

    cv::imshow("View Detected", m_imgLast);

    if( bDoWait )
    {
      cv::waitKey(10);
    }
  }
}
