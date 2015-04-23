////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// ROS Node:  faces_detector
//
// File:      faces_detector.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS faces_detector node class interface.
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

#ifndef _FACES_DETECTOR_H
#define _FACES_DETECTOR_H

#include <sys/types.h>

#include <string>
#include <map>
#include <vector>

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
#include <boost/thread.hpp>

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

namespace eyesxing
{
  //----------------------------------------------------------------------------
  // DetectorPElem Class
  //----------------------------------------------------------------------------
  /*!
   * \brief Detector pipeline element class.
   */
  class DetectorPElem : public EyeClassifier
  {
  public:
    /*!
     * \brief Default initialization constructor.
     *
     * \parm strName        Object/class tag for this classifier.
     *                      For example: 'face_profiles'.
     * \param op            Set operation with any upstream set of OoIs.
     */
    DetectorPElem(const std::string strName=UNKNOWN_OOI, SetOp op=SetOpUnion) :
        EyeClassifier(strName), m_eSetOp(op)
    {
    }

    /*!
     * \brief Shallow copy constructor.
     *
     * \note Any loaded classifier is not copied.
     */
    DetectorPElem(const DetectorPElem &src) :
        EyeClassifier(src.m_strName), m_eSetOp(src.m_eSetOp)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~DetectorPElem()
    {
    }

    /*!
     * \brief Perfrom the given set operation.
     *
     * C = A op B.
     *
     * An OoI in A is considered to be the same object in B by the degree of
     * their overlapping bounding boxes.
     *
     * \param [in] a      Set A of OoIs.
     * \param [in] b      Set B of OoIs.
     * \param [out] c     Resultant set C of OoIs.
     * \param similarity  Simuiarity qualitatve metric 0.0 - 1.0.
     *
     * \return Returns the number of objects in C.
     */
    virtual size_t setop(const std::vector<EyeOOI> &a,
                         const std::vector<EyeOOI> &b,
                         std::vector<EyeOOI>       &c,
                         double                    similarity=0.9)
    {
      return EyeClassifier::setop(m_eSetOp, a, b, c, similarity);
    }

  protected:
    SetOp m_eSetOp;   ///< set operation bound to this detector pipeline element
  };


  //----------------------------------------------------------------------------
  // EyesXingFacesDetector Class
  //----------------------------------------------------------------------------

  /*!
   * \brief The class embodiment of the pan_tilt_control ROS node.
   */
  class EyesXingFacesDetector
  {
  public:
    /*! map of ROS services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*! pipeline of dectectors type */
    typedef std::vector<DetectorPElem> DetectorPipeline;

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    EyesXingFacesDetector(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~EyesXingFacesDetector();

    /*!
     * \brief Advertise all services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish();

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Update Objects of Interest message from current detector state.
     *
     * \param [out] msg   Objects of Interest message.
     */
    void updateOOIMsg(eyesxing_msgs::OOI &msg);

    /*!
     * \brief Convert seconds to loop counts.
     *
     * \param seconds Seconds.
     *
     * \return Count.
     */
    int countsPerSecond(double seconds)
    {
      return (int)(seconds * m_hz);
    }

    /*!
     * \brief View overlay of objects detected onto processed image frame.
     *
     * Usefull debugging call.
     *
     * \param bDoWait Do [not] call OpenCV waitKey() to force window update.
     *                The waitKey() forces update of the window and is required
     *                somewhere in the processing loop or windowing thread.
     */
    void view(bool bDoWait=true);

  protected:
    ros::NodeHandle            &m_nh;       ///< bound node handler instance
    double                      m_hz;       ///< application nominal loop rate
    boost::mutex                m_mutexImg; ///< image grab mutex
    cv::Mat                     m_imgLast;  ///< last image grabbed
    sensor_msgs::ImageConstPtr  m_msgImg;   ///< last image message
    DetectorPipeline            m_pipeline; ///< pipeline of detectors
    std::vector<EyeOOI>         m_oois;     ///< detected objects of interest

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< pan-tilt control services
    MapPublishers     m_publishers;     ///< pan-tilt control publishers
    MapSubscriptions  m_subscriptions;  ///< pan-tilt control subscriptions
    image_transport::Subscriber m_subImage; ///< image subscriber

    // Messages for published data.
    eyesxing_msgs::OOI    m_msgOOI;     ///< objects of interest message

    //..........................................................................
    // Service callbacks
    //..........................................................................

    /*!
     * \brief Clear object classifiers.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool clearClassifiers(eyesxing_faces::ClearClassifiers::Request  &req,
                          eyesxing_faces::ClearClassifiers::Response &rsp);

    /*!
     * \brief Load object classifier.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool loadClassifier(eyesxing_faces::LoadClassifier::Request  &req,
                        eyesxing_faces::LoadClassifier::Response &rsp);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    /*!
     * \brief Publish objects of interest topic.
     */
    void publishOOI();


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    /*!
     * \brief Execute joint trajectory subscibed topic callback.
     *
     * \param jt  Joint trajectory message.
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    
  };

} // namespace eyesxing


#endif // _FACES_DETECTOR_H
