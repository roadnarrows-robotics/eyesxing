////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// Library:   libeyesxing
//
// File:      eyeClassifier.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Classifier class interface.
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

#ifndef _EYE_CLASSIFIER_H
#define _EYE_CLASSIFIER_H

#include <sys/types.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "eyesxing/eyesxing.h"
#include "eyesxing/eyeOOI.h"

namespace eyesxing
{
  /*!
   * \brief Thin wrapper class around CascadeClassifier.
   */
  class EyeClassifier
  {
  public:
    /*!
     * \brief Default constructor.
     *
     * \parm strName    Object/class tag for this classifier.
     *                  For example: 'dogs'.
     */
    EyeClassifier(const std::string strName=UNKNOWN_OOI) :
        m_strName(strName)
    {
    }

    /*!
     * \brief Initialization constructor.
     *
     * \parm strName        Object/class tag for this classifier.
     *                      For example: 'face_profiles'.
     * \param strFileName   Name of file to load. The file may contain the old
     *                      HAAR classifer or a new cascade classifier trained
     *                      by traincascade.
     */
    EyeClassifier(const std::string strName,
                  const std::string &strFileName) :
        m_strName(strName)
    {
      load(strFileName);
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EyeClassifier()
    {
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Check if a classifier is loaded.
     *
     * \return Returns true or false.
     */
    bool empty() const
    {
      return m_classifier.empty();
    }

    /*!
     * \brief Get the object/class tag for this classifier.
     *
     * \return String.
     */
    std::string name()
    {
      return m_strName;
    }

    /*!
     * \brief Set the object/class tag for this classifier.
     *
     * \parm strName    Object/class tag for this classifier.
     *                  For example: 'frowning_cats'.
     */
    void name(const std::string &strName)
    {
      m_strName = strName;
    }

    /*!
     * \brief Get the underlining cascade classifier.
     *
     * \return Reference to classifier.
     */
    cv::CascadeClassifier &classifier()
    {
      return m_classifier;
    }
    
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Operations
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Load classifier from the given file.
     *
     * \param strFileName   Name of file to load. The file may contain the old
     *                      HAAR classifer or a new cascade classifier trained
     *                      by traincascade.
     *
     * \return Return 0 on success, \<0 on failure.
     */
    int load(const std::string &strFileName)
    {
      return m_classifier.load(strFileName)? EYE_OK: -EYE_ECODE_NO_EXEC;
    }
    
    /*!
     * \brief Detect objects of different sizes in the input image.
     *
     * \param [in] image    Input image. The image is converted to grayscale
     *                      before classifying objects.
     * \param [out] oois    Vector of Objects of Interests.
     * \param scaleFactor   Image size reduction at search iteration.
     * \param minNeighbors  Minimum candidate neighbors requried to retain.
     * \param minSize       Minimum object size. Objects less than minimum
     *                      are ignored.
     * \param maxSize       Maximum object size. Objects greater than maximum
     *                      are ignored.
     *
     * \return Returns the number of objects detected.
     */
    size_t detect(const cv::Mat       &image,
                  std::vector<EyeOOI> &oois,
                  double              scaleFactor=1.1,
                  int                 minNeighbors=3,
                  cv::Size            minSize=cv::Size(),
                  cv::Size            maxSize=cv::Size());

    /*!
     * \brief Perfrom the given set operation.
     *
     * C = A op B.
     *
     * An OoI in A is considered to be the same object in B by the degree of
     * their overlapping bounding boxes.
     *
     * \param op          Set operation.
     * \param [in] a      Set A of OoIs.
     * \param [in] b      Set B of OoIs.
     * \param [out] c     Resultant set C of OoIs.
     * \param similarity  Simuiarity qualitatve metric 0.0 - 1.0.
     *
     * \return Returns the number of objects in C.
     */
    virtual size_t setop(SetOp                     op,
                         const std::vector<EyeOOI> &a,
                         const std::vector<EyeOOI> &b,
                         std::vector<EyeOOI>       &c,
                         double                    similarity=0.9);

  protected:
    std::string           m_strName;      ///< object/class tag name
    cv::CascadeClassifier m_classifier;   ///< cascade classifier
  };

} // namespace eyexing


#endif // _EYE_CLASSIFIER_H
