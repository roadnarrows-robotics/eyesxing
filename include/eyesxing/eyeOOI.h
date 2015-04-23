////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// Library:   libeyesxing
//
// File:      eyeOOI.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Object Of Interest class interface.
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

#ifndef _EYE_OOI_H
#define _EYE_OOI_H

#include <string>

#include <opencv2/opencv.hpp>

#include "eyesxing/eyesxing.h"

namespace eyesxing
{
  static const std::string  UNKNOWN_OOI("?");

  /*!
   * \brief Object Of Interest class.
   */
  class EyeOOI
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EyeOOI() :
      m_bbox(0,0,0,0), 
      m_strId(UNKNOWN_OOI),
      m_strIsA(UNKNOWN_OOI),
      m_fConfidence(0.0)
    {
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param bbox        OoI non-rotated bounding box.
     * \param strId       OoI classified or identification string.
     * \param strIsA      OoI is a type string.
     * \param fConfidence OoI identification confidence 0.0 - 1.0.
     */
    EyeOOI(
        const cv::Rect    &bbox,
        const std::string &strId=UNKNOWN_OOI,
        const std::string &strIsA=UNKNOWN_OOI,
        const double      fConfidence=0.0) :
            m_bbox(bbox),
            m_strId(strId),
            m_strIsA(strIsA),
            m_fConfidence(fConfidence)
    {
    }

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    EyeOOI(const EyeOOI &src) :
            m_bbox(src.m_bbox),
            m_strId(src.m_strId),
            m_strIsA(src.m_strIsA),
            m_fConfidence(src.m_fConfidence)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EyeOOI()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    EyeOOI operator=(const EyeOOI &rhs)
    {
      m_bbox        = rhs.m_bbox;
      m_strId       = rhs.m_strId;
      m_strIsA      = rhs.m_strIsA;
      m_fConfidence = rhs.m_fConfidence;

      return *this;
    }
    
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    
    /*!
     * \brief Get this Object Of Interest's bounding box.
     *
     * \return Rect bounding box.
     */
    cv::Rect bbox() const
    {
      return m_bbox;
    }

    /*!
     * \brief Set this Object Of Interest's bounding box.
     *
     * \param bbox  OoI non-rotated bounding box.
     */
    void bbox(const cv::Rect &bbox)
    {
      m_bbox = bbox;
    }
    
    /*!
     * \brief Get this Object Of Interest's identification.
     *
     * \return String id.
     */
    std::string id() const
    {
      return m_strId;
    }

    /*!
     * \brief Set this Object Of Interest's identification.
     *
     * \param strId   OoI identification string.
     */
    void id(const std::string &strId)
    {
      m_strId = strId;
    }
    
    /*!
     * \brief Get this Object Of Interest's is-a identification.
     *
     * \return String id.
     */
    std::string isa() const
    {
      return m_strIsA;
    }

    /*!
     * \brief Set this Object Of Interest's is-a identification.
     *
     * \param strId   OoI identification string.
     */
    void isa(const std::string &strIsA)
    {
      m_strIsA = strIsA;
    }
    
    /*!
     * \brief Get this Object Of Interest's confidence.
     *
     * \return Confidence value 0.0 - 1.0.
     */
    double confidence() const
    {
      return m_fConfidence;
    }

    /*!
     * \brief Set this Object Of Interest's identification.
     *
     * \param fConfidence OoI confidence 0.0 - 1.0.
     */
    void confidence(const double fConfidence)
    {
      m_fConfidence = cap(fConfidence, 0.0, 1.0);
    }
    
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Operations
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Calculate the similarity of this and the given OoI by the amount
     * of their respective overlapping bounding boxes.
     *
     * \param ooi   Object of Interset to compare against.
     *
     * \return Return simuiarity qualitatve metric 0.0 - 1.0.
     */
    double similarByIntersection(const EyeOOI &ooi) const
    {
      return similarByIntersection(ooi.bbox());
    }

    /*!
     * \brief Calculate the similarity between this OoI'bounding box to the
     * given rectangle.
     *
     * Simularity is determined by the reletive amount of intersection.
     *
     * If B is this OOI's bounding box, and C is the intersecting rectangle
     * formed from A and B, then:\n
     *
     * similarity = (2 * area(C))/(area(A) + area(B))
     *
     * \param a   Non-rotated rectangle A.
     *
     * \return Return similarity qualitatve metric 0.0 - 1.0.
     */
    virtual double similarByIntersection(const cv::Rect &a) const;

    /*!
     * \brief Calculate the area, in pixels, of the intersection of rectangles
     * A and B.
     *
     * \param a   Non-rotated rectangle A.
     * \param b   Non-rotated rectangle B.
     *
     * \return Area.
     */
    static int intersection(const cv::Rect &a, const cv::Rect &b);

  protected:
    cv::Rect    m_bbox;         ///< ooi non-rotated bounding box
    std::string m_strId;        ///< ooi classified or recognition string
    std::string m_strIsA;       ///< ooi 'is a type of' string
    double      m_fConfidence;  ///< ooi identification confidence 0.0 - 1.0 
  };

} // namespace eyexing


#endif // _EYE_OOI_H
