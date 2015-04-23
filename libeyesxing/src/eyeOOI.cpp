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
 * \brief Object Of Interest class implementation.
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

#include <string>

#include <opencv2/opencv.hpp>

#include "eyesxing/eyesxing.h"
#include "eyesxing/eyeOOI.h"

using namespace std;
using namespace cv;
using namespace eyesxing;


double EyeOOI::similarByIntersection(const Rect &a) const
{
  double d;

  d = (double)a.area() + (double)m_bbox.area();

  if( d <= 0.0 )
  {
    return 0.0;
  }
  else
  {
    return (2.0 * (double)intersection(m_bbox, a)) / d;
  }
}

int EyeOOI::intersection(const Rect &a, const Rect &b)
{
  cv::Point  a_br = a.br();   // bottom right corner of A
  cv::Point  b_br = b.br();   // bottom right corner of B

  int x_overlap = min(0, min(a_br.x, b_br.x) - max(a.x, b.x));
  int y_overlap = min(0, min(a_br.y, b_br.y) - max(a.y, b.y));

  return x_overlap * y_overlap;
}
