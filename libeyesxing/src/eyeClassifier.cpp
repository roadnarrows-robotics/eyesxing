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
 * \brief Classifier class implementation.
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

#include <sys/types.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "eyesxing/eyesxing.h"
#include "eyesxing/eyeOOI.h"
#include "eyesxing/eyeClassifier.h"

using namespace std;
using namespace cv;
using namespace eyesxing;

size_t EyeClassifier::detect(const Mat      &image,
                             vector<EyeOOI> &oois,
                             double         scaleFactor,
                             int            minNeighbors,
                             Size           minSize,
                             Size           maxSize)
{
  if( empty() )
  {
    return 0;
  }

  vector< Rect_<int> >  objects;  // find the objects in the frame
  Mat                   gray;     // image gray-scaled
  size_t                i;        // working index/count

  // convert the current frame to grayscale
  cvtColor(image, gray, CV_BGR2GRAY);

  m_classifier.detectMultiScale(gray, objects, scaleFactor, minNeighbors, 0,
                                  minSize, maxSize);

  for(i=0; i<objects.size(); ++i)
  {
    EyeOOI  ooi(objects[i], m_strName);
    oois.push_back(ooi);
  }

  return i;
}

size_t EyeClassifier::setop(SetOp                     op,
                            const std::vector<EyeOOI> &a,
                            const std::vector<EyeOOI> &b,
                            std::vector<EyeOOI>       &c,
                            double                    similarity)
{
  size_t    i, j;
  bool      bAlike;

  switch(op)
  {
    case SetOpIntersection:
      c.clear();
      for(i=0; i<a.size(); ++i)
      {
        for(j=0; j<b.size(); ++j)
        {
          if( a[i].similarByIntersection(b[j]) >= similarity )
          {
            c.push_back(a[i]);
          }
        }
      }
      break;

    case SetOpRelCompliment:
      c.clear();
      for(i=0; i<a.size(); ++i)
      {
        for(j=0, bAlike=false; j<b.size() && !bAlike; ++j)
        {
          if( a[i].similarByIntersection(b[j]) >= similarity )
          {
            bAlike = true;
          }
        }
        if( !bAlike )
        {
          c.push_back(a[i]);
        }
      }
      break;

    case SetOpUnion:
    default:
      c = a;
      for(j=0; j<b.size(); ++j)
      {
        c.push_back(b[j]);
      }
      break;
  }

  return c.size();
}
