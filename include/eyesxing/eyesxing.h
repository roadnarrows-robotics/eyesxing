////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// Library:   libeyesxing
//
// File:      eyesxing.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Top-level eyesxing library include file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _EYESXING_H
#define _EYESXING_H

#include <sys/types.h>
#include <sys/time.h>
#include <math.h>

#include <opencv2/opencv.hpp>

#define M_TAU (2.0 * M_PI)    ///< tau = 2 * pi

/*!
 *  \brief The eyesxing namespace encapsulates all Eyes Xing related constructs.
 */
namespace eyesxing
{
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup eyesxing_h
   * \defgroup ey_ecodes  Eyes Xing Error Codes
   *
   * EyesXing package-wide error codes.
   *
   * \{
   */
  static const int EYE_OK               =  0; ///< not an error, success

  static const int EYE_ECODE_GEN        =  1; ///< general, unspecified error
  static const int EYE_ECODE_SYS        =  2; ///< system (errno) error
  static const int EYE_ECODE_INTERNAL   =  3; ///< internal error (bug)
  static const int EYE_ECODE_BAD_VAL    =  4; ///< bad value general error
  static const int EYE_ECODE_TOO_BIG    =  5; ///< value/list/size too big
  static const int EYE_ECODE_TOO_SMALL  =  6; ///< value/list/size too small
  static const int EYE_ECODE_RANGE      =  7; ///< value out-of-range
  static const int EYE_ECODE_BAD_OP     =  8; ///< invalid operation error
  static const int EYE_ECODE_TIMEDOUT   =  9; ///< operation timed out error
  static const int EYE_ECODE_NO_DEV     = 10; ///< device not found error
  static const int EYE_ECODE_NO_RSRC    = 11; ///< no resource available error
  static const int EYE_ECODE_BUSY       = 12; ///< resource busy error
  static const int EYE_ECODE_NO_EXEC    = 13; ///< cannot execute error
  static const int EYE_ECODE_PERM       = 14; ///< no permissions error
  static const int EYE_ECODE_FORMAT     = 15; ///< bad format
  static const int EYE_ECODE_NO_FILE    = 16; ///< file not found
  static const int EYE_ECODE_XML        = 17; ///< XML error
  static const int EYE_ECODE_INTR       = 18; ///< operation interrupted

  static const int EYE_ECODE_BADEC      = 19; ///< bad error code

  static const int EYE_ECODE_NUMOF      = 20; ///< number of error codes
  /*! \} */


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup eyesxing_h
   * \defgroup ey_types  Eyes Xing Types
   *
   * \{
   */

  /*!
   * \brief Set operations.
   */
  enum SetOp
  {
    SetOpUnion          = 0,  ///< union of A and B
    SetOpIntersection   = 1,  ///< intersection A and B
    SetOpRelCompliment  = 2   ///< A but not B
  };

  /*! \} */


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup eyesxing_h
   * \defgroup ey_utils  Eyes Xing Utilities
   *
   * \{
   */

  /*!
   * \brief Get the error string describing the pan-tilt error code.
   *
   * The absolute value of the error code is taken prior retrieving the string.
   * An unknown or out-of-range error code will be mapped to
   * \ref PT_ECODE_BADEC.
   *
   * \param  ecode  Instance of \ref pt_ecodes.
   *
   * \return Returns the appropriate error code string.
   */
  extern const char *getStrError(const int ecode);

  /*!
   * \brief Convert degrees to radians
   *
   * \param d   Degrees.
   *
   * \return Radians.
   */
  inline double degToRad(double d)
  {
    return d / 360.0 * M_TAU;
  }

  /*!
   * \brief Convert radians to degrees
   *
   * \param r   Radians.
   *
   * \return Degrees.
   */
  inline double radToDeg(double r)
  {
    return r / M_TAU * 360.0;
  }

  /*!
   * \brief Integer absolute value.
   *
   * \param a   Integer value.
   *
   * \return |a|
   */
  inline int abs(int a)
  {
    return a>=0? a: -a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline double cap(int a, int min, int max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline double cap(double a, double min, double max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Find minimum value.
   *
   * \param a     Value a
   * \param b     Value b.
   *
   * \return min(a,b)
   */
  inline int min(int a, int b)
  {
    return a<=b? a: b;
  }

  /*!
   * \brief Find minimum value.
   *
   * \param a     Value b.
   * \param b     Value b.
   *
   * \return min(a,b)
   */
  inline double min(double a, double b)
  {
    return a<=b? a: b;
  }

  /*!
   * \brief Find maximum value.
   *
   * \param a     Value a
   * \param b     Value b.
   *
   * \return max(a,b)
   */
  inline int max(int a, int b)
  {
    return a>=b? a: b;
  }

  /*!
   * \brief Find maximum value.
   *
   * \param a     Value b.
   * \param b     Value b.
   *
   * \return max(a,b)
   */
  inline double max(double a, double b)
  {
    return a>=b? a: b;
  }

  /*!
   * \brief Boolean to string.
   *
   * \param b   Boolean value.
   *
   * \return Pointer to null-terminated constant character string.
   */
  inline const char *boolstr(bool b)
  {
    return b? "true": "false";
  }

  /*!
   * \brief Compare operator to test if left hand side time is earlier than
   * the right hand side time.
   *
   * \term lhs \h_lt rhs \<==\> lhs is an earlier time than rhs.
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  inline bool operator<(const struct timeval& lhs, const struct timeval& rhs)
  {
    if( lhs.tv_sec < rhs.tv_sec )
    {
      return true;
    }
    else if( (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec < rhs.tv_usec) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /*!
   * \brief Compare operator to test if left hand side time equals
   * the right hand side time.
   *
   * \term lhs == rhs \<==\> lhs time is the same time as the rhs.
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  inline bool operator==(const struct timeval& lhs, const struct timeval& rhs)
  {
    return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec == rhs.tv_usec)?
              true: false;
  }

  /*!
   * \brief Find the center of a non-rotated rectangle.
   *
   * \param r   Rectangle R.
   *
   * \return Center point.
   */
  inline cv::Point center(const cv::Rect r)
  {
    return cv::Point(r.x+r.width/2, r.y+r.height/2);
  }

  /*!
   * \brief Set operation to string.
   *
   * \param setop   One of the defined set operations.
   *
   * \return String.
   */
  extern const char *setopstr(const int setop);

  /*! \} */
} // namespace eyesxing


#endif // _EYESXING_H
