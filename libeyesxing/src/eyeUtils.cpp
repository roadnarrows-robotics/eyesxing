////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Eyes Xing Faces ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/eyesxing
//
// Library:   libeyesxing
//
// File:      eyeUtils.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Eyes Xing utilities.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
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
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <errno.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "eyesxing/eyesxing.h"

using namespace std;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

/*!
 * \ingroup libpan_tilt
 * \brief Pan-Tilt Error Code String Table.
 *
 * Table is indexed by pan-tilt error codes (see \ref pt_ecodes). Keep
 * in sync.
 */
static const char *EcodeStrTbl[] =
{
  "Ok",                                     ///< [EYE_OK]

  "Error",                                  ///< [EYE_ECODE_GEN]
  "System error",                           ///< [EYE_ECODE_SYS]
  "Internal error",                         ///< [EYE_ECODE_INTERNAL]
  "Bad value",                              ///< [EYE_ECODE_BAD_VAL]
  "Too big",                                ///< [EYE_ECODE_TOO_BIG]
  "Too small",                              ///< [EYE_ECODE_TOO_SMALL]
  "Value out-of-range",                     ///< [EYE_ECODE_RANGE]
  "Invalid operation",                      ///< [EYE_ECODE_BAD_OP]
  "Operation timed out",                    ///< [EYE_ECODE_TIMEDOUT]
  "Device not found",                       ///< [EYE_ECODE_NO_DEV]
  "No resource available",                  ///< [EYE_ECODE_NO_RSRC]
  "Resource busy",                          ///< [EYE_ECODE_BUSY]
  "Cannot execute",                         ///< [EYE_ECODE_NO_EXEC]
  "Permissions denied",                     ///< [EYE_ECODE_PERM]
  "Bad format",                             ///< [EYE_ECODE_FORMAT]
  "File not found",                         ///< [EYE_ECODE_NO_FILE]
  "XML error",                              ///< [EYE_ECODE_XML]
  "Operation interrupted",                  ///< [EYE_ECODE_INTR]

  "Invalid error code"                      ///< [EYE_ECODE_BADEC]
};


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

const char *eyesxing::getStrError(const int ecode)
{
  int ec = ecode >= 0 ? ecode : -ecode;

  if( ec >= arraysize(EcodeStrTbl) )
  {
    ec = EYE_ECODE_BADEC;
  }

  return EcodeStrTbl[ec];
}

const char *eyesxing::setopstr(const int setop)
{
  switch( setop )
  {
    case SetOpUnion:
      return "union";
    case SetOpIntersection:
      return "intersection";
    case SetOpRelCompliment:
      return "rel_compliment";
    default:
      return "?";
  }
}
