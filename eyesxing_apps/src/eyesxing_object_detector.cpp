////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eyes Xing
//
// Program:   eyesxing_object_detector   
//
// File:      eyesxing_object_detector.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Object detection in video base on a Cascade Classifier.
 * application.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows and WoundZoom
 * (http://www.RoadNarrows.com)
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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/*!
 * \ingroup eyesxing_apps
 * \defgroup eyesxing_object_detector eyesxing_object_detector
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

const int     VideoDevMajor = 81;             ///< video major device number

static char  *Argv0;                          ///< the command
static char  *OptsVideoDevName = (char *)"/dev/video0";
                                              ///< video device name
static char  *OptsVideoRes = (char *)"";      ///< video resolution
char         *OptsClassifierData = (char *)
          "/opt/share/opencv/haarcascades/haarcascade_frontalface_default.xml";

static int  VideoDeviceId;
static Size VideoRes(0, 0);

/*! 
 * \brief Package information.
 */
static const PkgInfo_T PkgInfo =
{
  "Eyes Xing",
  "1.0.0",
  "2014.05.19",
  "2014.05.19",
  "Eyes Xing",
  "Robin Knight",
  "RoadNarrows",
  "This software is under the MIT license."
};

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Detect trained objects in a video stream and show."

  // long_desc = 
  "The %P application detects trained objects in a video stream.\n"
  "The video device is opened, the trained classifier data set is loaded, and "
  "the video stream is shown in a window along with dectected objects "
  "annotated.",

  // diagnostics
  NULL
};

//
// Forward Declarations
//
static int OptsCvtArgRes(const char *argv0, const char *sOptName,
                         char *optarg, void *pOptVal);

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // --classifier, -c
  {
    "classifier",         // long_opt
    'c',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsClassifierData,  // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<file>",             // arg_name
                          // opt desc
    "Load classifier from file."
  },

  // --resolution, -r
  {
    "resolution",         // long_opt
    'r',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsVideoRes,        // opt_addr
    OptsCvtArgRes,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<wxh>",              // arg_name
                          // opt desc
    "Video device resolution specified as \"<width>x<height>\".\n"
    "Default is current/default camera resolution."
  },

  // --video, -v
  {
    "video",              // long_opt
    'v',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsVideoDevName,    // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<device>",           // arg_name
                          // opt desc
    "Video device name." 
  },

  {NULL, }
};

/*!
 * \brief Convert command-line option to camera resolution size.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \exception OPTSBADARG()
 *
 * \return If function returns, then returns 0.
 */
static int OptsCvtArgRes(const char *argv0, const char *sOptName,
                         char *optarg, void *pOptVal)
{
  string  res(optarg);
  string  w, delim("x"), h;
  size_t  pos;
  Size    sz;

  pos = res.find(delim);
  w = res.substr(0, pos);
  h = res.substr(pos+1, -1);

  if( w.empty() || h.empty() )
  {
    OptsInvalid(Argv0, "'%s': Invalid '%s' argument '<width>x<height>' syntax.",
             optarg, sOptName);
  }

  sz.width  = atoi(w.c_str());
  sz.height = atoi(h.c_str());

  if( (sz.width <= 0) || (sz.height <= 0) )
  {
    OptsInvalid(Argv0, "'%s': Invalid '%s' argument '<width>x<height>' values.",
             optarg, sOptName);
  }

  VideoRes = sz;

  return 0;
}

static int getVideoIndex(const string &strVideoDevName)
{
  struct stat statVid;
  uint_t      uMajor;
  uint_t      uMinor;

  if( strVideoDevName.empty() )
  {
    LOGERROR("No video device name.");
    return RC_ERROR;
  }

  else if( access(strVideoDevName.c_str(), F_OK|R_OK|W_OK) != 0 )
  {
    LOGSYSERROR("%s.", strVideoDevName.c_str());
    return RC_ERROR;
  }

  else if( stat(strVideoDevName.c_str(), &statVid) != 0 )
  {
    LOGSYSERROR("%s.", strVideoDevName.c_str());
    return RC_ERROR;
  }
  
  uMajor = major(statVid.st_rdev);
  uMinor = minor(statVid.st_rdev);

  if( uMajor != VideoDevMajor )
  {
    LOGERROR("%s: Not a video device",
                "Device major number %u != expected number %d.",
        strVideoDevName.c_str(), uMajor, VideoDevMajor);
    return RC_ERROR;
  }

  return (int)uMinor;
}

static Size setCameraResolution(VideoCapture &cap, const Size &res)
{
  Size  actual;

  // set
  if( (res.width > 0) && (res.height > 0) )
  {
    cap.set(CV_CAP_PROP_FRAME_WIDTH,  res.width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, res.height);
  }

  // read back
  actual.width  = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
  actual.height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  return actual;
}

/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( (VideoDeviceId = getVideoIndex(OptsVideoDevName)) < 0 )
  {
    OptsInvalid(Argv0, "'%s': Invalid '%s' argument '<device>'.",
             "<device>", "video");
  }
}

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char* argv[])
{
  char  key;
  Size  res;

  // parse, validate command-line options and argument and initialize app
  mainInit(argc, argv);

  // get a handle to the video device
  VideoCapture cap(VideoDeviceId);

  // check if we can use this device at all
  if( !cap.isOpened() )
  {
    LOGERROR("Capture device %s: cannot open.",  OptsVideoDevName);
    return -1;
  }

  // set resolution
  res = setCameraResolution(cap, VideoRes);

  CascadeClassifier classifier;

  // load trained data
  classifier.load(OptsClassifierData);

  cout << endl;
  cout << "Video device:  " << OptsVideoDevName << " at "
    << res.width << "x" << res.height << endl;
  cout << "Training data: " << OptsClassifierData << endl;
  cout << endl;

  cout << "Press <ESC> in window to terminate." << endl;

  // holds the current frame from the video device
  Mat frame;

  while( true )
  {
    cap >> frame;

    // clone the current frame
    Mat original = frame.clone();

    Mat gray;

    // convert the current frame to grayscale
    cvtColor(original, gray, CV_BGR2GRAY);

    // find the objects in the frame
    vector< Rect_<int> > objects;

    classifier.detectMultiScale(gray, objects);

    // At this point you have the position of the objects in the video.
    for(int i = 0; i < objects.size(); i++)
    {
      // Process face by face:
      Rect obj_i = objects[i];

      // annotate
      rectangle(original, obj_i, CV_RGB(0, 255,0), 1);
    }

    imshow("Eyesxing Object Detector", original);

    key = (char)waitKey(20);

    // ESC
    if( key == 0x1b )
    {
      break;
    }
  }

  return APP_EC_OK;
}

/*!
 * \}
 */
