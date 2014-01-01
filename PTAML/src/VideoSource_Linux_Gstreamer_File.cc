/*!
 * This file is free software; you can redistribute it and/or
 * modify as you see fit. It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * To compile and link this file with PTAM, install glib and gstreamer and modify
 * the COMPILEFLAGS and LINKFLAGS in the Makefile as follows:
 *
 * Add the following to COMPILEFLAGS:
 * `pkg-config --cflags glib-2.0` `pkg-config --cflags gstreamer-0.10`
 *
 * Add the following to LINKFLAGS:
 * `pkg-config --libs glib-2.0` `pkg-config --libs gstreamer-0.10`
 *
 * This implementation of the VideoSource class uses gstreamer to create a video
 * decoding pipeline for the file specified in the settings.cfg file or
 * GST_DEFAULT_VIDEO_SOURCE_FILE_NAME, if the GStreamerVideoFilename tag is not
 * present in settings.cfg.
 *
 * @author: James Y. Wilson (www.learningce.com, www.embedded101.com)
 */

#include "VideoSource_Linux_Gstreamer_File.h"

#include <string.h>
#include <glib.h>

#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappbuffer.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

/*!
 * NOTE: The value of the source video file can also be set in the settings.cfg
 * file as shown below.  The default video source file name is defined in the
 * constant GST_DEFAULT_VIDEO_SOURCE_FILE_NAME.
 * ...
 * VideoFilename={the full path of your video source file}
 * ...
 */


/*!
 * Default constructor for the VideoSource class.
 */
VideoSource_Linux_Gstreamer_File::VideoSource_Linux_Gstreamer_File(const std::string& videoSourceFile)
  : mRgbVideoSink(NULL)
  , mGrayVideoSink(NULL)
  , mSourcePipeline(NULL)
  , mFrameNumber(0)
{
  gchar *pipelineString = NULL;

  static bool bInitialized = false;
  static int id = 0;

  /*!
   * Initialize the gstreamer library, without the command-line parms.
   */

  ++id;

  if (!bInitialized) {
    gst_init(NULL, NULL);
    bInitialized = true;
  }

  int videoWidth = GV3::get<int>("VideoSource.Width", 640, HIDDEN);
  int videoHeight = GV3::get<int>("VideoSource.Height", 480, HIDDEN);
  double contrast = GV3::get<double>("VideoSource.Contrast", 1.0, HIDDEN);

  cout << "Contrast: " << contrast << endl;

  mirSize = ImageRef(videoWidth, videoHeight);

  std::stringstream ss; ss << "rgbvideo" << id;
  std::string rgbSinkName = ss.str();
  ss.str(""); ss << "grayvideo" << id;
  std::string bwSinkName = ss.str();
  ss.str(""); ss << "t" << id;
  std::string teeName = ss.str();


  /*!
   * Setup the source pipeline to read from a file, automatically select the proper decoder,
   * convert each frame to two new color spaces (specified later), scale the video (specified later),
   * filter capabilities to RGB and Gray level at 640x480 (used by the previous color space and scale filters),
   * and finally send the result to separate sink filters which will allow access to the resulting
   * RGB and Gray level video data (limited to 2 buffers).
   */
  pipelineString =
      g_strdup_printf(
        "filesrc location=\"%s\" ! "
        "decodebin ! "
        "videobalance contrast=%.1f ! "
        "tee name=%s ! "
        "videoscale ! "
        "ffmpegcolorspace ! "
        "video/x-raw-rgb,width=%d,height=%d,bpp=24,depth=24 ! "
        "queue ! "
        "appsink name=%s max-buffers=2 drop=false %s. ! "
        "ffmpegcolorspace ! "
        "videoscale ! "
        "video/x-raw-gray,width=%d,height=%d ! "
        "queue ! "
        "appsink name=%s max-buffers=2 drop=false",
        videoSourceFile.c_str(),
        contrast,
        teeName.c_str(),
        videoWidth, videoHeight,
        rgbSinkName.c_str(),
        teeName.c_str(),
        videoWidth, videoHeight,
        bwSinkName.c_str());

  g_print("gstreamer pipeline:\n%s\n", pipelineString);
  mSourcePipeline = gst_parse_launch(pipelineString, NULL);
  g_free(pipelineString);

  if (!mSourcePipeline)
  {
    g_print("An error occurred when attempting to create the video source pipeline.\n");
    return;
  }
  /*!
   * Obtain a reference to the appsink element in the pipeline for later use when pulling a buffer.
   */
  mRgbVideoSink = gst_bin_get_by_name(GST_BIN(mSourcePipeline), rgbSinkName.c_str());
  mGrayVideoSink = gst_bin_get_by_name(GST_BIN(mSourcePipeline), bwSinkName.c_str());

  if (!mRgbVideoSink || !mGrayVideoSink)
  {
    g_print("The video sink filters could not be created.\n");
    gst_object_unref(mSourcePipeline);
    mSourcePipeline = NULL;
    return;
  }
  /*!
   * Activate the video source pipeline, so it is ready when we request a buffer.
   */
  gst_element_set_state(mSourcePipeline, GST_STATE_PLAYING);
}

/*!
 * Destructructor for the VideoSource class.
 */
VideoSource_Linux_Gstreamer_File::~VideoSource_Linux_Gstreamer_File()
{
  if (mSourcePipeline)
  {
    gst_element_set_state(mSourcePipeline, GST_STATE_NULL);
    gst_object_unref(mSourcePipeline);
    mSourcePipeline = NULL;
  }
}

/*!
 * Retrieves the next frame from the video source file, as decoded by the video source
 * pipeline.  This function blocks until a frame becomes available in the pipeline,
 * a period of time determined by the frame rate of the original recording.  When
 * all frames have been retrieved from the file, the imBW and imRGB parameters will remain
 * unchanged, and the function will return.
 *
 * @param imBW  Return parameter containing the current video frame decoded in monochrome.
 * @param imRGB Return parameter containing the current video frame decoded in RGB.
 */
void VideoSource_Linux_Gstreamer_File::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  GstBuffer *rgbVideoBuffer = NULL;
  GstBuffer *grayVideoBuffer = NULL;

  /*!
   * Be sure the video sinks are available before requesting a buffer with video
   * data.
   */
  if (mRgbVideoSink != NULL && mGrayVideoSink != NULL)
  {
    rgbVideoBuffer = gst_app_sink_pull_buffer(GST_APP_SINK(mRgbVideoSink));
    grayVideoBuffer = gst_app_sink_pull_buffer(GST_APP_SINK(mGrayVideoSink));

    /*!
     * If either of the buffers are NULL then assume that we have reached the end of
     * the video stream and just return.
     */
    if (rgbVideoBuffer != NULL && grayVideoBuffer != NULL)
    {
      BasicImage<Rgb<byte> > rgbVideoFrame((Rgb<byte> *)rgbVideoBuffer->data, mirSize);
      BasicImage<byte> grayVideoFrame((byte *)grayVideoBuffer->data, mirSize);

      /* Copy the streamed image into caller's params. */
      imRGB.copy_from(rgbVideoFrame);
      imBW.copy_from(grayVideoFrame);

      /* Release the gst buffer since it is already copied to the caller. */
      gst_buffer_unref(rgbVideoBuffer);
      gst_buffer_unref(grayVideoBuffer);

      /* Maintain the running total of frames. */
      mFrameNumber++;
    }
  }
}
}
