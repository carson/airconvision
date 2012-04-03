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

#include <string.h>
#include <glib.h>
#include "VideoSource.h"

#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappbuffer.h>



extern std::string videoSourceFileName;
extern int videoSourceSizeHeight;
extern int videoSourceSizeWidth;

namespace PTAMM{
  using namespace CVD;
  using namespace std;
  using namespace GVars3;
  
  /*!
   * NOTE: The value of the source video file can also be set in the settings.cfg
   * file as shown below.  The default video source file name is defined in the
   * constant GST_DEFAULT_VIDEO_SOURCE_FILE_NAME.
   * ...
   * GStreamerVideoFilename={the full path of your video source file}
   * ...
   */
  
  static const char GST_DEFAULT_VIDEO_SOURCE_FILE_NAME[] = "movie/example.avi";
  static const int GST_MAX_WIDTH = 640;
  static const int GST_MAX_HEIGHT = 480;
/*  
static const int GST_MAX_WIDTH = 1920;
static const int GST_MAX_HEIGHT = 1080;
*/

  static GstElement *mRgbVideoSink;
  static GstElement *mGrayVideoSink;
  static GstElement *mSourcePipeline;
  static double mFrameNumber;
  //static ImageRef mFrameSize(videoSourceSizeWidth, videoSourceSizeHeight);
  static ImageRef mFrameSize(GST_MAX_WIDTH, GST_MAX_HEIGHT);
  
  /*!
   * Default constructor for the VideoSource class.
   */
  VideoSource::VideoSource()
  {
    gchar *pipelineString = NULL;
    mFrameNumber = 0;
    mRgbVideoSink = mGrayVideoSink = NULL;
    
    /*!
     * Retrieve the name of the video source file from settings.cfg.
     */
    string videoSourceFile="";
    if(videoSourceFileName==""){
      videoSourceFile = GVars3::GV2.GetString("GStreamerVideoFilename",
					      GST_DEFAULT_VIDEO_SOURCE_FILE_NAME);
    }
    else{
      videoSourceFile = GVars3::GV2.GetString("GStreamerVideoFilename",
					      videoSourceFileName);
    }
     /*!
      * Initialize the gstreamer library, without the command-line parms.
      */
    gst_init(NULL, NULL);

    /*!
     * Setup the source pipeline to read from a file, automatically select the proper decoder,
     * convert each frame to two new color spaces (specified later), scale the video (specified later),
     * filter capabilities to RGB and Gray level at 640x480 (used by the previous color space and scale filters),
     * and finally send the result to separate sink filters which will allow access to the resulting
     * RGB and Gray level video data (limited to 2 buffers).
     */
    // for firefly gst-launch -v dc1394src ! video/x-raw-gray, width=640, height=480, framerate=60/1 ! ffmpegcolorspace ! jpegenc ! filesink location=lapse1.mjpeg
    pipelineString =
        g_strdup_printf(
			//"filesrc location=\"%s\" ! "
			"-v dc1394src ! "
			"tee ! "
			"ffmpegcolorspace ! "
			"videoscale ! "			
			"video/x-raw-rgb,width=%d,height=%d,framerate=60/1 ! " 
			"queue ! "
			"appsink name=rgbvideo max-buffers=2 drop=false tee0. ! "
			"ffmpegcolorspace ! "
			"videoscale ! "
			"video/x-raw-gray,width=%d,height=%d,framerate=60/1 ! "
			"queue ! "
			"appsink name=grayvideo max-buffers=2 drop=false",
			//videoSourceFile.c_str(), 
			//videoSourceSizeWidth,videoSourceSizeHeight,
			//videoSourceSizeWidth,videoSourceSizeHeight);
	    GST_MAX_WIDTH, GST_MAX_HEIGHT, GST_MAX_WIDTH, GST_MAX_HEIGHT);
    g_print("gstreamer pipeline:\n%s\n", pipelineString);
    mSourcePipeline = gst_parse_launch(pipelineString, NULL);
    g_free(pipelineString);

    if (mSourcePipeline == NULL)
    {
        g_print("An error occurred when attempting to create the video source pipeline.\n");
        return;
    }
    /*!
     * Obtain a reference to the appsink element in the pipeline for later use when pulling a buffer.
     */
    mRgbVideoSink = gst_bin_get_by_name(GST_BIN(mSourcePipeline), "rgbvideo");
    mGrayVideoSink = gst_bin_get_by_name(GST_BIN(mSourcePipeline), "grayvideo");

    if (mRgbVideoSink == NULL || mGrayVideoSink == NULL)
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
 *//*`
VideoSource::~VideoSource()
{
    if (mSourcePipeline != NULL)
    {
        gst_element_set_state(mSourcePipeline, GST_STATE_NULL);
        gst_object_unref(mSourcePipeline);
        mSourcePipeline = NULL;
    }
}*/

/*!
 * Returns the size of the video frames generated by the video decode pipeline,
 * as specified in the GST_MAX_WIDTH and GST_MAX_HEIGHT constants.
 *
 * @return ImageRef object containing the width and height in the x and y member
 * functions.
 */
ImageRef VideoSource::Size()
{ 
    return mFrameSize;
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
void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
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
            BasicImage<Rgb<byte> > rgbVideoFrame((Rgb<byte> *)rgbVideoBuffer->data, mFrameSize);
            BasicImage<byte> grayVideoFrame((byte *)grayVideoBuffer->data, mFrameSize);

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
