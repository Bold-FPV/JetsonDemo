#include <stdio.h>
#include <string>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "cap_gstreamer.hpp"

#include <time.h>
#include <sys/time.h>

#include "vision.h"
#include "networktables/NetworkTable.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

/**
G++ COMMAND TO RUN:
libtool --mode=link g++ -o a.out main.cpp `pkg-config --cflags --libs opencv gstreamer-1.0` -lgstapp-1.0 -lgstriff-1.0 -lgstbase-1.0 -lgstvideo-1.0 -lgstpbutils-1.0
*/

//NetworkTables helper functions
shared_ptr<NetworkTable> mNetworkTable;
string netTableAddress = "192.168.1.35";
void putNumber (const string &name, const double value)
{
	mNetworkTable -> PutNumber (name, value);
}

void FillCircle( cv::Mat img, int rad, cv::Point center )
{
 int thickness = -1;
 int lineType = 8;

 cv::circle( img,
         center,
         rad,
         cv::Scalar( 0, 0, 255 ),
         thickness,
         lineType );
}

string create_read_pipeline (int vid_device, int width, int height, int framerate, bool mjpeg)
{
  char buff[500];

  if (mjpeg) {
    sprintf (buff,
      "v4l2src device=/dev/video%d ! "
      "image/jpeg,format=(string)BGR,width=(int)%d,height=(int)%d,framerate=(fraction)%d/1 ! "
      "jpegdec ! autovideoconvert ! appsink",
      vid_device, width, height, framerate);
  } 
  else {
    sprintf (buff,
      "v4l2src device=/dev/video%d ! "
      "video/x-raw,format=(string)BGR,width=(int)%d,height=(int)%d,framerate=(fraction)%d/1 ! "
      "autovideoconvert ! appsink",
      vid_device, width, height, framerate);
  }

  string pipstring = buff;
  printf ("read string: %s\n", pipstring.c_str());
  return pipstring;
}

string create_read_pipeline_split (
  int vid_device, int width, int height, int framerate, bool mjpeg,
  int bitrate, string ip, int port)
{
  char buff[500];

  if (mjpeg) {
    sprintf (buff,
      "v4l2src device=/dev/video%d ! "
      "image/jpeg,format=(string)BGR,width=(int)%d,height=(int)%d,framerate=(fraction)%d/1 ! jpegdec ! "
      "tee name=split "
        "split. ! queue ! videoconvert ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream !  h264parse ! rtph264pay ! "
          "udpsink host=%s port=%d "
        "split. ! queue ! autovideoconvert ! appsink",
      //"appsink",
      vid_device, width, height, framerate, bitrate, ip.c_str(), port);
  } 
  else {
    sprintf (buff,
      "v4l2src device=/dev/video%d ! "
      "video/x-raw,format=(string)BGR,width=(int)%d,height=(int)%d,framerate=(fraction)%d/1 ! "
      "tee name=split "
        "split. ! queue ! videoconvert ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream !  h264parse ! rtph264pay ! "
          "udpsink host=%s port=%d "
        "split. ! queue ! autovideoconvert ! appsink",
      //"appsink",
      vid_device, width, height, framerate, bitrate, ip.c_str(), port);
  }

  string pipstring = buff;
  printf ("read string: %s\n", pipstring.c_str());
  return pipstring;
}

string create_write_pipeline (int width, int height, int framerate, 
  int bitrate, string ip, int port)
{
  char buff[500];

  sprintf (buff,
    "appsrc ! "
    "video/x-raw, format=(string)BGR, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
    "videoconvert ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream !  h264parse ! rtph264pay ! "
    "udpsink host=%s port=%d",
    width, height, framerate, bitrate, ip.c_str(), port);

   string pipstring = buff;
  
  printf ("write string: %s\n", pipstring.c_str());
  return pipstring;
}

void ClearScreen()
{
  for (int i = 0; i < 500; i++)
  {
    cout << endl;
  }
}

int main ()
{
  srand (time (NULL));


  int 
  device = 0,
  width = 320, 
  height = 240, 
  framerate = 15, 
  mjpeg = false;

  int
  bitrate = 600000,
  port_stream = 5806,
  port_thresh = 5805;

  string ip = "192.168.1.34";


  char setting_script[100];
  sprintf (setting_script, "bash camera_settings.sh %d", device);
  system (setting_script);

  NetworkTable::SetClientMode();
  NetworkTable::SetDSClientEnabled(false);
  NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
	
  NetworkTable::Initialize();
  cout<<"initialized table"<<endl;	
  mNetworkTable = NetworkTable::GetTable("CVResultsTable");

  CvCapture_GStreamer mycam;
  mycam.open (CV_CAP_GSTREAMER_FILE, 
    create_read_pipeline_split (device, width, height, framerate, mjpeg, 
      bitrate, ip, port_stream).c_str());
    //create_read_pipeline (device, width, height, framerate, mjpeg).c_str());
  //mycam.open (CV_CAP_GSTREAMER_FILE, );
  printf ("succesfully opened camera!\n");

  printf ("width: %d, height: %d\n", width, height);

  CvVideoWriter_GStreamer mywriter;
  mywriter.open (
    create_write_pipeline (width, height, framerate, 
      bitrate, ip, port_thresh).c_str(),
    // create_write_pipeline (width, height, framerate, 
    //   600000, "192.168.1.34", 5805).c_str(),
    //"appsrc ! video/x-raw, format=(string)BGR, width=(int)640, height=(int)480 ! videoconvert ! omxh264enc bitrate=600000 ! video/x-h264, stream-format=(string)byte-stream !  h264parse ! rtph264pay ! udpsink host=192.168.1.34 port=5805",
    0, framerate, cv::Size(width, height), true);

  cv::Mat cameraFrame, greenImage;
  cv::Point2d ul (-2, -2), ur(-2, -2), ll(-2, -2), lr(-2, -2);


  //take each frame from the pipeline
  for (long long frame = 0; ; frame++) {
    bool success = mycam.grabFrame();

    printf ("frame: %lld\n", frame);

    if (success)
    {
      IplImage *img = mycam.retrieveFrame(0);
      cameraFrame = cv::cvarrToMat (img);
      greenImage = cameraFrame;
  	  try {
		//process the image, put the information into network tables
        VisionResultsPackage info = calculate(cameraFrame, ul, ur, ll, lr, greenImage);
			putNumber("Hue", info.hsv[0]);
			putNumber("Saturation", info.hsv[1]);
			putNumber("Value", info.hsv[2]);
    		putNumber("CenterX", info.midPoint.x);
    		putNumber("CenterY", info.midPoint.y);
    		putNumber("TopWidth", info.widths[0]);
    		putNumber("BottomWidth", info.widths[1]);
    		putNumber("LeftHeight", info.heights[0]);
    		putNumber("RightHeight", info.heights[1]);
    		putNumber("Frame#", frame);
    		putNumber("UpperLeftX", ul.x);
    		putNumber("UpperLeftY", ul.y);
    		putNumber("UpperRightX", ur.x);
    		putNumber("UpperRightY", ur.y);
    		putNumber("LowerLeftX", ll.x);
    		putNumber("LowerLeftY", ll.y);
    		putNumber("LowerRightX", lr.x);
    		putNumber("LowerRightY", lr.y);
		    mNetworkTable -> Flush();
  	  }
  	  catch(...) {
        //cout << "no contours" << endl;
      }

      // ClearScreen();
      // for (int r = 0; r < 50; r++)
      // {
      //   for (int c = 0; c < 100; c++)
      //   {
      //     printf ("%1d", greenImage.at<uchar>(r, c));
      //   }
      //   printf ("\n");
      // }

      //cout << "GREEN IMG type before conversion: " << greenImage.type() << endl;
      
      //cout << "GREEN IMG type after conversion: " << greenImage.type() << endl;
      //cout << "GREEN IMG type camera image: " << cameraFrame.type() << endl;
      
	  //pass the results back out
	  IplImage outImage = (IplImage) greenImage;
      //outImage depth = 16;

      printf ("depth: %d, nchannels: %d\n", outImage.depth, outImage.nChannels);
      mywriter.writeFrame (&outImage);
    }

    //cv::imshow("cam", cameraFrame);

    if (cv::waitKey(30) >= 0)
    {
      break;
    }

    usleep (10);
  }

  mywriter.close();
  mycam.close();
  return 0;
}
