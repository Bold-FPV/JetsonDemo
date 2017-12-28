#include "helper.hpp"
#include "gst_pipeline.hpp"
#include "GripPipeline.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

shared_ptr<NetworkTable> myNetworkTable; //our networktable for reading/writing
shared_ptr<NetworkTable> myContoursNetworkTable;
string netTableAddress = "192.168.1.130"; //address of the rio

//useful for testing OpenCV drawing to see you can modify an image
void fillCircle (cv::Mat img, int rad, cv::Point center);
void pushToNetworkTables (std::vector<std::vector<cv::Point> >* data);

//GRIP Pipeline
grip::GripPipeline myGrip;

//camera parameters
int 
device = 1, //bc we are using video0 in this case
width = 320, 
height = 240, 
framerate = 15, 
mjpeg = false; //mjpeg is not better than just grabbing a raw image in this case

//network parameters
int
bitrate = 600000, //kbit/sec over network
port_stream = 5806, //destination port for raw image
port_thresh = 5805; //destination port for thresholded image
string ip = "192.168.1.130"; //destination ip

string tableName = "GripResults";
string contourTableName = tableName + "/" + "MyContoursReport";

bool verbose = true;

struct ContourReport {
	i64 timestamp;
	std::vector<cv::Rect> boundingbox;
	std::vector<double> centerY, centerX, width, height, area, solidity;
};

void flash_good_settings() {
    char setting_script[100];
    sprintf (setting_script, "bash good_settings.sh %d", device);
    system (setting_script);
}

void flash_bad_settings() {
    char setting_script[100];
    sprintf (setting_script, "bash bad_settings.sh %d", device);
    system (setting_script);
}

int main () {
    //call the bash script to set camera settings
    flash_good_settings();

    //initialize NetworkTables
    NetworkTable::SetClientMode();
    NetworkTable::SetDSClientEnabled(false);
    NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
    NetworkTable::Initialize();
    if (verbose) printf ("Initialized table\n");
    myNetworkTable = NetworkTable::GetTable(tableName);
    myContoursNetworkTable = NetworkTable::GetTable(contourTableName);

    //open camera using CvCapture_GStreamer class
    CvCapture_GStreamer mycam;
    string read_pipeline = createReadPipelineSplit (
            device, width, height, framerate, mjpeg, 
            bitrate, ip, port_stream);
    if (verbose) {
        printf ("GStreamer read pipeline: %s\n", read_pipeline.c_str()); 
    }
    mycam.open (CV_CAP_GSTREAMER_FILE, read_pipeline.c_str());

    if (verbose) {
        printf ("Succesfully opened camera with dimensions: %dx%d\n",
            width, height);
    }

    //open vidoe writer using CvVideoWriter_GStreamer class
    CvVideoWriter_GStreamer mywriter;
    string write_pipeline = create_write_pipeline (width, height, framerate, 
            bitrate, ip, port_thresh);
    if (verbose) {
        printf ("GStreamer write pipeline: %s\n", write_pipeline.c_str());
    }
    mywriter.open (write_pipeline.c_str(), 
        0, framerate, cv::Size(width, height), true);

    //initialize raw & processed image matrices
    cv::Mat cameraFrame, processedImage;

    //take each frame from the pipeline
    for (long long frame = 0; ; frame++) {
        //have to alternate from bad settings to good settings on some cameras
        //because of weird firmware issues, sometimes the flash doesn't stick 
        //otherwise
        if (frame < 10) {
            flash_bad_settings();
        }
        else if (frame == 50) {
            flash_good_settings();
        }

        bool success = mycam.grabFrame();

        if (verbose) printf ("frame #%lld\n", frame);

        if (success) {
            IplImage *img = mycam.retrieveFrame(0); //store frame in IplImage
            cameraFrame = cv::cvarrToMat (img); //convert IplImage to cv::Mat
            processedImage = cameraFrame;
                
            //process the image, put the information into network tables
		myGrip.Process(cameraFrame);
		pushToNetworkTables(myGrip.GetFilterContoursOutput());
		processedImage = (myGrip.GetHslThresholdOutput()) -> clone();
		cv::cvtColor(processedImage, processedImage, CV_GRAY2BGR);
		IplImage outimage = (IplImage) processedImage;

            //pass the results back out
            mywriter.writeFrame (&outimage); //write output image over network
        }

        //delay for 10 millisecondss
        usleep (10);
    }

    mywriter.close();
    mycam.close();
    return 0;
}

void fillCircle (cv::Mat img, int rad, cv::Point center) {
    int thickness = -1;
    int lineType = 8;
    cv::circle (img, center, rad, cv::Scalar(0, 0, 255), thickness, lineType);
}

void pushToNetworkTables (std::vector<std::vector<cv::Point> >* data) {
std::vector<std::vector<cv::Point> >::const_iterator i;
ContourReport report;
    
myNetworkTable -> PutNumber ("NumContoursFound", data -> size());
for (i=data->begin(); i != data->end(); ++i)
{
	cv::Rect rect = cv::boundingRect(*i);
	double area = cv::contourArea(*i);
	std::vector<cv::Point> hull;
	cv::convexHull(*i, hull,false, true);
	

	report.boundingbox.push_back(rect);
	report.area.push_back(area);
	report.solidity.push_back(area / cv::contourArea(hull));
	

	report.width.push_back(rect.width);
	report.height.push_back(rect.height);
	report.centerX.push_back(rect.x + (rect.width/2.0));
	report.centerY.push_back(rect.y + (rect.height/2.0));
	
}

myContoursNetworkTable -> PutNumberArray ("centerX", report.centerX);
myContoursNetworkTable -> PutNumberArray ("centerY", report.centerY);
myContoursNetworkTable -> PutNumberArray ("width", report.width);
myContoursNetworkTable -> PutNumberArray ("height", report.height);
myContoursNetworkTable -> PutNumberArray ("area", report.area);
myContoursNetworkTable -> PutNumberArray ("solidity", report.solidity);
    myNetworkTable -> Flush();
}

