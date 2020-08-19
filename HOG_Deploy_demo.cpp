#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;
using namespace std::chrono;

std_msgs::String msg;
std::stringstream sign_speed;
ros::Publisher chatter_pub;

int counter_stop = 0;
int counter_120 = 0;
int counter_80 = 0;
int counter_40 = 0;
int limit = 20;
string last_sign;

void detectorSigns();

void detectorSigns()
{
	cout << "Running trained detector..." << endl;

	HOGDescriptor hogSigns;
	HOGDescriptor hogStop;
	HOGDescriptor hog120;
	HOGDescriptor hog80;
	hogSigns.load("./models/signs.yml");
	hogStop.load("./models/stop.yml");
	hog120.load("./models/120.yml");
	hog80.load("./models/80.yml");
	int counters[4];
	int largest_counter = 0;
	int this_sign; 

	VideoCapture cap("/video/video.m4v"); // Open camera
	if (!cap.isOpened())
	{
		cout << "Error opening camera or video." << endl;
	}

	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CAP_PROP_FPS, 60);


	// Initiate video writer
	VideoWriter video("outcpp.mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(1280, 720));

	int delay = 0;

	high_resolution_clock::time_point counter = high_resolution_clock::now();

	// Loop through the frames
	for (size_t i = 0;; i++)
	{
		// Timer start for FPS calculation
		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		Mat full_frame;
		Mat cropped;
		Mat sign;
		// These are switched to true when their signs are detected
		bool isStop = false;
		bool isOneTwenty = false;
		bool isEighty = false;
		bool isFourty = false;

		cap >> full_frame;
		delay = 1;

		// Marks the end of the video
		if (full_frame.empty())
		{
			cout << "End of video.";
			return;
		}

		// Bounding box top left coordinates
		int boundingBoxX;												
		int boundingBoxY;

		// Crop the frame to that will be processed
		full_frame(Rect(Point(640, 120), Point(1280, 480))).copyTo(cropped);

		vector<Rect> detectionsSigns;
		vector<Rect> detectionsStop;
		vector<Rect> detections120;
		vector<Rect> detections80;
		vector<double> foundWeightsSigns;
		vector<double> foundWeightsStop;
		vector<double> foundWeights120;
		vector<double> foundWeights80;

		// Run the detector on the cropped frame
		hogSigns.detectMultiScale(cropped, detectionsSigns, foundWeightsSigns);

		// For all signs that are detected
		for (size_t j = 0; j < detectionsSigns.size(); j++)
		{
			Scalar color = Scalar(0, foundWeightsSigns[j] *foundWeightsSigns[j] *200, 0);

			// Saved and used for text labels later
			boundingBoxX = detectionsSigns[j].tl().x;
			boundingBoxY = detectionsSigns[j].tl().y;

			// Draw a bounding box with offset to appear on the full frame
			rectangle(full_frame,
			{
				detectionsSigns[j].tl().x + 640, detectionsSigns[j].tl().y + 120
			},
			{
				detectionsSigns[j].br().x + 640, detectionsSigns[j].br().y + 120
			}, color, cropped.cols / 400 + 1);

			// Crop out the detected sign and save it into sign
			cropped(Rect(Point(detectionsSigns[j].tl().x, detectionsSigns[j].tl().y), Point(detectionsSigns[j].br().x, detectionsSigns[j].br().y))).copyTo(sign);

			// Run the stop detector on the sign
			hog80.detectMultiScale(sign, detections80, foundWeights80);

			// Run all the detectors on the frame
			if (detections80.size())
			{
				isEighty = true;
			}
			else
			{
				hog120.detectMultiScale(sign, detections120, foundWeights120);
				if (detections120.size())
				{
					isOneTwenty = true;
				}
				else
				{
					hogStop.detectMultiScale(sign, detectionsStop, foundWeightsStop);
					if (detectionsStop.size())
					{
						isStop = true;
					}
					else // is 40
					{
						isFourty = true;
					}
				}
			}
		}

		// Part of the algorithm to decide which sign it is
		counter_stop--;
		counter_120--;
		counter_80--;
		counter_40--;

		// For every frame: detection -> +2
		if (isStop){
			counter_stop = counter_stop + 2;
			isStop = false;
		}
		if (isOneTwenty){
			counter_120 = counter_120 + 2;
			isOneTwenty = false;
		}
		if (isEighty){
			counter_80 = counter_80 + 2;
			isEighty = false;
		}
		if (isFourty){
			counter_40 = counter_40 + 2;
			isFourty = false;
		}

		// Fix negative counters
		if (counter_stop < 0)
			counter_stop = 0;
		if (counter_120 < 0)
			counter_120 = 0;
		if (counter_80 < 0)
			counter_80 = 0;
		if (counter_40 < 0)
			counter_40 = 0;

		// Text log
		// cout << "Stop counter: " << counter_stop << " | 120 counter: " << counter_120 << " | 80 counter: " << counter_80 << " | 40 counter: "  << counter_40 << endl;

		// If a counter has surpassed the limit, then there is a sign
		if (counter_stop > limit || counter_120 > limit || counter_80 > limit || counter_40 > limit)
		{
			counters[0] = counter_stop;
			counters[1] = counter_120;
			counters[2] = counter_80;
			counters[3] = counter_40;
			// Determine the sign type based on the largest counter
			for(int i = 0; i < 4; i++){
				if(largest_counter < counters[i]){
					largest_counter = counters[i];
					this_sign = i;
				}
			}
			if(this_sign == 0){
				ROS_INFO("%s", "0");
				sign_speed.str("0");
				last_sign = "0";
			}
			else if(this_sign == 1){
				ROS_INFO("%s", "120");
				sign_speed.str("120");
				last_sign = "120";
			}
			else if(this_sign == 2){
				ROS_INFO("%s", "80");
				sign_speed.str("80");
				last_sign = "80";
			}
			else if(this_sign == 3){
				ROS_INFO("%s", "40");
				sign_speed.str("40");
				last_sign = "40";
			}
			largest_counter = 0;
		}
		msg.data = sign_speed.str();
		chatter_pub.publish(msg);
		cv::putText(full_frame,
		"Last seen sign:",
		cv::Point(10, 50),	// Coordinates
		cv::FONT_HERSHEY_TRIPLEX,	// Font
		2,	// Scale. 2.0 = 2x bigger
		cv::Scalar(255, 0, 255));

		cv::putText(full_frame,
		last_sign,
		cv::Point(10, 100),	// Coordinates
		cv::FONT_HERSHEY_TRIPLEX,	// Font
		2,	// Scale. 2.0 = 2x bigger
		cv::Scalar(255, 0, 255));
		// Timer calculations and output
		cout << std::endl;
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		duration<double> time_span = duration_cast<duration < double>> (t2 - t1);
		cout << "FPS: " << 1/time_span.count() << endl;

		// Write frame to video
		video.write(full_frame);
		// Show frame
		imshow("Preview", full_frame);

		if (waitKey(delay) == 27)
		{
			destroyAllWindows();
			return;
		}
	}

	cap.release();
	video.release();

	// Close all windows
	destroyAllWindows();
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "pi4");
	ros::NodeHandle n;
	ros::Rate poll_rate(1000);
	chatter_pub = n.advertise<std_msgs::String>("speed", 1000);

	// Run the detector
	detectorSigns();

	return 0;
}
