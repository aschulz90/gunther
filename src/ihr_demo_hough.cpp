/*
 *  OpenCV Demo for ROS
 *  Copyright (C) 2010, I Heart Robotics
 *  I Heart Robotics <iheartrobotics@gmail.com>
 *  http://www.iheartrobotics.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include "marker.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define PI 3.14159265

// ROS/OpenCV HSV Demo
// Based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

class Demo
{

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  cv::Mat img_in_;
  cv::Mat img_hsv_;
  cv::Mat img_hue_;
  cv::Mat img_sat_;
  cv::Mat img_bin_;
  cv::Mat img_out_;
  IplImage *cv_input_;

public:

    ros::Publisher marker_pub;
    bool seen_once;
    std_msgs::Float32 angle;
    std_msgs::Float32MultiArray array;
    Demo (ros::NodeHandle & nh):nh_ (nh), it_ (nh_)
  {
    // Listen for image messages on a topic and setup callback
    image_sub_ = it_.subscribe ("/camera/image_raw", 1, &Demo::imageCallback, this);
    seen_once = false;
    marker_pub = nh.advertise<std_msgs::Float32MultiArray>("marker_data", 1);
    // Open HighGUI Window
    cv::namedWindow ("input", 1);
    cv::namedWindow ("binary image", 1);
    cv::namedWindow ("detected output", 1);
    marker_pub = nh.advertise<std_msgs::Float32MultiArray>("marker_data", 1);
    
  }

  

  void imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr)
  {
    // Convert ROS Imput Image Message to IplImage
    try
    {
      cv_input_ = bridge_.imgMsgToCv (msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR ("CvBridge Input Error");
    }

    // Convert IplImage to cv::Mat
    img_in_ = cv::Mat (cv_input_).clone ();
    // output = input
    img_out_ = img_in_.clone ();
    // Convert Input image from BGR to HSV
    cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);
    // Zero Matrices
    img_hue_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    img_sat_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    img_bin_ = cv::Mat::zeros(img_hsv_.rows, img_hsv_.cols, CV_8U);
    // HSV Channel 0 -> img_hue_ & HSV Channel 1 -> img_sat_
    int from_to[] = { 0,0, 1,1};
    cv::Mat img_split[] = { img_hue_, img_sat_};
    cv::mixChannels(&img_hsv_, 3,img_split,2,from_to,2);

    for(int i = 0; i < img_out_.rows; i++)
    {
      for(int j = 0; j < img_out_.cols; j++)
      {
        // The output pixel is white if the input pixel
        // hue is orange and saturation is reasonable
        if(img_hue_.at<uchar>(i,j) > 0 &&
           img_hue_.at<uchar>(i,j) < 18 &&
           img_sat_.at<uchar>(i,j) > 210) {
          img_bin_.at<uchar>(i,j) = 255;
        } else {
          img_bin_.at<uchar>(i,j) = 0;
        }
      }
    }

    // strel_size is the size of the structuring element
    cv::Size strel_size;
    strel_size.width = 3;
    strel_size.height = 3;
    // Create an elliptical structuring element
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,strel_size);
    // Apply an opening morphological operation using the structing element to the image twice
    cv::morphologyEx(img_bin_,img_bin_,cv::MORPH_OPEN,strel,cv::Point(-1, -1),2);

    // Convert White on Black to Black on White by inverting the image
    cv::bitwise_not(img_bin_,img_bin_);
    // Blur the image to improve detection
    cv::GaussianBlur(img_bin_, img_bin_, cv::Size(7, 7), 2, 2 );

    // See http://opencv.willowgarage.com/documentation/cpp/feature_detection.html?highlight=hough#HoughCircles
    // The vector circles will hold the position and radius of the detected circles
    cv::vector<cv::Vec3f> circles;
    // Detect circles That have a radius between 20 and 400 that are a minimum of 170 pixels apart
    cv::HoughCircles(img_bin_, circles, CV_HOUGH_GRADIENT, 1, 170, 140, 15, 15, 400 );

    for( size_t i = 0; i < circles.size(); i++ )
    {
         // round the floats to an int
         cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         cv::circle( img_out_, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         cv::circle( img_out_, center, radius+1, cv::Scalar(0,0,255), 2, 8, 0 );
         // Debugging Output
         //ROS_INFO("x: %d y: %d r: %d",center.x,center.y, radius);
    }

    image_data pub;
    array.data.clear();
    if (circles.size() == 1) {
    seen_once = true;
    array.data.push_back(cvRound(circles[0][0])); // x - 0
    array.data.push_back(cvRound(circles[0][1])); // y - 1
    array.data.push_back(cvRound(circles[0][2])); // r - 2
    array.data.push_back(asin ( (array.data.at(0) - 320) / (sqrt( pow((array.data.at(0) - 320), 2) + pow((480 - array.data.at(1)), 2) ) ) ) );
    ROS_INFO("x: %d y: %d r: %d w: %f",array.data.at(0),array.data.at(1), array.data.at(2), array.data.at(3)); //winkel - 3
    array.data.push_back(1); // visible - 4
    array.data.push_back(1); //seen_once - 5
    } else {
    array.data.push_back(0);
    array.data.push_back(0);
    array.data.push_back(0);
    array.data.push_back(0);
    array.data.push_back(0);
    if (seen_once) array.data.push_back(1); else array.data.push_back(0);
    }

    marker_pub.publish(array);

    // Display Input image
    cv::imshow ("input", img_in_);
    // Display Binary Image
    cv::imshow ("binary image", img_bin_);
    // Display morphed image
    cv::imshow ("detected output", img_out_);

    // Needed to  keep the HighGUI window open
    cv::waitKey (3);
  }

};


int
main (int argc, char **argv)
{
  // Initialize ROS Node
  ros::init (argc, argv, "ihr_demo1");
  // Start node and create a Node Handle
  ros::NodeHandle nh;
  // Instaniate Demo Object
  Demo d (nh);
  // Spin ...
  ros::spin ();
  // ... until done
  return 0;
}
