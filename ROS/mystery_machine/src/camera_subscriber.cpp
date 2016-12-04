#include <string>
#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc_imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
 
static const std::string raw_window = "Raw";
static const std::string filtered_window = "Filtered";
static const std::string contoured_window = "Contoured";
 
image_transport::Publisher pub;

int blur = 1;
int lowerH = 0;
int upperH = 23;
int lowerS = 165;
int upperS = 256;
int lowerV = 203;
int upperV = 256;

void callback(const sensor_msgs::ImageConstPtr& original_image)
{
    // Convert from ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    cv::medianBlur(cv_ptr->image, cv_ptr->image, blur*2+1);

    // Apply HSV filter
    cv::Mat img_mask, img_hsv;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(lowerH,lowerS,lowerV),cv::Scalar(upperH,upperS,upperV),img_mask);
    cv::imshow(filtered_window, img_mask);
    
    // Find contours and create bounded rectangles
    cv::findContours(img_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f>center( contours.size() );
    std::vector<float>radius( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
      cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
      boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
    }
    for( size_t i = 0; i< contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar(255, 0, 0);
      cv::rectangle(cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), color, 1, 8, 0 );
    }
    cv::imshow(raw_window, cv_ptr->image);

    cv::waitKey(3);

    // Convert from OpenCV image to ROS image message and publish
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    const std::string control_window = "Filter Controls";

    cv::namedWindow(control_window);
    cv::createTrackbar("blur", control_window, &blur, 10, NULL);
    cv::createTrackbar("lowerH", control_window, &lowerH, 180, NULL);
    cv::createTrackbar("upperH", control_window, &upperH, 180, NULL);
    cv::createTrackbar("lowerS", control_window, &lowerS, 256, NULL);
    cv::createTrackbar("upperS", control_window, &upperS, 256, NULL);
    cv::createTrackbar("lowerV", control_window, &lowerV, 256, NULL);
    cv::createTrackbar("upperV", control_window, &upperV, 256, NULL);

    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, callback);

    //    cv::destroyWindow(main_window);

    ros::spin();
}
