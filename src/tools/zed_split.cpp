#include "opencv2/opencv.hpp" 
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>

using namespace cv;
int main(int argc, char** argv)
{
    VideoCapture cap(atoi(argv[1])); 
    if(!cap.isOpened()) 
        return -1;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1344);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 376);
    cap.set(cv::CAP_PROP_FPS, 100);

    ros::init(argc, argv, "zed_split");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_pub = it.advertise("/zed/left/image_raw_color", 1);
    image_transport::Publisher right_pub = it.advertise("/zed/right/image_raw_color", 1);

    // namedWindow("frame",1);
    int counter=0;
    ros::Rate loop_rate(20);
    while (nh.ok())
    {
        Mat frame;
        cap >> frame; 
        // imshow("frame", frame);
        // std::cout<<frame.size()<<std::endl;

        Mat left_frame = frame(Rect(0, 0, frame.cols/2, frame.rows));
        Mat right_frame = frame(Rect(frame.cols/2, 0, frame.cols/2, frame.rows));

        cvtColor(left_frame, left_frame, CV_RGB2GRAY);
        cvtColor(right_frame, right_frame, CV_RGB2GRAY);

        std_msgs::Header header; 
        header.seq = counter++; 
        header.stamp = ros::Time::now(); 

        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(header, "mono8", left_frame).toImageMsg();
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(header, "mono8", right_frame).toImageMsg();

        left_pub.publish(left_msg);
        right_pub.publish(right_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}