#include "opencv2/opencv.hpp" 
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
    string folderName = argv[1];
    string timeName = folderName + "times.txt";
    string img0Folder = folderName + "image_0/";
    string img1Folder = folderName + "image_1/";
    std::ifstream ifs;
    ifs.open (timeName, std::ifstream::in);

    size_t img_c = 0;
    vector<double> ts;
    vector<string> img0Names;
    vector<string> img1Names;
    double t;
    while(ifs>>t) {
        stringstream ss;
        ss << setw(6) << std::setfill('0') << img_c;
        string img0Name = img0Folder + ss.str() + ".png";
        string img1Name = img1Folder + ss.str() + ".png";

        ts.push_back(t);
        img0Names.push_back(img0Name);
        img1Names.push_back(img1Name);

        img_c++;
    }

    ros::init(argc, argv, "kitti_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_pub = it.advertise("/kitti/left/image_raw_color", 1);
    image_transport::Publisher right_pub = it.advertise("/kitti/right/image_raw_color", 1);

    img_c = 0;
    ros::Rate loop_rate(20);
    while (nh.ok())
    {
        Mat left_frame = imread(img0Names[img_c]);
        Mat right_frame = imread(img1Names[img_c]);

        cvtColor(left_frame, left_frame, CV_RGB2GRAY);
        cvtColor(right_frame, right_frame, CV_RGB2GRAY);

        std_msgs::Header header; 
        header.seq = img_c; 
        header.stamp = ros::Time(ts[img_c]); 

        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(header, "mono8", left_frame).toImageMsg();
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(header, "mono8", right_frame).toImageMsg();

        left_pub.publish(left_msg);
        right_pub.publish(right_msg);
        
        img_c++;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}