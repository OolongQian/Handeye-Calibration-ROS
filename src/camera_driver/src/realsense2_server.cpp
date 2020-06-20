#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rgbd_srv/rgbd.h"

<<<<<<< HEAD

=======
>>>>>>> upstream/master
using namespace std;
using namespace cv;

sensor_msgs::Image rgb_image;
sensor_msgs::Image depth_image;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_rgb;
	image_transport::Subscriber image_sub_depth;

public:
<<<<<<< HEAD

	ImageConverter()
	: it_(nh_)
	{
    	// Subscrive to input video feed and publish output video feed
		image_sub_rgb = it_.subscribe("/realsense_sr300/ylx/rgb", 1, &ImageConverter::imageCb_rgb, this);
		image_sub_depth = it_.subscribe("/realsense_sr300/ylx/depth", 1, &ImageConverter::imageCb_depth, this);
=======
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_rgb = it_.subscribe("/realsense/rgb",
									  1,
									  &ImageConverter::imageCb_rgb,
									  this);

		image_sub_depth = it_.subscribe("/realsense/depth",
										1,
										&ImageConverter::imageCb_depth,
										this);
>>>>>>> upstream/master
	}

	~ImageConverter()
	{
	}

<<<<<<< HEAD
	void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
	{
		rgb_image = *msg;
	}
	void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
	{  
		depth_image = *msg;
	}

=======
	void imageCb_rgb(const sensor_msgs::ImageConstPtr &msg)
	{
		rgb_image = *msg;
	}
	void imageCb_depth(const sensor_msgs::ImageConstPtr &msg)
	{
		depth_image = *msg;
	}
>>>>>>> upstream/master
};

bool realsense2_server(rgbd_srv::rgbd::Request &req, rgbd_srv::rgbd::Response &res)
{
<<<<<<< HEAD
	if (req.start) 
=======
	if (req.start)
>>>>>>> upstream/master
	{
		res.rgb_image = rgb_image;
		res.depth_image = depth_image;
	}
	ROS_INFO("success");
	return true;
}

<<<<<<< HEAD

int main(int argc, char** argv)
{    

	ros::init(argc, argv, "realsense2_server");
	ImageConverter ic; 
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("realsense2_server", realsense2_server);

	ros::Rate loop_rate(200);
=======
int main(int argc, char **argv)
{

	ros::init(argc, argv, "realsense2_server");
	ImageConverter ic;
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("realsense2_server", realsense2_server);

	ros::Rate loop_rate(60);
>>>>>>> upstream/master
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}
<<<<<<< HEAD

=======
>>>>>>> upstream/master
