#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>
#include <sys/time.h>
#include <thread>

#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "PAL.h"
#include "filters/filter_chain.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace cv;
using namespace PAL;

static const float Pi = 3.1415926535898f;

#define PROPERTIES_FILE_PATH "../catkin_ws/src/dreamvu_pal_navigation/src/SavedPalProperties.txt"
                              

static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub1, stereoleftpub1, stereorightpub1; 

std::vector<PAL::Data::ODOA_Data> data1;

Mat getColorMap(Mat img, float scale)
{
    Mat img_new = img * scale;
    img_new.convertTo(img_new, CV_8UC1);
    img_new = 255-img_new;
    applyColorMap(img_new, img_new, COLORMAP_JET);
    return img_new;
}

void publishimage(cv::Mat imgmat, image_transport::Publisher &pub, string encoding, timeval timestamp)
{
	int type;
	if (encoding == "mono8")
		type = CV_8UC1;
	else if (encoding == "mono16")
		type = CV_16SC1;
	else
		type = CV_8UC3;

    std_msgs::Header header;
    header.stamp.sec = timestamp.tv_sec;
    header.stamp.nsec = timestamp.tv_usec*1000;
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, encoding, imgmat).toImageMsg();

	pub.publish(imgmsg);
}


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "dreamvu_stereo_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	leftpub1 = it.advertise("/dreamvu/pal/odoa/get/left", 1);		
	stereoleftpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/left", 1);		
	stereorightpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/right", 1);				 
	
	int width, height;
    std::vector<int> camera_indexes{5};
    
    PAL::Mode init_mode = PAL::Mode::STEREO;
	
	char path[1024];
	sprintf(path,"/usr/local/bin/data/pal/data%d/",camera_indexes[0]);

	char path2[1024];
	sprintf(path2,"/usr/local/bin/data/pal/data%d/",6);

	PAL::SetPathtoData(path, path2);

	if (PAL::Init(width, height, camera_indexes, &init_mode) != PAL::SUCCESS) //Connect to the PAL Mini camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	std::vector<PAL::Data::Stereo> stereo_data;

	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(PROPERTIES_FILE_PATH, &g_CameraProperties);
	usleep(100000);
	
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL Mini settings from properties file at default location.\n\n"
				 "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in pal_camera_node.cpp and run catkin_make to build the package again.");
		ROS_INFO("Setting default properties to PAL Mini.");

	}

    PAL::SetAPIMode(PAL::API_Mode::STEREO);
    usleep(100000);
	for(int i=0; i<5; i++)
		stereo_data = PAL::GetStereoData();
	
	usleep(100000);
	
	
	ros::Rate loop_rate(20);
	g_bRosOK = ros::ok();
    

    PAL::SetAPIMode(14);		
    stereo_data = PAL::GetStereoData();
    
	while (g_bRosOK)
	{

		int g_emode = 0;
		//Getting no of subscribers for each publisher
		int left1Subnumber = leftpub1.getNumSubscribers();
		int stereoleft1Subnumber = stereoleftpub1.getNumSubscribers();				
		int stereoright1Subnumber = stereorightpub1.getNumSubscribers();

		int subnumber = left1Subnumber+/*laserscan1Subnumber+depth1Subnumber+pointcloudSubnumber+floorSubnumber+*/stereoright1Subnumber+stereoleft1Subnumber;
        bool overlaid1 = false;

		if (stereoleft1Subnumber > 0)
		{
            publishimage(stereo_data[0].stereo_left, stereoleftpub1, "bgr8", stereo_data[0].timestamp);
            g_emode = g_emode | PAL::API_Mode::STEREO;	            	
        }        
		if (stereoright1Subnumber > 0)
		{				
            publishimage(stereo_data[0].stereo_right, stereorightpub1, "bgr8", stereo_data[0].timestamp);	
            g_emode = g_emode | PAL::API_Mode::STEREO;	            	

        }
		if (left1Subnumber > 0 && g_emode!=PAL::API_Mode::STEREO)
		{
            publishimage(data1[0].marked_left, leftpub1, "bgr8", data1[0].timestamp);
            g_emode = g_emode | PAL::API_Mode::RANGE_SCAN;	
        }

		if (subnumber > 0)
		{
			if(g_emode & PAL::API_Mode::STEREO)
			{
				g_emode = PAL::API_Mode::STEREO;
				PAL::SetAPIMode(g_emode);
				stereo_data = PAL::GetStereoData();
			}	
		}

		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}

