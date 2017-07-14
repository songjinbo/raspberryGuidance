/***********************************************************
Author:Song Jinbo
Date:2017-04-05
Description:1:get image info combined with GPS and attitude without 
	    time_stamp correction
	    2:save data in raspberry
************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include "DJI_guidance.h"
#include "DJI_utility.h"
#include <cmath> 
#include <sstream>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

using namespace cv;
using namespace std;

DJI_event g_event;
DJI_lock g_lock;

string itos(double i)
{
	stringstream ss;

	ss << i;

	return ss.str();
}

#define Thres  50 //threshold for GPS and attitude
#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)
#define RETURN_IF_ERR(err_code) { if( err_code ){ printf( "USB error code:%d in file %s %d\n", err_code, __FILE__, __LINE__ );}}
#define RELEASE_IF_ERR(err_code) { if( err_code ){ release_transfer(); printf( "USB error code:%d in file %s %d\n", err_code, __FILE__, __LINE__ );}}

#define GPS_Queue_MAX 2000//the maximum of  GPS queue
#define Image_Queue_MAX 10//the maximum of attitude queue

//the GPS structure
struct gps_data
{
	float gps_x; //gps data
	float gps_y;
	float gps_z;
	bool gps_status;//the gps validity
	int gps_time_stamp;//time stamp of GPS
};

struct attitude_data
{
	int attitude_time_stamp;//time stamp of attitude

	float pitch;//pitch angle
	float roll;//roll angle
	float yaw;//yaw angle
};

//parameter
e_vbus_index selected_vbus = e_vbus1;  // select front vbus
e_image_data_frequecy frequency = e_frequecy_10;//set the frequency of image
string path = "../data/"; //the saving path
string path_init = "../data/";

image_data image;
Mat g_imleft(HEIGHT, WIDTH, CV_8U);
Mat g_imright(HEIGHT, WIDTH, CV_8U);
Mat	g_depth(HEIGHT, WIDTH, CV_16SC1);
int image_time_stamp = 0;

motion motion_data;
gps_data gps_tmp;

imu imu_data;
attitude_data attitude_tmp;

bool image_update = 0;
static int image_count = 0;

int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();

	if (e_image == data_type && NULL != content)
	{
		image_update = 1;
		image_count++;
		memcpy((char*)&image, content, sizeof(image));
	}

	else if (e_motion == data_type && NULL != content)
	{
		memcpy((char*)&motion_data, content, sizeof(motion_data));

		gps_tmp.gps_x = motion_data.position_in_global_x;
		gps_tmp.gps_y = motion_data.position_in_global_y;
		gps_tmp.gps_z = motion_data.position_in_global_z;
		gps_tmp.gps_time_stamp = motion_data.time_stamp;
		gps_tmp.gps_status = (motion_data.position_status & 0x00000004) && (motion_data.position_status & 0x00000002) && (motion_data.position_status & 0x00000001);

	}

	else if (e_imu == data_type && NULL != content)
	{
		memcpy((char*)&imu_data, content, sizeof(imu_data));

		attitude_tmp.attitude_time_stamp = imu_data.time_stamp;

		float a, b, c, d;
		a = imu_data.q[0];
		b = imu_data.q[1];
		c = imu_data.q[2];
		d = imu_data.q[3];

		attitude_tmp.roll = atan2(2 * (a*b + c*d), pow(a, 2) - pow(b, 2) - pow(c, 2) + pow(d, 2));//roll angle
		attitude_tmp.pitch = -asin(2 * (b*d - a*c));//pitch angle
		attitude_tmp.yaw = atan2(2 * (a*d + b*c), pow(a, 2) + pow(b, 2) - pow(c, 2) - pow(d, 2));//yaw angle
	}

	g_lock.leave();
	g_event.set_event();
	return 0;
}


void sigroutine(int dunno)
{
	int err_code = release_transfer();
	RETURN_IF_ERR(err_code);
	cout<<"transfer is released!"<<endl;
	exit(0);
}
int main()
{
	signal(SIGINT,sigroutine);
	signal(SIGSEGV,sigroutine);
	signal(SIGTSTP,sigroutine);
	signal(SIGPIPE,SIG_IGN);
	reset_config();

	int err_code = init_transfer();
	RELEASE_IF_ERR(err_code);

	err_code = select_greyscale_image(selected_vbus, true);
	RELEASE_IF_ERR(err_code);

	err_code = select_greyscale_image(selected_vbus, false);
	RELEASE_IF_ERR(err_code);

	err_code = select_depth_image(selected_vbus);
	RELEASE_IF_ERR(err_code);

	set_image_frequecy(frequency);

	select_motion();
	select_imu();

	err_code = set_sdk_event_handler(my_callback);
	RELEASE_IF_ERR(err_code);

	err_code = start_transfer();
	RELEASE_IF_ERR(err_code);

	for(int i=1;;i++)
	{
		path = path_init + itos(i) + "/";
		if(access(path.c_str(),0)==-1)
		{
			system(("mkdir "+path).c_str());
			break;	
		}
	}

	while (1)
	{
		g_event.wait_event();
		if (image_update)
		{
			memcpy(g_imleft.data, image.m_greyscale_image_left[selected_vbus], IMAGE_SIZE);
			memcpy(g_imright.data, image.m_greyscale_image_right[selected_vbus], IMAGE_SIZE);
			memcpy(g_depth.data, image.m_depth_image[selected_vbus], IMAGE_SIZE * 2);
			image_time_stamp = image.time_stamp;

			FileStorage f_depth(path + "depth" + itos(image_count) + ".xml", FileStorage::WRITE);
			FileStorage f_left(path + "left" + itos(image_count) + ".xml", FileStorage::WRITE);
			FileStorage f_right(path + "right" + itos(image_count) + ".xml", FileStorage::WRITE);

			f_depth << "gps" << "{" << "gps_time_stamp" << gps_tmp.gps_time_stamp << "gps_x" << gps_tmp.gps_x << "gps_y" << gps_tmp.gps_y << "gps_z" << gps_tmp.gps_z << "}";
			f_depth << "attitude" << "{" << "attitude_time_stamp" << attitude_tmp.attitude_time_stamp << "yaw" << attitude_tmp.yaw << "pitch" << attitude_tmp.pitch << "roll" << attitude_tmp.roll << "}";
			f_depth << "image" << "{ " << "image_time_stamp" << image_time_stamp << "depth" << g_depth << "}";

			f_left << "gps" << "{" << "gps_time_stamp" << gps_tmp.gps_time_stamp << "gps_x" << gps_tmp.gps_x << "gps_y" << gps_tmp.gps_y << "gps_z" << gps_tmp.gps_z << "}";
			f_left << "attitude" << "{" << "attitude_time_stamp" << attitude_tmp.attitude_time_stamp << "yaw" << attitude_tmp.yaw << "pitch" << attitude_tmp.pitch << "roll" << attitude_tmp.roll << "}";
			f_left << "image" << "{" << "image_time_stamp" << image_time_stamp << "left" << g_imleft << "}";

			f_right << "gps" << "{" << "gps_time_stamp" << gps_tmp.gps_time_stamp << "gps_x" << gps_tmp.gps_x << "gps_y" << gps_tmp.gps_y << "gps_z" << gps_tmp.gps_z << "}";
			f_right << "attitude" << "{" << "attitude_time_stamp" << attitude_tmp.attitude_time_stamp << "yaw" << attitude_tmp.yaw << "pitch" << attitude_tmp.pitch << "roll" << attitude_tmp.roll << "}";
			f_right << "image" << "{" << "image_time_stamp" << image_time_stamp << "right" << g_imright << "}";

			imwrite(path + "left" + itos(image_count) + ".png", g_imleft);

			f_depth.release();
			f_left.release();
			f_right.release();

			image_update = 0; 
			printf("saving image is completed!  ......\n");
		}
	}

	err_code = stop_transfer();
	RELEASE_IF_ERR(err_code);

	err_code = release_transfer();
	RETURN_IF_ERR(err_code);

	return 0;
}
