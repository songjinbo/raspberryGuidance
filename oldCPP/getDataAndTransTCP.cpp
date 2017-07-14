/***********************************************************
Author:Song Jinbo
Date:2017-04-25
Description:1:get image info combined with GPS and attitude
	    without time stamp correction. 
	    2:transfer info to PC using TCP
	    3:change the IP address according to the fact.
************************************************************/
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include "DJI_guidance.h"
#include "DJI_utility.h"
#include <cmath> 
#include <sstream>
#include <string>
#include <vector>

#include <errno.h>
#include <netinet/in.h>    // for sockaddr_in
#include <unistd.h>
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <arpa/inet.h>
#include <csignal>
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
	int gps_status;//the gps validity
	int gps_time_stamp;//time stamp of GPS
};

struct attitude_data
{
	int attitude_time_stamp;//time stamp of attitude

	float pitch;//pitch angle
	float roll;//roll angle
	float yaw;//yaw angle
};

struct MulDataStream
{
	char head[5];
	int count;

	int image_time_stamp;
	unsigned char left[WIDTH*HEIGHT];
	unsigned char right[WIDTH*HEIGHT];
	unsigned char depth[WIDTH*HEIGHT*2];

	gps_data posi;
	attitude_data attitude;
	char tail[5];
};

//parameter
e_vbus_index selected_vbus = e_vbus1;  // select front vbus
e_image_data_frequecy frequency = e_frequecy_20;//set the frequency of image

image_data image;
motion motion_data;
imu imu_data;
MulDataStream data;
bool image_update = 0;

//return 1 when succeed, 0 when failed
bool SendData(int sock, void *buf, int size)
{
	int err;
	int index = 0;
	while (size != 0)
	{
		err = send(sock, (char*)buf + index, size, 0);
		if (err == -1) break;
		else if (err == 0) break;
		size -= err;
		index += err;
	}
	return size == 0;
}
int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();

	if (e_image == data_type && NULL != content)
	{
		image_update = 1;
		data.count++;
		memcpy((char*)&image, content, sizeof(image));
	}

	else if (e_motion == data_type && NULL != content)
	{
		memcpy((char*)&motion_data, content, sizeof(motion_data));

		data.posi.gps_x = motion_data.position_in_global_x;
		data.posi.gps_y = motion_data.position_in_global_y;
		data.posi.gps_z = motion_data.position_in_global_z;
		data.posi.gps_time_stamp = motion_data.time_stamp;
		data.posi.gps_status = (motion_data.position_status & 0x00000004) && (motion_data.position_status & 0x00000002) && (motion_data.position_status & 0x00000001);
	}

	else if (e_imu == data_type && NULL != content)
	{
		memcpy((char*)&imu_data, content, sizeof(imu_data));

		data.attitude.attitude_time_stamp = imu_data.time_stamp;

		float a, b, c, d;
		a = imu_data.q[0];
		b = imu_data.q[1];
		c = imu_data.q[2];
		d = imu_data.q[3];

		data.attitude.roll = atan2(2 * (a*b + c*d), pow(a, 2) - pow(b, 2) - pow(c, 2) + pow(d, 2));//roll angle
		data.attitude.pitch = -asin(2 * (b*d - a*c));//pitch angle
		data.attitude.yaw = atan2(2 * (a*d + b*c), pow(a, 2) + pow(b, 2) - pow(c, 2) - pow(d, 2));//yaw angle
	}

	g_lock.leave();
	g_event.set_event();
	return 0;
}

const char IP_address[]={"192.168.3.3"};
int ConnectGuidance();
int Transfer();

void sigroutine(int dunno)
{
	int err_code = release_transfer();
	RETURN_IF_ERR(err_code);
	cout<<"transfer is released!"<<endl;
	exit(0);
}

int main()
{
		signal(SIGINT,sigroutine);//;ctrl+c
		signal(SIGTSTP,sigroutine);//ctrl+z
		signal(SIGSEGV,sigroutine);//segmentation fault
		signal(SIGPIPE,SIG_IGN);
		ConnectGuidance(); 
		while(1)
		{
			Transfer();
		}
		printf("transfer is over!\n");

		int err_code = release_transfer();
		RETURN_IF_ERR(err_code);
		cout<<"transfer is released!"<<endl;
		return 0;
}

int Transfer()
{
	int client_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (client_socket < 0)
	{
		printf("Create Socket Failed!\n");
		exit(1);
	}

	struct sockaddr_in server_addr;
	bzero(&server_addr, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(IP_address);
	server_addr.sin_port = htons(6001);

	int err;
	int snd_size = 310*1024*2; // the size of send buffer is 310*4K
	socklen_t snd_len = sizeof(int);
	err = setsockopt(client_socket, SOL_SOCKET, SO_SNDBUF, (char *)&snd_size,snd_len);
	if (err<0){
		printf("error when set the size of send buffer!\n");
	}

	while (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
	{
		printf("Can Not Connect!\n");
		sleep(2000000);
	}
	printf("connnect with server succeed!\n");

	int err_code = start_transfer();
	RELEASE_IF_ERR(err_code);
	cout<<"transfer is started!"<<endl;	

	const int length = sizeof(MulDataStream);
	char data_stream[length];
	memset(data_stream, 0, length);
	
	memcpy(data.head,"head",5);
	memcpy(data.tail,"tail",5);

	data.count = 0;

	while (1)
	{
		g_event.wait_event();
		if (image_update)
		{

			data.image_time_stamp = image.time_stamp;
			memcpy(data.left, image.m_greyscale_image_left[selected_vbus], IMAGE_SIZE);
			memcpy(data.right, image.m_greyscale_image_right[selected_vbus], IMAGE_SIZE);
			memcpy(data.depth, image.m_depth_image[selected_vbus], IMAGE_SIZE * 2);
			memset(data_stream,0,length);
			memcpy(data_stream,&data,length);

			int ret = SendData(client_socket, data_stream, length);
			if (ret == 0)
			{
				printf("data error!\n");		
				break;
			}
			//printf("one frame transfered!\n");
			image_update = 0;
		}
	}
	err_code = stop_transfer();
	RELEASE_IF_ERR(err_code);
	cout<<"transfer is stopped!"<<endl;

	close(client_socket);
	cout<<"socket is closed!"<<endl;
	return 1;
}

int ConnectGuidance()
{
	reset_config();
	int err_code = init_transfer();
	RETURN_IF_ERR(err_code);

	printf("connection with guidance succeed!\n");

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

	return 1;
}
