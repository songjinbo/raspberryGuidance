/***********************************************************
Author:Song Jinbo
Date:2017-04-25
Description:1:get image info combined with GPS and attitude
	    without time stamp correction. 
	    2:transfer info to PC using UDP
	    3:change the IP address according to the fact.
************************************************************/
#include <errno.h>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include <cmath> 
#include <sstream>
#include <string>
#include <vector>
#include <sys/resource.h>

#include <time.h>
#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <arpa/inet.h>
#include <csignal>

#include "DJI_guidance.h"
#include "DJI_utility.h"

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
	int count;

	int image_time_stamp;
	unsigned char left[WIDTH*HEIGHT];
	unsigned char right[WIDTH*HEIGHT];
	unsigned char depth[WIDTH*HEIGHT*2];

	gps_data posi;
	attitude_data attitude;
};

//parameter
e_vbus_index selected_vbus = e_vbus1;  // select front vbus
e_image_data_frequecy frequency = e_frequecy_20;//set the frequency of image

image_data image;
motion motion_data;
imu imu_data;
int count1 = 0;
bool image_update = 0;

ofstream outfile("out.txt",std::ios::out);
clock_t start,end;
int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		image_update = 1;
		count1++;
		memcpy((char*)&image, content, sizeof(image));
		end = clock();
		double t = (double)(end-start);
		outfile<<count1<<": "<<t/CLOCKS_PER_SEC*1000<<std::endl;
		std::cout<<count1<<": "<<t/CLOCKS_PER_SEC*1000<<std::endl;
		start = end;
	}

	else if (e_motion == data_type && NULL != content)
	{
		memcpy((char*)&motion_data, content, sizeof(motion_data));
	}

	else if (e_imu == data_type && NULL != content)
	{
		memcpy((char*)&imu_data, content, sizeof(imu_data));
	}

	g_lock.leave();
	g_event.set_event();
	return 0;
}

int ConnectGuidance();
int Transfer();
void sigroutine(int dunno)
{
	int err_code = stop_transfer();
	RELEASE_IF_ERR(err_code);
	cout<<"transfer is stopped by signal!"<<endl;
	err_code = release_transfer();
	RETURN_IF_ERR(err_code);
	cout<<"transfer is released by signal!"<<endl;
	exit(1);
}

const int MAXLEN = 1200;//IP协议一次传输1400字节，这里留有冗余量
struct sockaddr_in remote_addr; //服务器端网络地址结构体
struct SendUnit
{
	int count; //每一帧的编号
	int index; //每一片的编号
	char data[MAXLEN];//1200个字节的数据
};
struct RecvUnit
{
	int count;
	char data[6];
};
enum ERRCODE
 {
	SENDERROR=1,//sendto函数产生错误
	SENDTIMEOUT,//多次发送没应答
	ACKERROR,//应答错误
	SUCCESS,
};
ERRCODE SendData(int sock, char *buf, int size)
{
	int err;
	struct SendUnit snd_buf;//用来存储分片的数据
	snd_buf.count = count1; 
	int BlockNum = (size%MAXLEN)? size/MAXLEN+1:size/MAXLEN;
	for(int i = 0;i<BlockNum;i++)
	{	
		snd_buf.index = i;
		if(i<size/MAXLEN)
		{
			memcpy(snd_buf.data,buf+i*MAXLEN,MAXLEN);
		}
		else
		{
			memcpy(snd_buf.data,buf+i*MAXLEN,size-i*MAXLEN);
		}
		err = sendto(sock,&snd_buf,sizeof(struct SendUnit),0,(struct sockaddr *)&remote_addr,sizeof(remote_addr));
		if(err==-1)
		{
			perror("send error");
			return SENDERROR;
		}
		else if(err != sizeof(struct SendUnit))
		{
			cout<<"发送数据大小不正确:发送"<<err<<"个字节"<<endl;
			return SENDERROR;
		}
	}
//	std::cout<<count1<<std::endl;
	return SUCCESS;
}
const char IP_address[]={"192.168.3.3"};

int main()
{
	signal(SIGKILL,sigroutine);
	signal(SIGINT,sigroutine);
	signal(SIGSEGV,sigroutine);
	signal(SIGTSTP,sigroutine);
	signal(SIGPIPE,SIG_IGN);

	ConnectGuidance(); 

	int client_socket;
	if((client_socket=socket(AF_INET,SOCK_DGRAM,0))<0)
	{  
		perror("socket");
		return 0;
	}
	//设置client_socket为阻塞模式
	int flags =fcntl(client_socket,F_GETFL,0);
	fcntl(client_socket,F_SETFL,flags&~O_NONBLOCK);
	//设置发送缓冲区大小
	const int snd_size = 310*1024*4;
	if(setsockopt(client_socket, SOL_SOCKET, SO_SNDBUF, (char *)&snd_size, sizeof(snd_size))<0)
	{
		perror("socket");
		perror("set socket send buffer");
		return 0;
	}
	//设置服务器IP地址
	memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零
	remote_addr.sin_family=AF_INET; //设置为IP通信
	remote_addr.sin_addr.s_addr=inet_addr(IP_address);//服务器IP地址
	remote_addr.sin_port=htons(8000); //服务器端口号

	const int length = sizeof(MulDataStream);
	
	int err_code = start_transfer();
	RELEASE_IF_ERR(err_code);
	cout<<"transfer is started!"<<endl;	

	time_t start1,end1;
	MulDataStream data;
	count1 = 0;
	while (1)
	{
		g_event.wait_event();
		if (image_update)
		{
			g_lock.enter();
			data.image_time_stamp = image.time_stamp;
			memcpy(data.left, image.m_greyscale_image_left[selected_vbus], IMAGE_SIZE);
			memcpy(data.right, image.m_greyscale_image_right[selected_vbus], IMAGE_SIZE);
			memcpy(data.depth, image.m_depth_image[selected_vbus], IMAGE_SIZE * 2);
	
			data.posi.gps_x = motion_data.position_in_global_x;
			data.posi.gps_y = motion_data.position_in_global_y;
			data.posi.gps_z = motion_data.position_in_global_z;
			data.posi.gps_time_stamp = motion_data.time_stamp;
			data.posi.gps_status = (motion_data.position_status & 0x00000004) && (motion_data.position_status & 0x00000002) && (motion_data.position_status & 0x00000001);

			data.attitude.attitude_time_stamp = imu_data.time_stamp;
			float a, b, c, d;
			a = imu_data.q[0];
			b = imu_data.q[1];
			c = imu_data.q[2];
			d = imu_data.q[3];
			data.attitude.roll = atan2(2 * (a*b + c*d), pow(a, 2) - pow(b, 2) - pow(c, 2) + pow(d, 2));//roll angle
			data.attitude.pitch = -asin(2 * (b*d - a*c));//pitch angle
			data.attitude.yaw = atan2(2 * (a*d + b*c), pow(a, 2) + pow(b, 2) - pow(c, 2) - pow(d, 2));//yaw angle
			g_lock.leave();

			ERRCODE ret = SendData(client_socket, (char *)&data, length);
			end1 = clock();
			double t1 = (double)(end1-start1);
			outfile<<"------"<<count1<<": "<<t1/CLOCKS_PER_SEC*1000<<std::endl;
			std::cout<<"------"<<count1<<": "<<t1/CLOCKS_PER_SEC*1000<<std::endl;
			start1 = end1;	
			if (ret == SENDTIMEOUT)
			{
				cout<<"Timeout"<<endl;
			}
			else if(ret != SUCCESS)
			{
				cout<<"Send Error / ACK error"<<endl;
				break;
			}
			image_update = 0;
		}
	}

	close(client_socket);
	cout<<"socket is closed!"<<endl;

	cout<<"transfer is over!"<<endl;
	err_code = stop_transfer();
	RELEASE_IF_ERR(err_code);
	cout<<"transfer is stopped!"<<endl;
	err_code = release_transfer();
	RETURN_IF_ERR(err_code);
	cout<<"transfer is released!"<<endl;
	return 0;
}

int ConnectGuidance()
{
	reset_config();

	int err_code = init_transfer();
	RETURN_IF_ERR(err_code);
	cout<<"connection with guidance succeed!"<<endl;

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

