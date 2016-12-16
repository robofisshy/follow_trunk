/*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
//#include "OniSampleUtilities.h"
#include <OpenNI.h>
#include <sys/time.h>

/***************************
 * Motor Drivers
 ***************************/
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <wiringPi.h>
#include <math.h>
#define uchar unsigned char
#define uint unsigned int

#define Channal_A     1
#define Channal_B     2
#define Channal_AB    3

#define DIN     0       //Data input
#define SCLK    1       //Clk
#define CS      2       //Chip select
#define Alert1  3
#define Alert2  4
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace openni;
using namespace std;


int fd;
typedef struct point
{
	float x;
	float y;
	float z;
}point;

enum state{
    STOP=0,
    RUN
}state;
void send_vel_to_uno(int state_,double lspeed_,double rspeed_);
extern int serial_open();
extern int serial_Send(int fd, char *send_buf,int data_len);
extern int serial_Recv(int fd, char *rcv_buf,int data_len);
typedef struct velocity
{
	double l;
	double r;
}velocity;

inline int sign(double x)
{
	if(x>0)
		return 1;
	if(x==0)
		return 0;
	if(x<0)
		return -1;
};
/**********************************
 * Drivers function
 * ********************************
 * ********************************/
static void DA_conver(uint Dignum);
extern void Write_A_B(uint Data_A,uint Data_B,uchar Channal,int Model);
void GpioInit()
{
	wiringPiSetup();
	pinMode(DIN,OUTPUT);
	pinMode(SCLK,OUTPUT);
	pinMode(CS,OUTPUT);
	pinMode(Alert1,INPUT);
	pinMode(Alert2,INPUT);
}
void DA_conver(uint Dignum)
{
	uint Dig=0;
	uchar i=0;
	digitalWrite(SCLK,HIGH);        //
	digitalWrite(CS,LOW);           //Select
	for(i=0;i<16;i++)   //写入16为Bit的控制位和数据
	{
		Dig=Dignum&0x8000;
		if(Dig)
		{
			digitalWrite(DIN,HIGH);
		}
		else
		{
			digitalWrite(DIN,LOW);
		}
		digitalWrite(SCLK,LOW);
		asm("nop");
		Dignum<<=1;
		digitalWrite(SCLK,HIGH);
		asm("nop");
	}
	digitalWrite(SCLK,HIGH);
	digitalWrite(CS,HIGH);       //片选无效

}
void Write_A_B(uint Data_A,uint Data_B,uchar Channal,int Model)
{
	uint Temp;
	if(Model)
	{
		Temp=0x4000;
	}
	else
	{
		Temp=0x0000;
	}
	switch(Channal)
	{
		case Channal_A:         //A通道
			DA_conver(Temp|0x8000|(0x0fff&Data_A));
			break;
		case Channal_B:       //B通道
			DA_conver(Temp|0x0000|(0x0fff&Data_B));
			break;
		case Channal_AB:
			DA_conver(Temp|0x1000|(0x0fff&Data_B));        //A&B通道
			DA_conver(Temp|0x8000|(0x0fff&Data_A));
			break;
		default:
			break;
	}
}
void send_vel_to_uno(int state_,double lspeed_,double rspeed_)
{
	char send_vel[100];
    	char rspeed[20];
	char lspeed[20];
	char rcv_buf[5];
	int len_t;
	int len_r;
	
	//int ll=round(10*lspeed_);	//keep one decimal
	//int rr=round(10*rspeed_);
	int ll=round(30*lspeed_/0.6);	//keep one decimal
	int rr=round(30*rspeed_/0.6);
	if(state_==RUN){
		if(ll==0&&rr==0)
                	sprintf(send_vel, "%c", '0');
       		else
                	sprintf(send_vel, "%c", '1');
		cout<<"It's working time"<<endl;
	}
	else if(state_==STOP){
		sprintf(send_vel, "%c", '0');	
		cout<<"It's stop time"<<endl;
	}
	//ll=51+(ll-10)*5;	//51 means 1m/s, when add 0.1m/s,add 5 to ll
	//rr=51+(rr-10)*5;
    	sprintf(lspeed, "%d", rr);
    	sprintf(rspeed, "%d", ll);

    	strcat(send_vel,",");
    	strcat(send_vel,lspeed);
    	strcat(send_vel,",");
    	strcat(send_vel,rspeed);

	len_t= serial_Send(fd,send_vel,strlen(send_vel));
	cout<<"len_t is "<<len_t<<endl;
	cout<<send_vel<<endl;
}
int main(int argc, char** argv)
{
	double L_speed(0),R_speed(0);
	double s_theta(0),c(0);
	GpioInit(); 		//Initialize the GPIO
	fd=serial_open();
        tcflush(fd,TCIOFLUSH);
	double max_speed=0.6;
	double min_y_(-0.35), max_y_(0.35),
			min_x_(0.2), max_x_(0.5),
			max_z_(1.5), goal_z_(0.6),
			z_scale_(1), x_scale_(5);
	bool enabled_(true);
	double accel_lim_v(0.6);
	double accel_lim_w(5.4);
	double speed_lim_v(1);
	double speed_lim_w(3.5);
	double decel_factor(1);
	double k_factor(1);
	double theta(0);
    velocity current_cmd{0,0},last_cmd{0,0};
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	VideoStream depth;

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}

	VideoMode modeDepth;
	modeDepth.setResolution( 640, 480 );
	modeDepth.setFps( 30 );
	modeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
	depth.setVideoMode(modeDepth);

	double RealWorldXtoZ=2*tan(1.0226/2);
	double RealWorldYtoZ=2*tan(0.796616/2);
	//long int cal_pix_num;
	//cal_pix_num=640*480;
	double real_factor_x[307200];
	double real_factor_y[307200];
	int i,j;
	for( i=0;i<480;i++)
	{
		for(j=0;j<640;j++)
		{
			real_factor_x[i*640+j]= ((float)j / 640.0 - .5f)*RealWorldXtoZ;
			real_factor_y[i*640+j] = (.5f - (float)i / 480.0)*RealWorldYtoZ;
		}
	}

	if( device.isImageRegistrationModeSupported(
			IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
	{
		device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
	}

	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}
	VideoFrameRef frame;
	long double period(0.0);
	while (1){
		theta=0;
		struct timeval tpstart;
		gettimeofday(&tpstart,NULL);
		long double timenow = tpstart.tv_sec+tpstart.tv_usec/1000000.0;
		int changedStreamDummy;
		VideoStream* pStream = &depth;
		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}

		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		vector<point> points;
		point depth_point;
        float x(0.0),y(0.0),z(0.0),xx(0.0);
		DepthPixel *pDepth = (DepthPixel*)frame.getData();
		for(int i=0;i<frame.getHeight();i++)
		{
			for(int j=0;j<frame.getWidth();j++)
			{
				xx = pDepth[i*frame.getWidth()+j];
				//CoordinateConverter::convertDepthToWorld (depth,i,j,xx,&x,&y,&z);
				x=real_factor_x[i*640+j]*xx;
				y=real_factor_y[i*640+j]*xx;
				depth_point.y=x*0.001;
				depth_point.x=y*0.001;
				depth_point.z=xx*0.001;
				points.push_back(depth_point);
			}
		}

		//X,Y,Z of the centroid
		x = 0.0;
		y = 0.0;
		z = 1e6;
		//Number of points observed
		unsigned int n = 0;
		//Iterate through all the points in the region and find the average of the position
		for (vector<point>::iterator iter = points.begin(); iter != points.end(); iter++)
		{
			//First, ensure that the point's position is valid. This must be done in a seperate
			//if because we do not want to perform comparison on a nan value.
			if (!isnan(x) && !isnan(y) && !isnan(z))
			{
				//Test to ensure the point is within the aceptable box.
				//cout<<"*************"<<iter->x<<"   "<<iter->y<<"  "<<iter->z<<endl;
				if (iter->y > min_y_ && iter->y < max_y_ && iter->x < max_x_ && iter->x > min_x_ && iter->z < max_z_ && iter->z > 0.05)
				{
					//Add the point to the totals
					x += iter->x;
					y += iter->y;
					z = std::min(z, iter->z);
					n++;
				}
			}
		}
	//	cout<<"z="<<z<<endl;
		//If there are points, find the centroid and calculate the command goal.
		//If there are no points, simply publish a stop goal.
	//	cout<<"n="<<n<<endl;
		if (n>1000)
		{
			x /= n;
			y /= n;
			if(z > max_z_){

				if (enabled_)
				{
					 L_speed = 0;
					 R_speed = 0;
					//cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
				}
			}
			else
			{
				theta = atan2(y,z);
				cout<<"theta="<<theta<<endl;
				struct timeval tpend;
				gettimeofday(&tpend,NULL);
				period =tpend.tv_sec+ tpend.tv_usec/1000000.0 - timenow;
				if(theta>0.01 || theta<-0.01)
				{
					s_theta=abs(sin(theta));
					c=(sqrt(y*y+z*z)-goal_z_)/(2*s_theta);
					cout<<"c="<<c<<endl;
					if(theta>0)
					{
						cout<<"theta>0"<<endl;
						L_speed= 2*(c+0.17)*theta/period;
						R_speed= 2*(c-0.17)*theta/period;
					}
					if(theta<0)
					{
						cout<<"theta<0"<<endl;
						L_speed= -2*(c-0.17)*theta/period;
						R_speed= -2*(c+0.17)*theta/period;
					}
				}
				else
				{
					L_speed= (z-goal_z_)/period;
					R_speed= (z-goal_z_)/period;
				}
			}

		}
		else
		{
			 L_speed= 0;
			 R_speed= 0;
		}
		cout<<"first_L_speed= "<<L_speed<<endl;
		cout<<"first_R_speed= "<<R_speed<<endl; 

		if(L_speed<0)
		{
			R_speed=R_speed-L_speed;
			L_speed=0;
		}
		if(R_speed<0)
		{
			L_speed=L_speed-R_speed;
			R_speed=0;
		}
		if(L_speed>max_speed)
		{
			R_speed=R_speed*max_speed/L_speed;
			L_speed=max_speed;
		}
		if(R_speed>max_speed)
		{
			L_speed=L_speed*max_speed/R_speed;
			R_speed=max_speed;
		}
		//********************速度smooth*******************//
		if(L_speed>0)
		{
			double L_speed2= (L_speed-last_cmd.l) > 0 ? min(last_cmd.l+accel_lim_v, L_speed ) : max(last_cmd.l-accel_lim_v , L_speed);
			R_speed=R_speed*L_speed2/L_speed;
			L_speed=L_speed2;
		}
		if(R_speed>0)
		{
			double R_speed2= (R_speed-last_cmd.r) > 0 ? min(last_cmd.r+accel_lim_v, R_speed ) : max(last_cmd.r-accel_lim_v , R_speed);
			L_speed=L_speed*R_speed2/R_speed;
			R_speed=R_speed2;
		}

		cout<<"smooth_L="<<L_speed<<endl;
		cout<<"smooth_R="<<R_speed<<endl;

		last_cmd.l = L_speed;
		last_cmd.r = R_speed;


		
  	//	Write_A_B(L_speed,R_speed,Channal_AB,1);		//Control DA output
		if (digitalRead(3)==1 && digitalRead(4)==0)
                {
                        L_speed=0;
                        R_speed=0;
                        send_vel_to_uno(STOP,L_speed,R_speed);
                	//cout<<"Value is "<<digitalRead(3)<<" "<<digitalRead(4)<<endl;
		        //cout<<"Alert Alert"<<endl;
                }
		else
		{
			send_vel_to_uno(RUN,L_speed,R_speed);
                        //cout<<"Value is "<<digitalRead(3)<<" "<<digitalRead(4)<<endl;
		}
		cout<<"L_speed= "<<L_speed<<endl;
		cout<<"R_speed= "<<R_speed<<endl;
	}
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
