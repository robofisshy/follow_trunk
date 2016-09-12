/*****************************************************************************
*                                                                            *
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
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>
#define uchar unsigned char
#define uint unsigned int

#define Channal_A     1
#define Channal_B     2
#define Channal_AB    3

#define DIN     0       //Data input
#define SCLK    1       //Clk
#define CS      2       //Chip select
#define IN0	21		
#define IN1	22
#define IN2	23
#define IN3	24	
#define PWM1	25
#define PWM2	28

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace openni;
using namespace std;
double L_speed(0),R_speed(0);
typedef struct point
{
    float x;
    float y;
    float z;
}point;

typedef struct velocity
{
    double linear;
    double angular;
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
static void DA_conver(uint Dignum);
extern void Write_A_B(uint Data_A,uint Data_B,uchar Channal,int Model);
void GpioInit()
{
        wiringPiSetup();
        //pinMode(DIN,OUTPUT);
        //pinMode(SCLK,OUTPUT);
        //pinMode(CS,OUTPUT);
	pinMode(IN0,OUTPUT);
	pinMode(IN1,OUTPUT);
	pinMode(IN2,OUTPUT);
	pinMode(IN3,OUTPUT);
	//pinMode(PWM1,OUTPUT);
	//pinMode(PWM2,OUTPUT);
	digitalWrite(IN0,HIGH);
	digitalWrite(IN1,LOW);
	digitalWrite(IN2,LOW);
	digitalWrite(IN3,HIGH);	
}
void PWM_init()
{	
	softPwmCreate(PWM1,0,100);
	softPwmCreate(PWM2,0,100);
}
void PWM_write(int L,int R)
{
	softPwmWrite(PWM1,L_speed);
	softPwmWrite(PWM2,R_speed);
}
void DA_conver(uint Dignum)
{
        uint Dig=0;
        uchar i=0;
        digitalWrite(SCLK,HIGH);        //
        digitalWrite(CS,LOW);           //Select
        for(i=0;i<16;i++)   		//写入16为Bit的控制位和数据
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
        digitalWrite(CS,HIGH);          //片选无效

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

int main(int argc, char** argv)
{
    GpioInit();
    //wiringPiSetup();
    PWM_init();
    double min_y_(-0.35), max_y_(0.35),
            min_x_(-0.3), max_x_(0.1),
            max_z_(1.2), goal_z_(0.5),
            z_scale_(2), x_scale_(6);
    bool enabled_(true);
    double accel_lim_v(0.8);
    double accel_lim_w(5.5);
    double speed_lim_v(0.8);
    double speed_lim_w(5.4);
    double decel_factor(1);
    double k_factor(1);
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
    }	//坐标转换
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
    while (1)
    {
        struct timeval tpstart;
        gettimeofday(&tpstart,NULL);
        long double timenow = tpstart.tv_sec+tpstart.tv_usec/1000000.0;
        //float timenow = tpstart.tv_usec/1000000.0;
        //cout<<"timenow="<<timenow<<endl;

        int changedStreamDummy;
        VideoStream* pStream = &depth;
        rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
        if (rc != STATUS_OK)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
        PWM_write(0,0);	 
	   continue;
        }

        rc = depth.readFrame(&frame);
        if (rc != STATUS_OK)
        {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            PWM_write(0,0);
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
		depth_point.x=x*0.001;
                depth_point.y=y*0.001;
                depth_point.z=z*0.001;
                points.push_back(depth_point);
                //cout<<xx<<endl;
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
                if (-iter->y > min_y_ && -iter->y < max_y_ && iter->x < max_x_ && iter->x > min_x_ && iter->z < max_z_&& iter->z > 0.3)
                {
                    //Add the point to the totals
                    x += iter->x;
                    y += iter->y;
                    z = std::min(z, iter->z);
                    n++;
                }
            }
        }

        //If there are points, find the centroid and calculate the command goal.
        //If there are no points, simply publish a stop goal.
        if (n>500)
        {
            x /= n;
            y /= n;
            if(z > max_z_){

                if (enabled_)
                {
                    current_cmd.linear = 0;
                    current_cmd.angular = 0;
                    //cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
                }
            }
            else
            {
                current_cmd.linear = (z - goal_z_) * z_scale_;
                if(current_cmd.linear<0)
                    current_cmd.linear=0;
                current_cmd.angular = y * x_scale_;
                //cmdpub_.publish(cmd);
            }

        }
        else
        {
            current_cmd.linear = 0;
            current_cmd.angular = 0;
        }
                //********************速度smooth*******************//
        struct timeval tpend;
        gettimeofday(&tpend,NULL);
        period =tpend.tv_sec+ tpend.tv_usec/1000000.0 - timenow;
        //period = tpend.tv_usec/1000000.0 - timenow;
        cout<<"period="<<period<<endl;
        current_cmd.linear  =
                current_cmd.linear  > 0.0 ? std::min(current_cmd.linear,  speed_lim_v) : std::max(current_cmd.linear,  -speed_lim_v);
        current_cmd.angular =
                current_cmd.angular > 0.0 ? std::min(current_cmd.angular, speed_lim_w) : std::max(current_cmd.angular, -speed_lim_w);

        double v_inc, w_inc, max_v_inc, max_w_inc;

        v_inc = current_cmd.linear - last_cmd.linear;
        if (last_cmd.linear*current_cmd.linear < 0.0)
        {
            // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
            max_v_inc = decel_factor*accel_lim_v*period;
        }
        else
        {
            max_v_inc = ((v_inc*current_cmd.linear > 0.0)?accel_lim_v:decel_factor*accel_lim_v)*period;
        }
        w_inc = current_cmd.angular - last_cmd.angular;
        if (last_cmd.angular*current_cmd.angular < 0.0)
        {
            // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
            max_w_inc = decel_factor*accel_lim_w*period;
        }
        else
        {
            max_w_inc = ((w_inc*current_cmd.angular > 0.0)?accel_lim_w:decel_factor*accel_lim_w)*period;
        }

        double MA = sqrt(    v_inc *     v_inc +     w_inc *     w_inc);
        double MB = sqrt(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

        double Av = std::abs(v_inc) / MA;
        double Aw = std::abs(w_inc) / MA;
        double Bv = max_v_inc / MB;
        double Bw = max_w_inc / MB;
        double theta = atan2(Bw, Bv) - atan2(Aw, Av);

        if (theta < 0)
        {
            // overconstrain linear velocity
            max_v_inc = (max_w_inc*std::abs(v_inc))/std::abs(w_inc);
        }
        else
        {
            // overconstrain angular velocity
            max_w_inc = (max_v_inc*std::abs(w_inc))/std::abs(v_inc);
        }


        if (std::abs(v_inc) > max_v_inc)
        {
            // we must limit linear velocity
            current_cmd.linear  = last_cmd.linear  + sign(v_inc)*max_v_inc;
        }


        if (std::abs(w_inc) > max_w_inc)
        {
            // we must limit angular velocity
            current_cmd.angular = last_cmd.angular + sign(w_inc)*max_w_inc;
        }

        last_cmd = current_cmd;
        //cout<<"**********************linear= "<<current_cmd.linear<<endl;
        L_speed=k_factor*(current_cmd.linear-0.13*current_cmd.angular)*100/1.5;

        R_speed=k_factor*(current_cmd.linear+0.13*current_cmd.angular)*100/1.5;

        cout<<"linear= "<<current_cmd.linear<<endl;
        cout<<"angular= "<<current_cmd.angular<<endl;
        if(L_speed<0)
            L_speed=0;
        if(R_speed<0)
            R_speed=0;
        //******************************************
        cout<<"L_speed= "<<L_speed<<endl;
        cout<<"R_speed= "<<R_speed<<endl;
	L_speed*=1.3;
	R_speed*=1.3;
	PWM_write((int)L_speed,(int)R_speed);
        //Write_A_B(L_speed,R_speed,Channal_AB,1);
	
        //******************************************
    }

    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
