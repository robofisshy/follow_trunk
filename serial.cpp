#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
volatile bool _quit=false;
void handle_signal(int sig)
{
    _quit=true;
}
int serial_open()
{
    const char *dev="/dev/ttyUSB0";
    struct termios serial;
    int fd=open(dev,O_RDWR|O_NOCTTY);         //should set the authority of the device
    if(tcgetattr(fd,&serial)<0)
	cout<<"get tty property error"<<endl;
    memset(&serial, 0, sizeof(serial));
    serial.c_iflag=IGNPAR;
    serial.c_cflag &= ~CSIZE;
    serial.c_cflag=B9600|HUPCL|CS8|CREAD|CLOCAL;
    serial.c_cc[VMIN]=1;
    serial.c_cflag &= ~CRTSCTS;
    serial.c_cflag &= ~CSTOPB;
    serial.c_cflag &=~PARENB;
    if(tcsetattr(fd,TCSANOW,&serial)<0)
    	cout<<"set tty property error"<<endl;
    return fd;
}
int serial_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec =3;
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
	if(len==-1){
	     perror("read :");
	     tcflush(fd,TCIOFLUSH);		
	}
        return len;
    }
    else
    {
       	cout<<"error,error"<<endl;
	tcflush(fd,TCIOFLUSH);
	return -1;
    }
}
int serial_Send(int fd, char *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return -1;
    }
}
void serial_Close(int fd)
{
    close(fd);
}


