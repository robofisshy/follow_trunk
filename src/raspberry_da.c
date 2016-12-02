#include <wiringPi.h>
#include <math.h>
#define uchar unsigned char
#define uint unsigned int

#define Channal_A     1    
#define Channal_B     2    
#define Channal_AB    3    

#define DIN	0	//Data input
#define SCLK	1	//Clk
#define CS	2	//Chip select
 
static void DA_conver(uint Dignum);
extern void Write_A_B(uint Data_A,uint Data_B,uchar Channal,int Model);
void GpioInit();

uint n1=0,n2=0;

void GpioInit()
{
	wiringPiSetup();
	pinMode(DIN,OUTPUT);
	pinMode(SCLK,OUTPUT);
	pinMode(CS,OUTPUT);
}
//Name	  :	void DA_conver(uint Dignum)
//Function:	Digital to analog
//Para	  :     Dignum
void DA_conver(uint Dignum)
{
	uint Dig=0;
	uchar i=0;
	digitalWrite(SCLK,HIGH);	//	
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
void main()
{
	GpioInit();
	n1=0;
	n2=0;	
	while(1)
	{
		Write_A_B(n1,n2,Channal_AB,1);   
	}
}

