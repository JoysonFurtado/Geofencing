#include<lpc21xx.h>
#include"header.h"

void lcd_cmd(volatile unsigned char command)
{
	unsigned int i;

	IOCLR1=0X03000000;
	i=(command&0xf0);
	i=i<<17;
	IOSET0=i;
	IOCLR0=~i;

	IOCLR1=0X03000000;
	IOSET1=0X02000000;
	delay();
	IOCLR1=0X02000000;

	IOCLR1=0X03000000;
	i=(command&0x0f);
	i=i<<21;
	IOSET0=i;
	IOCLR0=~i;
	IOCLR1=0X03000000;
	IOSET1=0X02000000;
	delay();
	IOCLR1=0X02000000;
}

/*		LCD DATA FUNCTION		*/

void lcd_data(volatile unsigned char command)
{
	unsigned int i;

	IOCLR1=0X03000000;
	i=(command&0xf0);
	i=i<<17;
	IOSET0=i;
	IOCLR0=~i;
	IOCLR1=0X03000000;
	IOSET1=0X03000000;
	delay();
	IOCLR1=0X02000000;

	IOCLR1=0X03000000;
	i=(command&0x0f);
	i=i<<21;
	IOSET0=i;
	IOCLR0=~i;
	IOCLR1=0X03000000;
	IOSET1=0X03000000;
	delay();
	IOCLR1=0X02000000;
}


void delay()
{
	int i,j;
	for(i=0;i<65000;i++);
	for(j=0;j<65000;j++);
}

void lcd_init()
{
	unsigned char cmd[]={0x28,0x0e,0x06,0x80,0x01};
	int i;

		IODIR0 |= 0X01E00000;
//		IOCLR0=0X00000000;
		IODIR1 |= 0X03000000;
		for(i=0;i<5;i++)
		{
			lcd_cmd(cmd[i]);		   // SENDING COMMANDS
			delay();
		}					   

}
