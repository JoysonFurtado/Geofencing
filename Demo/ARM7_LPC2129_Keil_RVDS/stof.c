#include<LPC21xx.h>
#include<math.h>
# include"header.h"

int stof_lati(unsigned char* data)
{
	unsigned int res = 0, res1 = 0, sum = 0;
	res = (((*data)-0x30) * 10);
	res = res + ((*(++data)) - 0x30);
	res = res * 60;
	*(++data);
	res1 = (((*data)-0x30) * 10);
	res1 = res1 + ((*(++data)) - 0x30);
	sum = res + res1;
	return sum;
}
int stof_longi(unsigned char* data)
{
	unsigned int res = 0, res1 = 0, sum = 0;
	res = (((*data) - 0x30) * 100)+(((*(++data))-0x30) * 10);
	res = res + ((*(++data)) - 0x30);
	res = res * 60;
	*(++data);
	res1 = (((*data)-0x30) * 10);
	res1 = res1 + ((*(++data)) - 0x30);
	sum = res + res1;
	return sum;
}
