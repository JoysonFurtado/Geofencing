#include<LPC21xx.h>
# include<math.h>
# include"header.h"

extern unsigned char bx[10];

void ftos(  float s)		
{
	int x=0,k=0,t=0,i=0,m=0;
	unsigned char ax[10];	
	float f;
	x = (int)s;
	f=s-x;
	while(x!=0)
	{
		m=x%10;
		m=m+0x30;
		ax[i]=m;
		x=x/10;
		i++;
	}
	i--;
	while(i > -1)
	{
	bx[t]=ax[i];
	t++;
	i--;
	}
	i=t;
	t=0;
	bx[i]='.';
	i++;
	while((f!=0) && (t<5))
	{
		f=f*10;
		k=(int)f;
		f=f-k;
		k=k+0x30;
		bx[i]=k;
		k=0;
		i++;
		t++;
	}
	bx[i]='\0';
//	d3=i;
//	i=0;
	
	for(i=0;bx[i]!='\0';i++)
	{	
		lcd_data(bx[i]);
	   	delay();
	
	}
}

