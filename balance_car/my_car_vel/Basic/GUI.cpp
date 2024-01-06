#include "Basic.hpp"
#include <stdlib.h>
#include <stdio.h>
#include "drv_LCD.hpp"
#include "GUI_Images.hpp"
#include "GUI.hpp"
#include "Font.hpp"
#include "Commulink.hpp"

extern float Pitch,Roll,Yaw;
extern float Voltage;
extern int Encoder_Left,Encoder_Right;
extern float Velocity_Left,Velocity_Right;
extern int Volt_flag;

/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}

/******************************************************************************
函数说明：显示基本信息
      入口数据：无
      返回值：  无
******************************************************************************/
void LCD_ShowMAIN()
{
	int Encoder_Left_temp,Encoder_Right_temp;
	float Velocity_Left_temp,Velocity_Right_temp;
	u8 str[50];
	u8 BAT[]="VDDA:";
	u8 Rol[]={"Pit:"};
	u8 Pit[]={"Rol:"};
	u8 YAW[]={"Yaw:"};
	u8 VOLT[]={"Please charge"};
	u8 ENL[]={"EnL:"};
	u8 ENR[]={"EnR:"};
	u8 VL[]={"VL:"};
	u8 VR[]={"VR:"};
	float VDDA = 0;
	LCD_ShowString(0,0,BAT,BLACK,WHITE,16,0);
	LCD_ShowString(0,16,Rol,BLACK,WHITE,16,0);
	LCD_ShowString(0,32,Pit,BLACK,WHITE,16,0);
  LCD_ShowString(0,48,YAW,BLACK,WHITE,16,0);
	LCD_ShowString(0,64,ENL,BLACK,WHITE,16,0);
	LCD_ShowString(0,80,ENR,BLACK,WHITE,16,0);
	LCD_ShowString(0,96,VL,BLACK,WHITE,16,0);
	LCD_ShowString(0,112,VR,BLACK,WHITE,16,0);
	//显示电压
	VDDA = ((float)Voltage)/100;
	sprintf((char*)str, "%4.2f", VDDA);
	LCD_ShowString(40,0,str,RED,WHITE,16,0);
	sprintf((char*)str, "V");
	LCD_ShowString(80,0,str,RED,WHITE,16,0);
	//显示欧拉角
	//pitch
	sprintf((char*)str, "%3.2f", Pitch);
	LCD_ShowString(32,16,str,BLUE,WHITE,16,0);
	//roll
	sprintf((char*)str, "%3.2f", Roll);
	LCD_ShowString(32,32,str,BLUE,WHITE,16,0);
	//yaw
	sprintf((char*)str, "%3.2f", Yaw);
	LCD_ShowString(32,48,str,BLUE,WHITE,16,0);
	//显示编码器读数
	if(Encoder_Left<0)
	{
		LCD_ShowChar(32,64,'-',PURPLE,WHITE,16,0);
		Encoder_Left_temp=0-Encoder_Left;
	}
	else
	{
		LCD_ShowChar(32,64,' ',WHITE,WHITE,16,0);
		Encoder_Left_temp=Encoder_Left;
	}
	if(Encoder_Right<0)
	{
		LCD_ShowChar(32,80,'-',PURPLE,WHITE,16,0);
		Encoder_Right_temp=0-Encoder_Right;
	}
	else
	{
		LCD_ShowChar(32,80,' ',WHITE,WHITE,16,0);
		Encoder_Right_temp=Encoder_Right;
	}
	LCD_ShowIntNum(40,64,Encoder_Left_temp,3,PURPLE,WHITE,16);
	LCD_ShowIntNum(40,80,Encoder_Right_temp,3,PURPLE,WHITE,16);
	/*获取电机的速度*/
	if(Velocity_Left<0)
	{
		LCD_ShowChar(24,96,'-',PURPLE,WHITE,16,0);
		Velocity_Left_temp=0-Velocity_Left;
	}
	else
	{
		LCD_ShowChar(24,96,' ',WHITE,WHITE,16,0);
		Velocity_Left_temp=Velocity_Left;
	}
	if(Velocity_Right<0)
	{
		LCD_ShowChar(24,112,'-',PURPLE,WHITE,16,0);
		Velocity_Right_temp=0-Velocity_Right;
	}
	else
	{
		LCD_ShowChar(24,112,' ',WHITE,WHITE,16,0);
		Velocity_Right_temp=Velocity_Right;
	}
	LCD_ShowFloatNum1(32,96,Velocity_Left_temp,7,PURPLE,WHITE,16);
	LCD_ShowFloatNum1(32,112,Velocity_Right_temp,7,PURPLE,WHITE,16);
	//如果电压过低 提示充电
//	if(Volt_flag==1)
//		LCD_ShowString(32,80,VOLT,RED,WHITE,24,0);
}

static void GUI_Server()
{
	float t=0;
	u8 WhuFly[]={"Whu-Fly"};
	u8 Auther[]={"Auther:WX"};
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	u8 HthA[]={"HthA:  0%"};
	u8 Rol[]={"Rol:  0.0"};
	u8 Pit[]={"Pit:  0.0"};
	u8 Yaw[]={"Yaw:  0.0"};
	u8 Vz[]={"Vz:   0.0"};
	u8 HthP[]={"HthP:  0%"};
	u8 Vx[]={"Vx:   0.0"};
	u8 Vy[]={"Vy:   0.0"};
	u8 Mag[]={"Mag:    x"};
	u8 Pos[]={"Pos:    x"};
	u8 Sensor[]={"xxxxxxxxx"};
	sendLedSignal(LEDSignal_Start2);
	LCD_ShowString(130,0,WhuFly,RED,WHITE,24,0);
	LCD_ShowString(135,24,Auther,BLUE,WHITE,16,0);
	LCD_ShowPicture(0,0,91,57,gImage_Competition);
	LCD_ShowString(94,60,HthA,RED,WHITE,16,0);
	LCD_ShowString(94,76,Rol,RED,WHITE,16,0);
	LCD_ShowString(94,92,Pit,RED,WHITE,16,0);
	LCD_ShowString(94,108,Yaw,RED,WHITE,16,0);
	LCD_ShowString(94,124,Vz,RED,WHITE,16,0);
	LCD_ShowString(94,140,HthP,RED,WHITE,16,0);
	LCD_ShowString(94,156,Vx,RED,WHITE,16,0);
	LCD_ShowString(94,172,Vy,RED,WHITE,16,0);
	LCD_ShowString(94,188,Mag,RED,WHITE,16,0);
	LCD_ShowString(94,204,Pos,RED,WHITE,16,0);
	LCD_ShowString(94,220,Sensor,RED,WHITE,16,0);
}

void init_GUI()
{
	GUI_Server();
}