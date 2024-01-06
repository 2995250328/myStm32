#include <stdio.h>
#include "hardwareserial.h"
#include <ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "rospy_tutorials/AddTwoInts.h"

using rospy_tutorials::AddTwoInts;

//Ros节点句柄
ros::NodeHandle nh;

//服务端回调函数
void add(const AddTwoInts::Request &req, AddTwoInts::Response &res)
{
	res.sum = req.a + req.b;
}

//LED初始化
void init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

//LED开关
void on_off(bool status)
{
	if(status == true)
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_0);
	}
	else
	{
		GPIO_SetBits(GPIOE, GPIO_Pin_0);
	}
}

//回调函数
void command_callback( const std_msgs::Float64& cmd_msg)
{
	if(cmd_msg.data==1)
		on_off(true);
	else
		on_off(false);
}

//回调函数
void ledservice_callback( const std_msgs::Float64& cmd_msg)
{
	if(cmd_msg.data==1)
		on_off(true);
	else
		on_off(false);
}

//发布消息话题
std_msgs::String str_msg;
//发布者
ros::Publisher chatter("chatter", &str_msg);
//发布消息内容
char hello[] = "Hello world!";
//订阅LED话题
ros::Subscriber<std_msgs::Float64> cmd_sub("cmd_led", command_callback);
//声明服务端
ros::ServiceServer<AddTwoInts::Request,AddTwoInts::Response> server("add_two_ints", &add);

/*
 *      WHU-FLY
 * 所属单位：武汉大学
 * 版本：1.0
 * 修改时间：2023-05-23
 * Brand: WHU
 * Version:1.0
 * Update：2023-05-23
 * All rights reserved ByWX
 */

//主函数
int main(void) 
{
	bool OnOff = true;
	//系统时钟初始化
	SystemInit();
	//delay初始化
	initialise();
	//LED初始化
	init();
	//ROS节点初始化
	nh.initNode();	
	//等待ROS连接
	while(!nh.connected())
	{
		nh.spinOnce();
	}
	//建立话题 /chatter
	nh.advertise(chatter);
	//订阅话题 /cmd_led
	nh.subscribe(cmd_sub);
	//服务端 /add_two_ints
	nh.advertiseService(server);
	//提示连接成功
	nh.loginfo("Whufly Connection Successful!");
	nh.loginfo("Welcome to use Whufly!");
	while(1)
	{ 
		//发布消息
		str_msg.data = hello;
		chatter.publish(&str_msg);
		//循环
		nh.spinOnce();
	}
}


