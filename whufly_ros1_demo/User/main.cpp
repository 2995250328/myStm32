#include <stdio.h>
#include "hardwareserial.h"
#include <ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "rospy_tutorials/AddTwoInts.h"

using rospy_tutorials::AddTwoInts;

//Ros�ڵ���
ros::NodeHandle nh;

//����˻ص�����
void add(const AddTwoInts::Request &req, AddTwoInts::Response &res)
{
	res.sum = req.a + req.b;
}

//LED��ʼ��
void init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

//LED����
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

//�ص�����
void command_callback( const std_msgs::Float64& cmd_msg)
{
	if(cmd_msg.data==1)
		on_off(true);
	else
		on_off(false);
}

//�ص�����
void ledservice_callback( const std_msgs::Float64& cmd_msg)
{
	if(cmd_msg.data==1)
		on_off(true);
	else
		on_off(false);
}

//������Ϣ����
std_msgs::String str_msg;
//������
ros::Publisher chatter("chatter", &str_msg);
//������Ϣ����
char hello[] = "Hello world!";
//����LED����
ros::Subscriber<std_msgs::Float64> cmd_sub("cmd_led", command_callback);
//���������
ros::ServiceServer<AddTwoInts::Request,AddTwoInts::Response> server("add_two_ints", &add);

/*
 *      WHU-FLY
 * ������λ���人��ѧ
 * �汾��1.0
 * �޸�ʱ�䣺2023-05-23
 * Brand: WHU
 * Version:1.0
 * Update��2023-05-23
 * All rights reserved ByWX
 */

//������
int main(void) 
{
	bool OnOff = true;
	//ϵͳʱ�ӳ�ʼ��
	SystemInit();
	//delay��ʼ��
	initialise();
	//LED��ʼ��
	init();
	//ROS�ڵ��ʼ��
	nh.initNode();	
	//�ȴ�ROS����
	while(!nh.connected())
	{
		nh.spinOnce();
	}
	//�������� /chatter
	nh.advertise(chatter);
	//���Ļ��� /cmd_led
	nh.subscribe(cmd_sub);
	//����� /add_two_ints
	nh.advertiseService(server);
	//��ʾ���ӳɹ�
	nh.loginfo("Whufly Connection Successful!");
	nh.loginfo("Welcome to use Whufly!");
	while(1)
	{ 
		//������Ϣ
		str_msg.data = hello;
		chatter.publish(&str_msg);
		//ѭ��
		nh.spinOnce();
	}
}


