#pragma once
 
//������֡���ݻ�����
extern unsigned char DataScope_OutPut_Buffer[42];	   
//дͨ��������������֡���ݻ�����
extern "C"{
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    
//����֡�������ɺ���
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);   
}




