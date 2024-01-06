#pragma once

//	a_matrix:ת�ú�ľ���
//	b_matrix:ת��ǰ�ľ���
//	krow    :����
//	kline   :����
void matrix_t(double **a_matrix, const double **b_matrix, int krow, int kline);

//	a_matrix=b_matrix+c_matrix
//	 krow   :����
//	 kline  :����
//	 ktrl   :����0: �ӷ�  ������0:����
void matrix_a(double **a_matrix, const double **b_matrix, const double **c_matrix, 
					int krow, int kline, int ktrl);

//	a_matrix=b_matrix*c_matrix
//	krow  :����
//	kline :����
//  kmiddle���ڶ��������������Ҳ�ǵ����������������
//	ktrl  :	����0:��������������� ������0:����������Ը������� 
void matrix_m(double **a_matrix, const double **b_matrix, const double **c_matrix,
				int krow, int kline, int kmiddle, int ktrl);

//	a_matrix:����
//	ndimen :ά��		
int  matrix_inv(double **a_matrix, int ndimen);
 		
 


 
