#pragma once

//	a_matrix:转置后的矩阵
//	b_matrix:转置前的矩阵
//	krow    :行数
//	kline   :列数
void matrix_t(double **a_matrix, const double **b_matrix, int krow, int kline);

//	a_matrix=b_matrix+c_matrix
//	 krow   :行数
//	 kline  :列数
//	 ktrl   :大于0: 加法  不大于0:减法
void matrix_a(double **a_matrix, const double **b_matrix, const double **c_matrix, 
					int krow, int kline, int ktrl);

//	a_matrix=b_matrix*c_matrix
//	krow  :行数
//	kline :列数
//  kmiddle：第二个矩阵的列数，也是第三个矩阵的行数。
//	ktrl  :	大于0:两个正数矩阵相乘 不大于0:正数矩阵乘以负数矩阵 
void matrix_m(double **a_matrix, const double **b_matrix, const double **c_matrix,
				int krow, int kline, int kmiddle, int ktrl);

//	a_matrix:矩阵
//	ndimen :维数		
int  matrix_inv(double **a_matrix, int ndimen);
 		
 


 
