#include <stdio.h>
#include <math.h>
#include "matrix.h"

void matrix_t(double **a_matrix, const double **b_matrix, int krow, int kline)
 
//	a_matrix:转置后的矩阵
//	b_matrix:转置前的矩阵
//	krow    :行数
//	kline   :列数
 
{
	int k, k2;   
 
	for (k = 0; k < krow; k++)
	{
		for(k2 = 0; k2 < kline; k2++)
		{
			a_matrix[k2][k] = b_matrix[k][k2];
		}
	}
}

void matrix_a(double **a_matrix, const double **b_matrix, const double **c_matrix, 
					int krow, int kline, int ktrl)
 
//	a_matrix=b_matrix+c_matrix
//	 krow   :行数
//	 kline  :列数
//	 ktrl   :大于0: 加法  不大于0:减法
 
{
	int k, k2;
 
	for (k = 0; k < krow; k++)
	{
		for(k2 = 0; k2 < kline; k2++)
		{
			a_matrix[k][k2] = b_matrix[k][k2]
				+ ((ktrl > 0) ? c_matrix[k][k2] : -c_matrix[k][k2]); 
		}
	}
}

void matrix_m(double **a_matrix, const double **b_matrix, const double **c_matrix,
				int krow, int kline, int kmiddle, int ktrl)
 
//	a_matrix=b_matrix*c_matrix
//	krow  :行数
//	kline :列数
//  kmiddle：第二个矩阵的列数，也是第三个矩阵的行数。
//	ktrl  :	大于0:两个正数矩阵相乘 不大于0:正数矩阵乘以负数矩阵
 
{
	int k, k2, k4;
	double stmp;
 
	for (k = 0; k < krow; k++)     
	{
		for (k2 = 0; k2 < kline; k2++)   
		{
			stmp = 0.0;
			for (k4 = 0; k4 < kmiddle; k4++)  
			{
				stmp += b_matrix[k][k4] * c_matrix[k4][k2];
			}
			a_matrix[k][k2] = stmp;
		}
	}
	if (ktrl <= 0)   
	{
		for (k = 0; k < krow; k++)
		{
			for (k2 = 0; k2 < kline; k2++)
			{
				a_matrix[k][k2] = -a_matrix[k][k2];
			}
		}
	}
}

int  matrix_inv(double **a_matrix, int ndimen)
 
//	a_matrix:矩阵
//	ndimen :维数
 
{
	double tmp, tmp2, b_tmp[20], c_tmp[20];
	int k, k1, k2, k3, j, i, j2, i2, kme[20], kmf[20];
	i2 = j2 = 0;
 
	for (k = 0; k < ndimen; k++)  
	{
		tmp2 = 0.0;
		for (i = k; i < ndimen; i++)  
		{
			for (j = k; j < ndimen; j++)  
			{
				if (fabs(a_matrix[i][j] ) <= fabs(tmp2)) 
					continue;
				tmp2 = a_matrix[i][j];
				i2 = i;
				j2 = j;
			}  
		}
		if (i2 != k) 
		{
			for (j = 0; j < ndimen; j++)   
			{
				tmp = a_matrix[i2][j];
				a_matrix[i2][j] = a_matrix[k][j];
				a_matrix[k][j] = tmp;
			}
		}
		if (j2 != k) 
		{
			for (i = 0; i < ndimen; i++)  
			{
				tmp = a_matrix[i][j2];
				a_matrix[i][j2] = a_matrix[i][k];
				a_matrix[i][k] = tmp;
			}    
		}
		kme[k] = i2;
		kmf[k] = j2;
		for (j = 0; j < ndimen; j++)  
		{
			if (j == k)   
			{
				b_tmp[j] = 1.0 / tmp2;
				c_tmp[j] = 1.0;
			}
			else 
			{
				b_tmp[j] = -a_matrix[k][j] / tmp2;
				c_tmp[j] = a_matrix[j][k];
			}
			a_matrix[k][j] = 0.0;
			a_matrix[j][k] = 0.0;
		}
		for (i = 0; i < ndimen; i++)  
		{
			for (j = 0; j < ndimen; j++)  
			{
				a_matrix[i][j] = a_matrix[i][j] + c_tmp[i] * b_tmp[j];
			}  
		}
	}
	for (k3 = 0; k3 < ndimen;  k3++)   
	{
		k  = ndimen - k3 - 1;
		k1 = kme[k];
		k2 = kmf[k];
		if (k1 != k)   
		{
			for (i = 0; i < ndimen; i++)  
			{
				tmp = a_matrix[i][k1];
				a_matrix[i][k1] = a_matrix[i][k];
				a_matrix[i][k] = tmp;
			}  
		}
		if (k2 != k)   
		{
			for(j = 0; j < ndimen; j++)  
			{
				tmp = a_matrix[k2][j];
				a_matrix[k2][j] = a_matrix[k][j];
				a_matrix[k][j] = tmp;
			}
		}
	}
	return (0);
}