#include <iostream>
#include "euler.h"
using namespace std;
const double alpha = 0.00001;

double normal(double *v,int sign)//向量归一化
{
	double result;

	result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if(1==sign)
	{
		if(fabs(result-0)>alpha)
		{
			v[0] = v[0]/result;
			v[1] = v[1]/result;
			v[2] = v[2]/result;
		}
	}
	return result;
}




void compute(double *q,double *vectorBefore,double *vectorAfter)//求两个向量变换的四元数
{
	double rotationAxis[3];//旋转轴
	double rotationAngle;//向量夹角
	double dot_sum;
	rotationAxis[0] = vectorBefore[1] * vectorAfter[2] - vectorBefore[2] * vectorAfter[1];
	rotationAxis[1] = vectorBefore[2] * vectorAfter[0] - vectorBefore[0] * vectorAfter[2];
	rotationAxis[2] = vectorBefore[0] * vectorAfter[1] - vectorBefore[1] * vectorAfter[0];
	normal(rotationAxis,1);
	dot_sum = vectorBefore[0] * vectorAfter[0] + vectorBefore[1] * vectorAfter[1] + vectorBefore[2] * vectorAfter[2];
	double angle = dot_sum/normal(vectorAfter,0)/normal(vectorBefore,0);//求向量夹角
	if (fabs(angle-1)<alpha)
		rotationAngle = acos(1.0);
	else
		rotationAngle = acos(angle);

	q[0] = cos(rotationAngle/2);
	q[1] = rotationAxis[0]*sin(rotationAngle/2);
	q[2] = rotationAxis[1]*sin(rotationAngle/2);
	q[3] = rotationAxis[2]*sin(rotationAngle/2);
}






int
main(int argc, char **argv)
{
	double a[3] = {1,1,1};
	double b[3] = {1,2,3};
	double q[4] = {0,0,0,0};
	compute(q,a,b);//计算四元数
	double res[3] = {0,0,0};
	Quaternion q_res(q[1],q[2],q[3],q[0]);
	quaternion2Euler(q_res,res,zyz);
	for (int i = 0;i < 3;i++)
	{
		std::cout<<res[i]<<std::endl;
	}
	return 1;   
}
