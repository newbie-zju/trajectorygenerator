#ifndef __DPTOUCHING_H__
#define __DPTOUCHING_H__
#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

enum DPstate{FREE,CRUISE,TRACK,APPROACH};//飞行器状态：巡航、跟踪、接近

//位置类
struct Dp_pos
{
	float x,y,z;
	Dp_pos()
	{
		x=0.0f;
		y=0.0f;
		z=0.0f;
	}
	Dp_pos(float x0,float y0,float z0)
	{
		x=x0;
		y=y0;
		z=z0;
	}
};


class DpTouching
{
public:
	#define  DP_N 50//步数
	float dT;//discrete system sample time
	vector<Mat> q;//飞机位置与速度
	Mat q_ugv;//目标
	float p_x[DP_N];//规划x
	float p_y[DP_N];
	float p_z[DP_N];
	//DPstate state;//飞机状态
	//float cruise_height,track_height,approach_height;//巡航、跟踪、接近的高度设置

	

	//默认构造函数
	DpTouching()
	{
		dT = 0.1;
		//state = FREE;
		//cruise_height = 2;
		//track_height = 1;
		//approach_height = 0;
	}

	//输入规划开始位置
	void getBeginPos(float x, float y, float z);
	//输入规划目标位置
	void getTargetPos(float x, float y, float z);
	//进行运算
	void runMethod(void);

};

//判断是否在矩形内
bool insideRec(float tx,float ty,float x1,float y1,float x2,float y2);


#endif