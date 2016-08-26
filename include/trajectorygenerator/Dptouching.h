#ifndef __DPTOUCHING_H__
#define __DPTOUCHING_H__
#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

enum DPstate{FREE,CRUISE,TRACK,APPROACH};//������״̬��Ѳ�������١��ӽ�

//λ����
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
	#define  DP_N 50//����
	float dT;//discrete system sample time
	vector<Mat> q;//�ɻ�λ�����ٶ�
	Mat q_ugv;//Ŀ��
	float p_x[DP_N];//�滮x
	float p_y[DP_N];
	float p_z[DP_N];
	//DPstate state;//�ɻ�״̬
	//float cruise_height,track_height,approach_height;//Ѳ�������١��ӽ��ĸ߶�����

	

	//Ĭ�Ϲ��캯��
	DpTouching()
	{
		dT = 0.1;
		//state = FREE;
		//cruise_height = 2;
		//track_height = 1;
		//approach_height = 0;
	}

	//����滮��ʼλ��
	void getBeginPos(float x, float y, float z);
	//����滮Ŀ��λ��
	void getTargetPos(float x, float y, float z);
	//��������
	void runMethod(void);

};

//�ж��Ƿ��ھ�����
bool insideRec(float tx,float ty,float x1,float y1,float x2,float y2);


#endif