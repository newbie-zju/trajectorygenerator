#ifndef __DPTOUCHING_H__
#define __DPTOUCHING_H__
#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include "iarc_mission/TG.h"
#include <iarc_tf/Velocity.h>
//����
/*
Dp_pos quadrotorPos = Dp_pos(1.5,1.5,2),//������λ��
	irobotPos = Dp_pos(2,3,0);//С��λ��
float irobotDir = 0;//С������
//���
Dp_pos outputPos;//���λ��
*/
//DPstate quadrotorState = APPROACH;//������״̬
#define xMax 5.0						//���ط�Χ#8.27
#define yMax 5.0						//���ط�Χ#8.27
//xMin �� yMin Ϊ 0
#define dxy 0.5				//ƫ����#8.28
#define tarV 0.5			//Ѳ���ٶȴ�С#8.28


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
	dji_sdk::LocalPosition quadrotorPosNED;
	dji_sdk::LocalPosition quadrotorPos;
	goal_detected::Pose3D irobotPos;
	geometry_msgs::Point32 outputPos;
	int cruiseStep;//
	float tarX;
	float tarY;
	float tarZ;
	float tarVx;
	float tarVy;
	enum VelState{NED,GROUND};
	ros::NodeHandle nh;
	//Ĭ�Ϲ��캯��
	DpTouching(ros::NodeHandle nh_);
	ros::Subscriber quadrotorPosNED_sub;
	ros::Subscriber quadrotorPosGround_sub;
	ros::ServiceServer TG_server;
	ros::ServiceClient tf_client;
	tf::TransformListener listener;
	//����滮��ʼλ��
	void getBeginPos(float x, float y, float z);
	//����滮Ŀ��λ��
	void getTargetPos(float x, float y, float z);
	//��������
	void runMethod(void);
	void initialize();
	bool calculateTrajectoryCallback(iarc_mission::TG::Request &req, iarc_mission::TG::Response &res);
	void quadrotorPosNEDCallback(const dji_sdk::LocalPosition::ConstPtr &msg);
	void quadrotorPosGroundCallback(const geometry_msgs::Point::ConstPtr &msg);
	bool insideRec(float tx,float ty,float x1,float y1,float x2,float y2);
};



#endif