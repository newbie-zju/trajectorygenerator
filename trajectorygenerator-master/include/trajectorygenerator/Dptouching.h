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
//输入
/*
Dp_pos quadrotorPos = Dp_pos(1.5,1.5,2),//四旋翼位置
	irobotPos = Dp_pos(2,3,0);//小车位置
float irobotDir = 0;//小车方向
//输出
Dp_pos outputPos;//输出位置
*/
//DPstate quadrotorState = APPROACH;//四旋翼状态
#define xMax 5.0						//场地范围#8.27
#define yMax 5.0						//场地范围#8.27
//xMin 和 yMin 为 0
#define dxy 0.5				//偏移量#8.28
#define tarV 0.5			//巡航速度大小#8.28


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
	//默认构造函数
	DpTouching(ros::NodeHandle nh_);
	ros::Subscriber quadrotorPosNED_sub;
	ros::Subscriber quadrotorPosGround_sub;
	ros::ServiceServer TG_server;
	ros::ServiceClient tf_client;
	tf::TransformListener listener;
	//输入规划开始位置
	void getBeginPos(float x, float y, float z);
	//输入规划目标位置
	void getTargetPos(float x, float y, float z);
	//进行运算
	void runMethod(void);
	void initialize();
	bool calculateTrajectoryCallback(iarc_mission::TG::Request &req, iarc_mission::TG::Response &res);
	void quadrotorPosNEDCallback(const dji_sdk::LocalPosition::ConstPtr &msg);
	void quadrotorPosGroundCallback(const geometry_msgs::Point::ConstPtr &msg);
	bool insideRec(float tx,float ty,float x1,float y1,float x2,float y2);
};



#endif
