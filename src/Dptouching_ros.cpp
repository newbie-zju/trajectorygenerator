#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include "Dptouching.h"
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include "iarc_mission/TG.h"
using namespace std;


#define xMax 20.0						//场地范围#8.27
#define yMax 20.0						//场地范围#8.27
//xMin 和 yMin 为 0

//输入
/*
Dp_pos quadrotorPos = Dp_pos(1.5,1.5,2),//四旋翼位置
	irobotPos = Dp_pos(2,3,0);//小车位置
float irobotDir = 0;//小车方向
//输出
Dp_pos outputPos;//输出位置
*/
//DPstate quadrotorState = APPROACH;//四旋翼状态
dji_sdk::LocalPosition quadrotorPos;
goal_detected::Pose3D irobotPos;
geometry_msgs::Point32 outputPos;
float tarX, tarY, tarZ;
float tarVx, tarVy, tarV = 0.5;//tarV:巡航速度大小
DpTouching DpTouch;
void quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPos.x = msg->x;
	quadrotorPos.y = msg->y;
	quadrotorPos.z = msg->z;
}
/*
void irobotPosCallback(const goal_detected::Pose3DConstPtr &msg)
{
	irobotPos.x = msg->x;
	irobotPos.y = msg->y;
	irobotPos.z = msg->z;
	irobotPos.theta = msg->theta;
}
*/
bool calculateTrajectoryCallback(iarc_mission::TG::Request &req, iarc_mission::TG::Response &res)
{
	switch(req.quadrotorState)
	{
		case CRUISE:	//TODO: here is not NED frame!!!
			ROS_INFO("DpTouching: CRUISE");
			tarZ = 1.6;
			if (insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.1*yMax,0.9*xMax,0.9*yMax))//在(0.1,0.1)(0.9,0.9)矩形里面
			{
				if(insideRec(quadrotorPos.x,quadrotorPos.y,0.15*xMax,0.15*yMax,0.85*xMax,0.85*yMax))
				{
					double theta_center2quad = atan2((quadrotorPos.y-yMax/2),(quadrotorPos.x-xMax/2));//场地中心指向四旋翼的向量角度
					tarVx = cos(theta_center2quad)*tarV;
					tarVy = sin(theta_center2quad)*tarV;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.1*yMax,0.85*xMax,0.15*yMax))
				{
					tarVx = tarV;
					tarVy = 0;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.9*xMax,0.1*yMax,0.85*xMax,0.85*yMax))
				{
					tarVx = 0;
					tarVy = tarV;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.9*xMax,0.85*yMax,0.15*xMax,0.9*yMax))
				{
					tarVx = -tarV;
					tarVy = 0;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.15*xMax,0.15*yMax,0.1*xMax,0.9*yMax))
				{
					tarVx = 0;
					tarVy = -tarV;
				}
				else
				{
					tarVx = 0;
					tarVy = 0;
				}

			}
			else//在(0.1,0.1)(0.9,0.9)矩形外面
			{
				double theta_quad2center = atan2((yMax/2-quadrotorPos.y),(xMax/2-quadrotorPos.x));//四旋翼指向场地中心的向量角度
				tarVx = cos(theta_quad2center)*tarV;
				tarVy = sin(theta_quad2center)*tarV;
			}
			res.flightCtrlDstx = tarVx;
			res.flightCtrlDsty = tarVy;
			res.flightCtrlDstz = tarZ;
			break;
		case TRACK:
			ROS_INFO("DpTouching: TRACK");
			tarX = req.irobotPosNEDx + 0.5 * cos(req.theta);
			tarY = req.irobotPosNEDy + 0.5 * sin(req.theta);
			tarZ = 1.6;

			DpTouch.getBeginPos(quadrotorPos.x,quadrotorPos.y,quadrotorPos.z);
			DpTouch.getTargetPos(tarX,tarY,tarZ);
			ROS_INFO("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f",quadrotorPos.x,quadrotorPos.y,quadrotorPos.z,tarX,tarY,tarZ);
			DpTouch.runMethod();
			res.flightCtrlDstx = DpTouch.p_x[40];
			res.flightCtrlDsty = DpTouch.p_y[40];
			res.flightCtrlDstz = DpTouch.p_z[49];
			break;
		case APPROACH:
			ROS_INFO("DpTouching: APPROACH");
			tarX = req.irobotPosNEDx + 2.3 * cos(req.theta);
			tarY = req.irobotPosNEDy + 2.3 * sin(req.theta);
			tarZ = -0.5;

			DpTouch.getBeginPos(quadrotorPos.x,quadrotorPos.y,quadrotorPos.z);
			DpTouch.getTargetPos(tarX,tarY,tarZ);
			ROS_INFO("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f",quadrotorPos.x,quadrotorPos.y,quadrotorPos.z,tarX,tarY,tarZ);
			DpTouch.runMethod();
			res.flightCtrlDstx = DpTouch.p_x[40];
			res.flightCtrlDsty = DpTouch.p_y[40];
			res.flightCtrlDstz = DpTouch.p_z[49];
			break;
	}
	return true;

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Dptouching_node");
	ros::NodeHandle nh;
	ros::Subscriber quadrotorPos_sub = nh.subscribe("/dji_sdk/local_position", 10, quadrotorPosCallback);
	//ros::Subscriber irobotPos_sub = nh.subscribe("/goal_detected/goal_pose", 10, irobotPosCallback);
	//ros::Publisher outputPos_pub = nh.advertise<geometry_msgs::Point32>("/TG/flight_ctrl_dst",10);
	ros::ServiceServer TG_server = nh.advertiseService("/TG/TG_service", calculateTrajectoryCallback);
	ros::spin();
	return 0;
}
